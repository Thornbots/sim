# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The MCB on the far end of a pty from dji_serial_bridge, driving the gz sentry.

Opens a pty and links its slave at `device_link` for the bridge's `device`.
Runs sim.mcb_emulator's 1 kHz loop on sim time in `batch_ms` batches. Reads
/sim/raw_odom and /sim/raw_joint_states; writes /head_pan_cmd,
/head_pitch_cmd and /cmd_vel. Each shot goes out on ~/shot, a
std_msgs/Header stamped with the sim time the indexer fired. Referee state
is parameters (settable live). see README.md for design rationale
"""
import math
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sim.mcb_emulator.drive import rotate
from sim.mcb_emulator.pty_link import PtyLink
from sim.mcb_emulator.sentry import DRIVE_STOP, Hardware, Sentry
from sim.mcb_emulator.subsystems import RefSerial
from std_msgs.msg import Float64, Header

HEADPITCH_LIMIT = 0.6  # sentry_v2.urdf.xacro headpitch
MAX_CATCH_UP_MS = 100  # a stalled clock doesn't make one huge batch
REFEREE_PARAMS = {
    'game_type': 4, 'game_stage': 4, 'stage_time_remaining': 300, 'robot_id': 107,
    'current_hp': 400, 'max_hp': 400, 'restoration_zone': False, 'exchange_zone': False,
    'central_buff_zone': False, 'shooter_power': True}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GzHardware(Hardware):
    """
    The gz sentry as the firmware senses it: IMU yaw and odometry zeroed at boot.

    gz yaws are CCW; headlink turns about -z, so the turret's world yaw is
    chassis yaw minus the joint. The Pico frame is x right, y forward of the
    turret's boot heading (OdometryPointForwardsCommand holds the pods there).
    """

    def __init__(self):
        self.ready = False
        self._seen = set()
        self.chassis_yaw = self.chassis_rate = 0.0
        self.pos = (0.0, 0.0)
        self.vel = (0.0, 0.0)  # world
        self.joint = {'headlink': (0.0, 0.0), 'headpitch': (0.0, 0.0)}
        self.boot = None  # (x, y, turret world yaw) at the first full sample
        self.yaw_cmd = self.pitch_cmd = None
        self.drive = None
        self.shots = []
        self.time_ms = 0

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.chassis_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.chassis_rate = msg.twist.twist.angular.z
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.vel = rotate(msg.twist.twist.linear.x, msg.twist.twist.linear.y, self.chassis_yaw)
        self._maybe_boot('odom')

    def on_joints(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint:
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                self.joint[name] = (msg.position[i], vel)
        self._maybe_boot('joints')

    def _maybe_boot(self, source):
        self._seen.add(source)
        if self.boot is None and self._seen == {'odom', 'joints'}:
            self.boot = (*self.pos, self.turret_world_yaw())
            self.ready = True

    def turret_world_yaw(self):
        return self.chassis_yaw - self.joint['headlink'][0]

    def imu_yaw(self):
        return wrap(self.turret_world_yaw() - self.boot[2])

    def imu_gz(self):
        return self.chassis_rate - self.joint['headlink'][1]

    def yaw_encoder(self):
        return -self.joint['headlink'][0]

    def yaw_encoder_rate(self):
        return -self.joint['headlink'][1]

    def pitch_encoder(self):
        return self.joint['headpitch'][0]

    def pitch_rate(self):
        return self.joint['headpitch'][1]

    def odom(self):
        dx, dy = self.pos[0] - self.boot[0], self.pos[1] - self.boot[1]
        fwd, left = rotate(dx, dy, -self.boot[2])
        vf, vl = rotate(*self.vel, -self.boot[2])
        return -left, fwd, -vl, vf

    def set_gimbal(self, world_yaw, pitch):
        if world_yaw is None:
            self.yaw_cmd = None
        else:
            joint_now = self.joint['headlink'][0]
            desired = self.chassis_yaw - (world_yaw + self.boot[2])
            self.yaw_cmd = joint_now + wrap(desired - joint_now)
        self.pitch_cmd = min(max(pitch, -HEADPITCH_LIMIT), HEADPITCH_LIMIT)

    def set_chassis(self, drive):
        self.drive = drive

    def shoot(self):
        self.shots.append(self.time_ms)


class McbEmulator(Node):

    def __init__(self):
        super().__init__('mcb_emulator')
        self.declare_parameter('device_link', '/tmp/mcb_emulator_pty')
        self.declare_parameter('drive', DRIVE_STOP)  # stop, simple or auto: SentryControl's
        self.declare_parameter('auto_fire', True)
        self.declare_parameter('batch_ms', 5)
        self.declare_parameter('stats_period_s', 5.0)
        for name, default in REFEREE_PARAMS.items():
            self.declare_parameter(name, default)
        self.ref = RefSerial()
        self._apply_referee({n: self.get_parameter(n).value for n in REFEREE_PARAMS})
        self.add_on_set_parameters_callback(self._on_params)

        self.hw = GzHardware()
        self.sentry = Sentry(self.hw, self.ref, self.get_parameter('auto_fire').value,
                             self.get_parameter('drive').value)
        self.pty = PtyLink(self.get_parameter('device_link').value)

        self.pan_pub = self.create_publisher(Float64, '/head_pan_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/head_pitch_cmd', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.shot_pub = self.create_publisher(Header, '~/shot', 10)
        self.create_subscription(Odometry, '/sim/raw_odom', self.hw.on_odom, 10)
        self.create_subscription(JointState, '/sim/raw_joint_states', self.hw.on_joints, 10)
        self.last_ms = None
        self.create_timer(self.get_parameter('batch_ms').value / 1000.0, self._tick)
        self.create_timer(self.get_parameter('stats_period_s').value, self._log_stats)
        self.get_logger().info(
            f'MCB emulator (MCBV3 708b8d6) on {self.pty.link} -> {os.ttyname(self.pty.slave)}, '
            f"drive={self.get_parameter('drive').value}, "
            f"auto_fire={self.get_parameter('auto_fire').value}")

    def _apply_referee(self, values):
        for name, value in values.items():
            setattr(self.ref, name, value)

    def _on_params(self, params):
        self._apply_referee({p.name: p.value for p in params if p.name in REFEREE_PARAMS})
        return SetParametersResult(successful=True)

    def _tick(self):
        now_ms = self.get_clock().now().nanoseconds // 1_000_000
        if not self.hw.ready:
            self.last_ms = now_ms
            return
        cycles = min(now_ms - self.last_ms, MAX_CATCH_UP_MS) if self.last_ms else 1
        self.last_ms = now_ms
        if cycles <= 0:
            return
        to_sim_ms = now_ms - cycles - self.sentry.drivers.time_ms  # firmware ms -> sim ms
        self.sentry.receive(self.pty.read(), cycles)
        for _ in range(cycles):
            self.sentry.step()
        for t_ms in self.hw.shots:
            stamp = rclpy.time.Time(nanoseconds=(t_ms + to_sim_ms) * 1_000_000)
            self.shot_pub.publish(Header(stamp=stamp.to_msg(), frame_id='muzzle'))
        self.hw.shots.clear()
        out = self.sentry.drain_tx()
        if out:
            self.pty.write(out)
        self._publish_commands()

    def _publish_commands(self):
        hw = self.hw
        if hw.yaw_cmd is not None:
            self.pan_pub.publish(Float64(data=hw.yaw_cmd))
        if hw.pitch_cmd is not None:
            self.pitch_pub.publish(Float64(data=hw.pitch_cmd))
        twist = Twist()
        if hw.drive is not None:
            right, fwd, spin = hw.drive
            twist.linear.x, twist.linear.y, twist.angular.z = fwd, -right, spin
        self.cmd_vel_pub.publish(twist)

    def _log_stats(self):
        self.get_logger().info(f'rx {self.sentry.stats() or "nothing"}; '
                               f'tx dropped {self.pty.dropped} B')

    def destroy_node(self):
        self.pty.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = McbEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
