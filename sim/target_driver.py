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
Simulated fast-moving-*robot* ground truth: chassis plus spin, not a single point.

There is no gz entity, model, or plugin. This node just integrates its
own chassis (x, y, z, yaw) state in a timer callback and publishes nav_msgs/Odometry on
/target/ground_truth_odom, the same way pose_emulator.py stands in for
real hardware without touching gz. Models the opponent-robot behavior
documented in ARCC_2026_SENTRY_CONTEXT.md's "Opponent robot
characteristics" section: chassis translates (lateral bounce, up to 4 m/s
per that doc) while continuously spinning in place at spin_hz (1-2 Hz
"wiggle" defense) -- cv_target_emulator.py derives the 4 armor-panel poses
from this chassis pose + a fixed panel layout, it is not itself a panel.
See README.md for the path shape and dwell-count rationale.

Frame: header.frame_id is set to match /sim/raw_odom's ('odom' by default,
see sentry.urdf.xacro's OdometryPublisher plugin) since cv_target_emulator
assumes both ground-truth topics share one world frame with no TF lookup.
"""
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class TargetDriver(Node):

    def __init__(self):
        super().__init__('target_driver')

        self.declare_parameter('target_speed', 2.0)
        self.declare_parameter('spin_hz', 1.5)
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('center_x', 3.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('half_width', 2.4)
        # Path direction: 0 runs along +y, across the view of a robot at the
        # origin facing +x; 90 runs along +x, straight down its camera ray.
        self.declare_parameter('path_angle_deg', 0.0)
        self.declare_parameter('target_z', 0.3)
        self.declare_parameter('frame_id', 'odom')
        # Acceleration limits, so the target brakes into each end of its path
        # and ramps to a new target_speed/spin_hz instead of jumping. Estimates.
        self.declare_parameter('max_accel', 6.0)  # m/s^2
        self.declare_parameter('max_spin_accel', 20.0)  # rad/s^2

        self.target_z = self.get_parameter('target_z').value
        self.frame_id = self.get_parameter('frame_id').value

        # Offset and speed along the path, from its centre.
        self.s = 0.0
        self.vs = 0.0
        self.direction = 1.0
        self.yaw = 0.0
        self.omega = 0.0
        self._last_time = None

        self.pub = self.create_publisher(Odometry, '/target/ground_truth_odom', 10)

        rate_hz = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        gp = self.get_parameter
        self.get_logger().info(
            f"target_driver ready: speed={gp('target_speed').value:.2f} m/s, "
            f"spin={gp('spin_hz').value:.2f} Hz, path centre=({gp('center_x').value:.2f}, "
            f"{gp('center_y').value:.2f}) +-{gp('half_width').value:.2f} m at "
            f"{gp('path_angle_deg').value:.0f} deg, z={self.target_z:.2f}, "
            f'frame_id={self.frame_id}'
        )

    def on_timer(self):
        # Advance by elapsed sim-time delta (self.get_clock().now(), which
        # resolves to /clock under use_sim_time), never by the assumed
        # timer period -- otherwise a real-time-factor != 1.0 makes true
        # speed/spin diverge from the target_speed/spin_hz params. First
        # tick has no prior sample to diff against, so it only seeds
        # _last_time.
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            self._publish(0.0, 0.0)
            return
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0:
            return

        # Read every tick, so the bench can change the path between cases.
        half = self.get_parameter('half_width').value
        accel = self.get_parameter('max_accel').value
        speed = self.get_parameter('target_speed').value
        # Brake so the target stops at the end it is heading for, then turn.
        to_end = half - self.s if self.direction > 0 else self.s + half
        if to_end <= 1e-3 and abs(self.vs) <= accel * dt:
            self.direction = -self.direction
            to_end = half - self.s if self.direction > 0 else self.s + half
        want = self.direction * min(speed, math.sqrt(2.0 * accel * max(to_end, 0.0)))
        self.vs += max(-accel * dt, min(accel * dt, want - self.vs))
        self.s = max(-half, min(half, self.s + self.vs * dt))

        spin_accel = self.get_parameter('max_spin_accel').value
        want_omega = 2.0 * math.pi * self.get_parameter('spin_hz').value
        self.omega += max(-spin_accel * dt, min(spin_accel * dt, want_omega - self.omega))
        self.yaw = (self.yaw + self.omega * dt + math.pi) % (2.0 * math.pi) - math.pi

        self._publish(self.vs, self.omega)

    def _publish(self, vs, omega):
        angle = math.radians(self.get_parameter('path_angle_deg').value)
        dx, dy = math.sin(angle), math.cos(angle)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = 'target'
        msg.pose.pose.position.x = float(self.get_parameter('center_x').value + self.s * dx)
        msg.pose.pose.position.y = float(self.get_parameter('center_y').value + self.s * dy)
        msg.pose.pose.position.z = float(self.target_z)
        # Yaw-only orientation (chassis spin, flat ground) as a quaternion.
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        # World-frame velocity, despite child_frame_id; consumers rely on it.
        msg.twist.twist.linear.x = float(vs * dx)
        msg.twist.twist.linear.y = float(vs * dy)
        msg.twist.twist.angular.z = float(omega)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
