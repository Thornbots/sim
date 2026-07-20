"""
Emulates the real Type-C board's POSE_MSG interface in sim: republishes
sim's raw ground-truth /sim/raw_odom + /sim/raw_joint_states (bridged
straight from gz, see sim.launch.py) as a dji_serial_bridge/msg/RobotPose
on /pose -- the same topic/message real hardware's Type-C board sends
(via ros2_dji_serial_bridge's dji_serial_bridge_node, also on /pose).
sentry_pkg's pose_translator is the only thing that ever consumes pose
data downstream of this, for both sim and real hardware, so sim's job
here is purely to speak the same wire format, not to do anything
SLAM-specific itself.
"""
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from dji_serial_bridge.msg import RobotPose


class PoseEmulator(Node):
    def __init__(self):
        super().__init__('pose_emulator')

        self.declare_parameter('yaw_joint_name', 'headlink')
        self.yaw_joint_name = self.get_parameter('yaw_joint_name').value
        self.head_yaw = 0.0

        # Real hardware's wheel odometry accumulates drift (wheel slip,
        # encoder error -- worse on the arena's "Bumpy Road" zone) that
        # slam_toolbox's map->odom correction exists to compensate for. Sim's
        # ground truth has none of that by default, which is fine for most
        # testing but leaves slam_toolbox's correction behavior completely
        # unexercised. These params optionally inject synthetic position
        # drift/noise so that path can actually be demonstrated in sim; off
        # by default so existing ground-truth behavior is unchanged unless
        # explicitly opted into.
        self.declare_parameter('odom_noise_enabled', False)
        # Random-walk step stddev (meters/callback) added to a persistent
        # drift offset each callback -- accumulates over time like real
        # wheel-slip drift, rather than resetting every sample.
        self.declare_parameter('odom_drift_stddev', 0.0005)
        # Independent per-sample jitter stddev (meters), added on top of the
        # drift offset each callback -- simulates ordinary encoder/sensor
        # noise, does not accumulate.
        self.declare_parameter('odom_jitter_stddev', 0.001)

        self._drift_x = 0.0
        self._drift_y = 0.0

        self.pose_pub = self.create_publisher(RobotPose, '/pose', 10)
        self.create_subscription(Odometry, '/sim/raw_odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/sim/raw_joint_states', self.joint_callback, 10)

    def joint_callback(self, msg):
        if self.yaw_joint_name in msg.name:
            self.head_yaw = msg.position[msg.name.index(self.yaw_joint_name)]

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        if self.get_parameter('odom_noise_enabled').value:
            drift_stddev = self.get_parameter('odom_drift_stddev').value
            jitter_stddev = self.get_parameter('odom_jitter_stddev').value

            # Persistent random walk -- this is the "drift" that slam_toolbox
            # should end up correcting for in map->odom.
            self._drift_x += random.gauss(0.0, drift_stddev)
            self._drift_y += random.gauss(0.0, drift_stddev)

            # Independent per-sample jitter on top, not accumulated.
            jitter_x = random.gauss(0.0, jitter_stddev)
            jitter_y = random.gauss(0.0, jitter_stddev)

            x += self._drift_x + jitter_x
            y += self._drift_y + jitter_y

        pose = RobotPose()
        pose.header.stamp = msg.header.stamp
        pose.x = x
        pose.y = y
        pose.vel_x = float(msg.twist.twist.linear.x)
        pose.vel_y = float(msg.twist.twist.linear.y)
        pose.head_pitch = 0.0
        pose.head_yaw = float(self.head_yaw)
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEmulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
