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

        self.pose_pub = self.create_publisher(RobotPose, '/pose', 10)
        self.create_subscription(Odometry, '/sim/raw_odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/sim/raw_joint_states', self.joint_callback, 10)

    def joint_callback(self, msg):
        if self.yaw_joint_name in msg.name:
            self.head_yaw = msg.position[msg.name.index(self.yaw_joint_name)]

    def odom_callback(self, msg):
        pose = RobotPose()
        pose.header.stamp = msg.header.stamp
        pose.x = float(msg.pose.pose.position.x)
        pose.y = float(msg.pose.pose.position.y)
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
