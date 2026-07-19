"""
Splits a single /cmd_vel Twist into the 3 per-joint velocity commands the
sim robot's planar joint chain needs (see sentry.urdf.xacro): X slide, Y
slide, yaw rotate. Exists because the robot's base is now kinematically
constrained to X/Y/yaw only (to make tipping over impossible), and driving
a joint-constrained base needs per-joint velocity control rather than the
single-link VelocityControl plugin used before that restructuring.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class CmdVelToJoints(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_joints')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub_x = self.create_publisher(Float64, '/planar_x_vel_cmd', 10)
        self.pub_y = self.create_publisher(Float64, '/planar_y_vel_cmd', 10)
        self.pub_yaw = self.create_publisher(Float64, '/yaw_vel_cmd', 10)

    def cb(self, msg):
        self.pub_x.publish(Float64(data=msg.linear.x))
        self.pub_y.publish(Float64(data=msg.linear.y))
        self.pub_yaw.publish(Float64(data=msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
