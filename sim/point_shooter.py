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
The aim bench's own chassis: odom->root TF and /pose from a truth odometry.

Subscribes /shooter/ground_truth_odom (a second target_driver, spin 0) and
republishes each sample at once, stamped with its sample time: odom->root on
/tf, and RobotPose on /pose with root-frame velocity, as the MCB would send.
No noise, no latency: C1's perfect model of our own motion.
"""
import math

from dji_serial_bridge.msg import RobotPose
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class PointShooter(Node):

    def __init__(self):
        super().__init__('point_shooter')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('root_frame', 'root')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.root_frame = self.get_parameter('root_frame').value
        self.tf_pub = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(RobotPose, '/pose', 10)
        self.create_subscription(Odometry, '/shooter/ground_truth_odom', self.on_truth, 50)

    def on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.root_frame
        tf.transform.translation.x, tf.transform.translation.y = p.x, p.y
        tf.transform.translation.z = p.z
        tf.transform.rotation = q
        self.tf_pub.sendTransform(tf)

        # target_driver's twist is world-frame; RobotPose's is the chassis's.
        yaw = 2.0 * math.atan2(q.z, q.w)
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        pose = RobotPose()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.root_frame
        pose.x, pose.y = p.x, p.y
        pose.vel_x = math.cos(yaw) * vx + math.sin(yaw) * vy
        pose.vel_y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = PointShooter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
