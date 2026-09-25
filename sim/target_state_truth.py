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
Ground-truth stand-in for target_tracker: the aim bench's perfect knowledge.

Publishes the target's true TargetState on /cv/target_state for each
/target/ground_truth_odom sample as it arrives (60 Hz, target_driver's
rate), stamped with that sample's time: valid, confidence 1, track id
TRACK_ID. Acceleration is the
velocity change over the last sample step, exact on target_driver's
constant-acceleration segments. Panel 0 (front) is the
tracked panel. Panel geometry matches cv_target_emulator (panel_radius_x/y,
panel_stagger_m). see README.md for design rationale
"""
import math

from dji_serial_bridge.msg import TargetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sim.cv_target_emulator import PANEL_RADIUS_X, PANEL_RADIUS_Y

TRACK_ID = 1  # one target, never switched


class TargetStateTruth(Node):

    def __init__(self):
        super().__init__('target_state_truth')
        self.declare_parameter('output_topic', '/cv/target_state')
        self.declare_parameter('panel_radius_x', PANEL_RADIUS_X)
        self.declare_parameter('panel_radius_y', PANEL_RADIUS_Y)
        self.declare_parameter('panel_stagger_m', 0.0)

        self._truth = None  # previous /target/ground_truth_odom
        self._yaw = None  # unwrapped
        self._accel = (0.0, 0.0)  # world-frame x, y, m/s^2
        self.pub = self.create_publisher(
            TargetState, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Odometry, '/target/ground_truth_odom', self.on_truth, 50)

    def on_truth(self, msg):
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)  # target_driver publishes yaw-only
        if self._yaw is None:
            self._yaw = yaw
        else:
            self._yaw += (yaw - self._yaw + math.pi) % (2.0 * math.pi) - math.pi
        prev = self._truth
        if prev is not None:
            dt = (Time.from_msg(msg.header.stamp)
                  - Time.from_msg(prev.header.stamp)).nanoseconds / 1e9
            if dt > 0.0:
                v, v0 = msg.twist.twist.linear, prev.twist.twist.linear
                self._accel = ((v.x - v0.x) / dt, (v.y - v0.y) / dt)
        self._truth = msg
        self.publish_state(msg, self._yaw)

    def publish_state(self, truth, yaw):
        radius = self.get_parameter('panel_radius_x').value
        half_stagger = self.get_parameter('panel_stagger_m').value / 2.0
        p = truth.pose.pose.position
        out = TargetState()
        out.header = truth.header
        out.robot_track_id = TRACK_ID
        out.confidence = 1.0
        out.center.x, out.center.y, out.center.z = p.x, p.y, p.z
        # target_driver writes world-frame velocity into its twist.
        out.velocity.x = truth.twist.twist.linear.x
        out.velocity.y = truth.twist.twist.linear.y
        out.acceleration.x, out.acceleration.y = self._accel
        out.panel.x = p.x + radius * math.cos(yaw)
        out.panel.y = p.y + radius * math.sin(yaw)
        out.panel.z = p.z + half_stagger
        out.yaw = float(yaw)
        out.yaw_rate = float(truth.twist.twist.angular.z)
        out.radius = [float(radius), float(self.get_parameter('panel_radius_y').value)]
        # Front/back sit above the center and left/right below, as in cv_target_emulator.
        out.z_offset = [float(half_stagger), float(-half_stagger)]
        out.valid = True
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TargetStateTruth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
