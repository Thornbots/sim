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

One TargetState on /cv/target_state per /cv/robot_panels message, with that
message's stamp and robot_track_id and the target's true state at that stamp
from /target/ground_truth_odom. Panel 0 (front) is the tracked panel. Panel
geometry matches cv_target_emulator (panel_radius_x/y, panel_stagger_m; keep
panel_stagger_m in step with the emulator's). see README.md for design rationale
"""
import collections
import math

from dji_serial_bridge.msg import PanelDetectionArray, TargetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sim.cv_target_emulator import PANEL_RADIUS_X, PANEL_RADIUS_Y

# A detection is stamped with the truth sample it came from, so a match
# further off than this means the sample aged out of the history.
MAX_STAMP_GAP_S = 0.02


class TargetStateTruth(Node):

    def __init__(self):
        super().__init__('target_state_truth')
        self.declare_parameter('robot_panels_topic', '/cv/robot_panels')
        self.declare_parameter('output_topic', '/cv/target_state')
        self.declare_parameter('panel_radius_x', PANEL_RADIUS_X)
        self.declare_parameter('panel_radius_y', PANEL_RADIUS_Y)
        self.declare_parameter('panel_stagger_m', 0.0)
        self.declare_parameter('history_s', 2.0)

        self._history = collections.deque()  # (t_s, msg, unwrapped yaw)
        self._yaw = None
        self.pub = self.create_publisher(
            TargetState, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Odometry, '/target/ground_truth_odom', self.on_truth, 50)
        self.create_subscription(
            PanelDetectionArray, self.get_parameter('robot_panels_topic').value,
            self.on_robot_panels, 10)

    def on_truth(self, msg):
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)  # target_driver publishes yaw-only
        if self._yaw is None:
            self._yaw = yaw
        else:
            step = (yaw - self._yaw + math.pi) % (2.0 * math.pi) - math.pi
            self._yaw += step
        t_s = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        self._history.append((t_s, msg, self._yaw))
        horizon = t_s - self.get_parameter('history_s').value
        while self._history and self._history[0][0] < horizon:
            self._history.popleft()

    def on_robot_panels(self, msg):
        if not msg.detections or not self._history:
            return
        t_s = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        sample_t, truth, yaw = min(self._history, key=lambda h: abs(h[0] - t_s))
        if abs(sample_t - t_s) > MAX_STAMP_GAP_S:
            self.get_logger().warn(
                f'no truth sample within {MAX_STAMP_GAP_S * 1e3:.0f} ms of a detection '
                f'({abs(sample_t - t_s) * 1e3:.0f} ms off); skipping it',
                throttle_duration_sec=2.0)
            return

        radius = self.get_parameter('panel_radius_x').value
        half_stagger = self.get_parameter('panel_stagger_m').value / 2.0
        p = truth.pose.pose.position
        out = TargetState()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = truth.header.frame_id
        out.robot_track_id = msg.detections[0].robot_track_id
        out.centre.x, out.centre.y, out.centre.z = p.x, p.y, p.z
        # target_driver writes world-frame velocity into its twist.
        out.velocity.x = truth.twist.twist.linear.x
        out.velocity.y = truth.twist.twist.linear.y
        out.panel.x = p.x + radius * math.cos(yaw)
        out.panel.y = p.y + radius * math.sin(yaw)
        out.panel.z = p.z + half_stagger
        out.yaw = float(yaw)
        out.yaw_rate = float(truth.twist.twist.angular.z)
        out.radius = float(radius)
        out.other_radius = float(self.get_parameter('panel_radius_y').value)
        # Front/back sit above the centre and left/right below, as in cv_target_emulator.
        out.z_offset = float(half_stagger)
        out.other_z_offset = float(-half_stagger)
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
