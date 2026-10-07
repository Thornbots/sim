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

"""Check moving and spinning target Odometry against ROS child-frame semantics."""
import math
from types import SimpleNamespace

from nav_msgs.msg import Odometry
import pytest
from rclpy.parameter import Parameter
from sim.odometry import velocity_in_parent
from sim.target_driver import TargetDriver
from sim.target_state_truth import TargetStateTruth


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -math.pi / 2, math.pi])
def test_target_driver_twist_is_child_frame(ros_context, yaw):
    driver = TargetDriver()
    messages = []
    driver.pub = SimpleNamespace(publish=messages.append)
    try:
        driver.set_parameters([Parameter('origin_yaw', value=0.3)])
        driver.yaw = yaw
        driver._publish(2.0, 8.0)
        msg = messages[-1]
        # Motion remains along the rotated path's +y, whatever the chassis yaw.
        assert velocity_in_parent(msg) == pytest.approx(
            (-2 * math.sin(0.3), 2 * math.cos(0.3), 0.0))
        assert msg.twist.twist.linear.x == pytest.approx(2 * math.sin(yaw))
        assert msg.twist.twist.linear.y == pytest.approx(2 * math.cos(yaw))
        assert msg.twist.twist.angular.z == 8.0
        assert msg.child_frame_id == 'target'
    finally:
        driver.destroy_node()


def test_spinning_constant_velocity_does_not_create_acceleration(ros_context):
    node = TargetStateTruth()
    states = []
    node.pub = SimpleNamespace(publish=states.append)
    try:
        for sec, yaw in [(1, 0.0), (2, math.pi / 2)]:
            msg = Odometry()
            msg.header.stamp.sec = sec
            msg.pose.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.pose.orientation.w = math.cos(yaw / 2)
            msg.twist.twist.linear.x = 2 * math.cos(yaw)
            msg.twist.twist.linear.y = -2 * math.sin(yaw)
            node.on_truth(msg)
        assert states[-1].velocity.x == pytest.approx(2.0)
        assert states[-1].velocity.y == pytest.approx(0.0, abs=1e-12)
        assert states[-1].acceleration.x == pytest.approx(0.0, abs=1e-12)
        assert states[-1].acceleration.y == pytest.approx(0.0, abs=1e-12)
        assert states[-1].header.stamp.sec == 2
    finally:
        node.destroy_node()
