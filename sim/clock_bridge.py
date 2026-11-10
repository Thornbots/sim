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

"""Bridge Gazebo time without sharing subscriber SHM port lifetimes."""
from launch_ros.actions import Node

from sim.dds_switch import udp_env


def clock_bridge_node(topic='/clock', condition=None):
    return Node(
        package='sim',
        executable='gz_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[topic + '@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[(topic, '/clock')],
        condition=condition,
        parameters=[{'use_sim_time': True}],
        # A departed reader's SHM port can block all future clock publication.
        # Reuse the robot's UDP-only profile (peers only with remote DDS on).
        additional_env=udp_env(),
    )
