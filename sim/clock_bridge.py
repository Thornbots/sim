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
import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def clock_bridge_node():
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        # A departed reader's SHM port can block all future clock publication.
        # Reuse the robot's UDP-only profile, including its discovery peers.
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(
            get_package_share_directory('thornbots_pkg'), 'config', 'fastdds_udp_only.xml')},
    )
