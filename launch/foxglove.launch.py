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
Foxglove websocket bridge, for watching a test run from another machine.

`ros2 launch sim foxglove.launch.py [port:=8765]`, next to any bench or sim
launch; it finds their topics over DDS. Open Foxglove, choose
"Open connection", "Foxglove WebSocket", `ws://<this host>:8765`. Only topics a
panel subscribes to are sent. use_sim_time follows /clock, which every bench
publishes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='Interface to listen on; 0.0.0.0 is all'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'address': LaunchConfiguration('address'),
                'use_sim_time': True,
            }],
        ),
    ])
