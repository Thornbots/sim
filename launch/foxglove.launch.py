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

`ros2 launch sim foxglove.launch.py [port:=8765]`. The test launches include
it (their `foxglove:=` arg). It stands down if the port is already taken, so a
standalone bridge and a test's don't collide. Read-only: viewers can't
publish, call services or set parameters on a running test. use_sim_time
follows /clock, which every bench publishes. latch_holder keeps /tf_static,
/map and /robot_description visible across the drift suite's stack restarts.
"""
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node


def _port_in_use(port):
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(('127.0.0.1', port)) == 0


def _bridge(context):
    port = int(context.launch_configurations['port'])
    if _port_in_use(port):
        return [LogInfo(msg=f'foxglove: port {port} already serving, not starting another')]
    # Keeps the bridge's latched subscriptions latched across stack restarts.
    holder = Node(package='sim', executable='latch_holder', output='log')
    return [holder, Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='log',
        parameters=[{
            'port': port,
            'address': context.launch_configurations['address'],
            'use_sim_time': True,
            'capabilities': ['assets'],
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='Interface to listen on; 0.0.0.0 is all'),
        OpaqueFunction(function=_bridge),
    ])
