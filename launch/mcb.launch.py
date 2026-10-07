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

"""Gazebo hardware fixture, compiled sentry firmware, and the real UART bridge."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from sim.display import display_error


def stack(context):
    windows = display_error() is None
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sim'), 'launch', 'sim.launch.py')),
        launch_arguments={'pose_emulator': 'false', 'camera': 'false',
                          'gui': str(windows).lower(), 'rviz': str(windows).lower(),
                          'real_time_factor': '1'}.items())
    mcb = Node(package='sim', executable='mcb_emulator', output='screen', parameters=[{
        'use_sim_time': True,
        'device_link': LaunchConfiguration('device_link'),
        'firmware_binary': LaunchConfiguration('firmware_binary'),
        'drive': LaunchConfiguration('drive'),
        'auto_fire': ParameterValue(LaunchConfiguration('auto_fire'), value_type=bool)}])
    bridge = Node(package='dji_serial_bridge', executable='dji_serial_bridge_node',
                  name='dji_serial_bridge', output='screen', parameters=[{
                      'use_sim_time': True, 'device': LaunchConfiguration('device_link'),
                      'debug_log': False}])
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        launch_arguments={'real_hardware': 'false', 'localization_mode': 'none',
                          'use_rf2o': 'false', 'patrol_enabled': 'false'}.items())
    relay = Node(package='thornbots_pkg', executable='mcb_relay', output='screen',
                 parameters=[{'use_sim_time': True}])
    fail = RegisterEventHandler(OnProcessExit(target_action=mcb, on_exit=[
        EmitEvent(event=Shutdown(reason='MCB firmware adapter exited'))]))
    return [sim, fail, mcb, TimerAction(period=2.0, actions=[bridge, relay]),
            TimerAction(period=8.0, actions=[robot])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('firmware_binary', default_value='',
                              description='Hosted MCB executable; default is its release build'),
        DeclareLaunchArgument('device_link', default_value='/tmp/mcb_emulator_pty'),
        DeclareLaunchArgument('drive', default_value='stop', choices=['stop', 'simple', 'auto']),
        DeclareLaunchArgument('auto_fire', default_value='true'),
        OpaqueFunction(function=stack),
    ])
