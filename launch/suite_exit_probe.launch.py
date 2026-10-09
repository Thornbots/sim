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

"""Launch API fixture for native tests of suite exit-status propagation."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from sim.suite_exit import finish_suite


def _probe(context):
    code = context.launch_configurations['code']
    child = ExecuteProcess(cmd=['sh', '-c', f'exit {int(code)}'])
    handler = RegisterEventHandler(OnProcessExit(
        target_action=child,
        on_exit=lambda event, context: finish_suite(event, context, 'probe')))
    return [handler, child]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('code', default_value='0'),
                              OpaqueFunction(function=_probe)])
