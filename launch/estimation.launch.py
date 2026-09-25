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
C2 estimation bench: target_tracker's TargetState against the truth, on gz.

`ros2 launch sim estimation.launch.py [speeds:='0.5 1'] [blackout:=true]`.
gz runs our sentry_v2 with target_driver's phantom target, cv_target_emulator
turns it into detections off the real head's camera, target_selector and
target_tracker build the TargetState, and point_to_cv_target plus cv_head_aim
keep the head on it. Nothing fires. pytest (test_estimation.py) scores each
state at its stamp. Stops when the tests finish; Ctrl-C stops everything.
`run_tests:=false` brings up the stack alone.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Installed as a symlink into share/sim/launch (--symlink-install), so the real
# path leads back to src/sim; the constant covers a copying install.
SOURCE_FALLBACK = '/workspaces/isaac_ros-dev/src/sim/test/cv'
TEST_FILE = 'test_estimation.py'


def _test_dir():
    here = os.path.dirname(os.path.realpath(__file__))
    for candidate in (os.path.join(here, '..', 'test', 'cv'), SOURCE_FALLBACK):
        if os.path.exists(os.path.join(candidate, TEST_FILE)):
            return os.path.normpath(candidate)
    raise RuntimeError(f'could not find {TEST_FILE} next to {here} or in {SOURCE_FALLBACK}')


def _is_true(context, name):
    return context.launch_configurations[name].lower() in ('true', '1', 'yes')


def cv_node(executable, package='thornbots_pkg', **params):
    return Node(package=package, executable=executable, name=executable,
                output='screen', parameters=[{'use_sim_time': True, **params}])


def _stack(context):
    cfg = context.launch_configurations
    headless = _is_true(context, 'headless')
    camera_latency = cfg['camera_latency_s']
    sim_args = {'spawn_target': 'true', 'target_speed': '0.0', 'target_spin_hz': '0.0',
                'real_time_factor': cfg['real_time_factor'],
                'cv_camera_latency_s': camera_latency}
    if headless:
        sim_args.update(gui='false', rviz='false')
    else:
        sim_args['rviz_config'] = os.path.join(
            get_package_share_directory('sim'), 'rviz', 'estimation.rviz')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sim'), 'launch', 'sim.launch.py')),
        launch_arguments=sim_args.items())

    # TF chain only: sim runs no robot_state_publisher. The enable_*:=false
    # args skip auto.launch.py's copies of the CV nodes below, and
    # localization_mode:=none skips map_server/amcl.
    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        launch_arguments={
            'real_hardware': 'false', 'localization_mode': 'none', 'use_ekf': 'false',
            'enable_cv_target_bridge': 'false', 'enable_target_selector': 'false',
            'enable_target_tracker': 'false',
        }.items())

    tracker = {'camera_latency_s': float(cfg['tracker_camera_latency_s'] or camera_latency)}
    if cfg['process_noise_accel']:
        tracker['process_noise_accel'] = float(cfg['process_noise_accel'])
    actions = [sim, robot_tf, cv_node('target_selector'),
               cv_node('target_tracker', **tracker), cv_node('point_to_cv_target')]
    if not headless:
        actions.append(cv_node('target_state_markers', package='sim'))
    if _is_true(context, 'run_tests'):
        actions += _tests(context, _test_dir())
    return actions


def _tests(context, test_dir):
    cfg = context.launch_configurations
    cmd = [sys.executable, '-m', 'pytest', os.path.join(test_dir, TEST_FILE),
           '-m', 'integration', '-v', '-s', '--external-stack',
           '--panel-layout', cfg['panel_layout'], '--target-path', cfg['target_path'],
           '--shooter-speed', cfg['shooter_speed'],
           '--camera-latency', cfg['camera_latency_s']]
    speeds = cfg['speeds'].replace(',', ' ').split()
    if speeds:
        cmd += ['--shot-speeds', ','.join(speeds)]
    for arg, opt in (('duration', '--shot-duration'), ('log_dir', '--log-dir')):
        if cfg[arg]:
            cmd += [opt, cfg[arg]]
    for arg in ('skip_stationary', 'only_stationary', 'blackout'):
        if _is_true(context, arg):
            cmd.append('--' + arg.replace('_', '-'))
    cmd += cfg['pytest_args'].split()

    tests = ExecuteProcess(cmd=cmd, name='estimation_tests', output='screen')
    done = RegisterEventHandler(OnProcessExit(
        target_action=tests,
        on_exit=lambda event, _: [
            LogInfo(msg=f'estimation tests exited with code {event.returncode}'),
            EmitEvent(event=Shutdown(reason='estimation tests finished')),
        ]))
    return [tests, done]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('run_tests', default_value='true',
                              description='false: bring up the stack only'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='skip the gz GUI and rviz2'),
        DeclareLaunchArgument('real_time_factor', default_value='0',
                              description='sim speed cap; 0 = as fast as it runs'),
        DeclareLaunchArgument('speeds', default_value='',
                              description="target speeds (m/s), e.g. '0.5 1'; "
                                          'empty = the aim bench sweep'),
        DeclareLaunchArgument('duration', default_value='',
                              description='sim-time seconds scored per case'),
        DeclareLaunchArgument('target_path', default_value='lateral',
                              choices=['lateral', 'radial', 'diagonal'],
                              description='across the view, down the camera ray, or both'),
        DeclareLaunchArgument('shooter_speed', default_value='0.0',
                              description='our chassis speed (m/s), bouncing along y'),
        DeclareLaunchArgument('blackout', default_value='false',
                              description='drop every detection 0.3 s in each 2 s'),
        DeclareLaunchArgument('camera_latency_s', default_value='0.0',
                              description="cv_target_emulator's late stamp (s), and "
                                          "target_tracker's camera_latency_s unless "
                                          'tracker_camera_latency_s says otherwise'),
        DeclareLaunchArgument('tracker_camera_latency_s', default_value='',
                              description="target_tracker's camera_latency_s; empty = "
                                          'camera_latency_s'),
        DeclareLaunchArgument('process_noise_accel', default_value='',
                              description="target_tracker's process_noise_accel (m/s^2); "
                                          'empty = its default'),
        DeclareLaunchArgument('panel_layout', default_value='both',
                              choices=['flat', 'staggered', 'both']),
        DeclareLaunchArgument('skip_stationary', default_value='false'),
        DeclareLaunchArgument('only_stationary', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value='',
                              description='where estimation.jsonl and '
                                          'estimation_states.jsonl go'),
        DeclareLaunchArgument('pytest_args', default_value='',
                              description="extra pytest args, e.g. '-k flat'"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_stack)])
