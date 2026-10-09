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
Launch the native drift/ground-truth suites and their test-owned ROS stacks.

The C++ harness starts the shared sim (part:=sim) and each robot stack
(part:=robot); restart_sim:=true starts both again per case.
All scored durations use sim time. See README.md for design rationale.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from sim.display import display_error
from sim.suite_exit import finish_suite

# sim.launch.py's odom noise model, forwarded when given (the harness sets them
# per scenario).
SIM_PASSTHROUGH = ('odom_noise_enabled', 'odom_drift_stddev', 'odom_jitter_stddev',
                   'odom_jerk_stddev', 'odom_jerk_bias_enabled', 'odom_jerk_bias_x',
                   'odom_jerk_bias_y', 'odom_slip_ratio')
# Head start for gz and the robot spawn before localization subscribes, so
# its early "waiting for transform" noise stays out of the scanned logs.
LOCALIZATION_DELAY_S = 8.0


def _is_true(context, name):
    return context.launch_configurations[name].lower() in ('true', '1', 'yes')


def _stack(context):
    config = context.launch_configurations
    part = config['part']
    windows = not _is_true(context, 'headless') and display_error() is None
    gui = 'true' if windows else 'false'
    # foxglove: this launch's own bridge (_foxglove) covers the whole run.
    sim_args = {'gui': gui, 'rviz': gui, 'foxglove': 'false',
                'real_time_factor': config['real_time_factor']}
    sim_args.update({k: config[k] for k in SIM_PASSTHROUGH if k in config})
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sim'), 'launch', 'sim.launch.py')),
        launch_arguments=sim_args.items())

    backend = config['backend']
    # No CV: nothing here scores it, and it costs CPU the stack under test needs.
    robot_args = {'real_hardware': 'false', 'localization_mode': backend,
                  'use_rf2o': config['use_rf2o'],
                  # mapping builds its map from a blank one at spawn, so its
                  # map frame matches the world's (drift_harness.TRUTH_SCORED).
                  'load_map': 'false' if backend == 'mapping' else 'true',
                  'enable_target_tracker': 'false', 'enable_target_selector': 'false',
                  'enable_cv_target_bridge': 'false'}
    if backend == 'slam':
        # slam_toolbox's localization mode needs a .posegraph, and none ships.
        raise RuntimeError('backend:=slam needs a saved pose graph; none ships '
                           '(sentry_localization README.md map_file)')
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        launch_arguments=robot_args.items())
    if part == 'sim':
        return [sim]
    if part == 'robot':
        return [robot]
    return [sim, TimerAction(period=LOCALIZATION_DELAY_S, actions=[robot])]


def _tests(context):
    config = context.launch_configurations
    suite = config['suite']
    cmd = [os.path.join(get_package_prefix('sim'), 'lib', 'sim', 'localization_suite'),
           '--suite', suite, '--real-time-factor', config['real_time_factor']]
    if _is_true(context, 'headless'):
        cmd.append('--headless')
    if config['speed']:
        cmd += ['--speed', config['speed']]
    if config['drive_accel']:
        cmd += ['--drive-accel', config['drive_accel']]
    if config['spawn_yaw_deg']:
        cmd += ['--spawn-yaw-deg', config['spawn_yaw_deg']]
    if _is_true(context, 'restart_sim'):
        cmd.append('--restart-sim')
    if suite == 'ekf':
        cmd.append('--run-on-demand')
    if suite == 'drift':
        cmd += ['--backend', config['backend'],
                '--use-rf2o' if _is_true(context, 'use_rf2o') else '--no-use-rf2o']
        if config['scenario']:
            cmd += ['--scenario', config['scenario']]
    else:
        for arg in ('ekf_slip_ratio', 'ekf_drift_stddev', 'ekf_seconds'):
            if config[arg]:
                cmd += ['--' + arg.replace('_', '-'), config[arg]]
    cmd += config['gtest_args'].split()
    cmd += config['pytest_args'].split()

    # A scenario's teardown can take ~15s before its process group is stopped.
    tests = ExecuteProcess(cmd=cmd, name='localization_tests', output='screen',
                           sigterm_timeout='30')
    done = RegisterEventHandler(OnProcessExit(
        target_action=tests,
        on_exit=lambda event, context: finish_suite(event, context, 'localization')))
    return [tests, done]


def _launch(context):
    return _tests(context) if _is_true(context, 'run_tests') else _stack(context)


def _foxglove():
    # launch/foxglove.launch.py; it stands down if the port is taken.
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sim'), 'launch', 'foxglove.launch.py')),
        condition=IfCondition(LaunchConfiguration('foxglove')))


def generate_launch_description():
    args = [
        DeclareLaunchArgument('run_tests', default_value='true',
                              description='false: bring up one scenario stack only'),
        DeclareLaunchArgument('suite', default_value='drift', choices=['drift', 'ekf']),
        DeclareLaunchArgument('part', default_value='all', choices=['all', 'sim', 'robot'],
                              description='run_tests:=false only: sim, robot stack, or both'),
        DeclareLaunchArgument('restart_sim', default_value='false',
                              description='fresh sim per scenario instead of one per run'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='skip the gz GUI and rviz2'),
        DeclareLaunchArgument('real_time_factor', default_value='0',
                              description='sim speed cap; 0 = as fast as it runs'),
        DeclareLaunchArgument('backend', default_value='amcl',
                              choices=['slam', 'mapping', 'amcl', 'none'],
                              description='who owns map->odom (drift suite)'),
        DeclareLaunchArgument('use_rf2o', default_value='true',
                              description='fuse rf2o into odom->root (drift suite)'),
        DeclareLaunchArgument('scenario', default_value='',
                              description='one drift scenario; empty = all'),
        DeclareLaunchArgument('speed', default_value='',
                              description='cornering-loop m/s; empty = 4.0'),
        DeclareLaunchArgument('drive_accel', default_value='',
                              description='m/s^2 ramp on every drift leg; empty = 20, 0 = step'),
        DeclareLaunchArgument('spawn_yaw_deg', default_value='',
                              description='drift reset heading off spawn, deg; empty = 0'),
        DeclareLaunchArgument('ekf_slip_ratio', default_value='',
                              description='ekf suite; empty = the suite default'),
        DeclareLaunchArgument('ekf_drift_stddev', default_value='',
                              description='ekf suite; empty = the suite default'),
        DeclareLaunchArgument('ekf_seconds', default_value='',
                              description='ekf suite; empty = the suite default'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='Foxglove bridge on :8765'),
        DeclareLaunchArgument('gtest_args', default_value='',
                              description='extra GoogleTest arguments'),
        DeclareLaunchArgument('pytest_args', default_value='',
                              description='deprecated alias for gtest_args'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch), _foxglove()])
