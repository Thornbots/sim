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
Localization suites: the drift scenarios (suite:=drift) or the EKF ground-truth check (suite:=ekf).

`ros2 launch sim localization_tests.launch.py [backend:=amcl] [scenario:=odom_stuck]`.
Runs pytest, which brings up a fresh stack per scenario through this file with
run_tests:=false (sim, then auto.launch.py 8s later). Stops when the tests
finish; Ctrl-C stops pytest, and its stack with it. real_time_factor:=0 (the
default) runs the sim unthrottled; the suites measure in sim time.
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
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

# Installed as a symlink into share/sim/launch (--symlink-install), so the real
# path leads back to src/sim; the constant covers a copying install.
SOURCE_FALLBACK = '/workspaces/isaac_ros-dev/src/sim/test/localization'
TEST_FILES = {'drift': 'test_localization_drift.py', 'ekf': 'test_ekf_ground_truth.py'}
# sim.launch.py's odom noise model, forwarded when given (the harness sets them
# per scenario).
SIM_PASSTHROUGH = ('odom_noise_enabled', 'odom_drift_stddev', 'odom_jitter_stddev',
                   'odom_jerk_stddev', 'odom_jerk_bias_enabled', 'odom_jerk_bias_x',
                   'odom_jerk_bias_y', 'odom_slip_ratio')
# Head start for gz and the robot spawn before localization subscribes, so
# its early "waiting for transform" noise stays out of the scanned logs.
LOCALIZATION_DELAY_S = 8.0


def _test_dir():
    here = os.path.dirname(os.path.realpath(__file__))
    for candidate in (os.path.join(here, '..', 'test', 'localization'), SOURCE_FALLBACK):
        if os.path.exists(os.path.join(candidate, TEST_FILES['drift'])):
            return os.path.normpath(candidate)
    raise RuntimeError(f'could not find the localization tests next to {here} '
                       f'or in {SOURCE_FALLBACK}')


def _is_true(context, name):
    return context.launch_configurations[name].lower() in ('true', '1', 'yes')


def _stack(context):
    config = context.launch_configurations
    gui = 'false' if _is_true(context, 'headless') else 'true'
    sim_args = {'gui': gui, 'rviz': gui, 'real_time_factor': config['real_time_factor']}
    sim_args.update({k: config[k] for k in SIM_PASSTHROUGH if k in config})
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sim'), 'launch', 'sim.launch.py')),
        launch_arguments=sim_args.items())

    backend = config['backend']
    robot_args = {'real_hardware': 'false', 'localization_mode': backend,
                  'use_ekf': config['use_ekf'], 'load_map': 'true'}
    if backend == 'slam':
        # slam_toolbox's localization mode needs a .posegraph, which the default
        # map (clean_map) lacks; ARCC26 is the one map that has one.
        robot_args['map_file'] = os.path.join(
            get_package_share_directory('sentry_localization'), 'map', 'ARCC26')
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        launch_arguments=robot_args.items())
    return [sim, TimerAction(period=LOCALIZATION_DELAY_S, actions=[robot])]


def _tests(context):
    config = context.launch_configurations
    suite = config['suite']
    cmd = [sys.executable, '-m', 'pytest', os.path.join(_test_dir(), TEST_FILES[suite]),
           '-m', 'integration', '-v', '-s',
           '--real-time-factor', config['real_time_factor']]
    if _is_true(context, 'headless'):
        cmd.append('--headless')
    if suite == 'drift':
        cmd += ['--backend', config['backend'],
                '--use-ekf' if _is_true(context, 'use_ekf') else '--no-use-ekf']
        for arg, opt in (('scenario', '--scenario'), ('speed', '--speed')):
            if config[arg]:
                cmd += [opt, config[arg]]
    else:
        for arg in ('ekf_slip_ratio', 'ekf_drift_stddev', 'ekf_seconds'):
            if config[arg]:
                cmd += ['--' + arg.replace('_', '-'), config[arg]]
    cmd += config['pytest_args'].split()

    # sigterm_timeout: a scenario's teardown can take ~15s; SIGTERM kills
    # pytest outright, before it stops its stack.
    tests = ExecuteProcess(cmd=cmd, name='localization_tests', output='screen',
                           sigterm_timeout='30')
    done = RegisterEventHandler(OnProcessExit(
        target_action=tests,
        on_exit=lambda event, _: [
            LogInfo(msg=f'localization tests exited with code {event.returncode}'),
            EmitEvent(event=Shutdown(reason='localization tests finished')),
        ]))
    return [tests, done]


def _launch(context):
    return _tests(context) if _is_true(context, 'run_tests') else _stack(context)


def generate_launch_description():
    args = [
        DeclareLaunchArgument('sim_engine',
                              default_value=EnvironmentVariable('SIM_ENGINE', default_value='gz'),
                              choices=['gz', 'sapien'],
                              description='gz or sapien; exported as SIM_ENGINE so pytest '
                                          'and every stack it launches use the same one'),
        DeclareLaunchArgument('run_tests', default_value='true',
                              description='false: bring up one scenario stack only'),
        DeclareLaunchArgument('suite', default_value='drift', choices=['drift', 'ekf']),
        DeclareLaunchArgument('headless', default_value='false',
                              description='skip the gz GUI and rviz2'),
        DeclareLaunchArgument('real_time_factor', default_value='0',
                              description='sim speed cap; 0 = as fast as it runs'),
        DeclareLaunchArgument('backend', default_value='amcl',
                              choices=['slam', 'amcl', 'none'],
                              description='who owns map->odom (drift suite)'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='EKF-fuse odom->root (drift suite)'),
        DeclareLaunchArgument('scenario', default_value='',
                              description='one drift scenario; empty = all'),
        DeclareLaunchArgument('speed', default_value='',
                              description='cornering-loop m/s; empty = 4.0'),
        DeclareLaunchArgument('ekf_slip_ratio', default_value='',
                              description='ekf suite; empty = the pytest default'),
        DeclareLaunchArgument('ekf_drift_stddev', default_value='',
                              description='ekf suite; empty = the pytest default'),
        DeclareLaunchArgument('ekf_seconds', default_value='',
                              description='ekf suite; empty = the pytest default'),
        DeclareLaunchArgument('pytest_args', default_value='',
                              description="extra pytest args, e.g. '-x'"),
    ]
    engine = SetEnvironmentVariable('SIM_ENGINE', LaunchConfiguration('sim_engine'))
    return LaunchDescription(args + [engine, OpaqueFunction(function=_launch)])
