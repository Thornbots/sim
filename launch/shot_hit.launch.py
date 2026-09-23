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
Shot-hit bench: the sim, the production CV pipeline and the scoring pytest in one launch tree.

`ros2 launch sim shot_hit.launch.py [speeds:='0.5 1'] [only_stationary:=true]`.
Stops when the tests finish; Ctrl-C (or SIGINT/SIGTERM to this launch) stops
the whole stack. `run_tests:=false` brings up the stack alone, which is how
test_shot_hit.py's cv_stack fixture launches it under a bare pytest/colcon test.
real_time_factor:=0 (the default) runs the sim unthrottled; cases are scored in
sim time.
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
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# Installed as a symlink into share/sim/launch (--symlink-install), so the real
# path leads back to src/sim; the constant covers a copying install.
SOURCE_FALLBACK = '/workspaces/isaac_ros-dev/src/sim/test/cv'
TEST_FILE = 'test_shot_hit.py'


def _test_dir():
    here = os.path.dirname(os.path.realpath(__file__))
    for candidate in (os.path.join(here, '..', 'test', 'cv'), SOURCE_FALLBACK):
        if os.path.exists(os.path.join(candidate, TEST_FILE)):
            return os.path.normpath(candidate)
    raise RuntimeError(f'could not find {TEST_FILE} next to {here} or in {SOURCE_FALLBACK}')


def _is_true(context, name):
    return context.launch_configurations[name].lower() in ('true', '1', 'yes')


def _stack(context):
    test_dir = _test_dir()
    # The harness owns the fire rate its score expects; read it rather than copy it.
    sys.path.insert(0, test_dir)
    import shot_hit_harness as harness

    headless = _is_true(context, 'headless')
    sim_args = {'spawn_target': 'true', 'target_speed': '0.0', 'target_spin_hz': '0.0',
                'real_time_factor': context.launch_configurations['real_time_factor']}
    if headless:
        sim_args.update(gui='false', rviz='false')
    else:
        sim_args['rviz_config'] = os.path.join(
            get_package_share_directory('sim'), 'rviz', 'cv_target.rviz')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sim'), 'launch', 'sim.launch.py')),
        launch_arguments=sim_args.items())

    # TF chain only: sim runs no robot_state_publisher. The enable_*:=false args
    # skip auto.launch.py's copies of the CV nodes below, which would
    # double-publish; localization_mode:=none skips map_server/amcl.
    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        launch_arguments={
            'real_hardware': 'false', 'localization_mode': 'none', 'use_ekf': 'false',
            'enable_cv_target_bridge': 'false', 'enable_target_selector': 'false',
            'enable_target_tracker': 'false',
        }.items())

    # The production CV pipeline, bare (no auto.launch.py params): target_selector
    # picks from the emulator's panel_detections, target_tracker estimates the
    # spin centre, point_to_cv_target solves the lead and fires at up to
    # TEST_FIRE_HZ, mcb_relay forwards it to /dji_serial_bridge/cv_target.
    def cv_node(executable, **params):
        return Node(package='thornbots_pkg', executable=executable, name=executable,
                    output='screen', parameters=[{'use_sim_time': True, **params}])

    actions = [
        sim, robot_tf,
        cv_node('target_selector'),
        cv_node('target_tracker'),
        cv_node('point_to_cv_target',
                cv_target_publish_rate_hz=harness.TEST_FIRE_HZ,
                fire_rate_hz=harness.TEST_FIRE_HZ + 10.0),
        cv_node('mcb_relay'),
    ]
    if not _is_true(context, 'run_tests'):
        return actions

    cmd = [sys.executable, '-m', 'pytest', os.path.join(test_dir, TEST_FILE),
           '-m', 'integration', '-v', '-s', '--external-stack',
           '--panel-layout', context.launch_configurations['panel_layout']]
    speeds = context.launch_configurations['speeds'].replace(',', ' ').split()
    if speeds:
        cmd += ['--shot-speeds', ','.join(speeds)]
    for arg, opt in (('duration', '--shot-duration'), ('hit_radius', '--hit-radius'),
                     ('log_dir', '--log-dir')):
        if context.launch_configurations[arg]:
            cmd += [opt, context.launch_configurations[arg]]
    for arg in ('skip_stationary', 'only_stationary'):
        if _is_true(context, arg):
            cmd.append('--' + arg.replace('_', '-'))
    cmd += context.launch_configurations['pytest_args'].split()

    tests = ExecuteProcess(cmd=cmd, name='shot_hit_tests', output='screen')
    done = RegisterEventHandler(OnProcessExit(
        target_action=tests,
        on_exit=lambda event, _: [
            LogInfo(msg=f'shot-hit tests exited with code {event.returncode}'),
            EmitEvent(event=Shutdown(reason='shot-hit tests finished')),
        ]))
    return actions + [tests, done]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('sim_engine',
                              default_value=EnvironmentVariable('SIM_ENGINE', default_value='gz'),
                              choices=['gz', 'sapien'],
                              description='gz or sapien; exported as SIM_ENGINE so pytest '
                                          'and every stack it launches use the same one'),
        DeclareLaunchArgument('run_tests', default_value='true',
                              description='false: bring up the stack only'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='skip the gz GUI and rviz2'),
        DeclareLaunchArgument('real_time_factor', default_value='0',
                              description='sim speed cap; 0 = as fast as it runs'),
        DeclareLaunchArgument('speeds', default_value='',
                              description="target speeds (m/s), e.g. '0.5 1'; "
                                          'empty = the harness default sweep'),
        DeclareLaunchArgument('duration', default_value='',
                              description='sim-time seconds scored per case'),
        DeclareLaunchArgument('hit_radius', default_value='',
                              description='miss distance (m) still counted as a hit'),
        DeclareLaunchArgument('panel_layout', default_value='both',
                              choices=['flat', 'staggered', 'both']),
        DeclareLaunchArgument('skip_stationary', default_value='false'),
        DeclareLaunchArgument('only_stationary', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value='',
                              description='where shots.jsonl and panel_hits.jsonl go; '
                                          'empty = the harness default'),
        DeclareLaunchArgument('pytest_args', default_value='',
                              description="extra pytest args, e.g. '-k flat'"),
    ]
    engine = SetEnvironmentVariable('SIM_ENGINE', LaunchConfiguration('sim_engine'))
    return LaunchDescription(args + [engine, OpaqueFunction(function=_stack)])
