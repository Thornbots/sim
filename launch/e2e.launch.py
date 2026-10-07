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
The match test's stack, stage E1: detector stand-in to the gimbal, our robot parked.

`ros2 launch sim e2e.launch.py [speeds:='0 2'] [paths:=lateral]` runs
test/e2e/test_e1.py, which brings this stack up with run_tests:=false and
stops it after; `run_tests:=false` alone brings up the stack. The stack:
gz with no camera; one ghost opponent (opponent_driver spawns it, its
OpponentMover system rides target_driver's path); detector_standin putting
gz truth on /cv/panel_detections in roi_depth_node's place; auto.launch.py's
selector, tracker and point_to_cv_target;
cv_head_aim on the head; a RefSysStatus stub putting us on blue. The
opponent is red, class 6. `stage:=e2` swaps pose_emulator, cv_head_aim and
the stub for the MCB emulator on a pty with dji_serial_bridge and mcb_relay.
../E2E_PLAN.md has the stages.
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
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from sim.auto_explore import SPAWN_YAW
from sim.display import display_error
from sim.match_scenario import E1_PATHS

SOURCE_FALLBACK = '/workspaces/isaac_ros-dev/src/sim/test/e2e'
TEST_FILE = 'test_e1.py'
TEST_FILES = {'e1': TEST_FILE, 'e2': 'test_e2.py'}
ROBOT_DELAY_S = 8.0  # as localization_tests.launch.py: the sim is up first
OPPONENT = 'opponent_0'
OPPONENT_CLASS = 6  # red; we are blue
MCB_PTY = '/tmp/mcb_emulator_pty'


def _sim(context):
    config = context.launch_configurations
    windows = not _is_true(context, 'headless') and display_error() is None
    gui = 'true' if windows else 'false'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sim'), 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui, 'rviz': gui, 'foxglove': config['foxglove'],
                          'camera': 'false', 'pose_emulator': str(config['stage'] == 'e1').lower(),
                          'real_time_factor': config['real_time_factor']}.items())]


def _test_dir():
    here = os.path.dirname(os.path.realpath(__file__))
    for candidate in (os.path.join(here, '..', 'test', 'e2e'), SOURCE_FALLBACK):
        if os.path.exists(os.path.join(candidate, TEST_FILE)):
            return os.path.normpath(candidate)
    raise RuntimeError(f'could not find {TEST_FILE} next to {here} or in {SOURCE_FALLBACK}')


def _is_true(context, name):
    return context.launch_configurations[name].lower() in ('true', '1', 'yes')


def _tests(context):
    config = context.launch_configurations
    cmd = [sys.executable, '-m', 'pytest', os.path.join(_test_dir(), TEST_FILES[config['stage']]),
           '-m', 'integration', '-v', '-s']
    for arg, opt in (('speeds', '--e2e-speeds'), ('paths', '--e2e-paths'),
                     ('duration', '--e2e-duration'), ('log_dir', '--log-dir')):
        if config[arg]:
            cmd += [opt, ','.join(config[arg].replace(',', ' ').split())]
    if _is_true(context, 'headless'):
        cmd.append('--headless')
    if not _is_true(context, 'firmware_fixes'):
        cmd.append('--no-firmware-fixes')
    cmd += config['pytest_args'].split()
    tests = ExecuteProcess(cmd=cmd, name='e2e_tests', output='screen')
    done = RegisterEventHandler(OnProcessExit(
        target_action=tests,
        on_exit=lambda event, _: [
            LogInfo(msg=f'e2e tests exited with code {event.returncode}'),
            EmitEvent(event=Shutdown(reason='e2e tests finished')),
        ]))
    return [tests, done]


def generate_launch_description():
    share = get_package_share_directory('sim')
    xacro_file = os.path.join(share, 'urdf', 'sentry_v2.urdf.xacro')

    standin = Node(
        package='sim', executable='detector_standin_node', name='detector_standin',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'opponents': [OPPONENT],
            'class_ids': [OPPONENT_CLASS],
        }])

    target_driver = Node(
        package='sim', executable='target_driver', name='target_driver', output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_speed': ParameterValue(LaunchConfiguration('target_speed'), value_type=float),
            'spin_hz': ParameterValue(LaunchConfiguration('target_spin_hz'), value_type=float),
            'origin_yaw': SPAWN_YAW,  # sim.launch.py's spawn heading: the path stays in front
            **E1_PATHS['lateral'],
        }])
    # The path the opponent's OpponentMover system rides, into gz.
    path_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='opponent_path_bridge',
        output='screen',
        arguments=[f'/model/{OPPONENT}/path@nav_msgs/msg/Odometry]gz.msgs.Odometry'],
        remappings=[(f'/model/{OPPONENT}/path', '/target/ground_truth_odom')],
        parameters=[{'use_sim_time': True}])
    opponent_driver = Node(
        package='sim', executable='opponent_driver', name='opponent_driver', output='screen',
        parameters=[{'use_sim_time': True, 'name': OPPONENT}])
    cv_head_aim = Node(
        package='sim', executable='cv_head_aim', name='cv_head_aim', output='screen',
        parameters=[{'use_sim_time': True}])
    # The referee stand-in until E2's MCB emulator sends REF_SYS.
    team_stub = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-r', '5', '/dji_serial_bridge/ref_sys',
             'dji_serial_bridge/msg/RefSysStatus', '{is_on_blue_team: true}'],
        name='team_stub', output='log')
    # E2: the firmware port parked (DrivetrainStopCommand) on a pty, as
    # auto.launch.py starts the bridge and relay with real_hardware:=true.
    mcb_emulator = Node(
        package='sim', executable='mcb_emulator', name='mcb_emulator', output='screen',
        parameters=[{'use_sim_time': True, 'device_link': MCB_PTY, 'drive': 'stop',
                     'firmware_binary': LaunchConfiguration('firmware_binary')}])
    bridge = Node(
        package='dji_serial_bridge', executable='dji_serial_bridge_node',
        name='dji_serial_bridge', output='screen',
        parameters=[{'use_sim_time': True, 'device': MCB_PTY, 'debug_log': False}])
    mcb_relay = Node(
        package='thornbots_pkg', executable='mcb_relay', name='mcb_relay', output='screen',
        parameters=[{'use_sim_time': True}])

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        # No patrol: the scorer fires on every /cv/target frame (the
        # firmware's legacy rule), and a patrol frame isn't a target.
        launch_arguments={'real_hardware': 'false', 'localization_mode': 'amcl',
                          'use_rf2o': 'true', 'load_map': 'true',
                          'patrol_enabled': 'false'}.items())

    common = [standin, target_driver, path_bridge, opponent_driver]
    e1 = [cv_head_aim, team_stub]
    # The bridge opens the pty once, so it starts after the emulator makes it.
    e2 = [mcb_emulator, TimerAction(period=2.0, actions=[bridge, mcb_relay])]

    def stack(context):
        stage = context.launch_configurations['stage']
        if stage not in ('e1', 'e2'):
            raise RuntimeError(f"stage must be e1 or e2, not '{stage}'")
        return (common + (e1 if stage == 'e1' else e2)
                + [TimerAction(period=ROBOT_DELAY_S, actions=[robot])])

    return LaunchDescription([
        DeclareLaunchArgument('stage', default_value='e1',
                              description='e1: cv_head_aim on the gimbal; e2: the MCB emulator '
                                          'on a pty with dji_serial_bridge'),
        DeclareLaunchArgument('firmware_fixes', default_value='true',
                              description='Deprecated: the native MCB runs its checked-out code'),
        DeclareLaunchArgument('firmware_binary', default_value='',
                              description='Hosted MCB executable; empty uses the firmware build'),
        DeclareLaunchArgument('run_tests', default_value='true',
                              description='false: bring up the stack only'),
        DeclareLaunchArgument('speeds', default_value='',
                              description="opponent speeds, m/s, 0 = stationary, e.g. '0 2'; "
                                          'empty = the harness default'),
        DeclareLaunchArgument('paths', default_value='',
                              description="target_driver paths, e.g. 'lateral radial'; "
                                          'empty = all three'),
        DeclareLaunchArgument('duration', default_value='',
                              description='sim seconds scored per cell'),
        DeclareLaunchArgument('log_dir', default_value='',
                              description='where stack.log, shots.jsonl and scores.jsonl go'),
        DeclareLaunchArgument('pytest_args', default_value='',
                              description="extra pytest args, e.g. '-x'"),
        DeclareLaunchArgument('headless', default_value='false',
                              description='skip the gz GUI and rviz2'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='Foxglove bridge on :8765'),
        DeclareLaunchArgument('real_time_factor', default_value='0',
                              description='sim speed cap; 0 = as fast as it runs'),
        DeclareLaunchArgument('target_speed', default_value='2.0',
                              description="the opponent's speed along its path, m/s"),
        DeclareLaunchArgument('target_spin_hz', default_value='1.5',
                              description="the opponent's chassis spin, Hz"),
        # run_tests: pytest alone, which brings the stack up as one process
        # group and kills the group after; a stack here outlived the tests'
        # Shutdown (gz sim survived its ruby wrapper, 2026-09-29).
        OpaqueFunction(function=lambda context: _tests(context) if _is_true(
            context, 'run_tests') else _sim(context) + stack(context)),
    ])
