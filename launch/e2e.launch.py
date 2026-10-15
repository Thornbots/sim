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
Launch the MCB emulator integration suites and their camera-free Gazebo stacks.

Every stage runs truth detections, the real CV stack, the selected MCB backend
and the real UART bridge. mcb_parked scores 12 opponent cells with our robot
parked; mcb_drive drives from spawn to center; mcb_match adds a 2v2 fight.
Tests own their stack process group; run_tests:=false launches only the stack.
See README.md for design rationale and ../E2E_PLAN.md for acceptance criteria.
"""
import json
import math
import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from sim.auto_explore import SPAWN_YAW
from sim.display import display_error
from sim.suite_exit import finish_suite

STAGES = ('mcb_parked', 'mcb_drive', 'mcb_match')
DRIVING = ('mcb_drive', 'mcb_match')
ROBOT_DELAY_S = 8.0  # as localization_tests.launch.py: the sim is up first
OPPONENT = 'opponent_0'
OPPONENT_CLASS = 6  # red; we are blue
MCB_PTY = '/tmp/mcb_emulator_pty'


def _sim(context):
    config = context.launch_configurations
    if _is_true(context, 'lockstep_localization') and not (
            _is_true(context, 'paused_startup') and _is_true(context, 'lockstep')):
        raise RuntimeError('lockstep_localization requires paused_startup and lockstep')
    if _is_true(context, 'paused_startup') and not config['uart_socket']:
        config['uart_socket'] = f'/tmp/mcb_uart_{os.getpid()}.sock'
    windows = not _is_true(context, 'headless') and display_error() is None
    gui = 'true' if windows else 'false'
    spawn = ({'x': '4.625', 'y': '0.0', 'yaw': str(math.pi)}
             if config['stage'] in DRIVING else
             {'yaw': str(math.pi)} if config['backend'] == 'renode' else {})
    view = 'e2e-parked' if config['stage'] == 'mcb_parked' else 'e2e'
    share = get_package_share_directory('sim')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sim'), 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui, 'rviz': gui, 'foxglove': config['foxglove'],
                          'rviz_config': os.path.join(share, 'rviz', view + '.rviz'),
                          'gz_gui_config': os.path.join(share, 'rviz', view + '-gz.config'),
                          'camera': 'false', 'pose_emulator': 'false',
                          'mcb_physics': 'true',
                          'mcb_drive': 'stop',
                          'mcb_auto_fire': 'false' if config['stage'] in DRIVING else 'true',
                          'firmware_binary': config['firmware_binary'],
                          'mcb_backend': config['backend'],
                          'mcb_uart_socket': config['uart_socket'],
                          'mcb_baudrate': '115200',
                          'start_paused': config['paused_startup'],
                          'random_seed': '2026' if _is_true(context, 'paused_startup') else '',
                          'mcb_aim_tick_topic': '/sim/lockstep/aim_tick'
                          if _is_true(context, 'paused_startup') else '',
                          'mcb_aim_period_s': '0.025',
                          'mcb_raw_pose_ack_topic': '/sim/lockstep/raw_pose_tf'
                          if _is_true(context, 'paused_startup') else '',
                          'mcb_scorer_pose_ack_topic': '/sim/lockstep/scorer_pose'
                          if _is_true(context, 'paused_startup') else '',
                          'mcb_ekf_input_ack_topic': '/sim/lockstep/ekf_input'
                          if _is_true(context, 'lockstep_localization') else '',
                          'mcb_game_stage': '4',
                          'real_time_factor': config['real_time_factor'],
                          'lockstep': config['lockstep'], **spawn}.items())]


def _is_true(context, name):
    return context.launch_configurations[name].lower() in ('true', '1', 'yes')


def _tests(context):
    config = context.launch_configurations
    cmd = ['ros2', 'run', 'sim', 'e2e_suite', '--stage', config['stage'],
           '--backend', config['backend']]
    for arg, opt in (('speeds', '--e2e-speeds'), ('paths', '--e2e-paths'),
                     ('pose_latency_s', '--pose-latency-s'),
                     ('gimbal_lag_s', '--gimbal-lag-s'),
                     ('firmware_latency_s', '--firmware-latency-s'),
                     ('chase_settle_s', '--chase-settle-s'),
                     ('chase_margin_s', '--chase-margin-s'),
                     ('real_time_factor', '--real-time-factor'),
                     ('duration', '--e2e-duration'), ('log_dir', '--log-dir')):
        if config[arg]:
            cmd += [opt, ','.join(config[arg].replace(',', ' ').split())]
    if _is_true(context, 'headless'):
        cmd.append('--headless')
    if not _is_true(context, 'firmware_fixes'):
        cmd.append('--no-firmware-fixes')
    if not _is_true(context, 'lockstep'):
        cmd.append('--no-lockstep')
    cmd += config['pytest_args'].split()
    tests = ExecuteProcess(cmd=cmd, name='e2e_tests', output='screen',
                           additional_env={'MCB_SIM_UART_SOCKET': config['uart_socket'],
                                           'MCB_LOCKSTEP_LOCALIZATION':
                                               config['lockstep_localization'],
                                           'MCB_PAUSED_STARTUP': config['paused_startup']})
    done = RegisterEventHandler(OnProcessExit(
        target_action=tests,
        on_exit=lambda event, context: finish_suite(event, context, 'e2e')))
    return [tests, done]


def generate_launch_description():
    constants = subprocess.run(
        ['ros2', 'run', 'sim', 'e2e_suite', '--print-launch-constants'],
        check=True, capture_output=True, text=True).stdout
    parked_paths = json.loads(constants)
    share = get_package_share_directory('sim')
    xacro_file = os.path.join(share, 'urdf', 'sentry_v2.urdf.xacro')

    def detector(opponents):
        return Node(
            package='sim', executable='detector_standin_node', name='detector_standin',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'zero_phase': ParameterValue(
                    LaunchConfiguration('paused_startup'), value_type=bool),
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file]), value_type=str),
                'opponents': opponents,
                'class_ids': [2 if name.startswith('ally') else OPPONENT_CLASS
                              for name in opponents],
            }])

    target_driver = Node(
        package='sim', executable='target_driver', name='target_driver', output='screen',
        parameters=[{
            'use_sim_time': True,
            'paused_startup': ParameterValue(
                LaunchConfiguration('paused_startup'), value_type=bool),
            'target_speed': ParameterValue(LaunchConfiguration('target_speed'), value_type=float),
            'spin_hz': ParameterValue(LaunchConfiguration('target_spin_hz'), value_type=float),
            'origin_yaw': ParameterValue(PythonExpression([
                str(math.pi), " if '", LaunchConfiguration('backend'),
                "' == 'renode' else ", str(SPAWN_YAW)]), value_type=float),
            **parked_paths['lateral'],
        }])
    # The path the opponent's OpponentMover system rides, into gz.
    path_topic = PythonExpression([
        "'/sim/match/opponent_0/path' if '", LaunchConfiguration('stage'),
        "' in ", repr(DRIVING), " else '/target/ground_truth_odom'"])
    path_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='opponent_path_bridge',
        output='screen',
        arguments=[f'/model/{OPPONENT}/path@nav_msgs/msg/Odometry]gz.msgs.Odometry'],
        remappings=[(f'/model/{OPPONENT}/path', path_topic)],
        parameters=[{'use_sim_time': True}])
    opponent_driver = Node(
        package='sim', executable='opponent_driver', name='opponent_driver', output='screen',
        parameters=[{'use_sim_time': True, 'name': OPPONENT,
                     'paused_startup': ParameterValue(
                         LaunchConfiguration('paused_startup'), value_type=bool)}],
        remappings=[('/target/ground_truth_odom', path_topic)])
    # The hosted firmware parked (DrivetrainStopCommand) on a pty, as
    # auto.launch.py starts the bridge and relay with real_hardware:=true.
    bridge = Node(
        package='dji_serial_bridge', executable='dji_serial_bridge_node',
        name='dji_serial_bridge', output='screen',
        parameters=[{'use_sim_time': True, 'device': MCB_PTY, 'debug_log': False,
                     'sim_uart_socket': LaunchConfiguration('uart_socket'),
                     'baudrate': 115200}])
    mcb_relay = Node(
        package='thornbots_pkg', executable='mcb_relay', name='mcb_relay', output='screen',
        parameters=[{'use_sim_time': True,
                     'lockstep_inputs': LaunchConfiguration('lockstep_localization')}])

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        # Patrol stays on, as on the robot: patrol frames clear `fire`, and
        # it reacquires an opponent that starts out of view or is lost.
        launch_arguments={'real_hardware': 'false', 'localization_mode': 'amcl',
                          'allow_latest_tf': 'false',
                          'cv_target_publish_rate_hz': '40.0',
                          'joint_tf_publish_rate_hz': PythonExpression([
                              "'1000.0' if '", LaunchConfiguration('paused_startup'),
                              "' == 'true' else ''"]),
                          'aim_raw_pose_ack_topic': PythonExpression([
                              "'/sim/lockstep/raw_pose_tf' if '",
                              LaunchConfiguration('paused_startup'), "' == 'true' else ''"]),
                          'aim_tick_topic': PythonExpression([
                              "'/sim/lockstep/aim_tick' if '",
                              LaunchConfiguration('paused_startup'),
                              "' == 'true' else ''"]),
                          'aim_state_ack_topic': PythonExpression([
                              "'/sim/lockstep/aim_state' if '",
                              LaunchConfiguration('paused_startup'),
                              "' == 'true' else ''"]),
                          'tracker_consumed_topic': PythonExpression([
                              "'/sim/lockstep/tracker_consumed' if '",
                              LaunchConfiguration('paused_startup'), "' == 'true' else ''"]),
                          'tracker_state_at_capture': LaunchConfiguration('paused_startup'),
                          'lockstep_inputs': LaunchConfiguration('lockstep_localization'),
                          'aim_state_hold_s': PythonExpression([
                              "'0.014' if '", LaunchConfiguration('paused_startup'),
                              "' == 'true' else '0.0'"]),
                          'tracker_empty_ack_topic': PythonExpression([
                              "'/sim/lockstep/aim_state' if '",
                              LaunchConfiguration('paused_startup'), "' == 'true' else ''"]),
                          'pose_latency_s': LaunchConfiguration('pose_latency_s'),
                          'gimbal_lag_s': LaunchConfiguration('gimbal_lag_s'),
                          'firmware_latency_s': LaunchConfiguration('firmware_latency_s'),
                          'chase_settle_s': LaunchConfiguration('chase_settle_s'),
                          'chase_margin_s': LaunchConfiguration('chase_margin_s'),
                          'v_muzzle': '24.0',
                          'use_rf2o': 'true', 'load_map': 'true', 'patrol_enabled': 'true',
                          'initial_x': PythonExpression([
                              "'4.625' if '", LaunchConfiguration('stage'),
                              "' in ", repr(DRIVING), " else '0.0'"])}.items())

    match_environment = Node(
        package='sim', executable='match_environment', name='match_environment', output='screen',
        parameters=[{'use_sim_time': True, 'stage': LaunchConfiguration('stage'),
                     'paused_startup': ParameterValue(
                         LaunchConfiguration('paused_startup'), value_type=bool)}])

    def uart_stack(context):
        config = context.launch_configurations
        socket_path = config['uart_socket']
        ready = Node(package='sim', executable='mcb_ready', arguments=[
            socket_path or MCB_PTY, '600' if config['backend'] == 'renode' else '30',
            '--socket' if socket_path else '--pty'], output='screen')
        start = RegisterEventHandler(OnProcessExit(
            target_action=ready,
            on_exit=lambda event, context: [bridge, mcb_relay] if event.returncode == 0 else [
                EmitEvent(event=Shutdown(reason='MCB UART readiness failed'))]))
        return [start, ready]

    common = [path_bridge, opponent_driver, OpaqueFunction(function=uart_stack)]
    # One board batch ahead avoids outrunning the wall deadline during learning.
    coordinator = Node(
        package='sim', executable='lockstep_coordinator', output='screen',
        parameters=[{'chains': ['mcb_batch'], 'mcb_batch.period_s': 0.005,
                     'start_held': ParameterValue(
                         LaunchConfiguration('paused_startup'), value_type=bool),
                     'learn_margin_s': ParameterValue(PythonExpression([
                         "'0.005' if '", LaunchConfiguration('backend'),
                         "' == 'renode' else '0.05'"]), value_type=float),
                     'mcb_batch.phase_s': 0.0, 'mcb_batch.acks': ['/mcb_emulator/tick']}],
        condition=IfCondition(LaunchConfiguration('lockstep')))

    def stack(context):
        active_coordinator = coordinator
        stage = context.launch_configurations['stage']
        if stage not in STAGES:
            raise RuntimeError(f"stage must be one of {', '.join(STAGES)}, not '{stage}'")
        extras = []
        if not _is_true(context, 'headless') and display_error() is None:
            extras.append(Node(package='sim', executable='target_state_markers',
                               parameters=[{'use_sim_time': True}]))
            extras.append(Node(package='sim', executable='e2e_map_view',
                               additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(
                                   get_package_share_directory('thornbots_pkg'), 'config',
                                   'fastdds_udp_only.xml')},
                               parameters=[{'use_sim_time': True}]))
        opponents = [OPPONENT]
        if stage == 'mcb_match':
            opponents += ['opponent_1', 'ally_0']
            for name in opponents[1:]:
                topic = f'/sim/match/{name}/path'
                extras += [Node(
                    package='sim', executable='opponent_driver',
                    name=f'opponent_driver_{name}', output='screen',
                    parameters=[{'use_sim_time': True, 'name': name,
                                 'paused_startup': ParameterValue(
                                     LaunchConfiguration('paused_startup'), value_type=bool)}],
                    remappings=[('/target/ground_truth_odom', topic)]), Node(
                    package='ros_gz_bridge', executable='parameter_bridge',
                    name=f'path_bridge_{name}', output='screen',
                    arguments=[f'/model/{name}/path@nav_msgs/msg/Odometry]gz.msgs.Odometry'],
                    remappings=[(f'/model/{name}/path', topic)],
                    parameters=[{'use_sim_time': True}])]
        start_robot = [TimerAction(period=ROBOT_DELAY_S, actions=[robot])]
        if _is_true(context, 'paused_startup'):
            active_coordinator = Node(
                package='sim', executable='lockstep_coordinator', output='screen',
                parameters=[{
                    'start_held': True,
                    'startup_ack_topics': ['/mcb_emulator/tick'],
                    'chains': ['mcb_batch', 'vision', 'target_path'],
                    'mcb_batch.period_s': 0.005,
                    'mcb_batch.acks': ['/mcb_emulator/tick'],
                    'vision.period_s': 1.0 / 60.0,
                    'vision.delay_s': 0.014,
                    'vision.acks': ['/sim/lockstep/tracker_consumed',
                                    '/sim/lockstep/aim_state'],
                    'target_path.period_s': 0.01 if stage in DRIVING else 1.0 / 60.0,
                    'target_path.acks': [f'gz:/model/{name}/path_received'
                                         for name in opponents],
                    **({'chains': ['mcb_batch', 'vision', 'target_path', 'ekf', 'scorer'],
                        'ekf.period_s': 1.0 / 30.0,
                        'ekf.acks': ['/sim/lockstep/ekf', '/sim/lockstep/relocalize_uart'],
                        'scorer.period_s': 0.05,
                        'scorer.acks': ['/sim/lockstep/scorer']}
                       if _is_true(context, 'lockstep_localization') else {}),
                }], condition=IfCondition(LaunchConfiguration('lockstep')))
            required = ['detector_standin', 'target_selector', 'target_tracker',
                        'point_to_cv_target', 'pose_translator', 'odom_tf_broadcaster',
                        'robot_state_publisher', 'lidar_self_filter', 'mcb_emulator',
                        'dji_serial_bridge', 'mcb_relay', 'ekf_filter_node',
                        'rf2o_laser_odometry', 'map_server', 'amcl']
            if _is_true(context, 'lockstep'):
                required.append('lockstep_coordinator')
            startup = Node(package='sim', executable='e2e_startup', output='screen',
                           parameters=[{'required_nodes': required,
                                        'ready_topics': [
                                            '/sim/lockstep/coordinator_ready',
                                            '/mcb_emulator/tick_ready']
                                            if _is_true(context, 'lockstep') else [],
                                        'models': ['sentry'] + opponents,
                                        'timeout_s': 600.0 if context.launch_configurations[
                                            'backend'] == 'renode' else 30.0}])
            startup_exit = RegisterEventHandler(OnProcessExit(
                target_action=startup,
                on_exit=lambda event, context: [] if event.returncode == 0 else [
                    EmitEvent(event=Shutdown(reason='Paused startup failed'))]))
            start_robot = [robot, startup_exit, startup]
        return ([active_coordinator, detector(opponents)] + extras + common
                + ([match_environment] if stage in DRIVING else [target_driver])
                + start_robot)

    return LaunchDescription([
        DeclareLaunchArgument('stage', default_value='mcb_parked',
                              description='mcb_parked: 12 opponent cells, robot parked; '
                                          'mcb_drive: spawn to center; mcb_match: 2v2'),
        DeclareLaunchArgument('lockstep_localization',
                              default_value=os.environ.get('MCB_LOCKSTEP_LOCALIZATION', 'false')),
        DeclareLaunchArgument('firmware_fixes', default_value='true',
                              description='Deprecated: the native MCB runs its checked-out code'),
        DeclareLaunchArgument('firmware_binary', default_value='',
                              description='MCB executable or board ELF; empty uses the '
                                          'selected backend release build'),
        DeclareLaunchArgument('backend', default_value='renode', choices=['hosted', 'renode']),
        DeclareLaunchArgument('uart_socket',
                              default_value=os.environ.get('MCB_SIM_UART_SOCKET', ''),
                              description='Experimental sim-time UART socket; empty uses PTY'),
        DeclareLaunchArgument('paused_startup',
                              default_value=os.environ.get('MCB_PAUSED_STARTUP', 'false'),
                              choices=['true', 'false']),
        DeclareLaunchArgument('run_tests', default_value='true',
                              description='false: bring up the stack only'),
        DeclareLaunchArgument('speeds', default_value='',
                              description="opponent speeds, m/s, 0 = stationary, e.g. '0 2'; "
                                          'empty = the harness default'),
        DeclareLaunchArgument('firmware_latency_s', default_value='0.094',
                              description='Decision-to-exit firing delay estimate in seconds'),
        DeclareLaunchArgument('chase_settle_s', default_value='-1.0'),
        DeclareLaunchArgument('chase_margin_s', default_value=PythonExpression([
            "'0.025' if '", LaunchConfiguration('backend'), "' == 'hosted' else '0.0'"])),
        DeclareLaunchArgument('gimbal_lag_s', default_value=PythonExpression([
            "'0.0' if '", LaunchConfiguration('backend'), "' == 'hosted' else '0.05'"]),
                              description='Hosted response calibration; board remains '
                                          'uncalibrated'),
        DeclareLaunchArgument('pose_latency_s', default_value='0.0',
                              description='Capture-time pose transport calibration, seconds'),
        DeclareLaunchArgument('paths', default_value='',
                              description="target_driver paths, e.g. 'lateral radial'; "
                                          'empty = all three'),
        DeclareLaunchArgument('duration', default_value='',
                              description='sim seconds scored per cell'),
        DeclareLaunchArgument('log_dir', default_value='',
                              description='where stack.log, shots.jsonl and scores.jsonl go'),
        DeclareLaunchArgument('pytest_args', default_value='',
                              description="extra gtest args, e.g. '--gtest_fail_fast'"),
        DeclareLaunchArgument('headless', default_value='false',
                              description='skip the gz GUI and rviz2'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='Foxglove bridge on :8765'),
        DeclareLaunchArgument('real_time_factor', default_value='0',
                              description='sim speed cap; 0 = as fast as it runs'),
        DeclareLaunchArgument('lockstep', default_value='true',
                              description='hold gz at MCB batch deadlines until its tick arrives'),
        DeclareLaunchArgument('target_speed', default_value='2.0',
                              description="the opponent's speed along its path, m/s"),
        DeclareLaunchArgument('target_spin_hz', default_value='1.5',
                              description="the opponent's chassis spin, Hz"),
        # run_tests: the C++ harness alone, which brings the stack up as one process
        # group and kills the group after; a stack here outlived the tests'
        # Shutdown (gz sim survived its ruby wrapper, 2026-09-29).
        OpaqueFunction(function=lambda context: _tests(context) if _is_true(
            context, 'run_tests') else _sim(context) + stack(context)),
    ])
