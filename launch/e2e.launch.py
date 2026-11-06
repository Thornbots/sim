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
See README.md for design rationale and ../../docs/E2E_PLAN.md for acceptance criteria.
"""
import atexit
import json
import math
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
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
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from launch_ros.parameter_descriptions import ParameterValue
from sim.auto_explore import SPAWN_YAW
from sim.dds_switch import FOXGLOVE_DEFAULT, udp_env
from sim.display import display_error
from sim.field import BLUE_START, team_start_args
from sim.mcb_link import default_mcb_link
from sim.suite_exit import finish_suite, suite_command

STAGES = ('mcb_parked', 'mcb_drive', 'mcb_match')
DRIVING = ('mcb_drive', 'mcb_match')
ROBOT_DELAY_S = 8.0  # as localization_tests.launch.py: the sim is up first
OPPONENT = 'opponent_0'
OPPONENT_CLASS = 6  # red; we are blue


def startup_wait_s(config):
    """Wall seconds the gate and spawn check may wait at epoch zero for the stack."""
    return float(config['epoch_startup_timeout_s'] or (
        1200 if config['backend'] == 'renode' else 90)) + 60


def _sim(context):
    config = context.launch_configurations
    if int(config['uart_baud']) not in (9600, 19200, 38400, 57600, 115200, 230400,
                                        460800, 921600, 2000000, 4000000):
        raise ValueError('uart_baud must be supported by dji_serial_bridge')
    if _uart_coordinated(context) and not _is_true(context, 'lockstep'):
        raise ValueError('uart_deterministic requires lockstep')
    windows = not _is_true(context, 'headless') and display_error() is None
    gui = 'true' if windows else 'false'
    spawn = {'x': str(BLUE_START[0]), 'y': str(BLUE_START[1]), 'yaw': str(SPAWN_YAW)}
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
                          'uart_relay_fixture': config['uart_relay_fixture'],
                          'uart_deterministic': 'true' if _uart_coordinated(context) else 'false',
                          'uart_epoch_zero': config['deterministic_epoch_zero'],
                          'lidar_keyed_noise': 'true' if _is_true(
                              context, 'producer_chains') else 'false',
                          'uart_pose_consumed': '/sim/e2e/pose_consumed' if _is_true(
                              context, 'producer_chains') else 'none',
                          'verify_epoch_zero_spawn': config['deterministic_epoch_zero'],
                          'uart_baud': config['uart_baud'],
                          'mcb_read_delay_ms': config['mcb_read_delay_ms'],
                          'mcb_device_link': config['mcb_device_link'],
                          'mcb_drive': 'stop',
                          'mcb_auto_fire': 'false',
                          'firmware_binary': config['firmware_binary'],
                          'mcb_backend': config['backend'],
                          'mcb_game_stage': '4',
                          'real_time_factor': config['real_time_factor'],
                          'startup_wait_s': str(startup_wait_s(config)),
                          'lockstep': config['lockstep'], **spawn}.items())]


def _is_true(context, name):
    return context.launch_configurations[name].lower() in ('true', '1', 'yes')


# Lockstep chains for the Jetson-side producers whose frames reach the firmware, and the
# data they consume. `acks` are groups (`a|b`: any one); `lag_s`/`align_s` hold after the
# tick (see lockstep_coordinator.cpp). Topic names: `<topic>#<type>` acks any message
# whose first field is a Header. Rates are the nodes' own timers and sources.
ODOM = 'nav_msgs/msg/Odometry'
EKF_PERIOD_S = 0.034  # simulation only: 29.4 Hz for the stock 30 Hz, a whole number of ms
PRODUCER_CHAINS = {
    # rf2o's verdict on a lidar scan: its quality diagnostic, one per scan even when the
    # match fails (no odometry then). gz runs the 11.2 Hz lidar on whole milliseconds, 89 ms
    # apart from 1 ms past a whole step, and rf2o grades from the second scan. It needs /odom
    # and TF past the scan stamp, and POSE skips every 100th slot, so wait 30 ms.
    'scan': {'period_s': 0.089, 'phase_s': 0.001, 'start_s': 0.17, 'lag_s': 0.03, 'acks': [
        '/scan_odom/quality#diagnostic_msgs/msg/DiagnosticArray']},
    # Releases rf2o's results to the EKF one step before the tick that may use them
    # (scan_odom_gate): the first such hold at least 30 ms after the scan, which is when
    # the scan chain has held for the match. Without it a result reached either tick.
    'ekf_pre': {'period_s': EKF_PERIOD_S, 'phase_s': 0.0, 'start_s': 0.1, 'lag_s': 0.0,
                'acks': ['/sim/e2e/scan_released']},
    # The EKF (stock robot_localization) ticks from a ROS timer created at its first non-zero
    # clock, which `boot_hold_s` makes 1 ms. EKF_PERIOD_S is a whole number of milliseconds so
    # every tick lands on one clock value. Its odometry is stamped with its newest
    # measurement and the relay's clock can trail the EKF's, so the ack is a count: one
    # relay frame or mark per tick.
    'ekf': {'period_s': EKF_PERIOD_S, 'phase_s': 0.001, 'start_s': 0.002, 'lag_s': 0.0,
            'acks': ['~/dji_serial_bridge/tx_ack/relocalize|~/sim/e2e/relocalize_none']},
    # point_to_cv_target's 40 Hz idle look.
    'cv_target': {'period_s': 0.025, 'phase_s': 0.0, 'lag_s': 0.0, 'acks': [
        '/dji_serial_bridge/tx_ack/cv_target|/sim/e2e/cv_target_none']},
    # detector_standin's 60 Hz frame through selector and tracker to TARGET_STATE; the
    # tracker needs TF 10 ms past the capture (pose_latency_s) and gz poses on a 4 ms grid.
    'detector': {'period_s': 1.0 / 60.0, 'phase_s': 0.0, 'lag_s': 0.025, 'align_s': 0.004,
                 'acks': ['/dji_serial_bridge/tx_ack/target_state|/sim/e2e/target_state_none']},
    # waypoint_nav's 10 Hz NAV_GOAL, held 1 ms after its tick so the EKF chain (same
    # instant every third tick) has been released first.
    'nav': {'period_s': 0.1, 'phase_s': 0.0, 'lag_s': 0.0, 'acks': [
        '/dji_serial_bridge/tx_ack/nav_goal|/sim/e2e/nav_goal_none']},
}
# Marks set on the robot launch; mcb_relay and waypoint_nav start here and are set above.
PRODUCER_MARKS = {
    # The detector chain holds at tick + 25 ms rounded to the 4 ms gz pose grid.
    'process_delay_s': 0.025,
    'process_align_s': 0.004,
    'cv_target_none_topic': '/sim/e2e/cv_target_none',
    'target_state_none_topic': '/sim/e2e/target_state_none',
}


def _resolve_modes(context):
    """
    Replace the `auto` startup modes with true or false for every consumer.

    Coordinated UART follows lockstep, which its gate needs. Epoch-zero startup
    (paused gz, readiness barrier, no wall-time bring-up) follows coordinated UART
    without the relay fixture. Explicit values are validated by `_validate_modes`.
    """
    config = context.launch_configurations
    uart = config['uart_deterministic'].lower()
    uart_on = _is_true(context, 'lockstep') if uart in ('', 'auto') else uart in (
        'true', '1', 'yes')
    config['uart_deterministic'] = 'true' if uart_on else 'false'
    epoch = config['deterministic_epoch_zero'].lower()
    epoch_on = (uart_on and not _is_true(context, 'uart_relay_fixture')
                if epoch in ('', 'auto') else epoch in ('true', '1', 'yes'))
    config['deterministic_epoch_zero'] = 'true' if epoch_on else 'false'
    chains = config['producer_chains'].lower()
    config['producer_chains'] = 'true' if (
        epoch_on and chains in ('', 'auto', 'true', '1', 'yes')) else 'false'


def _uart_coordinated(context):
    return _is_true(context, 'uart_deterministic')


def _barrier_nodes(stage):
    """
    Return the nodes whose graph presence marks the stack as constructed.

    AMCL and its map server are absent: they start on referee data (a pose cannot
    be set before TF exists), they are off the hit path, and so keep that gate.
    """
    nodes = ['detector_standin', 'dji_serial_bridge', 'mcb_relay', 'waypoint_nav',
             'mcb_emulator', 'lidar_scan', 'head_slider_relay', 'opponent_driver',
             'point_to_cv_target', 'target_selector', 'target_tracker', 'pose_translator',
             'odom_tf_broadcaster', 'robot_state_publisher', 'lidar_self_filter',
             'ekf_input_publisher', 'ekf_filter_node', 'rf2o_laser_odometry',
             'scan_navigation_publisher', 'map_pose_publisher']
    nodes.append('match_environment' if stage in DRIVING else 'target_driver')
    if stage == 'mcb_match':
        nodes += ['opponent_driver_opponent_1', 'opponent_driver_ally_0']
    return nodes


def _tests(context):
    config = context.launch_configurations
    if (config['deterministic_epoch_zero'] not in ('', 'auto') or
            config['uart_deterministic'] not in ('', 'auto') or
            _is_true(context, 'uart_relay_fixture') or
            config['uart_baud'] != '115200' or config['mcb_read_delay_ms'] != '0'):
        raise RuntimeError('UART fixture/baud overrides require run_tests:=false; '
                           'suite does not forward them')
    cmd = [*suite_command('e2e_suite'), '--stage', config['stage'],
           '--backend', config['backend']]
    for arg, opt in (('speeds', '--e2e-speeds'), ('paths', '--e2e-paths'),
                     ('pose_latency_s', '--pose-latency-s'),
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
    tests = ExecuteProcess(cmd=cmd, name='e2e_tests', output='screen')
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
    coordinated = LaunchConfiguration('uart_deterministic')  # resolved by _resolve_modes
    xacro_file = os.path.join(share, 'urdf', 'sentry_v2.urdf.xacro')

    def detector(opponents):
        return Node(
            package='sim', executable='detector_standin_node', name='detector_standin',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'deterministic_epoch_zero': ParameterValue(
                    LaunchConfiguration('deterministic_epoch_zero'), value_type=bool),
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
            'deterministic_epoch_zero': ParameterValue(
                LaunchConfiguration('deterministic_epoch_zero'), value_type=bool),
            'target_speed': ParameterValue(LaunchConfiguration('target_speed'), value_type=float),
            'spin_hz': ParameterValue(LaunchConfiguration('target_spin_hz'), value_type=float),
            'origin_yaw': -math.pi / 2,  # Preserve the field-safe parked paths.
            **parked_paths['lateral'],
        }])
    # The path the opponent's OpponentMover system rides, into gz.
    path_topic = PythonExpression([
        "'/sim/match/opponent_0/path' if '", LaunchConfiguration('stage'),
        "' in ", repr(DRIVING), " else '/target/ground_truth_odom'"])
    path_bridge = Node(
        package='sim', executable='gz_bridge', name='opponent_path_bridge',
        output='screen',
        arguments=[f'/model/{OPPONENT}/path@nav_msgs/msg/Odometry]gz.msgs.Odometry'],
        remappings=[(f'/model/{OPPONENT}/path', path_topic)],
        parameters=[{'use_sim_time': True}])
    # The hosted firmware parked (DrivetrainStopCommand) on a pty, as
    # auto.launch.py starts the bridge and relay with real_hardware:=true.
    marks = LaunchConfiguration('producer_chains')
    bridge = Node(
        package='dji_serial_bridge', executable='dji_serial_bridge_node',
        name='dji_serial_bridge', output='screen',
        additional_env=udp_env(),
        parameters=[{'use_sim_time': True,
                     'device': LaunchConfiguration('mcb_device_link'),
                     'baudrate': LaunchConfiguration('uart_baud'),
                     'sim_frame_metadata': ParameterValue(coordinated, value_type=bool),
                     'sim_tx_queue': ParameterValue(LaunchConfiguration('producer_chains'),
                                                    value_type=bool),
                     'debug_log': False}])
    mcb_relay = Node(
        package='thornbots_pkg', executable='mcb_relay', name='mcb_relay', output='screen',
        parameters=[{'use_sim_time': True,
                     'relocalize_none_topic': PythonExpression([
                         "'/sim/e2e/relocalize_none' if '", marks, "' == 'true' else ''"]),
                     'message_clock': ParameterValue(marks, value_type=bool),
                     'raw_odom_ack_topic': PythonExpression([
                         "'/sim/e2e/relay_odom' if '", marks, "' == 'true' else ''"])}])
    # auto.launch.py starts waypoint_nav only with real_hardware:=true, like the relay.
    waypoint_nav = Node(
        package='thornbots_nav', executable='waypoint_nav', name='waypoint_nav', output='screen',
        parameters=[os.path.join(get_package_share_directory('thornbots_nav'), 'config',
                                 'route.yaml'), {'use_sim_time': True,
                                                 'nav_goal_none_topic': PythonExpression([
                                                     "'/sim/e2e/nav_goal_none' if '", marks,
                                                     "' == 'true' else ''"])}])

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        launch_arguments={**team_start_args(),
                          'real_hardware': 'false', 'localization_mode': 'amcl',
                          'allow_latest_tf': PythonExpression([
                              "'false' if '", LaunchConfiguration('deterministic_epoch_zero'),
                              "' == 'true' else 'true'"]),
                          'tf_lookup_lag_s': '0.05',
                          'nav_tf_lookup_lag_s': '0.1',
                          # Its wall-clock ping protocol writes UART frames at wall-timed moments.
                          'enable_mmm': PythonExpression([
                              "'false' if '", LaunchConfiguration('producer_chains'),
                              "' == 'true' else 'true'"]),
                          'ekf_frequency_hz': PythonExpression([
                              "'", repr(1.0 / EKF_PERIOD_S), "' if '",
                              LaunchConfiguration('producer_chains'), "' == 'true' else ''"]),
                          # tf2 returns a sample exactly or slerps at ratio 0 (1 ulp off) by
                          # whether the next one has arrived; wait for a newer sample (at most
                          # 20 ms later, since POSE-grid TF skips every 100th slot; the scan
                          # chain holds 30 ms).
                          'scan_navigation_odom_topic': PythonExpression([
                              "'/scan_odom/gated' if '", LaunchConfiguration('producer_chains'),
                              "' == 'true' else '/scan_odom'"]),
                          'rf2o_extrinsic_lookahead_s': PythonExpression([
                              "'0.001' if '", LaunchConfiguration('producer_chains'),
                              "' == 'true' else ''"]),
                          'seed_known_team': PythonExpression([
                              "'blue' if '", LaunchConfiguration('deterministic_epoch_zero'),
                              "' == 'true' else 'none'"]),
                          'deterministic_epoch_zero': LaunchConfiguration(
                              'deterministic_epoch_zero'),
                          'pose_latency_s': LaunchConfiguration('pose_latency_s'),
                          'use_rf2o': 'true', 'load_map': 'true'}.items())

    match_environment = Node(
        package='sim', executable='match_environment', name='match_environment', output='screen',
        parameters=[{'use_sim_time': True, 'stage': LaunchConfiguration('stage'),
                     'deterministic_epoch_zero': ParameterValue(
                         LaunchConfiguration('deterministic_epoch_zero'), value_type=bool)}])
    # The bridge opens the pty once, so it starts after the emulator makes it.
    uart_ready = Node(package='sim', executable='mcb_ready', arguments=[
        LaunchConfiguration('mcb_device_link'),
        PythonExpression(["'600' if '", LaunchConfiguration('backend'),
                          "' == 'renode' else '30'"]),
        PythonExpression(["'device_only' if '", coordinated, "' == 'true' else 'frame'"]),
    ],
        output='screen')
    uart_start = RegisterEventHandler(OnProcessExit(
        target_action=uart_ready,
        on_exit=lambda event, context: [
            bridge, mcb_relay, waypoint_nav] if event.returncode == 0 else [
            OpaqueFunction(function=lambda context: finish_suite(
                event, context, 'MCB UART readiness')
                           if _is_true(context, 'deterministic_epoch_zero') else [
                               EmitEvent(event=Shutdown(reason='MCB UART readiness failed'))])]))
    common = [path_bridge, uart_start, uart_ready]

    # One board batch ahead avoids outrunning the wall deadline during learning.
    def coordinator(deterministic, stage, extra_ready, startup_timeout, harness_chain,
                    producers):
        params = {'chains': ['mcb_batch'], 'mcb_batch.period_s': 0.005,
                  'learn_margin_s': ParameterValue(PythonExpression([
                      "'0.005' if '", LaunchConfiguration('backend'),
                      "' == 'renode' else '0.05'"]), value_type=float),
                  'mcb_batch.phase_s': 0.0, 'mcb_batch.acks': ['/mcb_emulator/tick']}
        if harness_chain:
            # The scorer decides on a 50 ms tick, 1 ms after the other chains' ticks, so
            # that anything it sets is seen by the first tick after, never half of one.
            params['chains'] = ['mcb_batch', 'harness']
            params.update({'harness.period_s': 0.05, 'harness.phase_s': 0.001,
                           'harness.acks': ['/sim/e2e/harness_ack']})
        if producers:
            params['boot_hold_s'] = 0.001
            params['flush_topic'] = '/dji_serial_bridge/sim_tx_flush'
            params['flushed_topic'] = '/dji_serial_bridge/sim_tx_flushed'
        if producers:
            for name, chain in PRODUCER_CHAINS.items():
                params['chains'] = params['chains'] + [name]
                params[f'{name}.period_s'] = chain['period_s']
                params[f'{name}.phase_s'] = chain['phase_s']
                params[f'{name}.lag_s'] = chain['lag_s']
                params[f'{name}.align_s'] = chain.get('align_s', 0.0)
                params[f'{name}.start_s'] = chain.get('start_s', 0.0)
                params[f'{name}.acks'] = chain['acks']
        if deterministic:
            params.update(strict_startup=True, readiness_topics=[
                '/detector_standin/ready', '/point_to_cv_target/epoch_zero_ready',
                '/opponent_driver/ready', '/mcb_emulator/epoch_zero_ready',
                '/robot_spawn_ready/ready', '/startup_barrier/ready',
                '/match_environment/ready' if stage in DRIVING else '/target_driver/ready'] +
                (['/opponent_driver_opponent_1/ready', '/opponent_driver_ally_0/ready']
                 if stage == 'mcb_match' else []) + ([extra_ready] if extra_ready else []),
                startup_timeout_s=startup_timeout)
        return Node(
            package='sim', executable='lockstep_coordinator', output='screen',
            parameters=[params],
            condition=IfCondition(LaunchConfiguration('lockstep')))

    def stack(context):
        stage = context.launch_configurations['stage']
        if stage not in STAGES:
            raise RuntimeError(f"stage must be one of {', '.join(STAGES)}, not '{stage}'")
        deterministic = _is_true(context, 'deterministic_epoch_zero')
        if deterministic and (not _is_true(context, 'lockstep') or
                              not _uart_coordinated(context) or
                              _is_true(context, 'uart_relay_fixture')):
            raise RuntimeError('deterministic_epoch_zero requires lockstep and '
                               'uart_deterministic without uart_relay_fixture')
        opponent_params = {'use_sim_time': True, 'name': OPPONENT}
        # The board boots inside robot creation, so scene checks may wait that long.
        scene_wait = {'startup_timeout_s': startup_wait_s(context.launch_configurations)}
        if deterministic:
            opponent_params.update(scene_wait)
            opponent_params['initial_position'] = (
                parked_paths['initial_positions'][OPPONENT] if stage in DRIVING else
                [0.0, -parked_paths['lateral']['center_x']])
            if stage in DRIVING:
                opponent_params['initial_yaw'] = parked_paths['initial_yaws'][OPPONENT]
        opponent_driver = Node(
            package='sim', executable='opponent_driver', name='opponent_driver', output='screen',
            parameters=[opponent_params], remappings=[('/target/ground_truth_odom', path_topic)])
        extras = []
        if not _is_true(context, 'headless') and display_error() is None:
            extras.append(Node(package='sim', executable='target_state_markers',
                               parameters=[{'use_sim_time': True}]))
            extras.append(Node(package='sim', executable='e2e_map_view',
                               additional_env=udp_env(),
                               parameters=[{'use_sim_time': True}]))
        opponents = [OPPONENT]
        if stage == 'mcb_match':
            opponents += ['opponent_1', 'ally_0']
            for name in opponents[1:]:
                topic = f'/sim/match/{name}/path'
                extras += [Node(
                    package='sim', executable='opponent_driver',
                    name=f'opponent_driver_{name}', output='screen',
                    parameters=[{'use_sim_time': True, 'name': name, **(
                        {'initial_position': parked_paths['initial_positions'][name],
                         'initial_yaw': parked_paths['initial_yaws'][name], **scene_wait}
                        if deterministic else {})}],
                    remappings=[('/target/ground_truth_odom', topic)]), Node(
                    package='sim', executable='gz_bridge',
                    name=f'path_bridge_{name}', output='screen',
                    arguments=[f'/model/{name}/path@nav_msgs/msg/Odometry]gz.msgs.Odometry'],
                    remappings=[(f'/model/{name}/path', topic)],
                    parameters=[{'use_sim_time': True}])]
        robot_bringup = robot
        if stage == 'mcb_parked':
            # Jazzy's global SetParameter stringifies arrays as Python tuples.
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as params:
                json.dump({'point_to_cv_target': {'ros__parameters': {
                    'idle_zone': parked_paths['idle_zone']}}}, params)
            atexit.register(os.unlink, params.name)
            robot_bringup = GroupAction([
                SetParametersFromFile(params.name), robot])
        extra_ready = context.launch_configurations['epoch_readiness_extra']
        startup_timeout = float(context.launch_configurations['epoch_startup_timeout_s'] or (
            1200 if context.launch_configurations['backend'] == 'renode' else 90))
        if not math.isfinite(startup_timeout) or startup_timeout <= 0 or startup_timeout > 3600:
            raise ValueError('epoch_startup_timeout_s must be in (0, 3600]')
        if extra_ready and (not deterministic or not extra_ready.startswith('/')):
            raise ValueError('epoch_readiness_extra requires deterministic mode and an '
                             'absolute topic')
        harness_chain = _is_true(context, 'harness_chain')
        if harness_chain and not deterministic:
            raise ValueError('harness_chain requires epoch-zero startup')
        producers = _is_true(context, 'producer_chains')
        coordination = coordinator(deterministic, stage, extra_ready, startup_timeout,
                                   harness_chain, producers)
        delay = float(context.launch_configurations['epoch_coordinator_delay_s'])
        if not math.isfinite(delay) or delay < 0 or delay > 10 or (delay and not deterministic):
            raise ValueError('epoch_coordinator_delay_s requires deterministic mode and 0..10 s')
        failure = RegisterEventHandler(OnProcessExit(
            target_action=coordination,
            on_exit=lambda event, context: finish_suite(event, context, 'lockstep coordinator')
            if deterministic and event.returncode else []))
        coordinator_action = (TimerAction(period=delay, actions=[coordination])
                              if delay else coordination)
        barrier = Node(
            package='sim', executable='startup_barrier', name='startup_barrier',
            output='screen',
            parameters=[{'nodes': _barrier_nodes(stage), 'settle_s': 2.0,
                         'timeout_s': startup_timeout}]) if deterministic else None
        # With epoch-zero startup AMCL starts before any TF, so it is given its pose later.
        pose_setter = Node(
            package='sim', executable='initial_pose_setter', name='initial_pose_setter',
            output='screen',
            parameters=[{'x': BLUE_START[0], 'y': BLUE_START[1], 'yaw': 0.0}],
        ) if deterministic else None
        if producers:
            extras.append(Node(package='sim', executable='pose_consumed', output='screen',
                               parameters=[{'relay_topic': '/sim/e2e/relay_odom'}]))
            extras.append(Node(package='sim', executable='scan_odom_gate', output='screen',
                               parameters=[{'period_s': EKF_PERIOD_S}]))
        return ([failure, coordinator_action, detector(opponents), opponent_driver] + extras
                + ([barrier, pose_setter] if barrier else []) + common
                + ([match_environment] if stage in DRIVING else [target_driver])
                + ([GroupAction([SetParameter(name='tick_topic', value='/sim/e2e/aim_tick'),
                                 *[SetParameter(name=k, value=v) for k, v in
                                   (PRODUCER_MARKS.items() if producers else [])],
                                 robot_bringup])] if deterministic else
                   [TimerAction(period=ROBOT_DELAY_S, actions=[robot_bringup])]))

    def entry(context):
        if _is_true(context, 'run_tests'):
            return _tests(context)
        _resolve_modes(context)
        return _sim(context) + stack(context)

    return LaunchDescription([
        DeclareLaunchArgument('deterministic_epoch_zero', default_value='auto',
                              description='Paused-at-zero startup behind a readiness barrier; '
                                          'auto follows coordinated UART'),
        DeclareLaunchArgument('producer_chains', default_value='auto',
                              description='Lockstep chains for the Jetson-side producers, so '
                                          'every Jetson-to-MCB frame lands on a fixed step; '
                                          'auto follows epoch-zero startup'),
        DeclareLaunchArgument('harness_chain', default_value='false',
                              description='The E2E harness acks a 50 ms tick that holds physics'),
        DeclareLaunchArgument('epoch_coordinator_delay_s', default_value='0',
                              description='Diagnostic startup-order delay, wall seconds'),
        DeclareLaunchArgument('epoch_readiness_extra', default_value='',
                              description='Optional additional diagnostic readiness topic'),
        DeclareLaunchArgument('epoch_startup_timeout_s', default_value='',
                              description='Readiness deadline in wall seconds; empty is 90 for '
                                          'hosted and 1200 for the board, which boots first'),
        DeclareLaunchArgument('uart_deterministic', default_value='auto',
                              description='Coordinated sim-time UART (bytes x 10 / baud, '
                                          'per-frame acks); auto follows lockstep'),
        DeclareLaunchArgument('uart_relay_fixture', default_value='false',
                              description='Transport without deterministic timestamps or acks'),
        DeclareLaunchArgument('uart_baud', default_value='115200'),
        DeclareLaunchArgument('mcb_read_delay_ms', default_value='0'),
        DeclareLaunchArgument('stage', default_value='mcb_parked',
                              description='mcb_parked: 12 opponent cells, robot parked; '
                                          'mcb_drive: spawn to center; mcb_match: 2v2'),
        DeclareLaunchArgument('firmware_fixes', default_value='true',
                              description='Deprecated: the native MCB runs its checked-out code'),
        DeclareLaunchArgument('firmware_binary', default_value='',
                              description='MCB executable or board ELF; empty uses the '
                                          'selected backend release build'),
        DeclareLaunchArgument('backend', default_value='renode', choices=['hosted', 'renode']),
        DeclareLaunchArgument('run_tests', default_value='true',
                              description='false: bring up the stack only'),
        DeclareLaunchArgument('speeds', default_value='',
                              description="opponent speeds, m/s, 0 = stationary, e.g. '0 2'; "
                                          'empty = the harness default'),
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
        DeclareLaunchArgument('foxglove', default_value=FOXGLOVE_DEFAULT,
                              description='Foxglove bridge on :8765'),
        DeclareLaunchArgument('real_time_factor', default_value='0',
                              description='sim speed cap; 0 = as fast as it runs'),
        DeclareLaunchArgument('mcb_device_link', default_value=default_mcb_link(),
                              description='PTY link; unique per GZ_PARTITION/ROS_DOMAIN_ID'),
        DeclareLaunchArgument('lockstep', default_value='true',
                              description='hold gz at MCB batch deadlines until its tick arrives'),
        DeclareLaunchArgument('target_speed', default_value='2.0',
                              description="the opponent's speed along its path, m/s"),
        DeclareLaunchArgument('target_spin_hz', default_value='1.5',
                              description="the opponent's chassis spin, Hz"),
        # run_tests: the C++ harness alone, which brings the stack up as one process
        # group and kills the group after; a stack here outlived the tests'
        # Shutdown (gz sim survived its ruby wrapper, 2026-09-29).
        OpaqueFunction(function=entry),
    ])
