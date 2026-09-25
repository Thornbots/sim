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
C1 aim bench: point_to_cv_target against a perfectly known target, no gz.

`ros2 launch sim shot_hit.launch.py [speeds:='0.5 1'] [only_stationary:=true]`.
sim_clock publishes /clock, point_shooter puts root at POINT_SHOOTER
(bouncing along y at shooter_speed), target_driver moves the phantom target,
target_state_truth publishes its true TargetState, and the pytest sends each
shot from the shooter toward the newest aim (a perfect gimbal). Every miss is
point_to_cv_target's. gz belongs to Part 2's estimation bench, not here.
Stops when the tests finish; Ctrl-C stops the whole stack. `run_tests:=false`
brings up the stack alone (test_shot_hit.py's cv_stack fixture does that).
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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

    actions = _point_stack(context, harness, _is_true(context, 'headless'))
    if not _is_true(context, 'run_tests'):
        return actions
    return actions + _tests(context, test_dir)


def cv_node(executable, package='thornbots_pkg', **params):
    return Node(package=package, executable=executable, name=executable,
                output='screen', parameters=[{'use_sim_time': True, **params}])


def _aim_node(context, harness):
    cfg = context.launch_configurations
    return cv_node('point_to_cv_target',
                   cv_target_publish_rate_hz=harness.TEST_FIRE_HZ,
                   fire_rate_hz=harness.TEST_FIRE_HZ + 10.0,
                   gimbal_lag_s=float(cfg['gimbal_lag_s']),
                   chase_settle_s=float(cfg['chase_settle_s']),
                   chase_margin_s=float(cfg['chase_margin_s']))


def _point_stack(context, harness, headless):
    # No world, no robot: a /clock, our chassis as odom->root and /pose,
    # the phantom target and its true state, and the aim node. A perfect
    # gimbal: it holds each 40 Hz aim until the next, with no lag after.
    rate = float(context.launch_configurations['real_time_factor'])
    if rate <= 0.0:
        raise RuntimeError('real_time_factor must be > 0: sim_clock has no "unthrottled"')
    x, y, z = harness.POINT_SHOOTER
    actions = [
        Node(package='sim', executable='sim_clock', name='sim_clock', output='screen',
             parameters=[{'rate': rate}]),
        # Our chassis: a second target_driver, no spin, bouncing along y
        # through root at shooter_speed (0 holds it at POINT_SHOOTER).
        Node(package='sim', executable='target_driver', name='shooter_driver', output='screen',
             remappings=[('/target/ground_truth_odom', '/shooter/ground_truth_odom')],
             parameters=[{'use_sim_time': True, 'spin_hz': 0.0, 'center_x': x,
                          'center_y': y, 'target_z': z, 'path_angle_deg': 0.0,
                          'half_width': harness.SHOOTER_HALF_WIDTH,
                          'target_speed': float(context.launch_configurations['shooter_speed'])}]),
        cv_node('point_shooter', package='sim'),
        cv_node('target_driver', package='sim', target_speed=0.0, spin_hz=0.0),
        cv_node('target_state_truth', package='sim'),
        _aim_node(context, harness),
        cv_node('mcb_relay'),
    ]
    if not headless:
        actions.append(cv_node('target_state_markers', package='sim'))
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('sim'), 'rviz',
                                          'cv_target.rviz')],
            parameters=[{'use_sim_time': True}]))
    return actions


def _tests(context, test_dir):
    cmd = [sys.executable, '-m', 'pytest', os.path.join(test_dir, TEST_FILE),
           '-m', 'integration', '-v', '-s', '--external-stack',
           '--panel-layout', context.launch_configurations['panel_layout'],
           '--target-path', context.launch_configurations['target_path'],
           '--shooter-speed', context.launch_configurations['shooter_speed']]
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
    return [tests, done]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('run_tests', default_value='true',
                              description='false: bring up the stack only'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='skip rviz2'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0',
                              description="sim_clock's rate, sim seconds per wall second"),
        DeclareLaunchArgument('speeds', default_value='',
                              description="target speeds (m/s), e.g. '0.5 1'; "
                                          'empty = the harness default sweep'),
        DeclareLaunchArgument('duration', default_value='',
                              description='sim-time seconds scored per case'),
        DeclareLaunchArgument('hit_radius', default_value='',
                              description='miss distance (m) still counted as a hit'),
        DeclareLaunchArgument('target_path', default_value='lateral',
                              choices=['lateral', 'radial', 'diagonal'],
                              description='across the view, down the camera ray, or both'),
        DeclareLaunchArgument('shooter_speed', default_value='0.0',
                              description='our own chassis speed (m/s), bouncing along y '
                                          'through every case; 0 holds it still'),
        DeclareLaunchArgument('gimbal_lag_s', default_value='0.0',
                              description="point_to_cv_target's gimbal_lag_s; 0 is the "
                                          'perfect gimbal'),
        DeclareLaunchArgument('chase_settle_s', default_value='0.0',
                              description="point_to_cv_target's spin mode: < 0 shotgating "
                                          '(timed fire), >= 0 chase the facing panel'),
        DeclareLaunchArgument('chase_margin_s', default_value='0.0',
                              description="point_to_cv_target's chase_margin_s"),
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
    return LaunchDescription(args + [OpaqueFunction(function=_stack)])
