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

`ros2 launch sim e2e.launch.py [target_speed:=2.0] [target_spin_hz:=1.5]`.
gz with the depth camera; one ghost opponent (opponent_driver spawns it, its
OpponentMover system rides target_driver's path); detector_standin and the real roi_depth_node in
camera_container; auto.launch.py's selector, tracker and point_to_cv_target;
cv_head_aim on the head; a RefSysStatus stub putting us on blue. The
opponent is red, class 6. ../E2E_PLAN.md has the stages.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from sim.display import display_error

ROBOT_DELAY_S = 8.0  # as localization_tests.launch.py: the sim is up first
OPPONENT = 'opponent_0'
OPPONENT_CLASS = 6  # red; we are blue


def _sim(context):
    config = context.launch_configurations
    windows = config['headless'].lower() not in ('true', '1') and display_error() is None
    gui = 'true' if windows else 'false'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sim'), 'launch', 'sim.launch.py')),
        launch_arguments={'gui': gui, 'rviz': gui, 'foxglove': config['foxglove'],
                          'camera': 'true',
                          'real_time_factor': config['real_time_factor']}.items())]


def generate_launch_description():
    share = get_package_share_directory('sim')
    xacro_file = os.path.join(share, 'urdf', 'sentry_v2.urdf.xacro')

    # Same parameters as the robot's (isaac_ros_yolov8_realsense.launch.py).
    roi_depth = ComposableNode(
        package='roi_depth_query',
        plugin='roi_depth_query::RoiDepthNode',
        name='roi_depth_node',
        parameters=[{
            'use_sim_time': True,
            'depth_ns': '/depth', 'color_ns': '/color',
            'depth_scale': 0.001, 'min_depth_m': 0.1, 'max_depth_m': 10.0,
            'center_sample_fraction': 0.25, 'depth_max_age_s': 0.05, 'max_detections': 16,
            'detections_topic': '/detections_output',
            'network_width': 640, 'network_height': 640,
            'color_width': 640, 'color_height': 480,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    standin = ComposableNode(
        package='sim',
        plugin='sim::DetectorStandin',
        name='detector_standin',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'opponents': [OPPONENT],
            'class_ids': [OPPONENT_CLASS],
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    camera_nodes = LoadComposableNodes(target_container='/camera_container',
                                       composable_node_descriptions=[roi_depth, standin])
    extrinsics_relay = Node(
        package='roi_depth_query', executable='extrinsics_relay_node', name='extrinsics_relay',
        output='screen',
        parameters=[{'extrinsics_topic': '/extrinsics/depth_to_color',
                     'target_node': '/roi_depth_node'}])

    target_driver = Node(
        package='sim', executable='target_driver', name='target_driver', output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_speed': ParameterValue(LaunchConfiguration('target_speed'), value_type=float),
            'spin_hz': ParameterValue(LaunchConfiguration('target_spin_hz'), value_type=float),
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
    # The referee stand-in until E2's MCB emulator sends REF_SYS_MSG.
    team_stub = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-r', '5', '/dji_serial_bridge/ref_sys',
             'dji_serial_bridge/msg/RefSysStatus', '{is_on_blue_team: true}'],
        name='team_stub', output='log')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('thornbots_pkg'), 'launch', 'auto.launch.py')),
        launch_arguments={'real_hardware': 'false', 'localization_mode': 'amcl',
                          'use_rf2o': 'true', 'load_map': 'true'}.items())

    return LaunchDescription([
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
        OpaqueFunction(function=_sim),
        camera_nodes,
        extrinsics_relay,
        target_driver,
        path_bridge,
        opponent_driver,
        cv_head_aim,
        team_stub,
        TimerAction(period=ROBOT_DELAY_S, actions=[robot]),
    ])
