"""
Launches gz sim (Ignition/Gazebo Sim) loaded with the ARCC_Field_2026 world
and spawns the sentry robot (from sentry_urdf.xacro) into it.

Usage:
    ros2 launch sim sim.launch.py
    ros2 launch sim sim.launch.py gui:=false
    ros2 launch sim sim.launch.py world:=/absolute/path/to/other.sdf
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('sim')

    default_world = os.path.join(pkg_share, 'world', 'ARCC_Field_2026.sdf')
    default_xacro = os.path.join(pkg_share, 'urdf', 'sentry.urdf.xacro')

    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Full path to the .sdf world file to load'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='sentry',
        description='Name the robot is spawned with in the sim'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.05')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to false to run gz sim headless (server only)'
    )

    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')

    # --- Resource path so gz sim can resolve model://sim/... URIs used
    # in the world file (they point at the meshes shipped under share/sim/world).
    # The parent of the package share dir is what needs to be on the path, since
    # the URI itself already starts with the package name (sim/world/...).
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.dirname(pkg_share),
            pkg_share,
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )
    # Older Ignition (Fortress/Edifice) reads this variable name instead of
    # GZ_SIM_RESOURCE_PATH; set both so it works either way.
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.dirname(pkg_share),
            pkg_share,
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        ])
    )

    # --- Start gz sim (server + optional GUI) with the requested world.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': [world, ' -r'],  # -r == run immediately, not paused
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': [world, ' -r -s'],  # -s == server only, no GUI
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    # --- Robot description (xacro -> URDF) published on /robot_description.
    robot_description = ParameterValue(
        Command(['xacro ', default_xacro]), value_type=str
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # --- Spawn the robot into the running world from the /robot_description topic.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_sentry',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-allow_renaming', 'true',
        ],
    )

    # --- Bridge sim clock to ROS so use_sim_time works everywhere.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
    )

    # --- Bridge the gpu_lidar sensor's /scan topic (defined in sentry.urdf.xacro)
    # into ROS 2 so it's usable by rviz/SLAM/etc.
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        output='screen',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': True}],
    )

    # --- Bridge the JointStatePublisher gazebo plugin's output into ROS 2.
    # That plugin (see sentry.urdf.xacro) only publishes on the gz-transport
    # topic /world/<world>/model/<robot_name>/joint_state as ignition.msgs.Model
    # -- it does NOT talk to ROS on its own -- so robot_state_publisher never
    # sees /joint_states without this bridge, and TF for the head/lidar links
    # (which hang off the continuous "headlink" joint) never gets published.
    gz_joint_state_topic = [
        '/world/ARCC_Field_2026/model/', robot_name, '/joint_state'
    ]
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        output='screen',
        arguments=[gz_joint_state_topic + ['@sensor_msgs/msg/JointState[gz.msgs.Model']],
        remappings=[(gz_joint_state_topic, '/joint_states')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gui_arg,
        gz_resource_path,
        ign_resource_path,
        gz_sim,
        gz_sim_headless,
        clock_bridge,
        scan_bridge,
        joint_state_bridge,
        robot_state_publisher,
        spawn_robot,
    ])
