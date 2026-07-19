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
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
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

    # --- Spawn the robot into the running world.
    # NOTE: deliberately using -string (raw URDF text) here, NOT -topic
    # robot_description. -topic makes `create` subscribe to robot_description
    # over ROS, and that subscription reliably fails to receive the
    # TRANSIENT_LOCAL-cached message from robot_state_publisher -- confirmed
    # by watching it live: `ros2 topic echo /robot_description` instantly got
    # the same message via the same QoS while an already-matched spawn_sentry
    # sat waiting 30+ seconds. That's a bug in ros_gz_sim create's own ROS
    # subscription handling, not a startup-ordering race, so no amount of
    # delay fixes it. -string sidesteps ROS entirely for this one hand-off:
    # xacro's output is substituted directly into the process arguments.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_sentry',
        output='screen',
        arguments=[
            '-string', Command(['xacro ', default_xacro]),
            '-name', robot_name,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-allow_renaming', 'true',
        ],
    )
    # Keep a short delay before spawning so gz sim's entity-creation service
    # has time to come up first (unrelated to the robot_description bug above).
    delayed_spawn_robot = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[TimerAction(period=2.0, actions=[spawn_robot])],
        )
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

    # --- Bridge the OdometryPublisher gazebo plugin's output into ROS 2.
    # Same story as joint_state above: that plugin (see sentry.urdf.xacro)
    # only publishes on the gz-transport topic /model/<robot_name>/odometry
    # as ignition.msgs.Odometry, not to ROS. This is deliberately just the
    # raw bridge -- sim's job is to provide raw topics matching what real
    # hardware would produce, nothing more. Turning /odom into the
    # odom->root TF is sentry_pkg's job, see sentry_pkg/launch/auto.launch.py
    # -- that's the "brain" package, sim is not.
    gz_odom_topic = ['/model/', robot_name, '/odometry']
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        output='screen',
        arguments=[gz_odom_topic + ['@nav_msgs/msg/Odometry[gz.msgs.Odometry']],
        remappings=[(gz_odom_topic, '/odom')],
        parameters=[{'use_sim_time': True}],
    )

    # --- Bridge /cmd_vel (ROS) into the VelocityControl gz plugin (see
    # sentry.urdf.xacro) so the chassis can actually be driven in sim, e.g.
    # to sweep the room for mapping. root is joint-constrained (X/Y/yaw
    # only, see sentry.urdf.xacro), so driving it needs one JointController
    # per joint rather than a single Twist bridge -- cmd_vel_to_joints
    # splits /cmd_vel into the 3 topics these bridges forward into gz.
    # ROS->GZ direction only (']' not '[') for all three.
    cmd_vel_to_joints = Node(
        package='sim',
        executable='cmd_vel_to_joints',
        name='cmd_vel_to_joints',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    gz_planar_x_topic = ['/model/', robot_name, '/joint/world_to_planar_x/cmd_vel']
    planar_x_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='planar_x_vel_bridge',
        output='screen',
        arguments=[gz_planar_x_topic + ['@std_msgs/msg/Float64]gz.msgs.Double']],
        remappings=[(gz_planar_x_topic, '/planar_x_vel_cmd')],
        parameters=[{'use_sim_time': True}],
    )
    gz_planar_y_topic = ['/model/', robot_name, '/joint/planar_x_to_y/cmd_vel']
    planar_y_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='planar_y_vel_bridge',
        output='screen',
        arguments=[gz_planar_y_topic + ['@std_msgs/msg/Float64]gz.msgs.Double']],
        remappings=[(gz_planar_y_topic, '/planar_y_vel_cmd')],
        parameters=[{'use_sim_time': True}],
    )
    gz_yaw_topic = ['/model/', robot_name, '/joint/planar_y_to_root_yaw/cmd_vel']
    yaw_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='yaw_vel_bridge',
        output='screen',
        arguments=[gz_yaw_topic + ['@std_msgs/msg/Float64]gz.msgs.Double']],
        remappings=[(gz_yaw_topic, '/yaw_vel_cmd')],
        parameters=[{'use_sim_time': True}],
    )

    # --- Bridge for the head pan (see sentry.urdf.xacro's
    # JointPositionController on headlink). The head partially blocks the
    # lidar's own field of view at whatever bearing it currently sits at,
    # so the idea was to slowly sweep it (sim/head_sweep.py) to move that
    # blind wedge around and let SLAM fill it in over time.
    # NOT auto-started (head_sweep is deliberately absent from the
    # LaunchDescription below): tried it, and continuous head rotation
    # measurably corrupted the map -- lidar points get integrated at
    # whatever head angle robot_state_publisher's TF says *at the scan's
    # timestamp*, and with the head moving fast enough, any slack in TF
    # timestamp/interpolation smears wall traces across a spread of wrong
    # angles instead of the intended "different fixed angle per pass".
    # Bridge is left wired up so `ros2 run sim head_sweep` (or a much
    # slower version of it) can still be tried manually later.
    gz_headlink_topic = ['/model/', robot_name, '/joint/headlink/cmd_pos']
    head_pan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='head_pan_bridge',
        output='screen',
        arguments=[gz_headlink_topic + ['@std_msgs/msg/Float64]gz.msgs.Double']],
        remappings=[(gz_headlink_topic, '/head_pan_cmd')],
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
        odom_bridge,
        cmd_vel_to_joints,
        planar_x_vel_bridge,
        planar_y_vel_bridge,
        yaw_vel_bridge,
        head_pan_bridge,
        robot_state_publisher,
        delayed_spawn_robot,
    ])
