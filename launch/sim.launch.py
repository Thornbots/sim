"""
Launches gz sim (Ignition/Gazebo Sim) loaded with the ARCC_Field_2026 world
and spawns the sentry robot (from sentry_urdf.xacro) into it.

Usage:
    ros2 launch sim sim.launch.py
    ros2 launch sim sim.launch.py gui:=false
    ros2 launch sim sim.launch.py world:=/absolute/path/to/other.sdf
    ros2 launch sim sim.launch.py odom_noise_enabled:=true

To exercise the sudden-jerk mechanism (wheel slip on bumpy terrain / hitting
something -- a discrete jump, distinct from the smooth drift above), call
pose_emulator's trigger_jerk service once sim is up:
    ros2 service call /pose_emulator/trigger_jerk std_srvs/srv/Trigger
odom_jerk_stddev:= controls how large that one-time jump is (meters).
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
    z_arg = DeclareLaunchArgument('z', default_value='0.03')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to false to run gz sim headless (server only)'
    )

    # --- Optional synthetic wheel-odometry drift injection (pose_emulator.py).
    # Off by default -- sim's /pose stays exact ground truth unless explicitly
    # opted into, so this never changes existing behavior by accident. Turn
    # it on to exercise/demonstrate slam_toolbox's map->odom correction, which
    # otherwise has nothing real to correct against in sim.
    odom_noise_enabled_arg = DeclareLaunchArgument(
        'odom_noise_enabled', default_value='false',
        description='Enable synthetic position drift/jitter on sim/pose_emulator\'s /pose output'
    )
    odom_drift_stddev_arg = DeclareLaunchArgument(
        'odom_drift_stddev', default_value='0.0005',
        description='Stddev (m) of the per-callback random-walk step accumulated into drift'
    )
    odom_jitter_stddev_arg = DeclareLaunchArgument(
        'odom_jitter_stddev', default_value='0.001',
        description='Stddev (m) of independent, non-accumulating per-sample jitter'
    )
    # Sudden one-time position "jerk" (wheel slip / bumpy terrain / hitting
    # something), distinct from the smooth drift random-walk above. This is
    # event-triggered (call pose_emulator's ~/trigger_jerk service, see
    # module docstring) rather than fired automatically -- nothing here
    # decides when a jerk happens, that's left for a test harness (or a
    # real event source later) to decide.
    odom_jerk_stddev_arg = DeclareLaunchArgument(
        'odom_jerk_stddev', default_value='0.2',
        description='Stddev (m) of the one-time impulse applied when a jerk is triggered'
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

    # --- Bridge sim clock to ROS so use_sim_time works everywhere.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
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
    # has time to come up first. Triggered off clock_bridge's start (rather
    # than robot_state_publisher, which sim no longer runs -- sentry_pkg owns
    # robot_state_publisher/TF now, see auto.launch.py) since clock_bridge
    # always starts regardless of the gui:= setting.
    delayed_spawn_robot = RegisterEventHandler(
        OnProcessStart(
            target_action=clock_bridge,
            on_start=[TimerAction(period=2.0, actions=[spawn_robot])],
        )
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

    # --- Bridge the JointStatePublisher gazebo plugin's output into ROS 2, as
    # /sim/raw_joint_states -- ground-truth, sim-internal only. Real hardware
    # has no such topic (the Type-C board doesn't expose raw joint states),
    # so nothing outside sim should consume this directly; it only feeds
    # pose_emulator below, which repackages it into the same pose interface
    # real hardware speaks. That plugin (see sentry.urdf.xacro) only
    # publishes on the gz-transport topic
    # /world/<world>/model/<robot_name>/joint_state as ignition.msgs.Model --
    # it does NOT talk to ROS on its own, hence this bridge.
    gz_joint_state_topic = [
        '/world/ARCC_Field_2026/model/', robot_name, '/joint_state'
    ]
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        output='screen',
        arguments=[gz_joint_state_topic + ['@sensor_msgs/msg/JointState[gz.msgs.Model']],
        remappings=[(gz_joint_state_topic, '/sim/raw_joint_states')],
        parameters=[{'use_sim_time': True}],
    )

    # --- Bridge the OdometryPublisher gazebo plugin's output into ROS 2, as
    # /sim/raw_odom -- ground-truth, sim-internal only, for the same reason
    # as /sim/raw_joint_states above: real hardware has no raw /odom topic
    # either, only its Type-C pose interface. That plugin (see
    # sentry.urdf.xacro) only publishes on the gz-transport topic
    # /model/<robot_name>/odometry as ignition.msgs.Odometry, not to ROS.
    gz_odom_topic = ['/model/', robot_name, '/odometry']
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        output='screen',
        arguments=[gz_odom_topic + ['@nav_msgs/msg/Odometry[gz.msgs.Odometry']],
        remappings=[(gz_odom_topic, '/sim/raw_odom')],
        parameters=[{'use_sim_time': True}],
    )

    # --- Repackage /sim/raw_odom + /sim/raw_joint_states into the same
    # dji_serial_bridge/msg/RobotPose interface real hardware's Type-C board
    # publishes on /pose. sentry_pkg's pose_translator is the only thing
    # that consumes pose data downstream of this, for both sim and real
    # hardware, so sim's job is purely to speak the same wire format here --
    # that's the "brain" package, sim is not (see
    # sentry_pkg/launch/auto.launch.py).
    pose_emulator = Node(
        package='sim',
        executable='pose_emulator',
        name='pose_emulator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_noise_enabled': ParameterValue(
                LaunchConfiguration('odom_noise_enabled'), value_type=bool
            ),
            'odom_drift_stddev': ParameterValue(
                LaunchConfiguration('odom_drift_stddev'), value_type=float
            ),
            'odom_jitter_stddev': ParameterValue(
                LaunchConfiguration('odom_jitter_stddev'), value_type=float
            ),
            'odom_jerk_stddev': ParameterValue(
                LaunchConfiguration('odom_jerk_stddev'), value_type=float
            ),
        }],
    )

    # --- Drive the chassis in sim manually via /cmd_vel (sim/wasd_teleop.py).
    # root is a genuinely free link again (see sentry.urdf.xacro), so a
    # single VelocityControl plugin on it takes a Twist directly -- no more
    # splitting into per-joint commands the way the old prismatic-joint-chain
    # design needed.
    gz_cmd_vel_topic = ['/model/', robot_name, '/cmd_vel']
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[gz_cmd_vel_topic + ['@geometry_msgs/msg/Twist]gz.msgs.Twist']],
        remappings=[(gz_cmd_vel_topic, '/cmd_vel')],
        parameters=[{'use_sim_time': True}],
    )

    # --- Teleport the chassis in sim via sim/auto_explore.py's grid sweep.
    # root has no parent joint any more (see sentry.urdf.xacro), so gz's
    # physics now honors a direct world-pose write on it; auto_explore.py
    # calls gz's own `/world/<world>/set_pose` service directly (there's no
    # ROS-side equivalent to bridge here, it's a gz-transport-only service).

    # --- Bridge for the head pan (see sentry.urdf.xacro's
    # JointPositionController on headlink, reintroduced now that root's
    # inflated rotational inertia and auto_explore.py's before/after
    # joint resets make it safe again). Lets the gz sim GUI's "Joint
    # Position Controller" panel slider and sim/head_sweep.py drive the
    # head turn.
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
        odom_noise_enabled_arg,
        odom_drift_stddev_arg,
        odom_jitter_stddev_arg,
        odom_jerk_stddev_arg,
        gz_resource_path,
        ign_resource_path,
        gz_sim,
        gz_sim_headless,
        clock_bridge,
        scan_bridge,
        joint_state_bridge,
        odom_bridge,
        pose_emulator,
        cmd_vel_bridge,
        head_pan_bridge,
        delayed_spawn_robot,
    ])
