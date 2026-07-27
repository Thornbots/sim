"""
Launches gz sim loaded with the ARCC_Field_2026 world and spawns the
sentry robot (from sentry_urdf.xacro) into it.

Usage: `ros2 launch sim sim.launch.py [gui:=false] [rviz:=false]
[world:=/abs/path.sdf] [odom_noise_enabled:=true]`. To fire a one-time
odom "jerk" (odom_jerk_stddev:= sets its size in meters), once sim is up:
`ros2 service call /pose_emulator/trigger_jerk std_srvs/srv/Trigger`.
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
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')

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
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Set to false to skip launching rviz2'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config,
        description='Full path to the rviz2 config file to load'
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
    # Optional direction bias for the jerk above (see pose_emulator.py's
    # declare_parameter comment) -- pulls the jerk toward
    # (odom_jerk_bias_x, odom_jerk_bias_y) instead of firing in a
    # uniformly random direction, for test loops whose corners sit close
    # to real walls. Off by default: existing random-direction jerk
    # behavior is unchanged unless explicitly enabled.
    odom_jerk_bias_enabled_arg = DeclareLaunchArgument(
        'odom_jerk_bias_enabled', default_value='false',
        description='Bias the jerk\'s direction toward (odom_jerk_bias_x, '
                     'odom_jerk_bias_y) instead of a uniformly random direction'
    )
    odom_jerk_bias_x_arg = DeclareLaunchArgument(
        'odom_jerk_bias_x', default_value='0.0',
        description='X target (m) the jerk direction is biased toward, when enabled'
    )
    odom_jerk_bias_y_arg = DeclareLaunchArgument(
        'odom_jerk_bias_y', default_value='0.0',
        description='Y target (m) the jerk direction is biased toward, when enabled'
    )
    # Continuous wheel slip -- see pose_emulator.py's declare_parameter
    # comment for the full model (a FRACTION of every meter driven lost
    # from reported odometry, not a fixed amount lost per unit time like
    # drift above). 0.0 default: reported motion exactly tracks true
    # motion, unchanged from before this arg existed.
    odom_slip_ratio_arg = DeclareLaunchArgument(
        'odom_slip_ratio', default_value='0.0',
        description='Fraction (0-1) of true distance traveled lost from '
                     'reported odometry, e.g. 0.5 = wheels report only '
                     'half the distance actually driven'
    )

    # --- Optional fast-moving-target CV simulation (target_driver.py +
    # cv_target_emulator.py). Off by default -- no new nodes/topics run
    # unless spawn_target:=true is passed, and neither new node depends on
    # sentry_pkg's SLAM/AMCL/EKF stack (auto.launch.py); both only need
    # /sim/raw_odom + /sim/raw_joint_states, produced inside sim itself.
    # See README.md's ## Notes for the noise model / FK / dwell-count
    # rationale.
    spawn_target_arg = DeclareLaunchArgument(
        'spawn_target', default_value='false',
        description='Launch target_driver + cv_target_emulator for CV detection testing'
    )
    target_speed_arg = DeclareLaunchArgument(
        'target_speed', default_value='2.0',
        description='Target chassis lateral speed (m/s) for target_driver\'s traverse path'
    )
    target_spin_hz_arg = DeclareLaunchArgument(
        'target_spin_hz', default_value='1.5',
        description='Target chassis spin rate (Hz) -- the "wiggle" defense per ARCC_2026_SENTRY_CONTEXT.md (typically 1-2 Hz)'
    )
    cv_noise_pos_stddev_arg = DeclareLaunchArgument(
        'cv_noise_pos_stddev', default_value='0.03',
        description='Stddev (m) of Gaussian position noise injected into cv_target_emulator\'s roi_point'
    )
    cv_dropout_probability_arg = DeclareLaunchArgument(
        'cv_dropout_probability', default_value='0.0',
        description='Per-sample probability (0-1) cv_target_emulator drops an otherwise-valid detection'
    )
    cv_publish_latency_s_arg = DeclareLaunchArgument(
        'cv_publish_latency_s', default_value='0.0',
        description='Fixed publish latency (s) cv_target_emulator adds before publishing a detection'
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
    # NOTE: deliberately -string (raw URDF text), NOT -topic
    # robot_description -- ros_gz_sim create's -topic subscription
    # reliably fails to receive robot_state_publisher's TRANSIENT_LOCAL
    # message (confirmed bug, not a startup race). See README.md.
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
    # into ROS 2, remapped to scan_raw -- sentry_pkg's lidar_self_filter node
    # is the only thing that publishes the final /scan (see its docstring),
    # for both sim and real hardware.
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        output='screen',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/scan', '/scan_raw')],
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
            'odom_jerk_bias_enabled': ParameterValue(
                LaunchConfiguration('odom_jerk_bias_enabled'), value_type=bool
            ),
            'odom_jerk_bias_x': ParameterValue(
                LaunchConfiguration('odom_jerk_bias_x'), value_type=float
            ),
            'odom_jerk_bias_y': ParameterValue(
                LaunchConfiguration('odom_jerk_bias_y'), value_type=float
            ),
            'odom_slip_ratio': ParameterValue(
                LaunchConfiguration('odom_slip_ratio'), value_type=float
            ),
        }],
    )

    # --- Bridges gz sim GUI's slider panel (fixed gz-transport topic
    # naming, not GUI-configurable) into headlink/headpitch's actual
    # command topics -- see sim/head_slider_relay.py and README.md.
    # Not a ROS node (no rclpy), so no use_sim_time param.
    head_slider_relay = Node(
        package='sim',
        executable='head_slider_relay',
        name='head_slider_relay',
        output='screen',
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

    # --- Bridge for the head-mounted camera's pitch (see sentry.urdf.xacro's
    # JointPositionController on headpitch), same pattern as head_pan_bridge
    # above.
    gz_headpitch_topic = ['/model/', robot_name, '/joint/headpitch/cmd_pos']
    head_pitch_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='head_pitch_bridge',
        output='screen',
        arguments=[gz_headpitch_topic + ['@std_msgs/msg/Float64]gz.msgs.Double']],
        remappings=[(gz_headpitch_topic, '/head_pitch_cmd')],
        parameters=[{'use_sim_time': True}],
    )

    # --- Bridge the rgbd_camera sensor (defined in sentry.urdf.xacro,
    # <topic>camera</topic>) into ROS 2, remapped to the same topic names
    # realsense-ros uses on real hardware (see
    # realsense-yolov8-nitros-bridge/launch/isaac_ros_yolov8_realsense.launch.py)
    # so CV nodes written against the physical camera run unmodified in sim.
    camera_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_image_bridge',
        output='screen',
        arguments=['/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
        remappings=[('/camera/image', '/color/image_raw')],
        parameters=[{'use_sim_time': True}],
    )
    camera_depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_depth_bridge',
        output='screen',
        arguments=['/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'],
        remappings=[('/camera/depth_image', '/depth/image_rect_raw')],
        parameters=[{'use_sim_time': True}],
    )
    # rgbd_camera only publishes one set of intrinsics (for the color lens);
    # reused for both color and depth since this is a single fixed-baseline
    # rig, same simplification real D435 firmware makes when depth is
    # aligned to color.
    camera_color_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_color_info_bridge',
        output='screen',
        arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        remappings=[('/camera/camera_info', '/color/camera_info')],
        parameters=[{'use_sim_time': True}],
    )
    camera_depth_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_depth_info_bridge',
        output='screen',
        arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        remappings=[('/camera/camera_info', '/depth/camera_info')],
        parameters=[{'use_sim_time': True}],
    )

    # --- Fast-moving-target CV simulation, gated behind spawn_target
    # (default false, see arg declarations above). Pure-ROS nodes -- no gz
    # entity/bridge involved, target_driver publishes ground truth directly
    # and cv_target_emulator computes the noisy detection from it, so both
    # can run standalone against bare sim.launch.py.
    target_driver = Node(
        package='sim',
        executable='target_driver',
        name='target_driver',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_speed': ParameterValue(
                LaunchConfiguration('target_speed'), value_type=float
            ),
            'spin_hz': ParameterValue(
                LaunchConfiguration('target_spin_hz'), value_type=float
            ),
        }],
        condition=IfCondition(LaunchConfiguration('spawn_target')),
    )
    cv_target_emulator = Node(
        package='sim',
        executable='cv_target_emulator',
        name='cv_target_emulator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'noise_pos_stddev': ParameterValue(
                LaunchConfiguration('cv_noise_pos_stddev'), value_type=float
            ),
            'dropout_probability': ParameterValue(
                LaunchConfiguration('cv_dropout_probability'), value_type=float
            ),
            'publish_latency_s': ParameterValue(
                LaunchConfiguration('cv_publish_latency_s'), value_type=float
            ),
        }],
        condition=IfCondition(LaunchConfiguration('spawn_target')),
    )

    # --- rviz2, using sentry_pkg's config (same one sentry_pkg's own launch
    # files use) so sim and real-hardware runs look the same. use_sim_time
    # matches every other node above since sim's /clock is what's bridged in.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gui_arg,
        rviz_arg,
        rviz_config_arg,
        odom_noise_enabled_arg,
        odom_drift_stddev_arg,
        odom_jitter_stddev_arg,
        odom_jerk_stddev_arg,
        odom_jerk_bias_enabled_arg,
        odom_jerk_bias_x_arg,
        odom_jerk_bias_y_arg,
        odom_slip_ratio_arg,
        spawn_target_arg,
        target_speed_arg,
        target_spin_hz_arg,
        cv_noise_pos_stddev_arg,
        cv_dropout_probability_arg,
        cv_publish_latency_s_arg,
        gz_resource_path,
        ign_resource_path,
        gz_sim,
        gz_sim_headless,
        clock_bridge,
        scan_bridge,
        joint_state_bridge,
        odom_bridge,
        pose_emulator,
        head_slider_relay,
        target_driver,
        cv_target_emulator,
        cmd_vel_bridge,
        head_pan_bridge,
        head_pitch_bridge,
        camera_image_bridge,
        camera_depth_bridge,
        camera_color_info_bridge,
        camera_depth_info_bridge,
        delayed_spawn_robot,
        rviz,
    ])
