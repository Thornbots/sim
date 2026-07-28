"""
Turns sentry_pkg's /cv/target (dji_serial_bridge/msg/CVTarget) into
/head_pan_cmd + /head_pitch_cmd so sim's head joints actually track CV
detections -- the "real" reason the head moves during CV testing, as
opposed to head_sweep.py's unwired placeholder sweep. See README.md for
the FK/sign-convention/gain/control-rate rationale.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from dji_serial_bridge.msg import CVTarget

HEADPITCH_LOWER = -0.6          # sentry.urdf.xacro: headpitch lower
HEADPITCH_UPPER = 0.6           # sentry.urdf.xacro: headpitch upper


class CvHeadAim(Node):

    def __init__(self):
        super().__init__('cv_head_aim')

        self.declare_parameter('cv_target_topic', '/cv/target')
        self.declare_parameter('joint_states_topic', '/sim/raw_joint_states')
        self.declare_parameter('pan_cmd_topic', '/head_pan_cmd')
        self.declare_parameter('pitch_cmd_topic', '/head_pitch_cmd')
        self.declare_parameter('yaw_joint_name', 'headlink')
        self.declare_parameter('pitch_joint_name', 'headpitch')
        self.declare_parameter('gain', 0.1)
        self.declare_parameter('sign_yaw', 1.0)
        self.declare_parameter('sign_pitch', -1.0)
        self.declare_parameter('control_rate_hz', 15.0)

        self.yaw_joint_name = self.get_parameter('yaw_joint_name').value
        self.pitch_joint_name = self.get_parameter('pitch_joint_name').value
        self.gain = self.get_parameter('gain').value
        self.sign_yaw = self.get_parameter('sign_yaw').value
        self.sign_pitch = self.get_parameter('sign_pitch').value
        control_rate_hz = self.get_parameter('control_rate_hz').value

        self._head_yaw = 0.0
        self._head_pitch = 0.0
        self._have_joint_states = False
        self._latest_target = None  # most recent confidence>0 CVTarget, or None

        self.pan_pub = self.create_publisher(
            Float64, self.get_parameter('pan_cmd_topic').value, 10)
        self.pitch_pub = self.create_publisher(
            Float64, self.get_parameter('pitch_cmd_topic').value, 10)

        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self.on_joint_states, 10)
        self.create_subscription(
            CVTarget, self.get_parameter('cv_target_topic').value,
            self.on_cv_target, qos_profile_sensor_data)

        # Corrections are computed on this timer, not directly off each
        # /cv/target arrival (which can be much faster -- e.g.
        # cv_target_emulator's 60Hz default -- than the JointPositionController
        # can physically catch up). Reacting to every message stacked a fresh
        # gain*delta correction onto a self._head_yaw feedback value that
        # hadn't yet caught up to the previous command, so the commanded
        # setpoint raced ahead of the physical joint without bound (observed
        # empirically: continuous multi-turn spin, not a converging
        # oscillation). Decoupling the control rate from the detection rate
        # gives the physical joint time to close each correction before the
        # next one is computed off fresh feedback.
        self.control_timer = self.create_timer(
            1.0 / control_rate_hz, self.on_control_tick)

        self.get_logger().info(
            f"cv_head_aim ready: {self.get_parameter('cv_target_topic').value}"
            f" -> {self.get_parameter('pan_cmd_topic').value} /"
            f" {self.get_parameter('pitch_cmd_topic').value}"
            f" (gain={self.gain}, control_rate_hz={control_rate_hz})"
        )

    def on_joint_states(self, msg):
        if self.yaw_joint_name in msg.name:
            self._head_yaw = msg.position[msg.name.index(self.yaw_joint_name)]
        if self.pitch_joint_name in msg.name:
            self._head_pitch = msg.position[msg.name.index(self.pitch_joint_name)]
        self._have_joint_states = True

    def on_cv_target(self, msg):
        self._latest_target = msg if msg.confidence > 0.0 else None

    def on_control_tick(self):
        msg = self._latest_target
        if not self._have_joint_states or msg is None:
            return

        delta_yaw = math.atan2(msg.x, msg.z)
        delta_pitch = math.atan2(msg.y, math.hypot(msg.x, msg.z))

        # headlink is a continuous joint (no position limit -- see
        # sentry.urdf.xacro) so new_yaw is left unclamped and free to
        # accumulate past +-pi; headpitch has a real +-0.6 limit.
        new_yaw = self._head_yaw + self.gain * self.sign_yaw * delta_yaw
        new_pitch = self._head_pitch + self.gain * self.sign_pitch * delta_pitch

        new_pitch = max(HEADPITCH_LOWER, min(HEADPITCH_UPPER, new_pitch))

        self.pan_pub.publish(Float64(data=new_yaw))
        self.pitch_pub.publish(Float64(data=new_pitch))


def main(args=None):
    rclpy.init(args=args)
    node = CvHeadAim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
