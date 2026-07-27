"""
Emulates the real Type-C board's POSE_MSG interface in sim: republishes
sim's raw ground-truth /sim/raw_odom + /sim/raw_joint_states (bridged
from gz, see sim.launch.py) as a dji_serial_bridge/msg/RobotPose on
/pose -- the same topic/message real hardware's Type-C board sends.
sentry_pkg's pose_translator is the only downstream consumer for both
sim and real hardware, so sim's job here is purely wire-format parity.
"""
import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from dji_serial_bridge.msg import RobotPose

from sim.auto_explore import teleport


class PoseEmulator(Node):
    def __init__(self):
        super().__init__('pose_emulator')

        self.declare_parameter('yaw_joint_name', 'headlink')
        self.yaw_joint_name = self.get_parameter('yaw_joint_name').value
        self.head_yaw = 0.0
        self.declare_parameter('pitch_joint_name', 'headpitch')
        self.pitch_joint_name = self.get_parameter('pitch_joint_name').value
        self.head_pitch = 0.0

        # Real hardware's wheel odometry accumulates drift (wheel slip,
        # encoder error -- worse on the arena's "Bumpy Road" zone) that
        # slam_toolbox's map->odom correction exists to compensate for. Sim's
        # ground truth has none of that by default, which is fine for most
        # testing but leaves slam_toolbox's correction behavior completely
        # unexercised. These params optionally inject synthetic position
        # drift/noise so that path can actually be demonstrated in sim; off
        # by default so existing ground-truth behavior is unchanged unless
        # explicitly opted into.
        self.declare_parameter('odom_noise_enabled', False)
        # Random-walk step stddev (meters/callback) added to a persistent
        # drift offset each callback -- accumulates over time like real
        # wheel-slip drift, rather than resetting every sample.
        self.declare_parameter('odom_drift_stddev', 0.0005)
        # Independent per-sample jitter stddev (meters), added on top of the
        # drift offset each callback -- simulates ordinary encoder/sensor
        # noise, does not accumulate.
        self.declare_parameter('odom_jitter_stddev', 0.001)
        # One-time "jerk" event: models a discrete EXTERNAL displacement
        # (wheel slip, collision) that moves the real robot without wheel
        # encoders registering it -- trigger_jerk() moves the sim robot and
        # cancels that delta from the drift accumulator so /pose stays
        # continuous. Manual only: `ros2 service call
        # /pose_emulator/trigger_jerk std_srvs/srv/Trigger`. Stddev below.
        # See README.md for why this works "backwards".
        self.declare_parameter('odom_jerk_stddev', 0.2)

        # Optional bias on the jerk's DIRECTION only (magnitude still governed
        # by odom_jerk_stddev) -- pulls the drawn jerk toward a fixed target
        # point (e.g. a test loop's center) instead of firing uniformly at
        # random, so test loops whose corners sit close to walls don't risk
        # a jerk teleporting the robot into one. Off by default; x/y target
        # values are meaningless while disabled. See README.md for the
        # scenario that relies on this (jerk_with_motion).
        self.declare_parameter('odom_jerk_bias_enabled', False)
        self.declare_parameter('odom_jerk_bias_x', 0.0)
        self.declare_parameter('odom_jerk_bias_y', 0.0)

        # Continuous wheel slip -- distinct from drift (smooth, unbounded
        # accumulation) and jerk (one-time impulse): models wheels that
        # spin but don't fully grip (e.g. the arena's "Bumpy Road" zone),
        # losing a fixed FRACTION of every meter actually driven. 0.5 means
        # reported /pose only advances 0.5m per 1m actually moved. 0.0
        # (default) disables this.
        self.declare_parameter('odom_slip_ratio', 0.0)

        self._drift_x = 0.0
        self._drift_y = 0.0
        # Last known ground-truth position, updated every odom_callback, so
        # the jerk-trigger service can read "where is the robot right now"
        # without needing its own subscription.
        self._true_x = 0.0
        self._true_y = 0.0
        # Slip bookkeeping: separate from _true_x/_true_y (which trigger_jerk
        # reads and which must stay exact ground truth) and from
        # _drift_x/_drift_y (an additive offset, unaffected by slip so
        # trigger_jerk's cancellation logic keeps working unmodified).
        # _slipped_x/_slipped_y integrate only a FRACTION of each true
        # position delta -- None until the first callback establishes a
        # starting point, since slip needs a previous sample to compute a
        # delta from.
        self._slipped_x = None
        self._slipped_y = None
        self._prev_true_x = None
        self._prev_true_y = None

        # One-time, permanent "dead sensor" event: models a wheel encoder
        # that stops reporting real motion entirely and just sticks at
        # (0, 0) forever -- unlike jerk/drift/slip, there's no cancellation
        # math here, since a truly dead sensor isn't recoverable. Manual
        # only: `ros2 service call /pose_emulator/trigger_odom_stuck
        # std_srvs/srv/Trigger`. See README.md.
        self._odom_stuck = False

        self.pose_pub = self.create_publisher(RobotPose, '/pose', 10)
        self.create_subscription(Odometry, '/sim/raw_odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/sim/raw_joint_states', self.joint_callback, 10)
        # Test-only trigger surface for the jerk event above: nothing in sim
        # calls this on its own (no collision/contact sensor or arena-zone
        # geometry wired up to call it from) -- it's here so a jerk can be
        # fired on demand from a shell for testing/tuning, e.g.:
        #   ros2 service call /pose_emulator/trigger_jerk std_srvs/srv/Trigger
        self.create_service(Trigger, '~/trigger_jerk', self._trigger_jerk_srv)
        self.create_service(Trigger, '~/trigger_odom_stuck', self._trigger_odom_stuck_srv)

    def joint_callback(self, msg):
        if self.yaw_joint_name in msg.name:
            self.head_yaw = msg.position[msg.name.index(self.yaw_joint_name)]
        if self.pitch_joint_name in msg.name:
            self.head_pitch = msg.position[msg.name.index(self.pitch_joint_name)]

    def trigger_jerk(self):
        """Fires a one-time position jerk immediately: draws a random
        (dx, dy), teleports the real sim robot via
        sim.auto_explore.teleport(), and subtracts the same delta from the
        drift accumulator so REPORTED /pose stays continuous -- only the
        next scan match should notice. See README.md for the full model."""
        jerk_stddev = self.get_parameter('odom_jerk_stddev').value

        if self.get_parameter('odom_jerk_bias_enabled').value:
            # Direction biased toward (odom_jerk_bias_x, odom_jerk_bias_y)
            # instead of drawn uniformly at random; magnitude distribution
            # is unchanged (still hypot of two independent gaussian draws)
            # so this only reshapes WHERE the jerk points, not how big it
            # typically is.
            magnitude = math.hypot(random.gauss(0.0, jerk_stddev),
                                    random.gauss(0.0, jerk_stddev))
            target_x = self.get_parameter('odom_jerk_bias_x').value
            target_y = self.get_parameter('odom_jerk_bias_y').value
            to_target_x = target_x - self._true_x
            to_target_y = target_y - self._true_y
            if math.hypot(to_target_x, to_target_y) < 1e-6:
                # Already at (or on top of) the bias target -- no direction
                # to point toward, fall back to a random one rather than
                # dividing by ~zero.
                angle = random.uniform(-math.pi, math.pi)
            else:
                angle = math.atan2(to_target_y, to_target_x)
            dx = magnitude * math.cos(angle)
            dy = magnitude * math.sin(angle)
        else:
            dx = random.gauss(0.0, jerk_stddev)
            dy = random.gauss(0.0, jerk_stddev)

        teleport(self._true_x + dx, self._true_y + dy)

        # Cancel the same delta out of the drift accumulator so reported
        # pose = new_true_pose + drift = (true_pose + dx) + (drift - dx)
        # = true_pose + drift, i.e. unchanged from before the jerk.
        self._drift_x -= dx
        self._drift_y -= dy
        return dx, dy

    def _trigger_jerk_srv(self, request, response):
        dx, dy = self.trigger_jerk()
        response.success = True
        # Report the actual applied (dx, dy) -- Trigger has no dedicated
        # payload field, so it's encoded into `message` rather than adding
        # a custom service type just for this. A test harness that wants
        # to assert slam_toolbox's correction actually tracks the jerk's
        # real magnitude (as opposed to just its stddev parameter, which
        # any single random draw can undershoot or overshoot considerably)
        # needs the real (dx, dy) that was applied, not just the
        # distribution it was drawn from.
        response.message = f'jerk applied: dx={dx!r} dy={dy!r}'
        return response

    def _trigger_odom_stuck_srv(self, request, response):
        self._odom_stuck = True
        response.success = True
        response.message = 'odom stuck: /pose will report (0, 0) from now on'
        return response

    def odom_callback(self, msg):
        true_x = float(msg.pose.pose.position.x)
        true_y = float(msg.pose.pose.position.y)
        self._true_x = true_x
        self._true_y = true_y

        slip_ratio = self.get_parameter('odom_slip_ratio').value
        if slip_ratio > 0.0:
            if self._prev_true_x is None:
                # First callback: nothing to compute a delta from yet, so
                # start the slipped position exactly at true position (no
                # slip applied retroactively to distance already
                # "traveled" before this node existed).
                self._slipped_x, self._slipped_y = true_x, true_y
            else:
                self._slipped_x += (true_x - self._prev_true_x) * (1.0 - slip_ratio)
                self._slipped_y += (true_y - self._prev_true_y) * (1.0 - slip_ratio)
            x, y = self._slipped_x, self._slipped_y
        else:
            x, y = true_x, true_y
        self._prev_true_x, self._prev_true_y = true_x, true_y

        if self.get_parameter('odom_noise_enabled').value:
            drift_stddev = self.get_parameter('odom_drift_stddev').value
            jitter_stddev = self.get_parameter('odom_jitter_stddev').value

            # Persistent random walk -- this is the "drift" that slam_toolbox
            # should end up correcting for in map->odom.
            self._drift_x += random.gauss(0.0, drift_stddev)
            self._drift_y += random.gauss(0.0, drift_stddev)

            # Independent per-sample jitter on top, not accumulated.
            jitter_x = random.gauss(0.0, jitter_stddev)
            jitter_y = random.gauss(0.0, jitter_stddev)

            x += jitter_x
            y += jitter_y

        # Applied unconditionally (not gated behind odom_noise_enabled): a
        # jerk's cancellation offset (see trigger_jerk()) must still reach
        # the published pose even when odom_noise_enabled is False, or the
        # jerk leaks straight into reported /pose instead of staying hidden
        # until the next scan match -- defeats the whole point of a jerk.
        # _drift_x/_drift_y are 0.0 unless trigger_jerk() has set them, so
        # this is a no-op whenever no jerk has fired.
        x += self._drift_x
        y += self._drift_y

        vel_x = float(msg.twist.twist.linear.x)
        vel_y = float(msg.twist.twist.linear.y)
        if self._odom_stuck:
            # Dead sensor: fresh timestamps keep arriving, but position and
            # velocity are pinned at zero regardless of actual motion --
            # distinct from a stalled topic, which slam_toolbox/amcl would
            # notice via TF timeout. This should NOT.
            x, y, vel_x, vel_y = 0.0, 0.0, 0.0, 0.0

        pose = RobotPose()
        pose.header.stamp = msg.header.stamp
        pose.x = x
        pose.y = y
        pose.vel_x = vel_x
        pose.vel_y = vel_y
        pose.head_pitch = float(self.head_pitch)
        pose.head_yaw = float(self.head_yaw)
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEmulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
