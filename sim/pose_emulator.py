"""
Emulates the real Type-C board's POSE_MSG interface in sim: republishes
sim's raw ground-truth /sim/raw_odom + /sim/raw_joint_states (bridged
straight from gz, see sim.launch.py) as a dji_serial_bridge/msg/RobotPose
on /pose -- the same topic/message real hardware's Type-C board sends
(via ros2_dji_serial_bridge's dji_serial_bridge_node, also on /pose).
sentry_pkg's pose_translator is the only thing that ever consumes pose
data downstream of this, for both sim and real hardware, so sim's job
here is purely to speak the same wire format, not to do anything
SLAM-specific itself.
"""
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
        # Occasional sudden position "jerk" -- distinct from the smooth
        # drift random-walk above: models a discrete EXTERNAL event, like
        # wheel slip on a bump or hitting something, that actually displaces
        # the robot's real position without the wheel encoders having
        # driven (and therefore registered) that displacement themselves.
        # That means a jerk has to do the opposite of what it might look
        # like at first: it MOVES THE REAL SIMULATED ROBOT in gz by a random
        # (dx, dy), and simultaneously cancels that same (dx, dy) out of the
        # persistent drift accumulator so the REPORTED /pose does not jump
        # at all at the moment of the trigger -- wheel odometry has no way
        # to know the real displacement happened, so it should keep
        # reporting exactly what it would have reported anyway. The
        # resulting discrepancy between reported (wheel) odometry and the
        # robot's new true position only becomes visible later, when
        # slam_toolbox's next scan match against the map disagrees with
        # wheel odometry and corrects map->odom -- that correction is the
        # actual thing this is meant to exercise. This is event-triggered
        # (see trigger_jerk()/the ~/trigger_jerk service below) rather than
        # a per-callback random draw or tied to any real collision/contact
        # sensor or arena-zone geometry (sim has neither wired up), so
        # nothing in this file calls trigger_jerk() automatically -- it's a
        # manually-fired test/tuning surface, e.g.:
        #   ros2 service call /pose_emulator/trigger_jerk std_srvs/srv/Trigger
        # Stddev (meters) of the one-time (dx, dy) jerk impulse -- meaningfully
        # larger than a single odom_drift_stddev step so the resulting SLAM
        # correction reads as a sudden jump rather than blending into the
        # smooth drift.
        self.declare_parameter('odom_jerk_stddev', 0.2)

        # Continuous wheel slip -- distinct from both the drift random-walk
        # (smooth, unbounded accumulation) and jerk (one-time impulse):
        # this models wheels that spin but don't fully grip (e.g. the
        # arena's "Bumpy Road" zone, see ARCC_2026_SENTRY_CONTEXT.md),
        # losing a fixed FRACTION of every meter actually driven rather
        # than accumulating a fixed amount over time regardless of motion.
        # 0.5 means reported /pose only advances 0.5m for every 1m the
        # robot actually moves -- wheel odometry systematically
        # under-reports distance traveled, growing in proportion to how
        # far the robot has actually gone, not how long it's been running.
        # 0.0 (default) disables this -- reported motion exactly tracks
        # true motion, same as before this parameter existed.
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

        self.pose_pub = self.create_publisher(RobotPose, '/pose', 10)
        self.create_subscription(Odometry, '/sim/raw_odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/sim/raw_joint_states', self.joint_callback, 10)
        # Test-only trigger surface for the jerk event above: nothing in sim
        # calls this on its own (no collision/contact sensor or arena-zone
        # geometry wired up to call it from) -- it's here so a jerk can be
        # fired on demand from a shell for testing/tuning, e.g.:
        #   ros2 service call /pose_emulator/trigger_jerk std_srvs/srv/Trigger
        self.create_service(Trigger, '~/trigger_jerk', self._trigger_jerk_srv)

    def joint_callback(self, msg):
        if self.yaw_joint_name in msg.name:
            self.head_yaw = msg.position[msg.name.index(self.yaw_joint_name)]
        if self.pitch_joint_name in msg.name:
            self.head_pitch = msg.position[msg.name.index(self.pitch_joint_name)]

    def trigger_jerk(self):
        """Fire a one-time sudden position jerk, applied immediately (not
        deferred to the next odom_callback): draws a random (dx, dy),
        physically teleports the real simulated robot in gz by that delta
        via sim.auto_explore.teleport() (a blocking gz-service call, ~up to
        a few seconds worst case -- acceptable for this manually-triggered
        test service), and simultaneously subtracts (dx, dy) from the same
        persistent drift accumulator odom_callback already uses for the
        continuous random-walk drift. That cancellation is what keeps the
        REPORTED /pose continuous across the trigger: wheel odometry
        couldn't have known about the real displacement, so it shouldn't
        visibly react to it -- only slam_toolbox's next scan match should
        notice the robot isn't where wheel odometry claims."""
        jerk_stddev = self.get_parameter('odom_jerk_stddev').value
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

        pose = RobotPose()
        pose.header.stamp = msg.header.stamp
        pose.x = x
        pose.y = y
        pose.vel_x = float(msg.twist.twist.linear.x)
        pose.vel_y = float(msg.twist.twist.linear.y)
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
