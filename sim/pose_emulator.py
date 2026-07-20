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


class PoseEmulator(Node):
    def __init__(self):
        super().__init__('pose_emulator')

        self.declare_parameter('yaw_joint_name', 'headlink')
        self.yaw_joint_name = self.get_parameter('yaw_joint_name').value
        self.head_yaw = 0.0

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
        # drift random-walk above: models a discrete event like wheel slip
        # on bumpy terrain or the robot hitting something, rather than
        # continuous encoder error. This is event-triggered (see
        # trigger_jerk() below) rather than a per-callback random draw --
        # NOT tied to any real collision/contact sensor or arena-zone
        # geometry (sim has neither wired up yet), so nothing in this file
        # actually calls trigger_jerk() yet. It's exposed so a test harness
        # (or later, a real event source once one exists) can fire it on
        # demand to exercise/tune slam_toolbox's reaction to a sudden jump,
        # without committing to how jerks get triggered for real.
        # Stddev (meters) of the one-time impulse added to the persistent
        # drift accumulator when a jerk fires -- meaningfully larger than a
        # single odom_drift_stddev step so it reads as a sudden jump rather
        # than blending into the smooth drift.
        self.declare_parameter('odom_jerk_stddev', 0.05)

        self._drift_x = 0.0
        self._drift_y = 0.0
        # Set by trigger_jerk(); consumed (and cleared) on the next
        # odom_callback so the impulse is applied exactly once.
        self._jerk_pending = False

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

    def trigger_jerk(self):
        """Arm a one-time sudden position jerk, applied on the next
        odom_callback. Nothing in this module calls this on its own -- it
        exists so a test harness (this node's own trigger_jerk service, a
        future real event source, etc.) can fire a jerk on demand.
        Intentionally left unwired/untriggered by default."""
        self._jerk_pending = True

    def _trigger_jerk_srv(self, request, response):
        self.trigger_jerk()
        response.success = True
        response.message = 'jerk armed for next odom_callback'
        return response

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

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

            x += self._drift_x + jitter_x
            y += self._drift_y + jitter_y

            # Sudden jerk, if one was armed via trigger_jerk() -- a one-time
            # impulse added into the persistent drift accumulator (not the
            # per-sample jitter), so it sticks going forward as a discrete
            # jump rather than snapping back like a one-frame blip would.
            if self._jerk_pending:
                jerk_stddev = self.get_parameter('odom_jerk_stddev').value
                self._drift_x += random.gauss(0.0, jerk_stddev)
                self._drift_y += random.gauss(0.0, jerk_stddev)
                self._jerk_pending = False

        pose = RobotPose()
        pose.header.stamp = msg.header.stamp
        pose.x = x
        pose.y = y
        pose.vel_x = float(msg.twist.twist.linear.x)
        pose.vel_y = float(msg.twist.twist.linear.y)
        pose.head_pitch = 0.0
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
