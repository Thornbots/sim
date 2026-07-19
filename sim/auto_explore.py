"""
Autonomous room-sweeping explorer for sim mapping runs. Sim-only: drives
/cmd_vel reactively off /scan, no equivalent on real hardware.

The robot is holonomic and cmd_vel_to_joints commands world-frame X/Y
velocity directly (not root-relative), and this node never commands yaw,
so root stays at yaw=0 throughout -- meaning the lidar's own scan angles
ARE world-frame angles the whole time, with no frame transform needed.

Simple reactive strategy: keep driving in the current heading while the
lidar shows clearance ahead; when a wall gets close, scan all directions
for the most open one (excluding ones too close to straight back, so it
doesn't just ping-pong corner to corner) and switch to that heading. Fast
because each leg runs until it actually reaches a wall, not a fixed
timed/scripted distance.
"""
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

SPEED = 1.2              # m/s
STOP_DISTANCE = 0.6      # m -- start looking for a new heading below this
CONE_HALF_WIDTH = math.radians(20)
CANDIDATE_STEP = math.radians(30)
AVOID_REVERSE_WITHIN = math.radians(60)  # don't re-pick near-opposite of current heading
COMMIT_SECONDS = 1.5     # ignore new-heading checks for this long after switching, so
                         # it actually moves away from a corner instead of re-evaluating
                         # against a still-close wall every tick and thrashing in place
STUCK_CHECK_SECONDS = 4.0    # window to measure net displacement over
STUCK_DISTANCE = 0.4         # below this net movement in that window, we're stuck
                              # (e.g. oscillating in a narrow gap between two local
                              # walls) -- escape by picking the single best direction
                              # out of all 360 deg, ignoring the reverse exclusion,
                              # and holding it much longer than a normal commit


class AutoExplore(Node):
    def __init__(self):
        super().__init__('auto_explore')
        self.heading = 0.0
        self.have_scan = False
        self.scan = None
        self.committed_until = 0.0
        self.pos = None
        self.stuck_check_pos = None
        self.stuck_check_time = 0.0
        self.escaping_until = 0.0
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)

    def scan_cb(self, msg):
        self.scan = msg
        self.have_scan = True

    def odom_cb(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def range_at(self, angle):
        msg = self.scan
        n = len(msg.ranges)
        idx = int(((angle - msg.angle_min) % (2 * math.pi)) / msg.angle_increment) % n
        r = msg.ranges[idx]
        if math.isnan(r):
            return msg.range_max
        if math.isinf(r):
            # -inf means "too close to measure" (below range_min, i.e. an
            # obstacle right up against the sensor), NOT "no obstacle" --
            # only +inf means a genuine no-return/clear reading. Treating
            # -inf as clear (the bug here originally) makes the robot think
            # a wall it's already touching is wide open space forever.
            return msg.range_min if r < 0 else msg.range_max
        return r

    def clearance(self, angle, half_width=CONE_HALF_WIDTH, samples=7):
        vals = [
            self.range_at(angle + off)
            for off in [half_width * (i / (samples - 1) * 2 - 1) for i in range(samples)]
        ]
        return min(vals)

    def pick_new_heading(self):
        candidates = []
        a = -math.pi
        while a < math.pi:
            diff = abs((a - self.heading + math.pi) % (2 * math.pi) - math.pi)
            if diff < math.pi - AVOID_REVERSE_WITHIN:
                candidates.append((self.clearance(a), a))
            a += CANDIDATE_STEP
        if not candidates:
            return self.heading + math.pi  # boxed in, reverse
        candidates.sort(reverse=True)
        top = candidates[:max(1, len(candidates) // 3)]
        return random.choice(top)[1]

    def pick_escape_heading(self):
        # No reverse-exclusion, no randomness: single best direction out of
        # all 360, for shoving out of a local dead end/narrow gap.
        best = max(
            (
                (self.clearance(a, half_width=math.radians(10), samples=5), a)
                for a in [i * CANDIDATE_STEP for i in range(int(2 * math.pi / CANDIDATE_STEP))]
            ),
        )
        return best[1]

    def tick(self):
        if not self.have_scan or self.pos is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9

        if self.stuck_check_pos is None:
            self.stuck_check_pos = self.pos
            self.stuck_check_time = now
        elif now - self.stuck_check_time >= STUCK_CHECK_SECONDS:
            moved = math.hypot(
                self.pos[0] - self.stuck_check_pos[0], self.pos[1] - self.stuck_check_pos[1]
            )
            if moved < STUCK_DISTANCE and now >= self.escaping_until:
                self.heading = self.pick_escape_heading()
                self.committed_until = now + COMMIT_SECONDS * 3
                self.escaping_until = now + COMMIT_SECONDS * 3
                self.get_logger().info(
                    f'stuck (moved {moved:.2f}m in {STUCK_CHECK_SECONDS:.0f}s), '
                    f'escaping toward {math.degrees(self.heading):.0f} deg'
                )
            self.stuck_check_pos = self.pos
            self.stuck_check_time = now

        if now >= self.committed_until and self.clearance(self.heading) < STOP_DISTANCE:
            self.heading = self.pick_new_heading()
            self.committed_until = now + COMMIT_SECONDS
            self.get_logger().info(f'wall ahead, new heading: {math.degrees(self.heading):.0f} deg')
        twist = Twist()
        twist.linear.x = SPEED * math.cos(self.heading)
        twist.linear.y = SPEED * math.sin(self.heading)
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
