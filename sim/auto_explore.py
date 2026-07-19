"""
Autonomous room-sweeping explorer for sim mapping runs. Sim-only: drives
/cmd_vel reactively off /map, no equivalent on real hardware.

The robot is holonomic and cmd_vel_to_joints commands world-frame X/Y
velocity directly (not root-relative), and this node never commands yaw,
so root stays at yaw=0 throughout.

Two layers:
  1. Map-aware obstacle avoidance: clearance() raycasts through the
     occupancy grid SLAM has already built, so it only avoids walls it
     actually knows about. Simplified from an earlier version that also
     reacted to live /scan data directly -- that caught brand new
     obstacles slightly sooner, but added a second, noisier data source
     for not much benefit, since anything the lidar sees gets folded into
     /map within a scan or two anyway (10Hz scans, 0.5s min SLAM update
     interval) and a fresh approach toward genuinely unknown space is
     supposed to be probed somewhat blindly by design (that's what
     "frontier" means).
  2. Frontier bias: pure reactive exploration tends to wander the
     already-explored area near its start and never push into large
     unexplored regions on the far side of the map. Periodically scans
     /map for frontier cells (known-free cells next to unknown cells),
     picks the one farthest from the robot's current position as a target,
     and biases heading choices toward it -- still constrained by actual
     clearance, this doesn't override obstacle avoidance, it just breaks
     ties in a useful direction instead of randomly/locally.
"""
import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException

SPEED = 1.2              # m/s
STOP_DISTANCE = 0.6      # m -- start looking for a new heading below this
CANDIDATE_STEP = math.radians(30)
AVOID_REVERSE_WITHIN = math.radians(60)  # don't re-pick near-opposite of current heading
COMMIT_SECONDS = 1.5     # ignore new-heading checks for this long after switching, so
                         # it actually moves away from a corner instead of re-evaluating
                         # against a still-close wall every tick and thrashing in place
STUCK_CHECK_SECONDS = 1.5    # window to measure net displacement over
STUCK_DISTANCE = 0.2         # below this net movement in that window, we're stuck
                              # (e.g. oscillating in a narrow gap, or driving against
                              # something the map doesn't yet show as occupied) --
                              # escape by picking the single best direction out of all
                              # 360 deg, ignoring the reverse exclusion, and holding it
                              # much longer than a normal commit
FRONTIER_UPDATE_SECONDS = 3.0    # how often to rescan /map for a new frontier target
MAP_OCCUPIED_THRESH = 50    # cell value >= this counts as occupied
MAP_MAX_RAY_DIST = 1.5      # m -- how far ahead the map raycast looks
FRONTIER_STEER_WEIGHT = math.radians(45)  # candidates within this angle of the frontier
                                            # direction get a clearance bonus when picking
                                            # a new heading, so ties (and near-ties) break
                                            # toward unexplored territory instead of
                                            # randomly/toward wherever's locally most open


class AutoExplore(Node):
    def __init__(self):
        super().__init__('auto_explore')
        self.heading = 0.0
        self.committed_until = 0.0
        self.pos = None
        self.stuck_check_pos = None
        self.stuck_check_time = 0.0
        self.escaping_until = 0.0
        self.map = None
        self.frontier_target = None
        self.next_frontier_update = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)

    def update_pos(self):
        # Looked up via TF (map->root) rather than read off /odom: gz sim's
        # OdometryPublisher plugin hardcodes /odom's Y position (and
        # orientation) to 0 once the base is joint-constrained -- see
        # sentry_pkg/odom_to_tf.py's docstring -- so trusting /odom directly
        # here silently pinned this node's whole idea of "where am I" to
        # Y=0 regardless of the robot's real position, which explains a lot
        # of past "stuck"/desync behavior. map->root also matches the frame
        # clearance()/update_frontier_target() already assume when indexing
        # into /map's occupancy grid (map_to_odom drift correction included),
        # whereas raw /odom was in the odom frame -- a second, smaller
        # mismatch on top of the Y bug.
        try:
            t = self.tf_buffer.lookup_transform('map', 'root', rclpy.time.Time())
        except (LookupException, ExtrapolationException, ConnectivityException):
            return
        self.pos = (t.transform.translation.x, t.transform.translation.y)

    def map_cb(self, msg):
        self.map = msg

    def clearance(self, angle, max_dist=MAP_MAX_RAY_DIST):
        # Raycasts through the occupancy grid from the robot's current
        # position, returning distance to the first cell SLAM already
        # believes is occupied (or max_dist if clear/unknown/unmapped the
        # whole way).
        if self.map is None or self.pos is None:
            return max_dist
        msg = self.map
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        data = msg.data
        px, py = self.pos
        dist = res
        while dist <= max_dist:
            col = int((px + dist * math.cos(angle) - ox) / res)
            row = int((py + dist * math.sin(angle) - oy) / res)
            if 0 <= col < w and 0 <= row < h and data[row * w + col] >= MAP_OCCUPIED_THRESH:
                return dist
            dist += res
        return max_dist

    def update_frontier_target(self):
        msg = self.map
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        data = msg.data
        px, py = self.pos
        pcol = int((px - ox) / res)
        prow = int((py - oy) / res)

        best_dist = -1.0
        best_xy = None
        # Frontier cell: free (0 <= v < thresh) with an unknown (-1) neighbor.
        # Full-resolution scan of a ~200x250 grid is cheap enough at this
        # update rate (every few seconds), no need to subsample.
        for row in range(1, h - 1):
            base = row * w
            for col in range(1, w - 1):
                v = data[base + col]
                if v < 0 or v >= MAP_OCCUPIED_THRESH:
                    continue
                if (data[base + col - 1] < 0 or data[base + col + 1] < 0
                        or data[base - w + col] < 0 or data[base + w + col] < 0):
                    d = (col - pcol) ** 2 + (row - prow) ** 2
                    if d > best_dist:
                        best_dist = d
                        best_xy = (col, row)

        if best_xy is None:
            self.frontier_target = None
            return
        col, row = best_xy
        self.frontier_target = (ox + (col + 0.5) * res, oy + (row + 0.5) * res)
        self.get_logger().info(
            f'new frontier target: ({self.frontier_target[0]:.1f}, '
            f'{self.frontier_target[1]:.1f}), {math.sqrt(best_dist) * res:.1f}m away'
        )

    def frontier_angle(self):
        if self.frontier_target is None or self.pos is None:
            return None
        dx = self.frontier_target[0] - self.pos[0]
        dy = self.frontier_target[1] - self.pos[1]
        if math.hypot(dx, dy) < 0.3:
            return None  # basically arrived, no meaningful direction
        return math.atan2(dy, dx)

    def pick_new_heading(self):
        frontier_a = self.frontier_angle()
        candidates = []
        a = -math.pi
        while a < math.pi:
            diff = abs((a - self.heading + math.pi) % (2 * math.pi) - math.pi)
            if diff < math.pi - AVOID_REVERSE_WITHIN:
                score = self.clearance(a)
                if frontier_a is not None:
                    fdiff = abs((a - frontier_a + math.pi) % (2 * math.pi) - math.pi)
                    if fdiff < FRONTIER_STEER_WEIGHT:
                        score += (FRONTIER_STEER_WEIGHT - fdiff) * 2.0
                candidates.append((score, a))
            a += CANDIDATE_STEP
        if not candidates:
            return self.heading + math.pi  # boxed in, reverse
        candidates.sort(reverse=True)
        top = candidates[:max(1, len(candidates) // 3)]
        return random.choice(top)[1]

    def pick_escape_heading(self):
        # No reverse-exclusion, no randomness, no frontier bias: single best
        # clearance out of all 360, for shoving out of a local dead end.
        best = max(
            (self.clearance(i * CANDIDATE_STEP), i * CANDIDATE_STEP)
            for i in range(int(2 * math.pi / CANDIDATE_STEP))
        )
        return best[1]

    def tick(self):
        self.update_pos()
        if self.pos is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9

        if self.map is not None and now >= self.next_frontier_update:
            self.update_frontier_target()
            self.next_frontier_update = now + FRONTIER_UPDATE_SECONDS

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
