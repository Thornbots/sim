# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Spawn N box actors and walk each back and forth along a segment by gz set_pose.

Params: count, speeds (m/s), paths (x0,y0,x1,y1 per actor, world frame),
rate_hz (sim time, run with use_sim_time:=true), size, height, mass,
robot_radius, margin, lookahead_s. Reads /sim/raw_odom for the robot. A
position is only written if it clears the robot's keep-out capsule (current
pose swept ahead by its velocity); otherwise the actor hops forward along its
path to the next clear spot. The caller removes the actors (`<prefix>_<i>`).
see README.md for design rationale
"""
from concurrent.futures import ThreadPoolExecutor
import math
import subprocess

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sim.auto_explore import set_model_pose

# Default layout: three segments crossing the drift suite's 3m loop
# (corners +-1.5), clear of the ARCC26 map's walls by >= 0.4m.
DEFAULT_PATHS = [0.8, 0.2, 0.8, -2.8,    # south edge
                 -0.3, 0.3, -2.4, 0.3,   # west edge
                 0.2, -0.4, 0.2, 2.1]    # north edge
DEFAULT_SPEEDS = [1.0, 2.0, 0.5]
HOP_STEP = 0.05  # m, search step when hopping past the robot


def box_sdf(name, x, y, size, height, mass):
    """Build a dynamic (not static) box, so set_pose moves it; it collides, so lidar sees it."""
    i_xy = mass * (size * size + height * height) / 12.0
    i_z = mass * size * size / 6.0
    geom = f'<geometry><box><size>{size} {size} {height}</size></box></geometry>'
    return (
        f'<sdf version="1.6"><model name="{name}">'
        f'<pose>{x} {y} {height / 2.0} 0 0 0</pose><link name="link">'
        f'<inertial><mass>{mass}</mass><inertia><ixx>{i_xy}</ixx><iyy>{i_xy}</iyy>'
        f'<izz>{i_z}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
        f'<collision name="collision">{geom}</collision>'
        f'<visual name="visual">{geom}<material><ambient>0.8 0.4 0.1 1</ambient>'
        f'<diffuse>0.8 0.4 0.1 1</diffuse></material></visual>'
        f'</link></model></sdf>')


def dist_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    len2 = dx * dx + dy * dy
    t = 0.0 if len2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class Actor:
    """One box on a ping-pong segment; u in [0, 2*length) is its progress."""

    def __init__(self, name, x0, y0, x1, y1, speed):
        self.name = name
        self.a = (x0, y0)
        self.b = (x1, y1)
        self.length = math.hypot(x1 - x0, y1 - y0)
        self.speed = speed
        self.u = 0.0
        self.spawned = False

    def position(self, u):
        u %= 2.0 * self.length
        d = u if u < self.length else 2.0 * self.length - u
        f = d / self.length if self.length > 0 else 0.0
        return (self.a[0] + (self.b[0] - self.a[0]) * f,
                self.a[1] + (self.b[1] - self.a[1]) * f)


class ActorDriver(Node):

    def __init__(self):
        super().__init__('actor_driver')
        p = self.declare_parameter
        count = p('count', 3).value
        speeds = list(p('speeds', DEFAULT_SPEEDS).value)
        paths = list(p('paths', DEFAULT_PATHS).value)
        self.rate_hz = p('rate_hz', 10.0).value
        self.size = p('size', 0.3).value
        self.height = p('height', 0.8).value
        self.mass = p('mass', 20.0).value
        self.robot_radius = p('robot_radius', 0.5).value  # sentry_v2 incl. barrel
        self.margin = p('margin', 0.3).value
        self.lookahead_s = p('lookahead_s', 0.5).value
        prefix = p('name_prefix', 'moving_actor').value
        if len(paths) < 4 * count or len(speeds) < count:
            raise ValueError(f'count={count} needs {4 * count} path values '
                             f'and {count} speeds; got {len(paths)}, {len(speeds)}')
        self.actors = [Actor(f'{prefix}_{i}', *paths[4 * i:4 * i + 4], speeds[i])
                       for i in range(count)]
        self.keep_out = self.robot_radius + self.size / math.sqrt(2.0) + self.margin
        self.pool = ThreadPoolExecutor(max_workers=max(count, 1))
        self.robot = None  # (x, y, vx, vy, stamp_s)
        self.spawned = False
        self.last_tick = None
        self.ticks = 0
        self.tick_dt_sum = 0.0
        self.create_subscription(Odometry, '/sim/raw_odom', self._on_odom, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear  # child (root) frame
        vx = v.x * math.cos(yaw) - v.y * math.sin(yaw)
        vy = v.x * math.sin(yaw) + v.y * math.cos(yaw)
        p = msg.pose.pose.position
        self.robot = (p.x, p.y, vx, vy, self._now())

    def _clear(self, xy, lookahead):
        rx, ry, vx, vy, _ = self.robot
        return dist_to_segment(xy[0], xy[1], rx, ry,
                               rx + vx * lookahead, ry + vy * lookahead) >= self.keep_out

    def _first_clear(self, actor, u, lookahead):
        """Return the first progress >= u whose position is clear, or None."""
        for k in range(int(2.0 * actor.length / HOP_STEP) + 1):
            if self._clear(actor.position(u + k * HOP_STEP), lookahead):
                return u + k * HOP_STEP
        return None

    def _spawn(self):
        """Spawn every actor not yet spawned, each at a clear spot; False if one had none."""
        for actor in (a for a in self.actors if not a.spawned):
            u = self._first_clear(actor, 0.0, self.lookahead_s)
            if u is None:
                return False
            actor.u = u
            x, y = actor.position(u)
            sdf = box_sdf(actor.name, x, y, self.size, self.height, self.mass)
            result = subprocess.run(
                ['ros2', 'run', 'ros_gz_sim', 'create', '-string', sdf,
                 '-name', actor.name, '-allow_renaming', 'false'],
                capture_output=True, text=True, timeout=30.0)
            if result.returncode != 0:
                self.get_logger().error(
                    f'spawning {actor.name} failed: {result.stdout}{result.stderr}')
                raise RuntimeError(f'spawning {actor.name} failed')
            actor.spawned = True
            self.get_logger().info(f'spawned {actor.name} at ({x:.2f}, {y:.2f})')
        return True

    def _tick(self):
        now = self._now()
        if self.robot is None or now - self.robot[4] > 1.0:
            return  # no fresh truth pose: move nothing
        if not self.spawned:
            self.spawned = self._spawn()
            if self.spawned:
                self.get_logger().info(f'all {len(self.actors)} actors spawned')
                self.last_tick = self._now()
            return
        dt = max(0.0, now - self.last_tick)
        self.last_tick = now
        self.ticks += 1
        self.tick_dt_sum += dt
        if self.ticks % 100 == 0:
            self.get_logger().info(
                f'{self.ticks} ticks, mean {self.tick_dt_sum / self.ticks:.3f} s sim per tick')
        # Look further ahead when ticks lag (unthrottled sim), capped at 1s.
        lookahead = min(max(self.lookahead_s, 2.0 * dt), 1.0)
        moves = []
        for actor in self.actors:
            u = self._first_clear(actor, actor.u + actor.speed * dt, lookahead)
            if u is None:
                continue  # hold: no clear spot on the whole path
            actor.u = u % (2.0 * actor.length)
            x, y = actor.position(actor.u)
            moves.append((actor.name, x, y))
        results = self.pool.map(
            lambda m: (m[0], set_model_pose(m[0], m[1], m[2], self.height / 2.0)), moves)
        for name, ok in results:
            if not ok:
                self.get_logger().warning(f'set_pose {name} failed')


def main(args=None):
    rclpy.init(args=args)
    node = ActorDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.pool.shutdown(wait=False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
