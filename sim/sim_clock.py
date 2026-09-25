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
Publish /clock without gz, for stacks that run on sim time but have no world.

shot_hit.launch.py's point bench uses it. rate > 0 advances `rate` sim
seconds per wall second, in steps of the measured wall time (publish_rate_hz).
rate 0 runs as fast as the stack keeps up: sim time moves in step_s steps,
and never more than one period (plus a step) past the newest header.stamp on
any pace_topics entry ("topic pkg/msg/Type period_s"). A topic silent for
max_wait_s of wall time stops holding the clock until it publishes again;
with no topic holding it (at startup, say), the clock runs at 1x.
"""
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message


class SimClock(Node):

    def __init__(self):
        super().__init__('sim_clock')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('publish_rate_hz', 1000.0)
        self.declare_parameter('step_s', 0.002)
        self.declare_parameter('pace_topics', [''])
        self.declare_parameter('max_wait_s', 0.5)
        gp = self.get_parameter
        self.rate = float(gp('rate').value)
        self.step_ns = int(float(gp('step_s').value) * 1e9)
        self.max_wait_s = float(gp('max_wait_s').value)
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self._sim_ns = 0
        self._last_wall = time.monotonic()
        if self.rate > 0.0:
            self.create_timer(1.0 / gp('publish_rate_hz').value, self.on_timer)
            self.get_logger().info(f'sim_clock: /clock at {self.rate}x wall time')
            return
        self._cond = threading.Condition()
        self._gates = {}  # topic -> [period_ns, last stamp_ns or None, live]
        for spec in gp('pace_topics').value:
            if not spec.strip():
                continue
            topic, type_name, period = spec.split()
            self._gates[topic] = [int(float(period) * 1e9), None, False]
            self.create_subscription(get_message(type_name), topic,
                                     lambda msg, t=topic: self._on_paced(t, msg),
                                     qos_profile_sensor_data)
        self.get_logger().info(
            f'sim_clock: as fast as {sorted(self._gates)} keep up, '
            f'{self.step_ns / 1e6:g} ms steps')

    def on_timer(self):
        wall = time.monotonic()
        self._sim_ns += int((wall - self._last_wall) * self.rate * 1e9)
        self._last_wall = wall
        self._publish()

    def _publish(self):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(self._sim_ns, 1_000_000_000)
        self.pub.publish(msg)

    def _on_paced(self, topic, msg):
        stamp = msg.stamp if hasattr(msg, 'stamp') else msg.header.stamp  # a bare Header too
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        with self._cond:
            gate = self._gates[topic]
            gate[1] = stamp_ns if gate[1] is None else max(gate[1], stamp_ns)
            gate[2] = True
            self._cond.notify()

    def _blocking(self, next_ns):
        """Return the live gates that must publish before sim time reaches next_ns."""
        return [t for t, (period, last, live) in self._gates.items()
                if live and last + period + self.step_ns < next_ns]

    def run_paced(self):
        """Step the clock forever; run on its own thread while the executor spins."""
        report_wall, report_sim = time.monotonic(), 0
        while rclpy.ok():
            next_ns = self._sim_ns + self.step_ns
            with self._cond:
                deadline = time.monotonic() + self.max_wait_s
                while (waiting := self._blocking(next_ns)) and time.monotonic() < deadline:
                    self._cond.wait(timeout=deadline - time.monotonic())
                for topic in waiting:  # silent too long: stop holding the clock for it
                    self._gates[topic][2] = False
                    self.get_logger().warn(f'sim_clock: {topic} silent, no longer pacing on it')
                idle = not any(live for _, _, live in self._gates.values())
            if idle:
                time.sleep(self.step_ns / 1e9)
            self._sim_ns = next_ns
            self._publish()
            wall = time.monotonic()
            if wall - report_wall >= 10.0:
                self.get_logger().info(
                    f'sim_clock: {(self._sim_ns - report_sim) / 1e9 / (wall - report_wall):.1f}x')
                report_wall, report_sim = wall, self._sim_ns


def main(args=None):
    rclpy.init(args=args)
    node = SimClock()
    if node.rate > 0.0:
        rclpy.spin(node)
    else:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        threading.Thread(target=node.run_paced, daemon=True).start()
        executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
