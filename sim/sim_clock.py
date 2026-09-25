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

shot_hit.launch.py's point bench (target_state:=truth) uses it. Advances
`rate` sim seconds per wall second, in steps of the measured wall time
(publish_rate_hz, 1000), so sim time never drifts from the rate.
"""
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class SimClock(Node):

    def __init__(self):
        super().__init__('sim_clock')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('publish_rate_hz', 1000.0)
        self.rate = float(self.get_parameter('rate').value)
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self._sim_ns = 0
        self._last_wall = time.monotonic()
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.on_timer)
        self.get_logger().info(f'sim_clock: /clock at {self.rate}x wall time')

    def on_timer(self):
        wall = time.monotonic()
        self._sim_ns += int((wall - self._last_wall) * self.rate * 1e9)
        self._last_wall = wall
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(self._sim_ns, 1_000_000_000)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimClock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
