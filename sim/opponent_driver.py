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
Spawn a ghost sentry_v2 opponent in gz at the start of target_driver's path.

Waits for /target/ground_truth_odom, then spawns `name` there once. The
model's OpponentMover system (sentry_v2.urdf.xacro, opponent:=true) moves
it from then on, from the same path bridged into gz. Params: name, z (root
height, sentry_v2 settled on its springs).
"""
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sim.auto_explore import model_names, spawn_model
from sim.opponents import opponent_urdf


class OpponentDriver(Node):

    def __init__(self):
        super().__init__('opponent_driver')
        self.declare_parameter('name', 'opponent_0')
        self.declare_parameter('z', -0.0115)
        self.name = self.get_parameter('name').value
        self.z = self.get_parameter('z').value
        self._start = None
        self.create_subscription(Odometry, '/target/ground_truth_odom', self._on_path, 10)
        self._timer = self.create_timer(0.5, self._spawn)

    def _on_path(self, msg):
        if self._start is None:
            self._start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _spawn(self):
        """Spawn once; create's reply can outlast its timeout, so check the world."""
        if self._start is None:
            return
        x, y = self._start
        names = model_names()
        if names is not None and self.name not in names:
            spawn_model(self.name, opponent_urdf(self.name), x, y, self.z)
            names = model_names()
        if names is not None and self.name in names:
            self._timer.cancel()
            self.get_logger().info(f'{self.name} spawned at ({x:.2f}, {y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
