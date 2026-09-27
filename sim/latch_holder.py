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
Hold a latched publisher on /tf_static, /map and /robot_description; never publish.

foxglove_bridge picks a subscription's durability from the publishers there
when a client subscribes, and keeps it. The drift suite restarts
robot_state_publisher and map_server every scenario, so a client that
subscribes in the gap got a volatile subscription and never saw those topics
again. With this publisher always present the bridge subscribes latched.
"""
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
TOPICS = {'/tf_static': TFMessage, '/map': OccupancyGrid, '/robot_description': String}


class LatchHolder(Node):

    def __init__(self):
        super().__init__('foxglove_latch_holder')
        self._pubs = [self.create_publisher(t, name, LATCHED) for name, t in TOPICS.items()]


def main(args=None):
    rclpy.init(args=args)
    node = LatchHolder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
