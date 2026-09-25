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
Draw a TargetState in rviz: its 4 panels, center, velocity and acceleration.

input_topic (/cv/target_state) -> output_topic (/cv/target_state_markers),
at most max_rate_hz (30) of wall time, the newest state only: rviz reads one
message per topic per frame, so a faster stream only queues. Panels follow
TargetState.msg's armor model, canted 15 deg per S122; orange when valid,
grey when not, the tracked panel (k = 0) brighter. Arrows: velocity x
velocity_scale_s (magenta), acceleration x accel_scale_s2 (red).
Visualization only.
"""
import math

from dji_serial_bridge.msg import TargetState
from geometry_msgs.msg import Point
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sim.cv_target_emulator import canted_panel_quat, PANEL_SIZE
from visualization_msgs.msg import Marker, MarkerArray

LIFETIME_NS = 500_000_000  # a state that stops arriving clears in 0.5 s


class TargetStateMarkers(Node):

    def __init__(self):
        super().__init__('target_state_markers')
        self.declare_parameter('input_topic', '/cv/target_state')
        self.declare_parameter('output_topic', '/cv/target_state_markers')
        self.declare_parameter('max_rate_hz', 30.0)
        self.declare_parameter('velocity_scale_s', 1.0)
        self.declare_parameter('accel_scale_s2', 0.25)
        gp = self.get_parameter
        self.velocity_scale_s = float(gp('velocity_scale_s').value)
        self.accel_scale_s2 = float(gp('accel_scale_s2').value)
        self._latest = None
        self.pub = self.create_publisher(MarkerArray, gp('output_topic').value, 10)
        self.create_subscription(TargetState, gp('input_topic').value, self.on_state, 10)
        self.create_timer(1.0 / float(gp('max_rate_hz').value), self.on_timer,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_state(self, msg):
        self._latest = msg

    def on_timer(self):
        if self._latest is None:
            return
        msg, self._latest = self._latest, None
        self.pub.publish(MarkerArray(markers=self.markers(msg)))

    def _marker(self, msg, ns, kind, rgb, alpha=1.0):
        m = Marker()
        m.header.stamp = msg.header.stamp  # the time the state describes
        m.header.frame_id = msg.header.frame_id or 'odom'
        m.ns, m.id, m.type, m.action = ns, 0, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = alpha
        m.lifetime.nanosec = LIFETIME_NS
        return m

    def _arrow(self, msg, ns, vec, scale, rgb):
        c = msg.center
        m = self._marker(msg, ns, Marker.ARROW, rgb)
        m.points = [Point(x=c.x, y=c.y, z=c.z),
                    Point(x=c.x + vec.x * scale, y=c.y + vec.y * scale, z=c.z + vec.z * scale)]
        m.scale.x, m.scale.y, m.scale.z = 0.02, 0.05, 0.0  # shaft, head, head length auto
        if math.hypot(vec.x, vec.y, vec.z) * scale < 1e-3:
            m.action = Marker.DELETE
        return m

    def markers(self, msg):
        """Return the MarkerArray contents for one TargetState."""
        panel_rgb = (1.0, 0.5, 0.0) if msg.valid else (0.6, 0.6, 0.6)
        out = []
        center = self._marker(msg, 'state_center', Marker.SPHERE, panel_rgb, 0.8)
        center.pose.position = msg.center
        center.scale.x = center.scale.y = center.scale.z = 0.08
        out.append(center)
        for k in range(4):
            yaw = msg.yaw + k * math.pi / 2.0
            r, dz = float(msg.radius[k % 2]), float(msg.z_offset[k % 2])  # numpy float32
            panel = self._marker(msg, 'state_panels', Marker.CUBE, panel_rgb,
                                 0.9 if k == 0 else 0.5)
            panel.id = k
            panel.pose.position = Point(x=msg.center.x + r * math.cos(yaw),
                                        y=msg.center.y + r * math.sin(yaw),
                                        z=msg.center.z + dz)
            o = panel.pose.orientation
            o.x, o.y, o.z, o.w = canted_panel_quat(yaw)
            panel.scale.x, panel.scale.y, panel.scale.z = 0.02, PANEL_SIZE, PANEL_SIZE
            out.append(panel)
        out.append(self._arrow(msg, 'state_velocity', msg.velocity,
                               self.velocity_scale_s, (1.0, 0.0, 1.0)))
        out.append(self._arrow(msg, 'state_accel', msg.acceleration,
                               self.accel_scale_s2, (1.0, 0.1, 0.1)))
        return out


def main(args=None):
    rclpy.init(args=args)
    node = TargetStateMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
