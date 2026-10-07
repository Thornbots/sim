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

"""Drive scripted spawn-to-center routes; the real MCB still owns aim and fire."""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sim.match_scenario import match_duration, route_duration, ROUTES, sample_match, sample_route


class MatchDriver(Node):

    def __init__(self):
        super().__init__('match_driver')
        self.declare_parameter('active', False)
        self.declare_parameter('stage', 'e3')
        self.start_s = None
        self.last_s = None
        self.position = ROUTES['sentry'][0]
        self.yaw = math.pi
        self.velocity = (0.0, 0.0)
        self.spin = 0.0
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reference = self.create_publisher(Odometry, '/sim/match/reference', 10)
        self.paths = {name: self.create_publisher(Odometry, f'/sim/match/{name}/path', 10)
                      for name in ['opponent_0']}
        self.create_subscription(Odometry, '/sim/raw_odom', self._on_odom, 10)
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(0.01, self._tick)
        self.get_logger().info(f'spawn-to-center scenario: {match_duration():.2f} sim seconds')

    def _on_params(self, params):
        for param in params:
            if param.name == 'active':
                self.start_s = self.get_clock().now().nanoseconds / 1e9 if param.value else None
                self.velocity, self.spin = (0.0, 0.0), 0.0
        return SetParametersResult(successful=True)

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.position = (p.x, p.y)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

    def _path_message(self, name, position, velocity, yaw, spin, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = name
        msg.pose.pose.position.x, msg.pose.pose.position.y = position
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        c, s = math.cos(yaw), math.sin(yaw)
        msg.twist.twist.linear.x = c * velocity[0] + s * velocity[1]
        msg.twist.twist.linear.y = -s * velocity[0] + c * velocity[1]
        msg.twist.twist.angular.z = spin
        return msg

    def _tick(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds / 1e9
        dt = min(max(seconds - (self.last_s or seconds), 0.0), 0.1)
        self.last_s = seconds
        elapsed = max(seconds - self.start_s, 0.0) if self.start_s is not None else 0.0
        pos, vel, spin, _ = sample_match(elapsed)
        if self.start_s is None:
            pos, vel, spin = ROUTES['sentry'][0], (0.0, 0.0), 0.0
        reference = self._path_message('root', pos, vel, self.yaw, spin, now.to_msg())
        # The reference is created at this tick in the field frame.
        self.reference.publish(reference)
        desired = [vel[i] + 2.0 * (pos[i] - self.position[i]) for i in range(2)]
        norm = math.hypot(*desired)
        if norm > 2.0:
            desired = [v * 2.0 / norm for v in desired]
        delta = [desired[i] - self.velocity[i] for i in range(2)]
        length = math.hypot(*delta)
        scale = min(1.0, 2.0 * dt / length) if length else 1.0
        self.velocity = tuple(self.velocity[i] + scale * delta[i] for i in range(2))
        self.spin += max(-20.0 * dt, min(20.0 * dt, spin - self.spin))
        cmd = Twist()
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        cmd.linear.x = c * self.velocity[0] + s * self.velocity[1]
        cmd.linear.y = -s * self.velocity[0] + c * self.velocity[1]
        cmd.angular.z = self.spin
        self.command.publish(cmd)
        for name, publisher in self.paths.items():
            p, v, finished = sample_route(ROUTES[name], elapsed)
            yaw = 0.0
            omega = 0.0
            if finished:
                fight = elapsed - route_duration(ROUTES[name])
                # Smooth motion inside the center lane, with a spinning chassis.
                p = (p[0], p[1] + 0.3 * math.sin(2.0 * fight))
                v = (0.0, 0.6 * math.cos(2.0 * fight))
                yaw, omega = 3.0 * math.pi * fight, 3.0 * math.pi
            publisher.publish(self._path_message(name, p, v, yaw, omega, now.to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = MatchDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
