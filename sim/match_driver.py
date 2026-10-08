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
import random
import threading

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sim.cv_head_aim_core import solve_head_angles
from sim.match_scenario import ingress_duration, match_duration, ROUTES, sample_match, sample_robot
from sim.odometry import velocity_in_parent
from std_msgs.msg import Header


class MatchDriver(Node):

    def __init__(self):
        super().__init__('match_driver')
        self.declare_parameter('active', False)
        self.declare_parameter('stage', 'mcb_drive')
        self.declare_parameter('seed', 2026)
        self.stage = self.get_parameter('stage').value
        self.random = random.Random(self.get_parameter('seed').value)
        self.hp = {name: 400 for name in ROUTES}
        self.world_velocity = (0.0, 0.0)
        self.truth = {}
        self.truth_lock = threading.Lock()
        self.next_fire = {}
        self.start_s = None
        self.last_s = None
        self.position = ROUTES['sentry'][0]
        self.yaw = math.pi
        self.velocity = (0.0, 0.0)
        self.spin = 0.0
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reference = self.create_publisher(Odometry, '/sim/match/reference', 10)
        self.paths = {name: self.create_publisher(Odometry, f'/sim/match/{name}/path', 10)
                      for name in (['opponent_0', 'opponent_1', 'ally_0']
                                   if self.stage == 'mcb_match' else ['opponent_0'])}
        self.create_subscription(Odometry, '/sim/raw_odom', self._on_odom, 10)
        if self.stage == 'mcb_match':
            self._setup_combat()
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(0.01, self._tick)
        self.get_logger().info(
            f'spawn-to-center scenario: {match_duration(self.stage):.2f} sim seconds')

    def _on_params(self, params):
        for param in params:
            if param.name == 'active':
                self.start_s = self.get_clock().now().nanoseconds / 1e9 if param.value else None
                self.velocity, self.spin = (0.0, 0.0), 0.0
        return SetParametersResult(successful=True)

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.position = (p.x, p.y)
        self.world_velocity = velocity_in_parent(msg)[:2]
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
        pos, vel, spin, _ = sample_match(elapsed, self.stage)
        if self.start_s is None:
            pos, vel, spin = ROUTES['sentry'][0], (0.0, 0.0), 0.0
        reference = self._path_message('root', pos, vel, self.yaw, spin, now.to_msg())
        # The reference is created at this tick in the field frame.
        self.reference.publish(reference)
        if self.hp['sentry'] == 0:
            vel, spin = (0.0, 0.0), 0.0
            pos = self.position
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
        samples = {'sentry': (self.position, self.world_velocity)}
        for name, publisher in self.paths.items():
            p, v, yaw, omega = sample_robot(name, elapsed, self.stage)
            samples[name] = (p, v)
            if self.hp[name] == 0:
                with self.truth_lock:
                    truth = self.truth.get(name)
                if truth is not None:
                    p = tuple(truth[1][:2, 3])
                    yaw = math.atan2(truth[1][1, 0], truth[1][0, 0])
                v, omega = (0.0, 0.0), 0.0
            publisher.publish(self._path_message(name, p, v, yaw, omega, now.to_msg()))
        if self.stage == 'mcb_match' and self.start_s is not None:
            if elapsed < match_duration(self.stage):
                self._combat(elapsed, now, samples)

    def _setup_combat(self):
        from gz.msgs10.double_pb2 import Double
        import gz.msgs10.pose_pb2  # noqa: F401; nested poses before parsing Pose_V
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self.gz = GzNode()
        from sim.combat import armor_offsets
        self.armor_offsets = armor_offsets()
        self.Double = Double
        self.head_commands = {
            name: [self.gz.advertise(f'/model/{name}/joint/{joint}/cmd_pos', Double)
                   for joint in ['headlink', 'headpitch']]
            for name in self.paths}
        self.shot_publishers = {
            name: self.create_publisher(Header, f'/sim/match/{name}/shot', 10)
            for name in self.paths}
        for name in ROUTES:
            self.gz.subscribe(Pose_V, f'/model/{name}/pose',
                              lambda msg, robot=name: self._on_truth(robot, msg))
        self.create_subscription(DiagnosticArray, '/sim/match/referee', self._on_referee, 10)

    def _on_referee(self, msg):
        for status in msg.status:
            for value in status.values:
                if value.key == 'hp' and status.name in self.hp:
                    self.hp[status.name] = int(value.value)

    def _on_truth(self, name, msg):
        from sim.combat import pose_matrix
        poses = {pose.name: pose for pose in msg.pose}
        if name not in poses or f'{name}::root' not in poses:
            return
        model = pose_matrix(poses[name])
        root = model @ pose_matrix(poses[f'{name}::root'])
        stamp = poses[name].header.stamp
        with self.truth_lock:
            self.truth[name] = (stamp.sec + stamp.nsec * 1e-9, root)

    def _combat(self, elapsed, now, samples):
        from sim.match_scenario import TEAMS
        with self.truth_lock:
            truth = dict(self.truth)
        seconds = now.nanoseconds / 1e9
        for name in self.paths:
            if self.hp[name] == 0 or name not in truth or seconds - truth[name][0] > .1:
                continue
            enemies = [robot for robot in truth if TEAMS[robot] != TEAMS[name]
                       and self.hp[robot] > 0 and seconds - truth[robot][0] <= .1]
            if not enemies:
                continue
            root = truth[name][1]
            target = min(enemies, key=lambda robot: np.linalg.norm(
                truth[robot][1][:2, 3] - root[:2, 3]))
            target_root = truth[target][1]
            # Ground-truth panel centers from the same URDF as scoring, with a flight-time lead.
            panels = [target_root @ offset for offset in self.armor_offsets]
            panel = max(panels, key=lambda p: np.dot(p[:3, 0], root[:3, 3] - p[:3, 3]))
            aim = panel[:3, 3].copy()
            flight = np.linalg.norm(aim - root[:3, 3]) / 25.0
            aim[:2] += np.array(samples[target][1]) * flight
            aim[2] += 9.80665 * flight**2 / 2
            relative = (np.linalg.inv(root) @ np.append(aim, 1.0))[:3]
            yaw, pitch = solve_head_angles(relative)
            yaw += self.random.gauss(0.0, .015)
            pitch += self.random.gauss(0.0, .015)
            self.head_commands[name][0].publish(self.Double(data=yaw))
            self.head_commands[name][1].publish(self.Double(data=max(-.6, min(.6, pitch))))
            if elapsed < ingress_duration(self.stage):
                continue
            if seconds >= self.next_fire.get(name, 0.0):
                self.shot_publishers[name].publish(
                    Header(stamp=now.to_msg(), frame_id=f'{name}/muzzle'))
                self.next_fire[name] = seconds + 0.5  # 2 Hz truth aimer, fixed seed


def main(args=None):
    rclpy.init(args=args)
    node = MatchDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
