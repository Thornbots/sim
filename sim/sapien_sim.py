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
SAPIEN engine for sim.launch.py (sim_engine:=sapien): a drop-in for gz plus its bridges.

Publishes /clock, /scan_raw, /sim/raw_odom, /sim/raw_joint_states with gz's frames and
rates; subscribes /cmd_vel (body-frame Twist), /head_pan_cmd, /head_pitch_cmd.
Chassis is kinematic like gz's collision-free sentry; head joints use gz's PD gains.
Lidar is Embree ray casting against the field STL plus spawned boxes.
Param spawn_box:=[x, y, size, height] adds an unmapped box obstacle mid-run.
see README.md for design rationale
"""
import math
import os
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import sapien
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector

JOINT_NAMES = ('headlink', 'headpitch', 'odowheel_x', 'odowheel_y')
# sentry.urdf.xacro's JointPositionController gains and gpu_lidar spec.
HEAD_P, HEAD_D, HEAD_EFFORT = 75.0, 0.125, 50.0
LIDAR_SAMPLES, LIDAR_MIN_ANGLE, LIDAR_MAX_ANGLE = 3000, -3.14, 3.14
LIDAR_RANGE_MIN, LIDAR_RANGE_MAX, LIDAR_NOISE = 0.2, 12.0, 0.03


def _yaw_quat(yaw):
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]  # sapien wxyz


def _ray_boxes(origins, dirs, boxes):
    """Distance along each horizontal ray to the nearest (x, y, size, height) box, inf if none."""
    best = np.full(len(dirs), np.inf)
    z = origins[0, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        for (bx, by, size, height) in boxes:
            if not 0.0 <= z <= height:
                continue
            h = size / 2.0
            lo = (np.array([bx - h, by - h]) - origins[:, :2]) / dirs[:, :2]
            hi = (np.array([bx + h, by + h]) - origins[:, :2]) / dirs[:, :2]
            t_near = np.nanmax(np.minimum(lo, hi), axis=1)
            t_far = np.nanmin(np.maximum(lo, hi), axis=1)
            hit = (t_far >= t_near) & (t_far > 0)
            t = np.where(t_near > 0, t_near, t_far)
            best = np.where(hit & (t < best), t, best)
    return best


class SapienSim(Node):

    def __init__(self):
        super().__init__('sapien_sim')
        share = get_package_share_directory('sim')
        self.declare_parameter('xacro', os.path.join(share, 'urdf', 'sentry.urdf.xacro'))
        self.declare_parameter('field_mesh', os.path.join(share, 'world', 'composite_part_1.stl'))
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.03)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('real_time_factor', 0.0)  # 0 = as fast as possible
        self.declare_parameter('physics_hz', 1000.0)     # gz world max_step_size 1 ms
        self.declare_parameter('clock_hz', 1000.0)
        self.declare_parameter('joint_state_hz', 1000.0)
        self.declare_parameter('odom_hz', 100.0)
        self.declare_parameter('scan_hz', 10.0)
        self.declare_parameter('seed', 0)
        self.declare_parameter('spawn_box', [0.0])
        self.add_on_set_parameters_callback(self._on_set_params)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.dt = 1.0 / gp('physics_hz')
        self.rtf = gp('real_time_factor')
        self.rng = np.random.default_rng(gp('seed'))
        self._every = {k: max(1, round(gp('physics_hz') / gp(k)))
                       for k in ('clock_hz', 'joint_state_hz', 'odom_hz', 'scan_hz')}

        self.scene = sapien.Scene()
        self.scene.set_timestep(self.dt)
        self.robot = self._load_robot(gp('xacro'))
        self.joints = {j.name: j for j in self.robot.active_joints}
        self._qidx = {n: i for i, n in enumerate(j.name for j in self.robot.active_joints)}
        for name in ('headlink', 'headpitch'):
            self.joints[name].set_drive_properties(HEAD_P, HEAD_D, HEAD_EFFORT, 'force')
        self.lidar_link = next(link for link in self.robot.links if link.name == 'lidar')

        self.field = RayMeshIntersector(trimesh.load(gp('field_mesh')))
        self.boxes = []
        angles = np.linspace(LIDAR_MIN_ANGLE, LIDAR_MAX_ANGLE, LIDAR_SAMPLES)
        self._beam_dirs = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], 1)

        self.pos = np.array([gp('x'), gp('y'), gp('z')])
        self.yaw = gp('yaw')
        self.body_vel = np.zeros(3)  # vx, vy (body frame), wz
        self.head_cmd = {'headlink': 0.0, 'headpitch': 0.0}
        self.lock = threading.Lock()
        self.sim_time = 0.0

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/sim/raw_odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/sim/raw_joint_states', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(
            Float64, '/head_pan_cmd', lambda m: self._on_head('headlink', m), 10)
        self.create_subscription(
            Float64, '/head_pitch_cmd', lambda m: self._on_head('headpitch', m), 10)

    def _load_robot(self, xacro_path):
        urdf = subprocess.run(['xacro', xacro_path], check=True,
                              capture_output=True, text=True).stdout
        share_parent = os.path.dirname(get_package_share_directory('sim'))
        urdf = urdf.replace('package://', share_parent + '/')
        path = f'/tmp/sapien_sim_{os.getpid()}.urdf'
        with open(path, 'w') as f:
            f.write(urdf)
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = True  # root is moved kinematically, like gz's VelocityControl
        robot = loader.load(path)
        for link in robot.links:
            link.disable_gravity = True  # matches the URDF's <gravity>false</gravity>
        return robot

    def _on_cmd_vel(self, msg):
        with self.lock:
            self.body_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def _on_head(self, name, msg):
        with self.lock:
            self.head_cmd[name] = msg.data

    def _on_set_params(self, params):
        for p in params:
            if p.name == 'spawn_box' and len(p.value) == 4:
                x, y, size, height = p.value
                # Lidar-only: the sentry has no collision, so nothing else can touch it.
                with self.lock:
                    self.boxes.append((x, y, size, height))
                self.get_logger().info(f'spawned box at ({x}, {y}) size {size} height {height}')
        return SetParametersResult(successful=True)

    def step(self, n):
        with self.lock:
            vx, vy, wz = self.body_vel
            head_cmd = dict(self.head_cmd)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.world_vel = np.array([c * vx - s * vy, s * vx + c * vy, 0.0])
        self.pos += self.world_vel * self.dt
        self.yaw += wz * self.dt
        self.wz = wz
        self.robot.set_root_pose(sapien.Pose(self.pos, _yaw_quat(self.yaw)))
        for name, target in head_cmd.items():
            self.joints[name].set_drive_target(target)
        self.scene.step()
        self.sim_time += self.dt

        stamp = self._stamp()
        if n % self._every['clock_hz'] == 0:
            self.clock_pub.publish(Clock(clock=stamp))
        if n % self._every['joint_state_hz'] == 0:
            self._publish_joints(stamp)
        if n % self._every['odom_hz'] == 0:
            self._publish_odom(stamp)
        if n % self._every['scan_hz'] == 0:
            self._publish_scan(stamp)

    def _stamp(self):
        sec = int(self.sim_time)
        nsec = round((self.sim_time - sec) * 1e9)
        return rclpy.time.Time(seconds=sec, nanoseconds=nsec).to_msg()

    def _publish_joints(self, stamp):
        qpos, qvel = self.robot.get_qpos(), self.robot.get_qvel()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINT_NAMES)
        msg.position = [float(qpos[self._qidx[n]]) for n in JOINT_NAMES]
        msg.velocity = [float(qvel[self._qidx[n]]) for n in JOINT_NAMES]
        msg.effort = [0.0] * len(JOINT_NAMES)
        self.joint_pub.publish(msg)

    def _publish_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'root'
        p = msg.pose.pose
        p.position.x, p.position.y, p.position.z = (float(v) for v in self.pos)
        p.orientation.w = math.cos(self.yaw / 2.0)
        p.orientation.z = math.sin(self.yaw / 2.0)
        # gz's OdometryPublisher reports twist in the child (root) frame.
        with self.lock:
            vx, vy, wz = self.body_vel
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = float(vx), float(vy)
        msg.twist.twist.angular.z = float(wz)
        self.odom_pub.publish(msg)

    def _publish_scan(self, stamp):
        pose = self.lidar_link.pose
        rot = pose.to_transformation_matrix()[:3, :3]
        dirs = self._beam_dirs @ rot.T
        dirs[:, 2] = 0.0  # a planar lidar stays planar if the head pitches the link
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        origins = np.tile(pose.p, (len(dirs), 1))
        ranges = np.full(len(dirs), np.inf)
        loc, ray_idx, _ = self.field.intersects_location(origins, dirs, multiple_hits=False)
        ranges[ray_idx] = np.linalg.norm(loc - origins[ray_idx], axis=1)
        with self.lock:
            boxes = list(self.boxes)
        if boxes:
            ranges = np.minimum(ranges, _ray_boxes(origins, dirs, boxes))
        hit = np.isfinite(ranges)
        ranges[hit] += self.rng.normal(0.0, LIDAR_NOISE, hit.sum())
        ranges[(ranges < LIDAR_RANGE_MIN) | (ranges > LIDAR_RANGE_MAX)] = np.inf

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = 'lidar'
        msg.angle_min, msg.angle_max = LIDAR_MIN_ANGLE, LIDAR_MAX_ANGLE
        msg.angle_increment = (LIDAR_MAX_ANGLE - LIDAR_MIN_ANGLE) / (LIDAR_SAMPLES - 1)
        msg.scan_time = 1.0 / self.get_parameter('scan_hz').value
        msg.range_min, msg.range_max = LIDAR_RANGE_MIN, LIDAR_RANGE_MAX
        msg.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(msg)

    def run(self):
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self)
        threading.Thread(target=self.executor_.spin, daemon=True).start()
        n, wall0, stats_t, stats_n = 0, time.monotonic(), time.monotonic(), 0
        while rclpy.ok():
            n += 1
            self.step(n)
            if self.rtf > 0:
                ahead = self.sim_time / self.rtf - (time.monotonic() - wall0)
                if ahead > 0:
                    time.sleep(ahead)
            if time.monotonic() - stats_t >= 10.0:
                rate = (n - stats_n) * self.dt / (time.monotonic() - stats_t)
                self.get_logger().info(f'sim t={self.sim_time:.1f}s, {rate:.1f}x real time')
                stats_t, stats_n = time.monotonic(), n


def main():
    rclpy.init()
    node = SapienSim()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.executor_.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
