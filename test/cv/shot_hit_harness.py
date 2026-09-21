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
Black-box shot-hit machinery: launches the sim plus the production CV pipeline.

Scores each firing CVTarget on /dji_serial_bridge/cv_target, knowing
nothing about how thornbots_pkg predicts (see sim/README.md's ## Notes for
why). Importable only -- test_shot_hit.py holds the assertions and
run_shot_hit_tests.py is the argparse wrapper.

For each shot: computes the muzzle pose (from /sim/raw_odom +
/sim/raw_joint_states, the same fixed FK chain cv_target_emulator.py
uses, duplicated here rather than imported so this doesn't silently
start passing/failing from an unrelated emulator refactor), simulates a
straight-line 25 m/s projectile (muzzle-speed cap per
ARCC_2026_SENTRY_CONTEXT.md), and checks the shot against all 4 of the
target's armor panels (layout also duplicated from
cv_target_emulator.py) at estimated impact time: a hit needs BOTH the
flight path to pass within hit_radius of a panel's 0.1m x 0.1m face AND
to arrive within that panel's 145-degree front exposure cone --
geometrically on-target from behind the panel still misses.

Requires mcb_relay.py to relay /cv/target onto
/dji_serial_bridge/cv_target (wired 2026-07-27, fire decision merged into
CVTarget 2026-09-20) and point_to_cv_target's placeholder fire trigger
(fire_rate_hz, no lead/HP/heat/power gating -- see
thornbots_pkg/README.md) to set fire=True on one.
Shots are observed today, so "zero shots" means something in the
launched stack is actually broken.
"""
import math
import os
import shlex
import signal
import subprocess
import time

from dji_serial_bridge.msg import CVTarget
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


MUZZLE_SPEED = 25.0  # m/s -- ARCC_2026_SENTRY_CONTEXT.md's muzzle-speed cap
TRUTH_HISTORY_S = 1.0  # ground truth kept for interpolating impact-time poses
# Static delay from a shot's fire time (CVTarget stamp + delay_ms) to the
# projectile leaving the muzzle; flight time at MUZZLE_SPEED comes on top.
FIRE_LATENCY_S = 0.05

DEFAULT_SPEEDS = [0.5, 1.0, 2.0, 4.0]
DEFAULT_DURATION = 30.0  # sim-time seconds of steady-state sampling per case
SETTLE_S = 3.0  # sim-time seconds after a motion change before scoring starts
# The bench fires far faster than the real launcher to stress tracking.
# point_to_cv_target fires at most once per publish tick, so the publish rate
# sets it; fire_rate_hz sits above so a tick a millisecond early still fires.
TEST_FIRE_HZ = 40.0
DEFAULT_LOG_DIR = '/tmp/shot_hit_test_logs'
# The stationary case (speed=0, spin=0) is the harness's own sanity
# check: a working pipeline hits a motionless target trivially. 0.5 is far
# below what a working stack does (90-96% on 2026-09-21), a floor that
# catches gross breakage, not a tuned figure. See CV_TEST_GAPS.md gap 7.
STATIONARY_MIN_HIT_RATE = 0.5
# The moving cases are what this bench exists for: aiming at a moving,
# spinning target. A placeholder stating intent, not a measurement; see
# CV_TEST_GAPS.md gap 8.
MOVING_MIN_HIT_RATE = 0.25
# Spin rate swept inversely to speed, spanning ARCC's documented
# "typically 1-2 Hz" range (ARCC_2026_SENTRY_CONTEXT.md).
SPIN_HZ_AT_MIN_SPEED = 2.0
SPIN_HZ_AT_MAX_SPEED = 1.0

POINT_TO_CV_TARGET_BIN = (
    '/workspaces/isaac_ros-dev/install/thornbots_pkg/lib/thornbots_pkg/point_to_cv_target'
)
MCB_RELAY_BIN = (
    '/workspaces/isaac_ros-dev/install/thornbots_pkg/lib/thornbots_pkg/mcb_relay'
)
TARGET_SELECTOR_BIN = (
    '/workspaces/isaac_ros-dev/install/thornbots_pkg/lib/thornbots_pkg/target_selector'
)
TARGET_TRACKER_BIN = (
    '/workspaces/isaac_ros-dev/install/thornbots_pkg/lib/thornbots_pkg/target_tracker'
)
CV_RVIZ_CONFIG = '/workspaces/isaac_ros-dev/install/sim/share/sim/rviz/cv_target.rviz'


# Same fixed FK chain as cv_target_emulator.py's _camera_pose (root -> body
# -> head(yaw) -> head_pitch(pitch) -> camera; cameralink is identity, and
# there's no separate muzzle link in this sim, so muzzle == camera pose).
def _rotation_from_rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _rotation_from_quaternion(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rotation_axis_angle(axis, angle):
    ax = np.array(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([
        [0, -ax[2], ax[1]],
        [ax[2], 0, -ax[0]],
        [-ax[1], ax[0], 0],
    ])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def _transform(rot, trans):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


_T_FASTENED_2 = _transform(_rotation_from_rpy(0, 0, math.pi), (0.0, 0.0, 0.0))
_HEADLINK_ORIGIN_R = _rotation_from_rpy(0, 0, math.pi)
_HEADLINK_ORIGIN_T = (0.0, 0.0, 0.252215)
_HEADLINK_AXIS = (0.0, 0.0, -1.0)
_HEADPITCH_ORIGIN_R = _rotation_from_rpy(0, 0, -0.38885)
_HEADPITCH_ORIGIN_T = (0.1, 0.0, 0.1218)
_HEADPITCH_AXIS = (0.0, 1.0, 0.0)

# Same 4-panel layout as cv_target_emulator.py's _panel_poses (front/left/
# back/right, spaced 90 degrees apart around the chassis center) --
# duplicated for the same "independent of an emulator refactor" reason as
# the FK constants above.
_PANEL_OFFSETS_RAD = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
_PANEL_USES_RADIUS_X = (True, False, True, False)
PANEL_RADIUS_X = 0.30  # chassis-center-to-panel offset, front/back
PANEL_RADIUS_Y = 0.24  # chassis-center-to-panel offset, left/right
# Small Armor Module is a flat 0.1m x 0.1m square (ARCC_2026_SENTRY_CONTEXT.md
# "What an armor panel actually looks like" / cv_target_emulator.PANEL_SIZE).
PANEL_SIZE = 0.1
DEFAULT_HIT_RADIUS = PANEL_SIZE / 2.0  # inscribed-circle half-side
# Hit-registration cone: a shot arriving from behind this angle couldn't
# have registered on the real panel even if geometrically on-target.
# Deliberately NOT the same number as cv_target_emulator.py's
# panel_view_half_angle (75 deg = 150 deg full) even though both are
# panel-facing-angle gates -- they answer different physical questions.
# panel_view_half_angle asks "could YOLO plausibly see this panel at all"
# (a detection-visibility gate, checked at detection time from the
# camera's bearing); this asks "did the panel's front face register the
# hit" (checked at impact time from the projectile's arrival direction).
# ARCC_2026_SENTRY_CONTEXT.md's 145 deg (line 237) is stated there as a
# MOUNTING-CLEARANCE requirement ("front 145 of the panel's exposure
# surface must stay unblocked"), not a hit-registration spec -- reusing it
# here is a deliberate stand-in, not a literal citation of an authoritative
# hit-registration cone (no such number exists in the doc). Chosen
# explicitly rather than left as an unexplained numeric coincidence with
# the emulator's 72.5 deg-vs-75 deg gap.
PANEL_EXPOSURE_HALF_ANGLE = math.radians(145.0 / 2.0)
# S122 (ARCC_2026_SENTRY_CONTEXT.md "Mounting angle"): panel outward normal
# makes a 75-degree angle with straight-up, i.e. canted ~15 degrees off
# pure-horizontal (90 degrees would be flush-vertical).
PANEL_NORMAL_ANGLE_FROM_UP = math.radians(75.0)


def _panel_poses(target_pos, target_rot):
    """
    Compute world (position, outward_normal_unit_vector) for each of the 4 armor panels.

    Mirrors cv_target_emulator.py's _panel_poses exactly. Position offset
    stays in the horizontal chassis plane; the outward normal is canted
    per S122, not flush-horizontal (z=0).
    """
    poses = []
    for offset, use_x in zip(_PANEL_OFFSETS_RAD, _PANEL_USES_RADIUS_X):
        radius = PANEL_RADIUS_X if use_x else PANEL_RADIUS_Y
        horiz_dir = np.array([math.cos(offset), math.sin(offset), 0.0])
        world_horiz = target_rot @ horiz_dir
        panel_pos = target_pos + radius * world_horiz

        local_normal = np.array([
            math.sin(PANEL_NORMAL_ANGLE_FROM_UP) * math.cos(offset),
            math.sin(PANEL_NORMAL_ANGLE_FROM_UP) * math.sin(offset),
            math.cos(PANEL_NORMAL_ANGLE_FROM_UP),
        ])
        world_normal = target_rot @ local_normal
        poses.append((panel_pos, world_normal))
    return poses


def spin_hz_for_speed(speed, speed_min, speed_max):
    if speed_max <= speed_min:
        return SPIN_HZ_AT_MIN_SPEED
    frac = (speed - speed_min) / (speed_max - speed_min)
    return SPIN_HZ_AT_MIN_SPEED + frac * (SPIN_HZ_AT_MAX_SPEED - SPIN_HZ_AT_MIN_SPEED)


class LaunchTree:
    """
    Launches a command as its own process group; SIGINT (then SIGKILL) the whole group on stop().

    Mirrors run_localization_drift_tests.py's LaunchTree -- see that for
    the orphan-process rationale.
    """

    def __init__(self, name, cmd, log_path):
        self.name = name
        self.cmd = cmd
        self.log_path = log_path
        self.proc = None
        self.log_file = None

    def start(self):
        self.log_file = open(self.log_path, 'w')
        self.proc = subprocess.Popen(
            self.cmd, stdout=self.log_file, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(f'[{self.name}] started pid={self.proc.pid} log={self.log_path} '
              f'cmd={shlex.join(self.cmd)}')

    def stop(self, timeout=15.0):
        if self.proc is None or self.proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(self.proc.pid)
        except ProcessLookupError:
            return
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                self.log_file.close()
                return
            time.sleep(0.2)
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait(timeout=10)
        self.log_file.close()


class ShotHitSampler(Node):
    """
    Subscribes the sim, ground-truth and cv_target topics the scorer needs.

    /sim/raw_odom + /sim/raw_joint_states give the muzzle FK,
    /target/ground_truth_odom the impact truth, and
    /dji_serial_bridge/cv_target the aim points that carry the fire
    decision.

    Each CVTarget with fire=True becomes one pending shot, resolved
    once ground-truth data at/after its estimated impact time arrives.
    """

    def __init__(self, hit_radius, marker_lifetime_s=5.0):
        # use_sim_time, or marker headers get stamped with wall-clock time
        # while the rest of the stack (sim, amcl's map->odom TF) runs on
        # sim time -- rviz then can't resolve the marker's TF at its
        # timestamp and silently drops it (this was why shot markers
        # never appeared).
        super().__init__(
            'shot_hit_test_sampler',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
            automatically_declare_parameters_from_overrides=True)
        self.hit_radius = hit_radius
        self.marker_lifetime = Duration(seconds=marker_lifetime_s).to_msg()

        self._root_pos = None
        self._root_rot = None
        self._root_frame_id = None
        self._head_yaw = 0.0
        self._head_pitch = 0.0
        self._target_pos = None
        self._truth_history = []  # [(stamp_s, pos, yaw)], last TRUTH_HISTORY_S
        self._target_rot = None

        self._pending_shots = []
        self._unlaunched = []  # [exit_time] of shots still inside FIRE_LATENCY_S
        self.shots_fired = 0
        self.hits = 0
        self.miss_distances = []
        self._shot_marker_id = 0

        self.create_subscription(Odometry, '/sim/raw_odom', self._on_root_odom, 10)
        self.create_subscription(
            JointState, '/sim/raw_joint_states', self._on_joint_states, 10)
        self.create_subscription(
            Odometry, '/target/ground_truth_odom', self._on_target_odom, 10)
        # Best-effort, matching mcb_relay's cv_target publisher.
        self.create_subscription(
            CVTarget, '/dji_serial_bridge/cv_target', self._on_cv_target,
            qos_profile_sensor_data)
        # So each shot's straight-line path is visible in rviz
        # (cv_target.rviz's ShotMarkers display) as it's resolved --
        # green = hit, red = miss. Not used for hit/miss judging itself,
        # purely a visualization aid.
        self.marker_pub = self.create_publisher(MarkerArray, '/shot_markers', 10)
        self.create_timer(1.0 / 30.0, self._publish_in_flight)

    @staticmethod
    def _stamp_s(header_stamp):
        return header_stamp.sec + header_stamp.nanosec / 1e9

    def _on_root_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._root_pos = np.array([p.x, p.y, p.z])
        self._root_rot = _rotation_from_quaternion(q.x, q.y, q.z, q.w)
        self._root_frame_id = msg.header.frame_id

    def _on_joint_states(self, msg):
        if 'headlink' in msg.name:
            self._head_yaw = msg.position[msg.name.index('headlink')]
        if 'headpitch' in msg.name:
            self._head_pitch = msg.position[msg.name.index('headpitch')]
        self._launch_due(self._stamp_s(msg.header.stamp))

    def _launch_due(self, now_s):
        """Launch commanded shots whose exit time has come, from the muzzle pose then."""
        due = [t for t in self._unlaunched if t <= now_s]
        if not due or self._root_pos is None or self._target_pos is None:
            return
        self._unlaunched = [t for t in self._unlaunched if t > now_s]
        muzzle_pos, aim_dir = self._muzzle_pose()
        shot_range = float(np.linalg.norm(self._target_pos - muzzle_pos))
        for exit_time in due:
            # Resolved once truth covers the latest plausible impact; each
            # panel is then judged at its own arrival time (see _resolve_pending).
            self._pending_shots.append({
                'fire_time': exit_time,
                'impact_time': exit_time + (shot_range + 1.0) / MUZZLE_SPEED,
                'muzzle_pos': muzzle_pos,
                'aim_dir': aim_dir,
                'shot_range': shot_range,
            })

    def _muzzle_pose(self):
        """
        Compute world (position, unit forward direction) of the muzzle via the fixed FK chain.

        No TF lookup -- see module docstring.
        """
        t_root = _transform(self._root_rot, self._root_pos)
        t_body = t_root @ _T_FASTENED_2
        t_headlink = _transform(
            _HEADLINK_ORIGIN_R @ _rotation_axis_angle(_HEADLINK_AXIS, self._head_yaw),
            _HEADLINK_ORIGIN_T)
        t_head = t_body @ t_headlink
        t_headpitch = _transform(
            _HEADPITCH_ORIGIN_R @ _rotation_axis_angle(_HEADPITCH_AXIS, self._head_pitch),
            _HEADPITCH_ORIGIN_T)
        t_muzzle = t_head @ t_headpitch
        pos = t_muzzle[:3, 3]
        # Camera-local +X is forward, not +Z -- see cv_target_emulator.py's
        # REP-103 conversion (rel_cam[0] is called 'fwd'). This was wrong
        # (used +Z) and is the likely cause of the earlier ~2.85m
        # near-constant miss distance across every speed.
        forward = t_muzzle[:3, :3] @ np.array([1.0, 0.0, 0.0])
        return pos, forward / np.linalg.norm(forward)

    def _on_target_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._target_pos = np.array([p.x, p.y, p.z])
        self._target_rot = _rotation_from_quaternion(q.x, q.y, q.z, q.w)
        stamp = self._stamp_s(msg.header.stamp)
        yaw = 2.0 * math.atan2(q.z, q.w)  # target_driver publishes yaw-only
        if self._truth_history:
            prev_yaw = self._truth_history[-1][2]
            yaw = prev_yaw + math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))
        self._truth_history.append((stamp, self._target_pos.copy(), yaw))
        self._truth_history = [h for h in self._truth_history if stamp - h[0] <= TRUTH_HISTORY_S]
        self._resolve_pending(stamp)

    def _truth_at(self, t):
        """Target (position, rotation) linearly interpolated to time t from the history."""
        hist = self._truth_history
        if t <= hist[0][0]:
            _, pos, yaw = hist[0]
        elif t >= hist[-1][0]:
            _, pos, yaw = hist[-1]
        else:
            i = next(k for k in range(1, len(hist)) if hist[k][0] >= t)
            (t0, p0, y0), (t1, p1, y1) = hist[i - 1], hist[i]
            a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            pos, yaw = p0 + a * (p1 - p0), y0 + a * (y1 - y0)
        return pos, _rotation_from_rpy(0.0, 0.0, yaw)

    def _on_cv_target(self, msg):
        if not msg.fire:
            return
        if self._root_pos is None or self._target_pos is None:
            return  # no muzzle pose / ground truth yet to evaluate against

        # The projectile leaves FIRE_LATENCY_S after the commanded fire
        # time, aimed wherever the muzzle points then (_launch_due).
        self.shots_fired += 1
        self._unlaunched.append(
            self._stamp_s(msg.header.stamp) + msg.delay_ms / 1000.0 + FIRE_LATENCY_S)

    def _resolve_pending(self, now_s):
        still_pending = []
        for shot in self._pending_shots:
            if now_s < shot['impact_time']:
                still_pending.append(shot)
                continue

            # Check all 4 panels, not just the chassis center -- a shot can
            # be close to the chassis but still miss every physical panel,
            # or land on whichever panel happens to be facing away. Each
            # panel is judged against interpolated truth at the moment the
            # projectile reaches it along the ray: at 1-2Hz spin, the
            # nearest truth sample (60Hz) is up to ~9 deg of yaw off.
            pos, rot = self._truth_at(shot['fire_time'] + shot['shot_range'] / MUZZLE_SPEED)
            arrivals = []
            for panel_pos, _ in _panel_poses(pos, rot):
                along = float(np.dot(panel_pos - shot['muzzle_pos'], shot['aim_dir']))
                arrivals.append(shot['fire_time'] + max(along, 0.0) / MUZZLE_SPEED)
            # Nearest panel among those facing the muzzle (within the front
            # 145 degrees, PANEL_EXPOSURE_HALF_ANGLE); the far side can line
            # up behind a hit but could never register one. Falls back to the
            # nearest panel of any facing only if none face the shooter.
            nearest_facing = nearest_any = None
            for k, t_arrive in enumerate(arrivals):
                pos, rot = self._truth_at(t_arrive)
                panel_pos, panel_normal = _panel_poses(pos, rot)[k]
                to_panel = panel_pos - shot['muzzle_pos']
                along = float(np.dot(to_panel, shot['aim_dir']))
                closest_on_ray = shot['muzzle_pos'] + along * shot['aim_dir']
                miss = float(np.linalg.norm(panel_pos - closest_on_ray))
                to_muzzle = shot['muzzle_pos'] - panel_pos
                to_muzzle_norm = to_muzzle / (np.linalg.norm(to_muzzle) + 1e-9)
                incidence = math.acos(np.clip(np.dot(panel_normal, to_muzzle_norm), -1.0, 1.0))
                cand = (miss, closest_on_ray, panel_pos)
                if nearest_any is None or miss < nearest_any[0]:
                    nearest_any = cand
                if incidence <= PANEL_EXPOSURE_HALF_ANGLE and (
                        nearest_facing is None or miss < nearest_facing[0]):
                    nearest_facing = cand

            best_miss, best_ray_pt, best_panel_pt = nearest_facing or nearest_any
            hit = nearest_facing is not None and best_miss <= self.hit_radius
            self.miss_distances.append(best_miss)
            if hit:
                self.hits += 1
            self._publish_shot_marker(hit, best_ray_pt, best_panel_pt)
        self._pending_shots = still_pending

    def _publish_in_flight(self):
        """Draw every airborne shot as a yellow dot at its current position."""
        now = self.get_clock().now().nanoseconds / 1e9
        marker = Marker()
        marker.header.frame_id = self._root_frame_id or 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'in_flight'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0
        for shot in self._pending_shots:
            flown = MUZZLE_SPEED * (now - shot['fire_time'])
            if 0.0 <= flown <= shot['shot_range']:
                p = shot['muzzle_pos'] + shot['aim_dir'] * flown
                marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        marker.action = Marker.ADD if marker.points else Marker.DELETE
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def _publish_shot_marker(self, hit, ray_pt, panel_pt):
        """
        Sphere where the shot passed nearest a facing panel (green hit, red miss).

        A miss adds an arrow from there to that panel as the shot reached it;
        its length is the miss distance.
        """
        end_pt = Point(x=float(ray_pt[0]), y=float(ray_pt[1]), z=float(ray_pt[2]))
        header_frame = self._root_frame_id or 'odom'
        stamp = self.get_clock().now().to_msg()

        marker = Marker()
        marker.header.frame_id = header_frame
        marker.header.stamp = stamp
        marker.ns = 'shots'
        marker.id = self._shot_marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = end_pt
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.03
        marker.color.a = 1.0
        if hit:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        marker.lifetime = self.marker_lifetime
        markers = [marker]

        if not hit:
            arrow = Marker()
            arrow.header.frame_id = header_frame
            arrow.header.stamp = stamp
            arrow.ns = 'miss_to_panel'
            arrow.id = self._shot_marker_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            c = panel_pt
            arrow.points = [end_pt, Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))]
            # shaft diameter, head diameter, head length
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.006, 0.02, 0.03
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.5, 0.0, 1.0
            arrow.lifetime = self.marker_lifetime
            markers.append(arrow)

        self._shot_marker_id += 1
        self.marker_pub.publish(MarkerArray(markers=markers))

    def reset_score(self):
        """Forget every shot so far, scored or in flight; ground truth is kept."""
        self.shots_fired = 0
        self.hits = 0
        self.miss_distances = []
        self._pending_shots = []
        self._unlaunched = []

    def finish(self):
        """
        Finish sampling, dropping shots still pending rather than counting them as misses.

        Pending means the impact time has not been reached yet, so there's
        no ground-truth sample to judge them against.
        """
        dropped = len(self._pending_shots) + len(self._unlaunched)
        self._pending_shots = []
        self._unlaunched = []
        return dropped

    def spin_for(self, seconds):
        """
        Spin for `seconds` of sim time, so a real-time factor under 1 doesn't shorten the case.

        Wall-clock cap of 3x (at least +5s) in case /clock stalls, as in the
        drift suite's drive().
        """
        wall_end = time.monotonic() + max(3.0 * seconds, seconds + 5.0)
        start = None
        while time.monotonic() < wall_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self.get_clock().now().nanoseconds / 1e9
            if start is None and now > 0.0:
                start = now
            if start is not None and now - start >= seconds:
                return
        print(f'[spin_for] wall-clock cap hit before {seconds}s of sim time elapsed')

    def wait_until(self, predicate, timeout, description):
        """
        Spin until predicate() is true or timeout (s) elapses.

        So the sampling window starts once the stack is actually
        publishing instead of after a guessed fixed sleep. Falls back to
        the full timeout (and a printed warning) if the predicate never
        fires -- the caller still proceeds rather than hanging forever on
        a genuinely broken launch.
        """
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f'[wait_until] timed out after {timeout}s waiting for: {description}')
        return False

    def nodes_up(self, *names):
        live = {n for n, _ in self.get_node_names_and_namespaces()}
        return all(n in live for n in names)


class CvStack:
    """
    The sim plus the production CV pipeline, launched once and reused for every case.

    Between cases only target_driver's target_speed/spin_hz change (it reads
    both every tick), so the target keeps its place and the stack its state.
    """

    def __init__(self, headless, log_dir):
        sim_cmd = ['ros2', 'launch', 'sim', 'sim.launch.py', 'spawn_target:=true',
                   'target_speed:=0.0', 'target_spin_hz:=0.0']
        if headless:
            sim_cmd += ['gui:=false', 'rviz:=false']
        else:
            sim_cmd.append(f'rviz_config:={CV_RVIZ_CONFIG}')

        def log(name):
            return os.path.join(log_dir, f'{name}.log')

        def node(name, binary):
            return LaunchTree(name, [binary, '--ros-args', '-p', 'use_sim_time:=true'],
                              log(name))

        self.sim = LaunchTree('sim', sim_cmd, log('sim'))
        # Full production pipeline standalone (auto.launch.py's CV node set
        # without dji_serial_bridge/lidar/localization): target_selector groups
        # and picks from the emulator's panel_detections, target_tracker
        # estimates the spin centre in odom, point_to_cv_target solves the lead
        # and emits /cv/target, mcb_relay forwards it to
        # /dji_serial_bridge/cv_target.
        self.pipeline = [
            node('target_selector', TARGET_SELECTOR_BIN),
            node('target_tracker', TARGET_TRACKER_BIN),
            LaunchTree('point_to_cv_target',
                       [POINT_TO_CV_TARGET_BIN, '--ros-args', '-p', 'use_sim_time:=true',
                        '-p', f'cv_target_publish_rate_hz:={TEST_FIRE_HZ}',
                        '-p', f'fire_rate_hz:={TEST_FIRE_HZ + 10.0}'],
                       log('point_to_cv_target')),
            node('mcb_relay', MCB_RELAY_BIN),
        ]
        # TF chain: sim runs no robot_state_publisher, and target_tracker's and
        # point_to_cv_target's TF lookups need one. The enable_*:=false args
        # skip auto.launch.py's own copies of the three CV nodes above, which
        # would double-publish; localization_mode:=none skips map_server/amcl.
        self.robot_tf = LaunchTree(
            'robot_tf',
            ['ros2', 'launch', 'thornbots_pkg', 'auto.launch.py',
             'real_hardware:=false', 'localization_mode:=none', 'use_ekf:=false',
             'enable_cv_target_bridge:=false', 'enable_target_selector:=false',
             'enable_target_tracker:=false'],
            log('robot_tf'))
        self.node = rclpy.create_node(
            'shot_hit_stack',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._set_params = self.node.create_client(
            SetParameters, '/target_driver/set_parameters')

    def start(self):
        probe = ShotHitSampler(hit_radius=0.0)
        try:
            self.sim.start()
            probe.wait_until(
                lambda: probe._root_pos is not None and probe._target_pos is not None,
                timeout=30.0,
                description='/sim/raw_odom + /target/ground_truth_odom publishing')
            self.robot_tf.start()
            for tree in self.pipeline:
                tree.start()
            ready = ['point_to_cv_target', 'mcb_relay', 'robot_state_publisher',
                     'target_selector', 'target_tracker']
            probe.wait_until(lambda: probe.nodes_up(*ready), timeout=15.0,
                             description=f'{", ".join(ready)} nodes up')
        finally:
            probe.destroy_node()

    def set_target_motion(self, speed, spin_hz):
        """Set target_driver's speed and spin rate; raise if it doesn't take them."""
        if not self._set_params.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/target_driver/set_parameters not available')
        req = SetParameters.Request(parameters=[
            Parameter('target_speed', Parameter.Type.DOUBLE, float(speed)).to_parameter_msg(),
            Parameter('spin_hz', Parameter.Type.DOUBLE, float(spin_hz)).to_parameter_msg(),
        ])
        future = self._set_params.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not all(r.successful for r in result.results):
            raise RuntimeError(f'target_driver rejected speed={speed} spin={spin_hz}')
        print(f'[stack] target_driver set to speed={speed} m/s, spin={spin_hz:.2f} Hz')

    def stop(self):
        for tree in reversed(self.pipeline):
            tree.stop()
        self.robot_tf.stop()
        self.sim.stop()
        self.node.destroy_node()


def run_case(stack, speed, spin_hz, duration, hit_radius):
    """
    Switch the target to (speed, spin_hz), let the tracker settle, then score.

    Shots fired during the SETTLE_S window are discarded.
    """
    stack.set_target_motion(speed, spin_hz)
    # At TEST_FIRE_HZ a 1s lifetime keeps ~40 markers; fast targets scatter
    # shots across the view, so theirs expire sooner still.
    sampler = ShotHitSampler(hit_radius=hit_radius,
                             marker_lifetime_s=1.0 / max(1.0, speed))
    try:
        sampler.spin_for(SETTLE_S)
        sampler.reset_score()
        sampler.spin_for(duration)
    finally:
        dropped = sampler.finish()
        sampler.destroy_node()
    return sampler, dropped


def score(sampler, duration):
    """
    Return (score, hits_per_expected, keep_up) for one case.

    Expects TEST_FIRE_HZ * duration shots. score is the mean of the hit rate
    and hits per expected shot, so hitting often counts as much as hitting
    accurately, and firing below TEST_FIRE_HZ costs points.
    """
    expected = TEST_FIRE_HZ * duration
    hit_rate = sampler.hits / sampler.shots_fired if sampler.shots_fired else 0.0
    hits_per_expected = sampler.hits / expected
    return (0.5 * (hit_rate + hits_per_expected), hits_per_expected,
            sampler.shots_fired / expected)


def summarize(label, sampler, dropped, duration):
    hit_pct = (100.0 * sampler.hits / sampler.shots_fired) if sampler.shots_fired else float('nan')
    miss = sampler.miss_distances
    miss_mean = sum(miss) / len(miss) if miss else float('nan')
    total, per_expected, keep_up = score(sampler, duration)
    print(
        f'{label:30s} | score={total:5.1%} | '
        f'shots={sampler.shots_fired:4d}/{int(TEST_FIRE_HZ * duration)} ({keep_up:5.1%}) | '
        f'hits={sampler.hits:4d} ({hit_pct:5.1f}% of fired, {per_expected:5.1%} of expected) | '
        f'miss mean={miss_mean:6.3f} m | dropped={dropped}'
    )
    return total
