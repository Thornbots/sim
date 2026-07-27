#!/usr/bin/env python3
"""
Black-box shot-hit test: checks sentry_pkg's fire decisions against ground
truth without knowing anything about how sentry_pkg predicts (see
sim/README.md's ## Notes for why -- the test only consumes the final
FireCommand on /dji_serial_bridge/fire_command, the same topic that would
actually reach the real launcher hardware per mcb_relay.py's "sole relay"
design).

For each fire_command received: computes the muzzle pose (from
/sim/raw_odom + /sim/raw_joint_states, the same fixed FK chain
cv_target_emulator.py uses -- duplicated here rather than imported so this
test doesn't silently start passing/failing from an unrelated emulator
refactor), simulates a straight-line 25 m/s projectile (muzzle-speed cap
per ARCC_2026_SENTRY_CONTEXT.md), and checks the shot against all 4 of the
target's armor panels (also duplicated from cv_target_emulator.py's
layout) at estimated impact time: a hit needs BOTH the flight path to pass
within --hit-radius of a panel's 0.1m x 0.1m face AND to arrive within
that panel's 145-degree front exposure cone (ARCC_2026_SENTRY_CONTEXT.md)
-- geometrically on-target from behind the panel still misses.

Requires mcb_relay.py to actually relay a FireCommand onto
/dji_serial_bridge/fire_command (wired 2026-07-27) and an upstream
firing-logic node to publish one (not yet written as of 2026-07-27).
Until that node exists this reports zero shots for every speed -- a real
result (no fire commands were ever sent), not a bug in this script.

Usage: python3 run_shot_hit_tests.py [--speeds 0.5 1 2 4]
[--duration 12.0] [--hit-radius 0.05] [--headless] [--skip-stationary]

Runs a completely stationary (speed=0, spin=0) baseline case before the
speed sweep, unless --skip-stationary -- a working pipeline should hit
this trivially, so a miss there means the harness itself is broken, not
that tracking/prediction is hard.
"""
import argparse
import math
import os
import shlex
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from dji_serial_bridge.msg import FireCommand


MUZZLE_SPEED = 25.0  # m/s -- ARCC_2026_SENTRY_CONTEXT.md's muzzle-speed cap

DEFAULT_SPEEDS = [0.5, 1.0, 2.0, 4.0]
# Spin rate swept inversely to speed, spanning ARCC's documented
# "typically 1-2 Hz" range (ARCC_2026_SENTRY_CONTEXT.md).
SPIN_HZ_AT_MIN_SPEED = 2.0
SPIN_HZ_AT_MAX_SPEED = 1.0

POINT_TO_CV_TARGET_BIN = (
    '/workspaces/isaac_ros-dev/install/sentry_pkg/lib/sentry_pkg/point_to_cv_target'
)
MCB_RELAY_BIN = (
    '/workspaces/isaac_ros-dev/install/sentry_pkg/lib/sentry_pkg/mcb_relay'
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
# "front 145 of the panel's exposure surface must stay unblocked"
# (ARCC_2026_SENTRY_CONTEXT.md line 237) -- a shot arriving outside this
# cone couldn't have registered on the real panel even if it's
# geometrically on-target, so the angle check matters as much as distance.
PANEL_EXPOSURE_HALF_ANGLE = math.radians(145.0 / 2.0)
# S122 (ARCC_2026_SENTRY_CONTEXT.md "Mounting angle"): panel outward normal
# makes a 75-degree angle with straight-up, i.e. canted ~15 degrees off
# pure-horizontal (90 degrees would be flush-vertical).
PANEL_NORMAL_ANGLE_FROM_UP = math.radians(75.0)


def _panel_poses(target_pos, target_rot):
    """World (position, outward_normal_unit_vector) for each of the 4 armor
    panels -- mirrors cv_target_emulator.py's _panel_poses exactly.
    Position offset stays in the horizontal chassis plane; the outward
    normal is canted per S122, not flush-horizontal (z=0)."""
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
    """Launches a command as its own process group; SIGINT (then SIGKILL)
    the whole group on stop(). Mirrors run_localization_drift_tests.py's
    LaunchTree -- see that for the orphan-process rationale."""

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
    """Subscribes /sim/raw_odom + /sim/raw_joint_states (muzzle FK),
    /target/ground_truth_odom (impact truth), and
    /dji_serial_bridge/fire_command (shot events). Each fire_command with
    fire=True becomes one pending shot, resolved once ground-truth data
    at/after its estimated impact time arrives."""

    def __init__(self, hit_radius):
        super().__init__('shot_hit_test_sampler')
        self.hit_radius = hit_radius

        self._root_pos = None
        self._root_rot = None
        self._head_yaw = 0.0
        self._head_pitch = 0.0
        self._target_pos = None
        self._target_rot = None

        self._pending_shots = []
        self.shots_fired = 0
        self.hits = 0
        self.miss_distances = []

        self.create_subscription(Odometry, '/sim/raw_odom', self._on_root_odom, 10)
        self.create_subscription(
            JointState, '/sim/raw_joint_states', self._on_joint_states, 10)
        self.create_subscription(
            Odometry, '/target/ground_truth_odom', self._on_target_odom, 10)
        self.create_subscription(
            FireCommand, '/dji_serial_bridge/fire_command', self._on_fire_command, 10)

    @staticmethod
    def _stamp_s(header_stamp):
        return header_stamp.sec + header_stamp.nanosec / 1e9

    def _on_root_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._root_pos = np.array([p.x, p.y, p.z])
        self._root_rot = _rotation_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_joint_states(self, msg):
        if 'headlink' in msg.name:
            self._head_yaw = msg.position[msg.name.index('headlink')]
        if 'headpitch' in msg.name:
            self._head_pitch = msg.position[msg.name.index('headpitch')]

    def _muzzle_pose(self):
        """World (position, unit forward direction) of the muzzle via the
        fixed FK chain, no TF lookup -- see module docstring."""
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
        self._resolve_pending(self._stamp_s(msg.header.stamp))

    def _on_fire_command(self, msg):
        if not msg.fire:
            return
        if self._root_pos is None or self._target_pos is None:
            return  # no muzzle pose / ground truth yet to evaluate against

        fire_time = self._stamp_s(msg.header.stamp) + msg.delay_ms / 1000.0
        muzzle_pos, aim_dir = self._muzzle_pose()

        # First-order flight-time estimate: distance to the target's
        # position *at fire time*, at the muzzle speed cap. Does not
        # re-converge on the target's motion during flight -- this test
        # checks the fire decision's lead against ground truth, not full
        # ballistics, so a single-step estimate is enough.
        flight_time = float(np.linalg.norm(self._target_pos - muzzle_pos)) / MUZZLE_SPEED
        impact_time = fire_time + flight_time

        self.shots_fired += 1
        self._pending_shots.append({
            'impact_time': impact_time,
            'muzzle_pos': muzzle_pos,
            'aim_dir': aim_dir,
        })

    def _resolve_pending(self, now_s):
        still_pending = []
        for shot in self._pending_shots:
            if now_s < shot['impact_time']:
                still_pending.append(shot)
                continue

            # Check all 4 panels, not just the chassis center -- a shot can
            # be close to the chassis but still miss every physical panel,
            # or land on whichever panel happens to be facing away.
            panels = _panel_poses(self._target_pos, self._target_rot)
            best_miss = None
            hit = False
            for panel_pos, panel_normal in panels:
                to_panel = panel_pos - shot['muzzle_pos']
                along = float(np.dot(to_panel, shot['aim_dir']))
                closest_on_ray = shot['muzzle_pos'] + along * shot['aim_dir']
                miss = float(np.linalg.norm(panel_pos - closest_on_ray))
                if best_miss is None or miss < best_miss:
                    best_miss = miss

                # Exposure-cone check: the shot must also arrive from
                # within the panel's front 145 degrees, or it couldn't have
                # registered even if geometrically on-target (see
                # PANEL_EXPOSURE_HALF_ANGLE).
                to_muzzle = shot['muzzle_pos'] - panel_pos
                to_muzzle_norm = to_muzzle / (np.linalg.norm(to_muzzle) + 1e-9)
                incidence = math.acos(np.clip(np.dot(panel_normal, to_muzzle_norm), -1.0, 1.0))
                if miss <= self.hit_radius and incidence <= PANEL_EXPOSURE_HALF_ANGLE:
                    hit = True

            self.miss_distances.append(best_miss)
            if hit:
                self.hits += 1
        self._pending_shots = still_pending

    def finish(self):
        """Shots still pending at sampling end (impact time not yet
        reached) are dropped, not counted as misses -- there's no
        ground-truth sample to judge them against."""
        dropped = len(self._pending_shots)
        self._pending_shots = []
        return dropped

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def run_one_speed(speed, spin_hz, duration, headless, log_dir, hit_radius):
    sim_cmd = [
        'ros2', 'launch', 'sim', 'sim.launch.py',
        'spawn_target:=true', f'target_speed:={speed}',
        f'target_spin_hz:={spin_hz}',
    ]
    if headless:
        sim_cmd += ['gui:=false', 'rviz:=false']
    else:
        sim_cmd.append(f'rviz_config:={CV_RVIZ_CONFIG}')
    sim = LaunchTree(
        'sim', sim_cmd, os.path.join(log_dir, f'sim_{speed}_spin{spin_hz:.2f}.log'))

    cv_bridge = LaunchTree(
        'point_to_cv_target',
        [POINT_TO_CV_TARGET_BIN, '--ros-args', '-p', 'use_sim_time:=true'],
        os.path.join(log_dir, f'point_to_cv_target_{speed}_spin{spin_hz:.2f}.log'),
    )
    mcb_relay = LaunchTree(
        'mcb_relay',
        [MCB_RELAY_BIN, '--ros-args', '-p', 'use_sim_time:=true'],
        os.path.join(log_dir, f'mcb_relay_{speed}_spin{spin_hz:.2f}.log'),
    )

    # Visualization only: sim itself intentionally runs no TF/
    # robot_state_publisher (nodes compute their own FK -- see README.md),
    # so nothing feeds cv_target.rviz's RobotModel display without this.
    # real_hardware:=false + localization_mode:=none skips SLAM/AMCL and
    # the real-hardware-only mcb_relay/point_to_cv_target it would
    # otherwise launch (no conflict with the ones started above); it just
    # gets robot_state_publisher + pose_translator + odom_tf_broadcaster
    # onto the graph, sourced from sim's own /pose (pose_emulator) and
    # /sim/raw_joint_states. Skipped entirely when headless -- nothing to
    # look at, not worth the extra process tree.
    robot_tf = LaunchTree(
        'robot_tf',
        ['ros2', 'launch', 'sentry_pkg', 'auto.launch.py',
         'real_hardware:=false', 'localization_mode:=none', 'use_ekf:=false'],
        os.path.join(log_dir, f'robot_tf_{speed}_spin{spin_hz:.2f}.log'),
    )

    rclpy.init()
    sampler = ShotHitSampler(hit_radius=hit_radius)
    try:
        sim.start()
        sampler.spin_for(6.0)
        cv_bridge.start()
        mcb_relay.start()
        if not headless:
            robot_tf.start()
        sampler.spin_for(2.0)

        sampler.spin_for(duration)
    finally:
        dropped = sampler.finish()
        cv_bridge.stop()
        mcb_relay.stop()
        if not headless:
            robot_tf.stop()
        sim.stop()
        sampler.destroy_node()
        rclpy.shutdown()

    return sampler, dropped


def summarize(speed, spin_hz, sampler, dropped):
    hit_pct = (100.0 * sampler.hits / sampler.shots_fired) if sampler.shots_fired else float('nan')
    miss = sampler.miss_distances
    miss_mean = sum(miss) / len(miss) if miss else float('nan')
    miss_max = max(miss) if miss else float('nan')
    print(
        f"speed={speed:5.2f} m/s | spin={spin_hz:4.2f} Hz | "
        f"shots={sampler.shots_fired:4d} | hits={sampler.hits:4d} ({hit_pct:5.1f}%) | "
        f"miss dist mean={miss_mean:6.3f} max={miss_max:6.3f} m | dropped={dropped}"
    )
    return sampler.shots_fired > 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds', type=float, nargs='+', default=DEFAULT_SPEEDS)
    parser.add_argument('--duration', type=float, default=12.0,
                         help='Seconds of steady-state sampling per speed (wall-clock)')
    parser.add_argument('--hit-radius', type=float, default=DEFAULT_HIT_RADIUS,
                         help='Perpendicular miss distance (m) still counted as a hit')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--skip-stationary', action='store_true',
                         help='Skip the speed=0/spin=0 baseline case run before the sweep')
    parser.add_argument('--only-stationary', action='store_true',
                         help='Run only the speed=0/spin=0 baseline case, skip the speed sweep entirely')
    parser.add_argument('--log-dir', default='/tmp/shot_hit_test_logs')
    args = parser.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    speed_min, speed_max = min(args.speeds), max(args.speeds)

    any_shots = False

    if not args.skip_stationary:
        # Completely stationary (speed=0, spin=0) baseline, not part of the
        # inverse speed<->spin sweep -- a real prediction/aim pipeline
        # should hit this trivially, so a miss here means the harness
        # itself is broken, not that tracking/prediction is hard.
        print('\n=== stationary baseline: speed=0.00 m/s, spin=0.00 Hz ===')
        sampler, dropped = run_one_speed(
            0.0, 0.0, args.duration, args.headless, args.log_dir, args.hit_radius)
        got_shots = summarize(0.0, 0.0, sampler, dropped)
        any_shots = any_shots or got_shots
        time.sleep(1.0)

    if not args.only_stationary:
        for speed in args.speeds:
            spin_hz = spin_hz_for_speed(speed, speed_min, speed_max)
            print(f'\n=== speed={speed} m/s, spin={spin_hz:.2f} Hz ===')
            sampler, dropped = run_one_speed(
                speed, spin_hz, args.duration, args.headless, args.log_dir, args.hit_radius)
            got_shots = summarize(speed, spin_hz, sampler, dropped)
            any_shots = any_shots or got_shots
            time.sleep(1.0)

    if not any_shots:
        print('\nNo shots observed at any speed -- sentry_pkg is not yet publishing on '
              '/dji_serial_bridge/fire_command (expected until its firing-logic node exists).')
        sys.exit(1)
    print('\nDone.')


if __name__ == '__main__':
    main()
