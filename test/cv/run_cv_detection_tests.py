#!/usr/bin/env python3
"""
Speed-sweep test/metric script for the CV target simulation
(target_driver.py + cv_target_emulator.py -> sentry_pkg's
point_to_cv_target.py, unmodified). For each swept speed: launches sim
headless with spawn_target:=true, runs point_to_cv_target against the
emulator's output, and reports tracking error vs. speed.

Frustum-dwell guard (hard requirement, not advisory -- see
sim/README.md's ## Notes): if any swept speed produces fewer than
--min-dwell consecutive in-frustum roi_point samples per transit, that
speed's numbers reflect the EMA velocity filter's warm-up lag rather than
real tracking degradation, and the run FAILS for that speed.

Usage: python3 run_cv_detection_tests.py [--speeds 0.5 1 2 4 6 8]
[--duration 8.0] [--min-dwell 10] [--headless] [--keep-running]
"""
import argparse
import math
import os
import shlex
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from dji_serial_bridge.msg import CVTarget


DEFAULT_SPEEDS = [0.5, 1.0, 2.0, 4.0, 6.0, 8.0]
# Built binary invoked by absolute path, bypassing `ros2 run`/`ros2 pkg
# prefix` package-name resolution -- see README.md's ## Notes: that
# resolution can silently pick the stale image-baked /workspaces/ros2_ws
# clone instead of this repo's bind-mounted build, which may lack this
# executable entirely.
POINT_TO_CV_TARGET_BIN = (
    '/workspaces/isaac_ros-dev/install/sentry_pkg/lib/sentry_pkg/point_to_cv_target'
)


class LaunchTree:
    """Launches a command as its own process group; SIGINT (then SIGKILL)
    the whole group on stop(). Mirrors
    run_localization_drift_tests.py's LaunchTree -- see that file for the
    orphan-process rationale."""

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


class CvTrackingSampler(Node):
    """Subscribes roi_point (dwell-run counting), /sim/raw_odom (root
    pose, for an independent camera-forward/-left approximation),
    /target/ground_truth_odom, and /cv/target (tracking error vs. that
    independent ground-truth projection).

    Approximation: this projection uses ROOT's yaw plus a manually-applied
    HEADPITCH_YAW_OFFSET, not cv_target_emulator's own FK matrix -- so it
    can catch a sign/rotation bug there independently rather than just
    re-deriving the same answer. The offset's sign was verified (not
    assumed) 2026-07-27 by comparing pos_err with +0.38885, -0.38885, and
    0 applied: -0.38885 collapsed mean pos_err to ~0.13m (near the 0.03m
    noise floor) vs. +0.38885's ~2.45m and 0's ~1.22m -- only -0.38885
    (this class's HEADPITCH_YAW_OFFSET) is consistent with
    cv_target_emulator's FK being correct. See README.md's ## Notes."""

    # Matches sentry.urdf.xacro's headpitch joint origin yaw, and
    # cv_target_emulator.py's _HEADPITCH_ORIGIN_R -- see the sign
    # verification in the class docstring above.
    HEADPITCH_YAW_OFFSET = -0.38885

    def __init__(self, target_speed, max_gap_s=0.05):
        super().__init__('cv_detection_test_sampler')
        self.target_speed = target_speed
        self.max_gap_s = max_gap_s

        self._last_roi_time = None
        self._dwell_run = 0
        self._first_run_pending = True  # first/last run may be clipped by
        # the sampling window's wall-clock start/stop, not a real transit
        # boundary -- see finish_dwell()'s docstring.
        self.dwell_counts = []  # interior (both-ends-bounded) runs only
        self.boundary_dwell_counts = []  # first + last run, reported but not enforced

        self._root_xy_yaw = None
        self._target_xy = None

        self.pos_errors = []
        self.vel_errors = []

        self.create_subscription(PointStamped, 'roi_point', self._on_roi_point, 10)
        self.create_subscription(Odometry, '/sim/raw_odom', self._on_root_odom, 10)
        self.create_subscription(
            Odometry, '/target/ground_truth_odom', self._on_target_odom, 10)
        self.create_subscription(
            CVTarget, '/cv/target', self._on_cv_target, qos_profile_sensor_data)

    @staticmethod
    def _stamp_s(header_stamp):
        return header_stamp.sec + header_stamp.nanosec / 1e9

    def _on_roi_point(self, msg):
        t = self._stamp_s(msg.header.stamp)
        if self._last_roi_time is not None and (t - self._last_roi_time) > self.max_gap_s:
            self._close_run()
        self._dwell_run += 1
        self._last_roi_time = t

    def _close_run(self):
        if self._dwell_run <= 0:
            return
        if self._first_run_pending:
            # This run's true start may predate the sampler's own
            # subscription (or the test's wait-for-ready window) -- not a
            # real transit boundary, so it doesn't count toward the guard.
            self.boundary_dwell_counts.append(self._dwell_run)
            self._first_run_pending = False
        else:
            self.dwell_counts.append(self._dwell_run)
        self._dwell_run = 0

    def _on_root_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))
        self._root_xy_yaw = (p.x, p.y, yaw)

    def _on_target_odom(self, msg):
        p = msg.pose.pose.position
        self._target_xy = (p.x, p.y)

    def _on_cv_target(self, msg):
        if msg.confidence <= 0.0 or self._root_xy_yaw is None or self._target_xy is None:
            return

        cv_speed = math.sqrt(msg.v_x ** 2 + msg.v_y ** 2 + msg.v_z ** 2)
        self.vel_errors.append(abs(cv_speed - self.target_speed))

        rx, ry, yaw = self._root_xy_yaw
        yaw += self.HEADPITCH_YAW_OFFSET
        tx, ty = self._target_xy
        dx, dy = tx - rx, ty - ry
        true_fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
        true_left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        # CVTarget is (right, up, forward); right = -left.
        self.pos_errors.append(math.hypot(msg.z - true_fwd, -msg.x - true_left))

    def finish_dwell(self):
        """Call once sampling stops. The run still open at that instant
        was cut off by the wall-clock sampling window, not a genuine
        transit exit -- goes to boundary_dwell_counts like the run
        possibly already in progress when sampling started (see
        _close_run), never counted toward the dwell guard."""
        if self._dwell_run > 0:
            self.boundary_dwell_counts.append(self._dwell_run)
            self._dwell_run = 0
        self._first_run_pending = False

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def run_one_speed(speed, duration, headless, log_dir):
    sim_cmd = [
        'ros2', 'launch', 'sim', 'sim.launch.py',
        f'spawn_target:=true', f'target_speed:={speed}',
        'rviz:=false',
    ]
    if headless:
        sim_cmd.append('gui:=false')
    sim = LaunchTree('sim', sim_cmd, os.path.join(log_dir, f'sim_{speed}.log'))

    cv_bridge = LaunchTree(
        'point_to_cv_target',
        [POINT_TO_CV_TARGET_BIN, '--ros-args', '-p', 'use_sim_time:=true'],
        os.path.join(log_dir, f'point_to_cv_target_{speed}.log'),
    )

    rclpy.init()
    sampler = CvTrackingSampler(target_speed=speed)
    try:
        sim.start()
        # Give sim + the two new nodes time to come up before starting
        # point_to_cv_target, mirroring sim.launch.py's own 2s spawn delay.
        sampler.spin_for(6.0)
        cv_bridge.start()
        sampler.spin_for(2.0)

        sampler.spin_for(duration)
    finally:
        sampler.finish_dwell()
        cv_bridge.stop()
        sim.stop()
        sampler.destroy_node()
        rclpy.shutdown()

    return sampler


def summarize(speed, sampler, min_dwell):
    # Only interior runs (both start and end seen as a real frustum
    # exit, not clipped by the sampling window's own start/stop) count
    # toward the guard -- see CvTrackingSampler._close_run/finish_dwell.
    dwell_counts = sampler.dwell_counts
    min_seen = min(dwell_counts) if dwell_counts else None
    dwell_ok = min_seen is not None and min_seen >= min_dwell

    pos = sampler.pos_errors
    vel = sampler.vel_errors
    pos_mean = sum(pos) / len(pos) if pos else float('nan')
    pos_max = max(pos) if pos else float('nan')
    vel_mean = sum(vel) / len(vel) if vel else float('nan')
    vel_max = max(vel) if vel else float('nan')

    min_str = f"{min_seen:4d}" if min_seen is not None else " n/a"
    print(
        f"speed={speed:5.2f} m/s | samples={len(pos):4d} | "
        f"pos_err mean={pos_mean:6.3f} max={pos_max:6.3f} m | "
        f"vel_err mean={vel_mean:6.3f} max={vel_max:6.3f} m/s | "
        f"dwell transits={len(dwell_counts):2d} min={min_str} "
        f"(boundary runs excluded: {sampler.boundary_dwell_counts}) "
        f"({'OK' if dwell_ok else 'FAIL: below --min-dwell=' + str(min_dwell)})"
    )
    return dwell_ok and len(pos) > 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds', type=float, nargs='+', default=DEFAULT_SPEEDS)
    parser.add_argument('--duration', type=float, default=12.0,
                         help='Seconds of steady-state sampling per speed (wall-clock, '
                              'not sim-time -- must exceed one full bounce period, '
                              '4*half_width/speed, at the SLOWEST swept speed, or that '
                              'speed sees zero interior dwell runs and reports min=n/a; '
                              '12s covers the default sweep down to 1.0 m/s with margin)')
    parser.add_argument('--min-dwell', type=int, default=10,
                         help='Minimum required consecutive in-frustum samples per transit')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--log-dir', default='/tmp/cv_detection_test_logs')
    args = parser.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)

    all_ok = True
    for speed in args.speeds:
        print(f'\n=== speed={speed} m/s ===')
        sampler = run_one_speed(speed, args.duration, args.headless, args.log_dir)
        ok = summarize(speed, sampler, args.min_dwell)
        all_ok = all_ok and ok
        time.sleep(1.0)  # let the container fully settle before the next launch

    if not all_ok:
        print('\nFAIL: one or more speeds failed the dwell guard or produced no samples.')
        sys.exit(1)
    print('\nAll speeds passed.')


if __name__ == '__main__':
    main()
