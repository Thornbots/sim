#!/usr/bin/env python3
"""Ground-truth accuracy diagnostic for EKF-fused odometry (--backend none
--use-ekf in run_localization_drift_tests.py's terms).
Answers what run_localization_drift_tests.py structurally can't (see
README.md): does fusing /scan_odom into /odom via ekf_node actually beat
raw /odom, scored against /sim/raw_odom? Usage: `python3
ekf_ground_truth_diag.py [--headless] [--slip-ratio 0.10]`. Exit 0 if
EKF beat raw /odom on mean error, 1 otherwise.
"""

import argparse
import importlib.util
import math
import os
import statistics
import sys
import time

import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

_HERE = os.path.dirname(os.path.abspath(__file__))

# Reuse the drift suite's stack lifecycle / driving helpers rather than
# duplicating them -- filename isn't a valid identifier for a plain import.
_spec = importlib.util.spec_from_file_location(
    'run_localization_drift_tests',
    os.path.join(_HERE, 'run_localization_drift_tests.py'))
_drift = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_drift)


class GroundTruthProbe(Node):
    """Samples ground truth, raw wheel odometry, and the EKF's fused
    `odom->root` TF simultaneously, so all three can be compared at the
    same instants."""

    def __init__(self):
        super().__init__('ekf_ground_truth_probe')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._truth = None
        self._odom = None
        self.create_subscription(
            Odometry, '/sim/raw_odom', self._on_truth, 10)
        self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

    def _on_truth(self, msg):
        p = msg.pose.pose.position
        self._truth = (p.x, p.y)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._odom = (p.x, p.y)

    def ekf_xy(self, timeout=0.5):
        """Fused estimate, read off the `odom->root` TF that ekf_node owns
        under localization_mode:=none (there is no map frame in this
        mode)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'root', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def sample(self):
        """One simultaneous (truth, raw odom, ekf) triple, or None if any
        leg isn't available yet."""
        truth, odom, ekf = self._truth, self._odom, self.ekf_xy()
        if truth is None or odom is None or ekf is None:
            return None
        return truth, odom, ekf


def _err(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _stats(errors):
    return {
        'mean': statistics.fmean(errors),
        'rms': math.sqrt(statistics.fmean(e * e for e in errors)),
        'max': max(errors),
    }


def run(gui, slip_ratio, drift_stddev, observe_seconds):
    sim_tree = sentry_tree = helper = None
    probe = None
    try:
        # Wheel odometry error ON -- the whole point (see module docstring).
        # backend='none' (no map layer), use_ekf=True -- the old standalone
        # 'ekf' backend, now expressed via run_localization_drift_tests.py's
        # two-axis backend/use_ekf split.
        sim_tree, sentry_tree, helper = _drift.run_stack(
            gui, 'none', True,
            odom_noise_enabled=True,
            odom_drift_stddev=drift_stddev,
            odom_slip_ratio=slip_ratio)

        probe = GroundTruthProbe()
        # Spin continuously on a background thread. This node must keep
        # draining its subscriptions -- especially the TransformListener's
        # /tf feed -- for the whole run, including while the *helper* node
        # is busy inside drive()/spin_for(). Sampling with only a handful of
        # spin_once() calls after each leg makes the tf2 buffer's newest
        # transform lag real time by seconds (it ingests a few of the many
        # queued /tf messages), which reads as the EKF trailing the robot
        # when it is really just the probe reading a stale buffer.
        executor = SingleThreadedExecutor()
        executor.add_node(probe)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        sc = _drift.Scenario('ekf_ground_truth', 'accuracy vs /sim/raw_odom')
        if not _drift.wait_for_stack_ready(sc, helper):
            print('FAIL: stack never reached a healthy /scan rate')
            return None

        if helper.wait_for_correction_tf(timeout=45.0) is None:
            print('FAIL: odom->root never became available within 45s')
            return None

        # Let all three streams line up before scoring anything.
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if probe.sample() is not None:
                break
            time.sleep(0.1)
        first = probe.sample()
        if first is None:
            print('FAIL: never got a simultaneous truth/odom/ekf sample')
            return None
        truth0, odom0, ekf0 = first
        print(f'initial truth={truth0}  odom={odom0}  ekf={ekf0}')

        # Reposition to the loop's start corner, same as the drift suite's
        # cornering scenarios, so the driving profile is comparable.
        helper.drive(-4.0, 0.0, 0.125)
        helper.drive(0.0, -4.0, 0.25)

        odom_errs = []
        ekf_errs = []
        t0 = time.monotonic()
        i = 0
        while time.monotonic() - t0 < observe_seconds:
            vx, vy, duration = _drift.OBSTACLE_LOOP_LEGS[
                i % len(_drift.OBSTACLE_LOOP_LEGS)]
            i += 1
            helper.drive(vx, vy, duration)
            helper.spin_for(_drift.OBSTACLE_LOOP_DWELL_SECONDS)

            # probe spins on its own thread, so everything is already
            # current here -- no manual draining needed.
            s = probe.sample()
            if s is None:
                continue
            truth, odom, ekf = s
            e_odom, e_ekf = _err(odom, truth), _err(ekf, truth)
            odom_errs.append(e_odom)
            ekf_errs.append(e_ekf)
            print(f't={time.monotonic() - t0:5.1f}s  '
                  f'truth=({truth[0]:6.3f},{truth[1]:6.3f})  '
                  f'odom=({odom[0]:6.3f},{odom[1]:6.3f})  '
                  f'ekf=({ekf[0]:6.3f},{ekf[1]:6.3f})  '
                  f'err_odom={e_odom:.4f}  err_ekf={e_ekf:.4f}')

        if len(odom_errs) < 3:
            print(f'FAIL: too few samples ({len(odom_errs)})')
            return None
        return _stats(odom_errs), _stats(ekf_errs), len(odom_errs)
    finally:
        if probe is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
            probe.destroy_node()
        _drift.teardown_stack(sim_tree, sentry_tree, helper)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true',
                        help='run gz-sim without its GUI')
    parser.add_argument('--slip-ratio', type=float, default=0.05,
                        help='fraction of every driven meter lost from '
                             'reported odometry (default 0.05)')
    parser.add_argument('--drift-stddev', type=float, default=0.002,
                        help='per-sample stddev of the odometry drift '
                             'random walk (default 0.002)')
    parser.add_argument('--seconds', type=float, default=45.0,
                        help='how long to drive the cornering loop')
    args = parser.parse_args()

    rclpy.init()
    try:
        result = run(not args.headless, args.slip_ratio, args.drift_stddev,
                     args.seconds)
    finally:
        rclpy.shutdown()

    if result is None:
        return 1
    odom_s, ekf_s, n = result
    improvement = (odom_s['mean'] - ekf_s['mean']) / odom_s['mean'] * 100.0
    print()
    print(f'=== {n} samples, slip_ratio={args.slip_ratio}, '
          f'drift_stddev={args.drift_stddev} ===')
    print(f'raw /odom   vs truth: mean={odom_s["mean"]:.4f} m  '
          f'rms={odom_s["rms"]:.4f}  max={odom_s["max"]:.4f}')
    print(f'ekf fused   vs truth: mean={ekf_s["mean"]:.4f} m  '
          f'rms={ekf_s["rms"]:.4f}  max={ekf_s["max"]:.4f}')
    print(f'EKF improvement over raw /odom: {improvement:+.1f}%')
    return 0 if improvement > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
