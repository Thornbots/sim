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
Ground-truth accuracy probe for EKF-fused odometry.

That is backend='none' plus use_ekf=True in drift_harness's terms. Answers
what the drift suite structurally can't (see README.md): does fusing
/scan_odom into /odom via ekf_node actually beat raw /odom, scored against
/sim/raw_odom? Importable machinery only -- test_ekf_ground_truth.py holds
the assertion, launch/localization_tests.launch.py suite:=ekf runs it.
"""
import math
import statistics
import threading
import time

import drift_harness as _drift
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


class GroundTruthProbe(Node):
    """
    Samples ground truth, raw wheel odometry, and the EKF's fused TF.

    The `odom->root` TF is read at the same instants as the other two,
    so all three can be compared directly.
    """

    def __init__(self):
        super().__init__(
            'ekf_ground_truth_probe',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._truth = None
        self._odom = None
        self._scan_odom = None
        self.head_yaw = None
        self.create_subscription(
            Odometry, '/sim/raw_odom', self._on_truth, 10)
        self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(
            Odometry, '/scan_odom', self._on_scan_odom, 10)
        self.create_subscription(
            JointState, '/sim/raw_joint_states', self._on_joints, 10)

    def _on_truth(self, msg):
        p = msg.pose.pose.position
        self._truth = (p.x, p.y)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._odom = (p.x, p.y)

    def _on_scan_odom(self, msg):
        p = msg.pose.pose.position
        self._scan_odom = (p.x, p.y)

    def _on_joints(self, msg):
        if 'headlink' in msg.name:
            self.head_yaw = msg.position[msg.name.index('headlink')]

    def ekf_xy(self, timeout=0.5):
        """
        Fused estimate, read off the `odom->root` TF that ekf_node owns.

        There is no map frame under localization_mode:=none.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'root', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def sample(self):
        """One simultaneous (truth, raw odom, ekf) triple, or None if any is missing."""
        truth, odom, ekf = self._truth, self._odom, self.ekf_xy()
        if truth is None or odom is None or ekf is None:
            return None
        return truth, odom, ekf


def _err(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def leg_vector_error(start, end, truth_start, truth_end):
    """
    (angle_deg, length_ratio) of one source's leg displacement against truth's.

    The angle is what catches a source pointing backwards; a length ratio
    alone scored a reversed rf2o at ~1% error once (sim/AGENTS.md).
    """
    dx, dy = end[0] - start[0], end[1] - start[1]
    tx, ty = truth_end[0] - truth_start[0], truth_end[1] - truth_start[1]
    t_len = math.hypot(tx, ty)
    if t_len < 1e-6 or math.hypot(dx, dy) < 1e-6:
        return float('nan'), float('nan')
    angle = math.degrees(math.atan2(tx * dy - ty * dx, tx * dx + ty * dy))
    return angle, math.hypot(dx, dy) / t_len


def _stats(errors):
    return {
        'mean': statistics.fmean(errors),
        'rms': math.sqrt(statistics.fmean(e * e for e in errors)),
        'max': max(errors),
    }


def run(gui, slip_ratio, drift_stddev, observe_seconds):
    stack = helper = None
    probe = None
    try:
        # Wheel odometry error ON -- the whole point (see module docstring).
        # backend='none' (no map layer), use_ekf=True -- the old standalone
        # 'ekf' backend, in the drift suite's two-axis backend/use_ekf terms.
        stack, helper = _drift.run_stack(
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
        _drift._reposition_to_loop_start(helper)

        odom_errs = []
        ekf_errs = []
        leg_angles = {'scan_odom': [], 'odom': [], 'ekf': []}
        t0 = helper.now_s()
        i = 0
        # probe spins on its own thread, so everything is already current
        # at each read -- no manual draining needed.
        prev = probe.sample()
        prev_scan = probe._scan_odom
        while helper.now_s() - t0 < observe_seconds:
            vx, vy, duration = _drift.OBSTACLE_LOOP_LEGS[
                i % len(_drift.OBSTACLE_LOOP_LEGS)]
            i += 1
            helper.drive(vx, vy, duration)
            helper.spin_for(_drift.OBSTACLE_LOOP_DWELL_SECONDS)

            s = probe.sample()
            scan = probe._scan_odom
            if s is None:
                prev, prev_scan = None, scan
                continue
            truth, odom, ekf = s
            if prev is not None:
                legs = [('odom', prev[1], odom), ('ekf', prev[2], ekf)]
                if prev_scan is not None and scan is not None:
                    legs.insert(0, ('scan_odom', prev_scan, scan))
                parts = []
                for name, a, b in legs:
                    ang, ratio = leg_vector_error(a, b, prev[0], truth)
                    leg_angles[name].append(ang)
                    parts.append(f'{name} {ang:+6.1f}deg x{ratio:.2f}')
                head = probe.head_yaw
                head_str = f'  head_yaw={head:+.2f}' if head is not None else ''
                print(f'  leg ({vx:+.1f},{vy:+.1f}) vs truth: '
                      + ', '.join(parts) + head_str)
            prev, prev_scan = s, scan
            e_odom, e_ekf = _err(odom, truth), _err(ekf, truth)
            odom_errs.append(e_odom)
            ekf_errs.append(e_ekf)
            print(f't={helper.now_s() - t0:5.1f}s  '
                  f'truth=({truth[0]:6.3f},{truth[1]:6.3f})  '
                  f'odom=({odom[0]:6.3f},{odom[1]:6.3f})  '
                  f'ekf=({ekf[0]:6.3f},{ekf[1]:6.3f})  '
                  f'err_odom={e_odom:.4f}  err_ekf={e_ekf:.4f}')

        if len(odom_errs) < 3:
            print(f'FAIL: too few samples ({len(odom_errs)})')
            return None
        for name, angles in leg_angles.items():
            finite = [abs(a) for a in angles if not math.isnan(a)]
            if finite:
                print(f'{name} leg direction error: mean {statistics.fmean(finite):.1f} '
                      f'deg, max {max(finite):.1f} deg over {len(finite)} legs')
        return _stats(odom_errs), _stats(ekf_errs), len(odom_errs)
    finally:
        if probe is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
            probe.destroy_node()
        _drift.teardown_stack(stack, helper)
        _drift.stop_sim()


def improvement_pct(odom_stats, ekf_stats):
    """
    Percentage reduction in mean ground-truth error, EKF versus raw /odom.

    NaN if odom's own mean error is zero, which a short or degenerate
    run can produce even though noise is always injected.
    """
    if odom_stats['mean'] <= 0.0:
        return float('nan')
    return (odom_stats['mean'] - ekf_stats['mean']) / odom_stats['mean'] * 100.0


def report(odom_stats, ekf_stats, n, slip_ratio, drift_stddev):
    print()
    print(f'=== {n} samples, slip_ratio={slip_ratio}, '
          f'drift_stddev={drift_stddev} ===')
    print(f'raw /odom   vs truth: mean={odom_stats["mean"]:.4f} m  '
          f'rms={odom_stats["rms"]:.4f}  max={odom_stats["max"]:.4f}')
    print(f'ekf fused   vs truth: mean={ekf_stats["mean"]:.4f} m  '
          f'rms={ekf_stats["rms"]:.4f}  max={ekf_stats["max"]:.4f}')
    print(f'EKF improvement over raw /odom: '
          f'{improvement_pct(odom_stats, ekf_stats):+.1f}%')
