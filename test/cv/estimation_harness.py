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
Estimation-bench machinery: scores target_tracker's TargetState against truth.

estimation.launch.py runs bench_world (the clock, the phantom target, our
chassis and head, and detections off our head's camera; C++, no gz) and the
real Part 2 (target_selector, target_tracker). point_to_cv_target and
bench_world's head controller keep the head on the target; nothing fires. Each TargetState is
compared with bench_world's truth at its own header.stamp, so a wrong stamp
scores as error. Importable only -- test_estimation.py holds the assertions.
see ../../README.md for design rationale
"""
import json
import math
import os
import zlib

from dji_serial_bridge.msg import TargetState
import estimation_limits_data
from estimation_metrics import METRICS, state_errors, summarize_case
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shot_hit_harness import (
    check_nodes, interpolate, LaunchTree, PANEL_RADIUS_X, PANEL_RADIUS_Y, SimTimeNode,
    TARGET_PATHS,
)
from sim import suite_timing
from std_msgs.msg import Header

DEFAULT_DURATION = 30.0  # sim seconds scored per case, after SETTLE_S
SETTLE_S = 3.0  # sim seconds after the track restarts before steady-state scoring
# bench_world's case_hold_s: detections off this long at each case start,
# past target_tracker's track_max_gap_s (0.5), so every case starts a fresh
# track, while bench_world turns the head to the true target.
RESET_S = 1.0
TRUTH_HISTORY_S = 2.0
DEFAULT_LOG_DIR = '/tmp/estimation_test_logs'
# Sim seconds of truth past a window's end before it is closed, so every state
# stamped inside it has arrived.
CLOSE_S = 0.2
# --blackout: blackout_s of no detections every period, shorter than
# track_max_gap_s, so the tracker coasts and must pick the target up again.
BLACKOUT = {'blackout_period_s': 2.0, 'blackout_s': 0.3}
# --shooter-speed: bench_world bounces our chassis along y within this of the origin.
SHOOTER_HALF_WIDTH = 1.0
# Steady-state p95 limits per cell; estimation_limits_data.py says where from.
LIMITS = estimation_limits_data.LIMITS


class EstimationSampler(SimTimeNode):
    """
    Collects TargetStates and scores each once truth covers its stamp.

    One per stack, so /bench/progress keeps pacing bench_world between cases.
    """

    def __init__(self):
        super().__init__('estimation_sampler')
        self.stagger = 0.0
        self._truth = []  # [(stamp_s, center, velocity, yaw unwrapped, yaw_rate)]
        self._root_xy = (0.0, 0.0)  # ours, from /sim/raw_odom
        self._pending = []  # [(stamp_s, arrival_s, TargetState)]
        self.records = []  # one dict per scored state
        self.create_subscription(Odometry, '/target/ground_truth_odom', self._on_truth, 50)
        self.create_subscription(TargetState, '/cv/target_state', self._on_state, 50)
        self.create_subscription(Odometry, '/sim/raw_odom', self._on_root_odom, 10)
        # The truth stamp scored up to, so sim_clock's paced mode (rate 0)
        # doesn't run ahead of this scorer.
        self.progress_pub = self.create_publisher(Header, '/bench/progress', 10)
        self.case = None  # (case_seed, start_s) from bench_world's /bench/case
        self.create_subscription(
            Header, '/bench/case', self._on_case,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def start_case(self, stagger):
        """Forget the last case: its truth jumped when bench_world reset."""
        self.stagger = stagger
        self._truth, self._pending, self.records = [], [], []
        self.case = None  # callbacks run only in spin, so the new case's message waits

    def _on_truth(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        stamp = self._stamp_s(msg.header.stamp)
        yaw = 2.0 * math.atan2(q.z, q.w)  # bench_world publishes yaw-only
        if self._truth:
            prev = self._truth[-1][3]
            yaw = prev + math.atan2(math.sin(yaw - prev), math.cos(yaw - prev))
        # The truth twist is world-frame, despite child_frame_id.
        self._truth.append((stamp, np.array([p.x, p.y, p.z]),
                            np.array([v.linear.x, v.linear.y, v.linear.z]),
                            yaw, v.angular.z))
        self._truth = [h for h in self._truth if stamp - h[0] <= TRUTH_HISTORY_S]
        self._resolve(stamp)
        self.progress_pub.publish(Header(stamp=msg.header.stamp))

    def _on_case(self, msg):
        self.case = (int(msg.frame_id), self._stamp_s(msg.stamp))

    def truth_s(self):
        """Newest truth stamp seen (s), or None."""
        return self._truth[-1][0] if self._truth else None

    def spin_until_truth(self, stamp_s):
        """Spin until truth reaches stamp_s, capped at 60 s wall."""
        ok = self.wait_until(lambda: (self.truth_s() or 0.0) >= stamp_s, timeout=60.0,
                             description=f'truth at {stamp_s:.3f} s')
        if not ok:
            print('[spin_until_truth] wall-clock cap hit')

    def _on_state(self, msg):
        self._pending.append((self._stamp_s(msg.header.stamp), self.now_s(), msg))

    def _resolve(self, truth_s):
        keep = []
        for stamp, arrival, msg in self._pending:
            if stamp > truth_s:
                keep.append((stamp, arrival, msg))
                continue
            if stamp < self._truth[0][0]:
                continue  # older than the truth kept; can't score it
            center, vel, yaw, yaw_rate = interpolate(self._truth, stamp)
            rec = state_errors(msg, (center, vel, yaw, yaw_rate), self.stagger,
                               (PANEL_RADIUS_X, PANEL_RADIUS_Y), self._root_xy)
            rec.update({'t': round(stamp, 4), 'valid': bool(msg.valid),
                        'age_on_arrival_s': round(arrival - stamp, 4),
                        'track_id': int(msg.robot_track_id)})
            self.records.append(rec)
        self._pending = keep

    def _on_root_odom(self, msg):
        p = msg.pose.pose.position
        self._root_xy = (p.x, p.y)


class EstimationStack:
    """
    The estimation bench stack, launched once and reused for every case.

    launch/estimation.launch.py defines it; with external=True it is already
    running (it started this pytest). Between cases only bench_world's target
    motion, panel stagger and masking change.
    """

    def __init__(self, headless, log_dir, external=False, extra_args=()):
        self.launch = None
        # Checked up at start and after every case (check_nodes).
        self.nodes = ['bench_world', 'target_selector', 'target_tracker', 'point_to_cv_target']
        if not headless:
            self.nodes += ['target_state_markers', 'rviz2']
        if not external:
            self.launch = LaunchTree(
                'stack',
                ['ros2', 'launch', 'sim', 'estimation.launch.py', 'run_tests:=false',
                 f'headless:={str(headless).lower()}', *extra_args],
                os.path.join(log_dir, 'stack.log'))
        self.states_path = os.path.join(log_dir, 'estimation_states.jsonl')
        self.summary_path = os.path.join(log_dir, 'estimation.jsonl')
        self.node = rclpy.create_node(
            'estimation_stack',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._client = self.node.create_client(SetParameters, '/bench_world/set_parameters')
        self.sampler = None

    def start(self):
        for path in (self.states_path, self.summary_path):
            open(path, 'w').close()
        with suite_timing.phase('bringup'):
            if self.launch is not None:
                self.launch.start()
            self.sampler = EstimationSampler()
            self.sampler.wait_until(lambda: bool(self.sampler._truth), timeout=120.0,
                                    description='/target/ground_truth_odom publishing')
            self.sampler.wait_until(lambda: self.sampler.nodes_up(*self.nodes), timeout=60.0,
                                    description=f'{", ".join(self.nodes)} nodes up')
            check_nodes(self.sampler, self.nodes)

    def set_params(self, **params):
        """Set bench_world's target and detection parameters."""
        client = self._client
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/bench_world/set_parameters not available')
        msgs = []
        for k, v in params.items():
            if isinstance(v, bool):
                param = Parameter(k, Parameter.Type.BOOL, v)
            elif isinstance(v, int):
                param = Parameter(k, Parameter.Type.INTEGER, v)
            else:
                param = Parameter(k, Parameter.Type.DOUBLE, float(v))
            msgs.append(param.to_parameter_msg())
        future = client.call_async(SetParameters.Request(parameters=msgs))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not all(r.successful for r in result.results):
            raise RuntimeError(f'bench_world rejected {params}')

    def stop(self):
        with suite_timing.phase('teardown'):
            if self.launch is not None:
                self.launch.stop()
        if self.sampler is not None:
            self.sampler.destroy_node()
        self.node.destroy_node()


def run_case(stack, cell, speed, spin_hz, duration, stagger=0.0, path='lateral',
             blackout=False, shooter_speed=0.0, target_yaw=0.0, chassis_spin=0.0):
    """
    Restart the track on (speed, spin_hz, stagger, path), then score every state.

    One set_params starts the case in bench_world (case_seed, the cell's
    CRC): the target at its path start and target_yaw (rad, 0 = a panel
    square to us), our chassis at the origin spinning at chassis_spin
    (rad/s) under the world-held head, noise keyed on the cell, and
    detections off for RESET_S while the head turns to the target. States
    are scored from there for SETTLE_S + duration.
    """
    sampler = stack.sampler
    seed = zlib.crc32(cell.encode()) & 0x7fffffff
    with suite_timing.phase('reset'):
        stack.set_params(
            target_speed=speed, spin_hz=spin_hz, **TARGET_PATHS[path],
            target_yaw=target_yaw, chassis_spin_rad_s=chassis_spin,
            panel_stagger_m=stagger, detections_enabled=True,
            **(BLACKOUT if blackout else {'blackout_period_s': 0.0}),
            shooter_speed=shooter_speed, shooter_half_width=SHOOTER_HALF_WIDTH,
            case_hold_s=RESET_S, case_seed=seed)
        sampler.start_case(stagger)
    suite_timing.set_sim_clock(sampler.now_s)
    try:
        # The window is bench_world's case start plus fixed offsets, so it
        # covers the same sim times on every run.
        with suite_timing.phase('reset'):
            if not sampler.wait_until(lambda: sampler.case and sampler.case[0] == seed,
                                      timeout=30.0, description=f'/bench/case {seed}'):
                raise RuntimeError(f'bench_world never started case {cell}')
            start_s = sampler.case[1] + RESET_S
            sampler.spin_until_truth(start_s)
        with suite_timing.phase('settle'):
            sampler.spin_until_truth(start_s + SETTLE_S)
        end_s = start_s + SETTLE_S + duration
        with suite_timing.phase('scored'):
            sampler.spin_until_truth(end_s + CLOSE_S)
        check_nodes(sampler, stack.nodes)
    finally:
        suite_timing.set_sim_clock(None)
    records = [r for r in sampler.records if start_s <= r['t'] <= end_s]
    summary = {'cell': cell, **summarize_case(records, start_s, SETTLE_S)}
    with open(stack.states_path, 'a') as f:
        for rec in records:
            f.write(json.dumps({'cell': cell, **rec}) + '\n')
    with open(stack.summary_path, 'a') as f:
        f.write(json.dumps(summary) + '\n')
    return summary


def print_summary(label, summary):
    print(f'{label}: {summary["states"]} states, '
          f'{summary.get("valid_fraction", 0):.0%} valid, '
          f'converged in {summary.get("converge_s")} s, '
          f'age on arrival {summary.get("age_on_arrival_s")} s')
    for key in METRICS:
        stat = summary.get(key)
        if stat:
            print(f'    {key:16s} mean {stat["mean"]:.4f}  p95 {stat["p95"]:.4f}')
