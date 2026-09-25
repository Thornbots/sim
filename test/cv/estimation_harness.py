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
C2 estimation-bench machinery: scores target_tracker's TargetState against truth.

estimation.launch.py runs gz with our sentry_v2, target_driver's phantom
target, cv_target_emulator's detections off the real head's camera, and the
real Part 2 (target_selector, target_tracker). point_to_cv_target and
cv_head_aim keep the head on the target; nothing fires. Each TargetState is
compared with target_driver's truth at its own header.stamp, so a wrong stamp
scores as error. Importable only -- test_estimation.py holds the assertions.
see ../../README.md for design rationale
"""
import json
import math
import os

from dji_serial_bridge.msg import TargetState
from estimation_metrics import METRICS, state_errors, summarize_case
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import Parameter
from shot_hit_harness import (
    interpolate, LaunchTree, PANEL_RADIUS_X, PANEL_RADIUS_Y, SimTimeNode, TARGET_PATHS,
)

DEFAULT_DURATION = 30.0  # sim seconds scored per case, after SETTLE_S
SETTLE_S = 3.0  # sim seconds after the track restarts before steady-state scoring
# Detections off this long at each case start, past target_tracker's
# track_max_gap_s (0.5), so every case starts a fresh track.
RESET_S = 1.0
TRUTH_HISTORY_S = 2.0
DEFAULT_LOG_DIR = '/tmp/estimation_test_logs'
# --blackout: blackout_s of no detections every period, shorter than
# track_max_gap_s, so the tracker coasts and must pick the target up again.
BLACKOUT = {'blackout_period_s': 2.0, 'blackout_s': 0.3}
# --shooter-speed drives our gz chassis along y within this of the origin.
SHOOTER_HALF_WIDTH = 1.0
SHOOTER_CMD_PERIOD_S = 0.05
# Steady-state p95 limits per cell, metric -> m or rad: the worst of three
# runs plus a margin, printed by tools/estimation_limits.py. None measured yet,
# so every cell only reports (CV_SPLIT_PLAN.md 2.0).
LIMITS = {}


class EstimationSampler(SimTimeNode):
    """Collects TargetStates and scores each once truth covers its stamp."""

    def __init__(self, stagger, shooter_speed=0.0):
        super().__init__('estimation_sampler')
        self.stagger = stagger
        self.shooter_speed = shooter_speed
        self._shooter_dir = 1.0
        self._last_cmd_s = None
        self._truth = []  # [(stamp_s, center, velocity, yaw unwrapped, yaw_rate)]
        self._root_xy = (0.0, 0.0)  # ours, from /sim/raw_odom
        self._pending = []  # [(stamp_s, arrival_s, TargetState)]
        self.records = []  # one dict per scored state
        self.create_subscription(Odometry, '/target/ground_truth_odom', self._on_truth, 50)
        self.create_subscription(TargetState, '/cv/target_state', self._on_state, 50)
        self.create_subscription(Odometry, '/sim/raw_odom', self._on_root_odom, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_truth(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        stamp = self._stamp_s(msg.header.stamp)
        yaw = 2.0 * math.atan2(q.z, q.w)  # target_driver publishes yaw-only
        if self._truth:
            prev = self._truth[-1][3]
            yaw = prev + math.atan2(math.sin(yaw - prev), math.cos(yaw - prev))
        # target_driver's twist is world-frame, despite child_frame_id.
        self._truth.append((stamp, np.array([p.x, p.y, p.z]),
                            np.array([v.linear.x, v.linear.y, v.linear.z]),
                            yaw, v.angular.z))
        self._truth = [h for h in self._truth if stamp - h[0] <= TRUTH_HISTORY_S]
        self._resolve(stamp)

    def _on_state(self, msg):
        self._pending.append((self._stamp_s(msg.header.stamp), self._now_s(), msg))

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
        """Keep our position; bounce our chassis along y at shooter_speed, x held at 0."""
        self._root_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.shooter_speed <= 0.0:
            return
        now_s = self._stamp_s(msg.header.stamp)
        if self._last_cmd_s is not None and now_s - self._last_cmd_s < SHOOTER_CMD_PERIOD_S:
            return
        self._last_cmd_s = now_s
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if y >= SHOOTER_HALF_WIDTH:
            self._shooter_dir = -1.0
        elif y <= -SHOOTER_HALF_WIDTH:
            self._shooter_dir = 1.0
        cmd = Twist()
        cmd.linear.x = max(-self.shooter_speed, min(self.shooter_speed, -2.0 * x))
        cmd.linear.y = self._shooter_dir * self.shooter_speed
        self.cmd_vel_pub.publish(cmd)

    def stop_shooter(self):
        if self.shooter_speed > 0.0:
            self.cmd_vel_pub.publish(Twist())


class EstimationStack:
    """
    The C2 stack, launched once and reused for every case.

    launch/estimation.launch.py defines it; with external=True it is already
    running (it started this pytest). Between cases only target_driver's
    motion and cv_target_emulator's stagger and masking change.
    """

    def __init__(self, headless, log_dir, external=False, extra_args=()):
        self.launch = None
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
        self._clients = {name: self.node.create_client(SetParameters, f'/{name}/set_parameters')
                         for name in ('target_driver', 'cv_target_emulator')}

    def start(self):
        for path in (self.states_path, self.summary_path):
            open(path, 'w').close()
        if self.launch is not None:
            self.launch.start()
        probe = EstimationSampler(stagger=0.0)
        try:
            probe.wait_until(lambda: bool(probe._truth), timeout=120.0,
                             description='/target/ground_truth_odom publishing')
            ready = ['target_driver', 'cv_target_emulator', 'target_selector',
                     'target_tracker', 'point_to_cv_target', 'cv_head_aim']
            probe.wait_until(lambda: probe.nodes_up(*ready), timeout=60.0,
                             description=f'{", ".join(ready)} nodes up')
        finally:
            probe.destroy_node()

    def set_params(self, name, **params):
        client = self._clients[name]
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'/{name}/set_parameters not available')
        msgs = []
        for k, v in params.items():
            kind = Parameter.Type.BOOL if isinstance(v, bool) else Parameter.Type.DOUBLE
            msgs.append(Parameter(k, kind, v if isinstance(v, bool) else float(v))
                        .to_parameter_msg())
        future = client.call_async(SetParameters.Request(parameters=msgs))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not all(r.successful for r in result.results):
            raise RuntimeError(f'{name} rejected {params}')

    def stop(self):
        if self.launch is not None:
            self.launch.stop()
        self.node.destroy_node()


def run_case(stack, cell, speed, spin_hz, duration, stagger=0.0, path='lateral',
             blackout=False, shooter_speed=0.0):
    """
    Restart the track on (speed, spin_hz, stagger, path), then score every state.

    Detections go off for RESET_S so the tracker drops the old track, then
    come back; states are scored from there for SETTLE_S + duration.
    """
    stack.set_params('target_driver', target_speed=speed, spin_hz=spin_hz,
                     **TARGET_PATHS[path])
    stack.set_params('cv_target_emulator', panel_stagger_m=stagger, detections_enabled=False,
                     **(BLACKOUT if blackout else {'blackout_period_s': 0.0}))
    sampler = EstimationSampler(stagger, shooter_speed)
    try:
        sampler.spin_for(RESET_S)
        sampler._pending, sampler.records = [], []
        stack.set_params('cv_target_emulator', detections_enabled=True)
        start_s = sampler._now_s()
        sampler.spin_for(SETTLE_S + duration)
    finally:
        sampler.stop_shooter()
        records = sampler.records
        sampler.destroy_node()
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
