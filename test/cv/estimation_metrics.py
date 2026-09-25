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
C2's scoring math, no ROS: one TargetState's errors against truth, and a case summary.

estimation_harness.py feeds it; test_estimation_metrics.py pins it.
"""
import math

import numpy as np

QUARTER_TURN = math.pi / 2.0
# Panel error the track must stay under to count as converged: half a panel.
CONVERGED_M = 0.05
METRICS = ('facing_panel_m', 'panel_m', 'center_m', 'velocity_m_s', 'yaw_rad', 'yaw_rate_rad_s',
           'radius_m', 'z_offset_m')


def cell_id(layout, speed, path, shooter_speed, blackout, camera_latency_s):
    """Name one bench cell, as LIMITS and estimation.jsonl key it."""
    case = 'stationary' if speed == 0.0 else f'speed{speed:g}'
    cell = f'{layout}-{case}-{path}-shooter{shooter_speed:g}'
    if blackout:
        cell += '-blackout'
    if camera_latency_s:
        cell += f'-camlat{camera_latency_s:g}'
    return cell


def state_errors(state, truth, stagger, radii, viewer):
    """
    Return one TargetState's errors against truth at its stamp.

    truth: (center, velocity, yaw, yaw_rate), yaw unwrapped; radii: the true
    (front/back, left/right) radii; viewer: our (x, y), for facing_panel_m, the
    error on the true panel facing us, the one Part 1 aims at. The state's
    panel k is matched to the
    true panel m quarter turns on, m the rounding of their yaw difference,
    so tracking another panel of the same robot is not an error; its pairs
    are matched the same way.
    """
    center, vel, yaw, yaw_rate = truth
    c = np.array([state.center.x, state.center.y, state.center.z])
    v = np.array([state.velocity.x, state.velocity.y, state.velocity.z])
    m = round((state.yaw - yaw) / QUARTER_TURN)
    true_dz = (stagger / 2.0, -stagger / 2.0)
    panel_err = []
    for k in range(4):
        pair, true_pair = k % 2, (k + m) % 2
        yaw_s, yaw_t = state.yaw + k * QUARTER_TURN, yaw + (k + m) * QUARTER_TURN
        est = c + np.array([state.radius[pair] * math.cos(yaw_s),
                            state.radius[pair] * math.sin(yaw_s), state.z_offset[pair]])
        true = center + np.array([radii[true_pair] * math.cos(yaw_t),
                                  radii[true_pair] * math.sin(yaw_t), true_dz[true_pair]])
        panel_err.append(float(np.linalg.norm(est - true)))
    to_viewer = math.atan2(viewer[1] - center[1], viewer[0] - center[0])
    facing = max(range(4), key=lambda j: math.cos(yaw + j * QUARTER_TURN - to_viewer))
    errors = {
        'facing_panel_m': panel_err[(facing - m) % 4],
        'panel_m': sum(panel_err) / 4.0,
        'panel_max_m': max(panel_err),
        'center_m': float(np.linalg.norm(c - center)),
        'velocity_m_s': float(np.linalg.norm(v - vel)),
        'yaw_rad': abs(state.yaw - yaw - m * QUARTER_TURN),
        'yaw_rate_rad_s': abs(state.yaw_rate - yaw_rate),
        'radius_m': max(abs(state.radius[p] - radii[(p + m) % 2]) for p in range(2)),
        'z_offset_m': max(abs(state.z_offset[p] - true_dz[(p + m) % 2]) for p in range(2)),
    }
    # TargetState's fixed arrays arrive as numpy float32, which json refuses.
    return {k: round(float(v), 5) for k, v in errors.items()}


def summarize_case(records, start_s, settle_s):
    """
    Return the case summary: convergence, valid fraction, steady-state mean and p95.

    converge_s runs from the first state to the first after which the
    facing panel's error stays under CONVERGED_M (None if it never does).
    Steady state is every valid state settle_s or more after start_s.
    """
    if not records:
        return {'states': 0}
    first = records[0]['t']
    last_bad = [r['t'] for r in records
                if not r['valid'] or r['facing_panel_m'] > CONVERGED_M]
    later = [r['t'] for r in records if not last_bad or r['t'] > last_bad[-1]]
    steady = [r for r in records if r['valid'] and r['t'] >= start_s + settle_s]
    out = {
        'states': len(records),
        'valid_fraction': round(sum(r['valid'] for r in records) / len(records), 4),
        'converge_s': round(later[0] - first, 3) if later else None,
        'age_on_arrival_s': round(float(np.mean([r['age_on_arrival_s'] for r in records])), 4),
        'steady_states': len(steady),
    }
    for key in METRICS:
        values = [r[key] for r in steady]
        out[key] = ({'mean': round(float(np.mean(values)), 4),
                     'p95': round(float(np.percentile(values, 95)), 4)}
                    if values else None)
    return out
