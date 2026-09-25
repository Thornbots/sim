#!/usr/bin/env python3
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
C2 without ROS or gz: ArmorTracker on emulated detections, scored like the bench.

`python3 tools/estimation_offline.py [--cells flat-speed2 ...] [--set q_accel=4]`.
Mirrors target_driver's path, cv_target_emulator's panels, gating, noise,
dropout and delivery delay, and target_tracker's reset, R and publish-time
prediction, then scores with test/cv/estimation_metrics. The camera sits on
our head, aimed at the target. For tuning in seconds; the gz bench decides.
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'test', 'cv'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'thornbots_pkg'))
from estimation_metrics import cell_id, METRICS, state_errors, summarize_case  # noqa: E402
from thornbots_pkg.target_tracker_core import (  # noqa: E402
    ArmorTracker, DZ, POS, R, ray_covariance, VEL, W, YAW,
)

# Copies of the ROS-side constants (cv_target_emulator, shot_hit_harness,
# estimation_harness, target_tracker defaults).
PANEL_RADII = (0.30, 0.24)
STAGGER_M = 0.09
CANT = math.radians(75.0)
VIEW_HALF_ANGLE = math.radians(75.0)
HFOV = 1.5184
CAMERA_Z = 0.45
RATE_HZ = 60.0
PATHS = {'lateral': (0.0, 3.0, 2.4), 'radial': (90.0, 3.5, 2.0), 'diagonal': (45.0, 3.0, 2.0)}
TRACKER = {'panel_radius_m': 0.27, 'meas_noise_base_m': 0.03, 'meas_noise_range_coeff': 0.01,
           'meas_noise_lateral_m': 0.04, 'q_accel': 2.0, 'q_yaw_accel': 5.0,
           'q_radius': 0.02, 'gate_nis': 16.3, 'max_outliers': 3, 'track_max_gap_s': 0.5,
           'facing_std': 0.3, 'extra': {'q_jerk': 3.0, 'accel_tau_s': 1.0}}
# 'noise' is isotropic; depth_coeff and lateral_rad add a D435-like ray model:
# depth std = depth_coeff * range^2, lateral std = lateral_rad * range.
EKF_KWARGS = ('q_jerk', 'accel_tau_s', 'accel_prior_std', 'q_dz', 'still', 'q_still_pos',
              'q_still_yaw')
EMULATOR = {'noise': 0.005, 'dropout': 0.1, 'latency': 0.06, 'depth_coeff': 0.0,
            'lateral_rad': 0.0}
D435 = {'noise': 0.0, 'depth_coeff': 0.0036, 'lateral_rad': 0.003}


class Target:
    """target_driver's integrator: braking bounce plus ramped spin."""

    def __init__(self, speed, spin_hz, path, accel=6.0, spin_accel=20.0):
        self.speed, self.w_want = speed, 2 * math.pi * spin_hz
        angle, self.cx, self.half = PATHS[path]
        self.dir_xy = (math.sin(math.radians(angle)), math.cos(math.radians(angle)))
        self.accel, self.spin_accel = accel, spin_accel
        self.s = self.vs = self.yaw = self.w = 0.0
        self.direction = 1.0

    def step(self, dt):
        a = self.accel
        to_end = self.half - self.s if self.direction > 0 else self.s + self.half
        if to_end <= 1e-3 and abs(self.vs) <= a * dt:
            self.direction = -self.direction
            to_end = self.half - self.s if self.direction > 0 else self.s + self.half
        want = self.direction * min(self.speed, math.sqrt(2 * a * max(to_end, 0.0)))
        self.vs += max(-a * dt, min(a * dt, want - self.vs))
        self.s = max(-self.half, min(self.half, self.s + self.vs * dt))
        self.w += max(-self.spin_accel * dt, min(self.spin_accel * dt, self.w_want - self.w))
        self.yaw += self.w * dt

    def truth(self):
        dx, dy = self.dir_xy
        return (np.array([self.cx + self.s * dx, self.s * dy, 0.3]),
                np.array([self.vs * dx, self.vs * dy, 0.0]), self.yaw, self.w)


def visible_panels(center, yaw, stagger, cam):
    """[(view_angle, position)] of the panels the emulator would publish."""
    out = []
    for j in range(4):
        a = yaw + j * math.pi / 2
        pos = center + np.array([PANEL_RADII[j % 2] * math.cos(a),
                                 PANEL_RADII[j % 2] * math.sin(a),
                                 stagger / 2 if j % 2 == 0 else -stagger / 2])
        normal = np.array([math.sin(CANT) * math.cos(a), math.sin(CANT) * math.sin(a),
                           math.cos(CANT)])
        to_cam = cam - pos
        view = math.acos(np.clip(normal @ to_cam / np.linalg.norm(to_cam), -1, 1))
        if view <= VIEW_HALF_ANGLE:
            out.append((view, pos))
    return out


def run_cell(layout, speed, spin_hz, path, shooter_speed, blackout, duration,
             tracker, emulator, seed, camera_latency=0.0, tracker_latency=None):
    rng = np.random.default_rng(seed)
    stagger = STAGGER_M if layout == 'staggered' else 0.0
    target = Target(speed, spin_hz, path)
    dt = 1.0 / RATE_HZ
    # Pre-roll so the target is up to speed, as the bench's earlier cases leave it.
    for _ in range(int(3.0 * RATE_HZ)):
        target.step(dt)
    tl = camera_latency if tracker_latency is None else tracker_latency
    ekf, last_stamp, n_updates = None, None, 0
    cam_y, cam_dir = 0.0, 1.0
    pending, records, t = [], [], 0.0
    start_s, settle = 0.0, 3.0
    while t < settle + duration:
        target.step(dt)
        t += dt
        if shooter_speed > 0.0:
            if cam_y >= 1.0:
                cam_dir = -1.0
            elif cam_y <= -1.0:
                cam_dir = 1.0
            cam_y += cam_dir * shooter_speed * dt
        cam = np.array([0.0, cam_y, CAMERA_Z])
        center, vel, yaw, w = target.truth()
        masked = blackout and (t % 2.0) < 0.3
        if not masked:
            vis = visible_panels(center, yaw, stagger, cam)
            # Winner first: the selector's most central panel, here the most head-on.
            vis.sort(key=lambda v: v[0])
            dets = []
            for _, pos in vis:
                if rng.uniform() < emulator['dropout']:
                    continue
                dets.append(pos + _noise(rng, pos - cam, emulator))
            if dets:
                pending.append((t + max(emulator['latency'], camera_latency),
                                t + camera_latency, dets, cam.copy()))
        # Deliver, as target_tracker would see it.
        while pending and pending[0][0] <= t + 1e-9:
            arrive, stamp, dets, cam_at = pending.pop(0)
            cap = stamp - tl
            if last_stamp is not None and cap - last_stamp > tracker['track_max_gap_s']:
                ekf, n_updates = None, 0
            last_stamp = cap
            for p in dets:
                rng_m = float(np.linalg.norm(p - cam_at))
                std = tracker['meas_noise_base_m'] + tracker['meas_noise_range_coeff'] * rng_m ** 2
                R = ray_covariance(p, cam_at, std, tracker['meas_noise_lateral_m'])
                if ekf is None:
                    ekf = ArmorTracker(p, cam_at, cap, R, tracker['panel_radius_m'],
                                       tracker['q_accel'], tracker['q_yaw_accel'],
                                       tracker['q_radius'], **tracker.get('extra', {}))
                else:
                    ekf.step(p, cam_at, cap, R, tracker['gate_nis'], tracker['max_outliers'],
                             tracker.get('facing_std') if len(dets) == 1 else None)
            n_updates += 1
            now = max(arrive, cap)
            state, _ = ekf.predicted(now)
            msg = _state_msg(state, ekf.other_r, n_updates >= 2)
            c, v, y, wr = target.truth()  # truth at `now` == t
            rec = state_errors(msg, (c, v, y, wr), stagger, PANEL_RADII, (cam[0], cam[1]))
            rec.update({'t': now, 'valid': msg.valid, 'age_on_arrival_s': 0.0})
            records.append(rec)
    return summarize_case(records, start_s, settle)


def _noise(rng, ray, emulator):
    rng_m = float(np.linalg.norm(ray))
    u = ray / rng_m
    lateral = rng.normal(0.0, emulator['lateral_rad'] * rng_m, 3)
    lateral -= (lateral @ u) * u
    return (rng.normal(0.0, emulator['noise'], 3) + lateral
            + u * rng.normal(0.0, emulator['depth_coeff'] * rng_m ** 2))


class _Vec:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Msg:
    pass


def _state_msg(state, other_r, valid):
    m = _Msg()
    m.center = _Vec(*state[POS])
    m.velocity = _Vec(*state[VEL])
    m.yaw, m.yaw_rate = float(state[YAW]), float(state[W])
    m.radius = [float(state[R]), float(other_r)]
    m.z_offset = [float(state[DZ]), float(-state[DZ])]
    m.valid = valid
    return m


def cells(speeds=(0.5, 1.0, 2.0, 4.0)):
    for layout in ('flat', 'staggered'):
        yield layout, 0.0, 0.0
        for s in speeds:
            frac = (s - min(speeds)) / (max(speeds) - min(speeds))
            yield layout, s, 2.0 - frac


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    p.add_argument('--path', default='lateral', choices=list(PATHS))
    p.add_argument('--shooter-speed', type=float, default=0.0)
    p.add_argument('--blackout', action='store_true')
    p.add_argument('--camera-latency', type=float, default=0.0)
    p.add_argument('--duration', type=float, default=30.0)
    p.add_argument('--seeds', type=int, default=3)
    p.add_argument('--only', default='', help='substring filter on the cell name')
    p.add_argument('--set', action='append', default=[], help='tracker param=value')
    p.add_argument('--d435', action='store_true', help='D435-like ray noise, not 5 mm')
    p.add_argument('--metrics', default='facing_panel_m,center_m,velocity_m_s,yaw_rate_rad_s')
    args = p.parse_args(argv)
    tracker = dict(TRACKER, extra=dict(TRACKER['extra']))
    for kv in args.set:
        k, v = kv.split('=')
        if k in EKF_KWARGS:
            tracker.setdefault('extra', {})[k] = float(v)
        else:
            tracker[k] = float(v)
    emulator = dict(EMULATOR, **(D435 if args.d435 else {}))
    keys = args.metrics.split(',')
    print(f'{"cell":44s} conv_s  valid ' + ' '.join(f'{k[:14]:>14s}' for k in keys))
    for layout, speed, spin in cells():
        cell = cell_id(layout, speed, args.path, args.shooter_speed, args.blackout,
                       args.camera_latency)
        if args.only and args.only not in cell:
            continue
        runs = [run_cell(layout, speed, spin, args.path, args.shooter_speed, args.blackout,
                         args.duration, tracker, emulator, seed, args.camera_latency)
                for seed in range(args.seeds)]
        conv = max((r['converge_s'] if r['converge_s'] is not None else 99) for r in runs)
        valid = min(r['valid_fraction'] for r in runs)
        cols = []
        for k in keys:
            p95 = max(r[k]['p95'] for r in runs if r.get(k))
            cols.append(f'{p95:14.4f}')
        print(f'{cell:44s} {conv:6.2f} {valid:5.2f} ' + ' '.join(cols))
    return 0


if __name__ == '__main__':
    assert set(METRICS)  # imported for --metrics names
    sys.exit(main())
