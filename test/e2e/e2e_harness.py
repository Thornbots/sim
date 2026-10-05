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
Match test machinery, stage E1: scores the real CV chain's aim from gz truth.

e2e.launch.py runs the stack. While /cv/target keeps sending aim
points, a shot leaves every 1/FIRE_HZ s (the
firmware's indexer rate while it holds a target) from the gz muzzle, along
the barrel, FIRE_LATENCY_S after the decision, at 25 m/s. It hits if it
crosses a canted armor face facing it within 72.5 deg, with each panel
taken from gz's pose at the shot's arrival. Shots CVTarget.fire asks for
are scored too, as `flag` shots, but don't set the pass. In stage E2 the
MCB emulator fires instead: each ~/shot it reports is an `mcb` shot,
launched FIRE_LATENCY_S after its stamp, and every shot falls under
gravity, since the firmware pitches up for it. Importable only; test_e1.py and
test_e2.py hold the assertions. Borrows the aiming bench's process and
node helpers and its face test from ../cv/shot_hit_harness.py.
"""
import bisect
import json
import math
import os
import subprocess
import sys
import threading
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from dji_serial_bridge.msg import CVTarget, TargetState
import numpy as np
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sim import suite_timing

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cv'))
import shot_hit_harness as bench  # noqa: E402, I100

OPPONENT = 'opponent_0'
MUZZLE_SPEED = bench.MUZZLE_SPEED
GRAVITY = 9.80665  # m/s^2, E2 shots only: E1's cv_head_aim aims straight
FIRE_HZ = 10.0  # the firmware's indexer rate while it holds a target
FIRE_LATENCY_S = bench.FIRE_LATENCY_S
TARGET_FRESH_S = 0.1  # a /cv/target older than this holds fire
EXPOSURE_HALF_ANGLE = bench.PANEL_EXPOSURE_HALF_ANGLE
HISTORY_S = 3.0  # gz poses kept for resolving shots
RESOLVE_AFTER_S = 0.5  # sim s after launch before a shot is judged
SETTLE_S = 3.0
DEFAULT_DURATION = 20.0
DEFAULT_SPEEDS = [0.0, 1.0, 2.0, 4.0]
DEFAULT_PATHS = ['lateral', 'radial', 'diagonal']
DEFAULT_LOG_DIR = '/tmp/e2e_test_logs'
# Every path runs the opponent through field obstacles (the user, 2026-10-05):
# no cell is valid until the paths move. pytest skips them all.
INVALID = 'opponent path runs through field obstacles; paths need moving'
PLACEHOLDER_FLOOR = 0.10  # until three runs give FLOORS
FLOORS = {}
# The nodes a run needs; the camera container can lose a load at startup.
STACK_NODES = ['detector_standin', 'depth_camera_emulator', 'roi_depth_node',
               'target_selector', 'target_tracker', 'point_to_cv_target', 'cv_head_aim',
               'opponent_driver', 'target_driver']
E2_NODES = [n for n in STACK_NODES if n != 'cv_head_aim'] + [
    'mcb_emulator', 'dji_serial_bridge', 'mcb_relay']


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _joint_origin(root, child):
    joint = next(j for j in root.iter('joint') if j.find('child').get('link') == child)
    origin = joint.find('origin')
    xyz = [float(v) for v in (origin.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
    t = np.eye(4)
    t[:3, :3] = _rpy(*rpy)
    t[:3, 3] = xyz
    return t


def urdf_offsets():
    """(root_T_armor list, head_pitch_T_muzzle) from sentry_v2's URDF."""
    xacro_file = os.path.join(get_package_share_directory('sim'), 'urdf', 'sentry_v2.urdf.xacro')
    root = ET.fromstring(subprocess.run(['xacro', xacro_file], check=True,
                                        capture_output=True, text=True).stdout)
    armors = sorted(link.get('name') for link in root.iter('link')
                    if link.get('name').startswith('armor_'))
    return [_joint_origin(root, a) for a in armors], _joint_origin(root, 'muzzle')


def _iso(p):
    return _iso_qt(p.orientation, p.position)


def _iso_qt(q, p):
    """4x4 from a quaternion and a translation (anything with w/x/y/z and x/y/z)."""
    w, x, y, z = q.w, q.x, q.y, q.z
    t = np.eye(4)
    t[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                 [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                 [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    t[:3, 3] = [p.x, p.y, p.z]
    return t


class PoseHistory:
    """A model's world_T_root and world_T_head_pitch from gz, looked up by sim time."""

    def __init__(self):
        self._lock = threading.Lock()
        self._t, self._root, self._head = [], [], []

    def add(self, name, msg):
        e = {p.name: p for p in msg.pose}
        if name not in e or f'{name}::root' not in e:
            return
        stamp = e[name].header.stamp
        model = _iso(e[name])
        with self._lock:
            t = stamp.sec + stamp.nsec * 1e-9
            if self._t and t <= self._t[-1]:
                return
            self._t.append(t)
            self._root.append(model @ _iso(e[f'{name}::root']))
            head = e.get(f'{name}::head_pitch')
            self._head.append(model @ _iso(head) if head is not None else None)
            while self._t and self._t[0] < t - HISTORY_S:
                del self._t[0], self._root[0], self._head[0]

    def newest(self):
        with self._lock:
            return self._t[-1] if self._t else None

    def at(self, t):
        """(world_T_root, world_T_head_pitch) at t: nearest sample, 4 ms apart."""
        with self._lock:
            if not self._t or t < self._t[0] or t > self._t[-1]:
                return None
            i = bisect.bisect_left(self._t, t)
            if i > 0 and (i == len(self._t) or t - self._t[i - 1] < self._t[i] - t):
                i -= 1
            return self._root[i], self._head[i]


def _dropped(barrel, along):
    """Return the unit chord from the muzzle to where a shot has fallen at range along."""
    t = along / MUZZLE_SPEED
    chord = along * barrel - np.array([0.0, 0.0, GRAVITY / 2.0 * t * t])
    norm = float(np.linalg.norm(chord))
    return chord / norm if norm > 0.0 else barrel


class E2EScorer(bench.SimTimeNode):
    """Fires by the firmware's rule on /cv/target and scores every shot from gz truth."""

    def __init__(self, armors, muzzle, log_path, stage='e1'):
        super().__init__('e2e_scorer')
        self.stage = stage
        self.armors, self.muzzle = armors, muzzle
        self.log_path = log_path
        self.ours, self.theirs = PoseHistory(), PoseHistory()
        import gz.msgs10.pose_pb2  # noqa: F401; before any Pose_V parse
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self._gz = GzNode()
        self._gz.subscribe(Pose_V, '/model/sentry/pose', lambda m: self.ours.add('sentry', m))
        self._gz.subscribe(Pose_V, f'/model/{OPPONENT}/pose',
                           lambda m: self.theirs.add(OPPONENT, m))
        self._target = None  # stamp_s of the newest aim point
        self._aims = []  # (stamp_s, odom point) of each confident /cv/target
        import tf2_ros
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)
        self.create_subscription(CVTarget, '/cv/target', self._on_target,
                                 qos_profile_sensor_data)
        self.create_subscription(TargetState, '/cv/target_state', self._on_state, 10)
        self.states = []  # per TargetState while scoring: its error against gz truth
        self._pending_states = []  # judged once truth 20 ms past their stamp is in
        self._pending = []
        self.scoring = False
        self.shots = {'rate': [], 'flag': [], 'mcb': []}
        self.create_timer(1.0 / FIRE_HZ, self._fire_tick)
        if stage == 'e2':
            from std_msgs.msg import Header
            self.create_subscription(Header, '/mcb_emulator/shot', self._on_mcb_shot, 100)

    def reset(self):
        self._pending, self.shots, self.scoring = [], {'rate': [], 'flag': [], 'mcb': []}, True
        self.states, self._pending_states = [], []

    def _on_target(self, msg):
        t = self._stamp_s(msg.header.stamp)
        self._target = t
        self._aims.append((t, np.array([msg.x, msg.y, msg.z])))
        while self._aims and self._aims[0][0] < t - HISTORY_S:
            del self._aims[0]
        if msg.fire and self.scoring:
            self._pending.append(('flag', t + msg.delay_ms / 1000.0 + FIRE_LATENCY_S))

    def _on_mcb_shot(self, msg):
        if self.scoring:
            self._pending.append(('mcb', self._stamp_s(msg.stamp) + FIRE_LATENCY_S))

    def _fire_tick(self):
        now = self.now_s()
        if self.scoring and self.stage == 'e1' and self._target is not None:
            if now - self._target <= TARGET_FRESH_S:
                self._pending.append(('rate', now + FIRE_LATENCY_S))
        ready = [m for m in self._pending_states if self._stamp_s(m.header.stamp) < now - 0.1]
        self._pending_states = [m for m in self._pending_states if m not in ready]
        for m in ready:
            self._judge_state(m)
        still = []
        for kind, t_launch in self._pending:
            if now < t_launch + RESOLVE_AFTER_S:
                still.append((kind, t_launch))
                continue
            shot = self._judge(t_launch)
            if shot is not None:
                shot['kind'] = kind
                self.shots[kind].append(shot)
                with open(self.log_path, 'a') as f:
                    f.write(json.dumps(shot) + '\n')
        self._pending = still

    def _judge(self, t_launch):
        ours = self.ours.at(t_launch)
        if ours is None or ours[1] is None:
            return None
        muzzle = ours[1] @ self.muzzle
        origin, barrel = muzzle[:3, 3], muzzle[:3, 0]
        best = None  # (off_face, miss, k, incidence, range, direction)
        for k, root_T_armor in enumerate(self.armors):
            theirs = self.theirs.at(t_launch)
            if theirs is None:
                return None
            centre = (theirs[0] @ root_T_armor)[:3, 3]
            along = max(float(np.dot(centre - origin, barrel)), 0.0)
            direction = _dropped(barrel, along) if self.stage == 'e2' else barrel
            theirs = self.theirs.at(t_launch + along / MUZZLE_SPEED)
            if theirs is None:
                return None
            panel = theirs[0] @ root_T_armor
            pos, normal = panel[:3, 3], panel[:3, 0]
            to_muzzle = origin - pos
            incidence = math.acos(float(np.clip(
                np.dot(normal, to_muzzle / np.linalg.norm(to_muzzle)), -1.0, 1.0)))
            along = float(np.dot(pos - origin, direction))
            miss = float(np.linalg.norm(pos - (origin + along * direction)))
            off = bench.off_face(origin, direction, (pos, normal, panel[:3, 1], panel[:3, 2]))
            facing = incidence <= EXPOSURE_HALF_ANGLE
            cand = (off if facing else math.inf, miss, k, incidence, along, direction)
            if best is None or cand[:2] < best[:2]:
                best = cand
        off, miss, k, incidence, rng, direction = best
        return {'t': round(t_launch, 4), 'hit': off == 0.0, 'panel': k,
                **self._aim_split(t_launch, origin, direction, k),
                'miss_m': round(miss, 4),
                'off_face_m': round(off, 4) if math.isfinite(off) else None,
                'incidence_deg': round(math.degrees(incidence), 1), 'range_m': round(rng, 3)}

    def _world_T_odom(self, t):
        """
        Our true pose times TF's root->odom at t, so localization error drops out.

        TF's root is heading-fixed, gz's root link turns with the spawn yaw:
        the true pose drops its yaw.
        """
        from rclpy.time import Time
        ours = self.ours.at(t)
        try:
            tr = self._tf.lookup_transform('odom', 'root', Time(seconds=t)).transform
        except Exception:  # noqa: B902; a stamp at `now` isn't in TF yet: take the newest
            try:
                tr = self._tf.lookup_transform('odom', 'root', Time()).transform
            except Exception:  # noqa: B902
                return None
        if ours is None:
            return None
        world_T_root = ours[0].copy()
        yaw = math.atan2(world_T_root[1, 0], world_T_root[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        unyaw = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        world_T_root[:3, :3] = unyaw @ world_T_root[:3, :3]
        return world_T_root @ np.linalg.inv(_iso_qt(tr.rotation, tr.translation))

    def _on_state(self, msg):
        if self.scoring and msg.valid:
            self._pending_states.append(msg)

    def _judge_state(self, msg):
        """Log a valid TargetState's centre, velocity and spin against gz truth at its stamp."""
        t = self._stamp_s(msg.header.stamp)
        world_T_odom = self._world_T_odom(t)
        before, after = self.theirs.at(t - 0.02), self.theirs.at(t + 0.02)
        if world_T_odom is None or before is None or after is None:
            return
        truth = self.theirs.at(t)[0]
        c = msg.center
        centre = (world_T_odom @ np.array([c.x, c.y, c.z, 1.0]))[:3]
        vel = world_T_odom[:3, :3] @ np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        true_vel = (after[0][:3, 3] - before[0][:3, 3]) / 0.04
        yaw = [math.atan2(m[0][1, 0], m[0][0, 0]) for m in (before, after)]
        dyaw = yaw[1] - yaw[0]
        true_rate = math.atan2(math.sin(dyaw), math.cos(dyaw)) / 0.04
        self.states.append({
            'centre_xy_m': float(np.hypot(*(centre - truth[:3, 3])[:2])),
            'vel_err': (vel - true_vel).tolist(),
            'yaw_rate_err': abs(msg.yaw_rate) - abs(true_rate),
        })

    def _aim_split(self, t_launch, origin, direction, k):
        """
        Split a shot's error: the barrel off the newest aim, and the aim off panel k's centre.

        The aim is an odom point, taken to the world through our true pose and
        TF's odom->root at the decision (localization error drops out).
        """
        t_decide = t_launch - FIRE_LATENCY_S
        aims = [a for a in self._aims if a[0] <= t_decide]
        world_T_odom = self._world_T_odom(t_decide)
        if not aims or world_T_odom is None:
            return {}
        aim = (world_T_odom @ np.append(aims[-1][1], 1.0))[:3]
        to_aim = (aim - origin) / np.linalg.norm(aim - origin)
        theirs = self.theirs.at(t_launch + float(np.dot(aim - origin, direction)) / MUZZLE_SPEED)
        out = {'barrel_off_aim_deg': round(math.degrees(math.acos(
            float(np.clip(np.dot(direction, to_aim), -1.0, 1.0)))), 2)}
        if theirs is not None:
            panel = (theirs[0] @ self.armors[k])[:3, 3]
            out['aim_off_panel_m'] = round(float(np.linalg.norm(aim - panel)), 3)
            out['aim_minus_panel'] = [round(float(v), 3) for v in aim - panel]
        return out


class E2EStack:
    """
    Brings up e2e.launch.py once per run, or waits on the one that started pytest.

    Between cases only target_driver's path, speed and spin change.
    """

    def __init__(self, headless, log_dir, external=False, stage='e1', firmware_fixes=True):
        self.launch = None
        self.log_dir = log_dir
        self.nodes = E2_NODES if stage == 'e2' else STACK_NODES
        if not external:
            self.launch = bench.LaunchTree(
                'stack', ['ros2', 'launch', 'sim', 'e2e.launch.py', 'run_tests:=false',
                          f'headless:={str(headless).lower()}', f'stage:={stage}',
                          f'firmware_fixes:={str(firmware_fixes).lower()}'],
                os.path.join(log_dir, 'stack.log'))
        self.shots_path = os.path.join(log_dir, 'shots.jsonl')
        self.scores_path = os.path.join(log_dir, 'scores.jsonl')
        self.node = rclpy.create_node(
            'e2e_stack',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._set_params = self.node.create_client(
            SetParameters, '/target_driver/set_parameters')
        armors, muzzle = urdf_offsets()
        self.scorer = E2EScorer(armors, muzzle, self.shots_path, stage)

    def start(self):
        for path in (self.shots_path, self.scores_path):
            open(path, 'w').close()
        with suite_timing.phase('bringup'):
            if self.launch is not None:
                self.launch.start()
            s = self.scorer
            s.wait_until(lambda: s.theirs.newest() is not None, timeout=120.0,
                         description=f'{OPPONENT} in gz')
            s.wait_until(lambda: s.nodes_up(*self.nodes), timeout=30.0,
                         description=f'{", ".join(self.nodes)} nodes up')
            bench.check_nodes(s, self.nodes)
            s.wait_until(lambda: s._target is not None,
                         timeout=60.0, description='a /cv/target aim point')

    def set_target(self, speed, spin_hz, path):
        if not self._set_params.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/target_driver/set_parameters not available')
        params = {'target_speed': speed, 'spin_hz': spin_hz, **bench.TARGET_PATHS[path]}
        req = SetParameters.Request(parameters=[
            Parameter(k, Parameter.Type.DOUBLE, float(v)).to_parameter_msg()
            for k, v in params.items()])
        future = self._set_params.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not all(r.successful for r in result.results):
            raise RuntimeError(f'target_driver rejected {params}')

    def stop(self):
        with suite_timing.phase('teardown'):
            if self.launch is not None:
                self.launch.stop()
        self.scorer.destroy_node()
        self.node.destroy_node()


def cell_id(speed, path, spin_override=None):
    """stationary-lateral, speed2-radial; with --e2e-spin, speed2-radial-spin0."""
    name = f"{'stationary' if speed == 0.0 else f'speed{speed:g}'}-{path}"
    return name if spin_override is None else f'{name}-spin{spin_override:g}'


def state_summary(states):
    """Medians of a case's TargetState errors, for the log."""
    if not states:
        return 'no valid TargetState'
    med = np.median
    v = np.array([x['vel_err'] for x in states])
    return (f"{len(states)} states: centre xy {med([x['centre_xy_m'] for x in states]):.3f} m, "
            f'|vel err| {med(np.linalg.norm(v, axis=1)):.2f} m/s (z {med(np.abs(v[:, 2])):.2f}), '
            f"|yaw rate| - true {med([x['yaw_rate_err'] for x in states]):+.2f} rad/s (p50)")


def run_case(stack, speed, spin_hz, path, duration):
    """Move the opponent to (speed, spin_hz, path), settle, then fire and score for duration."""
    s = stack.scorer
    with suite_timing.phase('reset'):
        stack.set_target(speed, spin_hz, path)
        s.scoring = False
    with suite_timing.phase('settle'):
        s.spin_for(SETTLE_S)
    s.reset()
    s.spin_for(duration + RESOLVE_AFTER_S)
    s.scoring = False
    bench.check_nodes(s, stack.nodes)
    print(state_summary(s.states))
    return s.shots


def hit_rate(shots):
    return sum(x['hit'] for x in shots) / len(shots) if shots else 0.0


def record_score(stack, cell, shots, duration):
    """One line per case in scores.jsonl, for a later FLOORS."""
    with open(stack.scores_path, 'a') as f:
        f.write(json.dumps({
            'cell': cell, 'duration_s': duration,
            'rate_shots': len(shots['rate']), 'rate_hit_rate': round(hit_rate(shots['rate']), 4),
            'flag_shots': len(shots['flag']), 'flag_hit_rate': round(hit_rate(shots['flag']), 4),
            'mcb_shots': len(shots['mcb']), 'mcb_hit_rate': round(hit_rate(shots['mcb']), 4),
        }) + '\n')
