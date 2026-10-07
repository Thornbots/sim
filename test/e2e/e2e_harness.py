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

"""Score E1-E4 shots and route diagnostics against stamped Gazebo truth.

E1 samples fresh CV aims at 10 Hz; E2 onward uses actual firmware shots.
Flag shots are hypothetical and never damage HP. E4 scores ballistic first
impacts for all four robots and returns referee state through the MCB UART.
See README.md for design rationale; test_e1.py through test_e4.py assert results.
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
from sim.match_scenario import E1_PATHS as TARGET_PATHS
from sim.match_scenario import ingress_duration, match_duration, sample_match, TEAMS

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
PLACEHOLDER_FLOOR = 0.10  # until three runs give FLOORS
FLOORS = {}
# The nodes a run needs.
STACK_NODES = ['detector_standin', 'target_selector', 'target_tracker', 'point_to_cv_target',
               'cv_head_aim', 'opponent_driver', 'target_driver']
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
        self.match_start = None
        self._reference = []
        self._pending_reference = []
        self.route_records = []
        self.launch_probe = None
        if stage in ('e3', 'e4'):
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, '/sim/match/reference', self._on_reference, 10)
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
        self.score_until = math.inf
        self.shot_kinds = ['rate', 'flag', 'mcb']
        if stage == 'e4':
            self._setup_match()
        self.shots = {kind: [] for kind in self.shot_kinds}
        self.create_timer(1.0 / FIRE_HZ, self._fire_tick)
        if stage != 'e1':
            from std_msgs.msg import Header
            self.create_subscription(Header, '/mcb_emulator/shot', self._on_mcb_shot, 100)

    def wait_until(self, predicate, timeout, description):
        def ready():
            if self.launch_probe is not None and self.launch_probe.proc.poll() is not None:
                raise RuntimeError('E2E stack exited during bring-up; see stack.log')
            return predicate()
        if not super().wait_until(ready, timeout, description):
            raise RuntimeError(f'E2E bring-up failed: {description}; see stack.log')

    def spin_for(self, seconds):
        start = self.now_s()
        super().spin_for(seconds)
        if self.now_s() - start < seconds - 0.01:
            raise RuntimeError('E2E clock stalled; scoring window did not complete')

    def reset(self):
        self._pending, self.shots, self.scoring = [], {k: [] for k in self.shot_kinds}, True
        self.states, self._pending_states = [], []
        self.route_records, self._pending_reference = [], []
        self.score_until = math.inf
        if self.stage == 'e4':
            self._flight_steps = {}

    def _on_target(self, msg):
        t = self._stamp_s(msg.header.stamp)
        self._target = t
        self._aims.append((t, np.array([msg.x, msg.y, msg.z])))
        while self._aims and self._aims[0][0] < t - HISTORY_S:
            del self._aims[0]
        if msg.fire and self.scoring and t < self.score_until:
            self._pending.append(('flag', t + msg.delay_ms / 1000.0 + FIRE_LATENCY_S))

    def _on_mcb_shot(self, msg):
        if self.scoring and self._stamp_s(msg.stamp) < self.score_until:
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
        pending_reference = []
        for t in self._pending_reference:
            if t > now - 0.1:
                pending_reference.append(t)
            else:
                self.route_records.append({'t': t, **self._pose_errors(t)})
        self._pending_reference = pending_reference
        still = []
        completed = []
        for kind, t_launch in self._pending:
            if self.stage == 'e4':
                shot = self._judge_match(kind, t_launch)
                if shot is None:
                    still.append((kind, t_launch))
                else:
                    completed.append((kind, shot))
                continue
            if now < t_launch + RESOLVE_AFTER_S:
                still.append((kind, t_launch))
                continue
            shot = self._judge(t_launch)
            if shot is not None:
                shot['kind'] = kind
                self.shots[kind].append(shot)
                with open(self.log_path, 'a') as f:
                    f.write(json.dumps(shot) + '\n')
        for kind, shot in sorted(completed, key=lambda item: item[1]['impact_t']):
            actual = kind != 'flag'
            shot['hit'] = self.referee.apply(shot) if actual else bool(shot['eligible'])
            shooter, victim = shot['shooter'], shot['victim']
            shot['friendly_intersection'] = bool(
                victim is not None and TEAMS[shooter] == TEAMS[victim])
            shot['enemy_hit'] = shot['hit'] and not shot['friendly_intersection']
            if actual and shot['hit'] and victim == 'sentry':
                self.hurt_armor_id = shot['panel']
            shot['hp'] = dict(self.referee.hp)
            shot['kind'] = kind
            self.shots[kind].append(shot)
            with open(self.log_path, 'a') as stream:
                stream.write(json.dumps(shot) + '\n')
        self._pending = still
        if self.stage == 'e4':
            self._sync_referee()

    def _setup_match(self):
        from diagnostic_msgs.msg import DiagnosticArray
        from sim.combat import Referee, ShotResolver
        from std_msgs.msg import Header
        self.histories = {'sentry': self.ours, 'opponent_0': self.theirs}
        from gz.msgs10.pose_v_pb2 import Pose_V
        for name in ['opponent_1', 'ally_0']:
            history = PoseHistory()
            self.histories[name] = history
            self._gz.subscribe(Pose_V, f'/model/{name}/pose',
                               lambda msg, robot=name, hist=history: hist.add(robot, msg))
        for name in ['opponent_0', 'opponent_1', 'ally_0']:
            self.shot_kinds.append(name)
            self.create_subscription(
                Header, f'/sim/match/{name}/shot',
                lambda msg, robot=name: self._on_other_shot(robot, msg), 100)
        self.resolver = ShotResolver(get_package_share_directory('sim'), self.armors)
        self.referee = Referee(TEAMS)
        self._flight_steps = {}
        self.hurt_armor_id = 0
        from dji_serial_bridge.msg import RefSysStatus
        self.referee_messages = []
        self.create_subscription(RefSysStatus, '/dji_serial_bridge/ref_sys',
                                 self.referee_messages.append, qos_profile_sensor_data)
        self._referee_client = self.create_client(SetParameters, '/mcb_emulator/set_parameters')
        self._referee_future = None
        self.referee_pub = self.create_publisher(DiagnosticArray, '/sim/match/referee', 10)

    def _on_other_shot(self, robot, msg):
        if self.scoring and self._stamp_s(msg.stamp) < self.score_until:
            self._pending.append((robot, self._stamp_s(msg.stamp) + FIRE_LATENCY_S))

    def _judge_match(self, kind, launch):
        shooter = 'sentry' if kind in ('mcb', 'flag') else kind
        available = min(history.newest() or 0.0 for history in self.histories.values())
        if launch > available:
            return None
        pose = self.histories[shooter].at(launch)
        if pose is None or pose[1] is None:
            raise RuntimeError(f'missing muzzle truth at {launch}: {shooter}')
        muzzle = pose[1] @ self.muzzle
        key = (kind, launch)
        impact = self.resolver.resolve(shooter, muzzle[:3, 3], muzzle[:3, 0], launch,
                                       self.histories, until=min(self.now_s() - .02, available),
                                       start_step=self._flight_steps.get(key, 0))
        if impact.get('pending'):
            self._flight_steps[key] = impact['next_step']
            return None
        self._flight_steps.pop(key, None)
        out = {'t': launch, 'shooter': shooter, **impact, **self._pose_errors(launch)}
        if shooter == 'sentry':
            victim = impact['victim']
            history = self.histories.get(victim, self.theirs)
            index = impact['panel'] if impact['panel'] is not None else 0
            out.update(self._aim_split(launch, muzzle[:3, 3], muzzle[:3, 0], index,
                                       history=history))
        return out

    def _sync_referee(self):
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        elapsed = self.now_s() - (self.match_start or self.now_s())
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, hp in self.referee.hp.items():
            msg.status.append(DiagnosticStatus(
                name=name, hardware_id=name,
                level=DiagnosticStatus.OK if hp else DiagnosticStatus.ERROR,
                message='alive' if hp else 'defeated',
                values=[KeyValue(key='hp', value=str(hp)),
                        KeyValue(key='team', value=TEAMS[name])]))
        self.referee_pub.publish(msg)
        if self._referee_future is not None and not self._referee_future.done():
            return
        if self._referee_future is not None:
            result = self._referee_future.result()
            if result is None or not all(r.successful for r in result.results):
                raise RuntimeError('MCB rejected match referee status')
        if not self._referee_client.service_is_ready():
            return
        params = {'current_hp': self.referee.hp['sentry'],
                  'game_stage': (5 if elapsed >= match_duration('e4') else
                                 4 if elapsed >= ingress_duration('e4') else 3),
                  'stage_time_remaining': max(0, 300 - int(elapsed)),
                  'hurt_armor_id': self.hurt_armor_id,
                  'shooter_power': self.referee.hp['sentry'] > 0}
        self._referee_future = self._referee_client.call_async(SetParameters.Request(
            parameters=[Parameter(name, value=value).to_parameter_msg()
                        for name, value in params.items()]))

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
            direction = _dropped(barrel, along) if self.stage != 'e1' else barrel
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
                **self._pose_errors(t_launch),
                'miss_m': round(miss, 4),
                'off_face_m': round(off, 4) if math.isfinite(off) else None,
                'incidence_deg': round(math.degrees(incidence), 1), 'range_m': round(rng, 3)}

    def _on_reference(self, msg):
        p = msg.pose.pose.position
        t = self._stamp_s(msg.header.stamp)
        self._reference.append((t, np.array([p.x, p.y])))
        if self.scoring:
            self._pending_reference.append(t)
        self._reference = [r for r in self._reference if r[0] >= t - HISTORY_S]

    def _pose_errors(self, t):
        """Route and map-localization error at the shot stamp, without a latest-TF fallback."""
        from rclpy.time import Time
        ours = self.ours.at(t)
        if ours is None or self.match_start is None:
            return {}
        segment = sample_match(t - self.match_start, self.stage)[3]
        refs = [p for stamp, p in self._reference if stamp <= t]
        out = {'segment': segment,
               'chassis_tilt_deg': math.degrees(math.acos(float(np.clip(ours[0][2, 2], -1, 1))))}
        world_T_odom = self._world_T_odom(t)
        try:
            tr = self._tf.lookup_transform('odom', 'head_pitch', Time(seconds=t)).transform
            head = world_T_odom @ _iso_qt(tr.rotation, tr.translation)
            diff = head[:3, :3].T @ ours[1][:3, :3]
            out['head_tf_error_deg'] = math.degrees(math.acos(float(np.clip(
                (np.trace(diff) - 1) / 2, -1, 1))))
            out['head_tf_translation_m'] = float(np.linalg.norm(head[:3, 3] - ours[1][:3, 3]))
        except Exception:  # noqa: B902; missing truth or TF remains visible in the shot record
            out['head_tf_error_deg'] = None
        if refs:
            out['route_error_m'] = float(np.linalg.norm(ours[0][:2, 3] - refs[-1]))
        try:
            tr = self._tf.lookup_transform('map', 'root', Time(seconds=t)).transform
            out['localization_error_m'] = float(np.hypot(
                tr.translation.x - ours[0][0, 3], tr.translation.y - ours[0][1, 3]))
        except Exception:  # noqa: B902; missing stamped TF is a diagnostic, never silently latest
            out['localization_error_m'] = None
        return out

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

    def _aim_split(self, t_launch, origin, direction, k, history=None):
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
        theirs = (history or self.theirs).at(
            t_launch + float(np.dot(aim - origin, direction)) / MUZZLE_SPEED)
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
        self.stage = stage
        self.nodes = STACK_NODES if stage == 'e1' else E2_NODES
        if stage in ('e3', 'e4'):
            self.nodes = [n for n in self.nodes if n != 'target_driver'] + ['match_driver']
        if stage == 'e4':
            self.nodes += ['opponent_driver_opponent_1', 'opponent_driver_ally_0']
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
            s = self.scorer
            suite_timing.set_sim_clock(s.now_s)
            if self.launch is not None:
                self.launch.start()
                s.launch_probe = self.launch
            s.wait_until(lambda: s.theirs.newest() is not None, timeout=120.0,
                         description=f'{OPPONENT} in gz')
            s.wait_until(lambda: s.nodes_up(*self.nodes), timeout=30.0,
                         description=f'{", ".join(self.nodes)} nodes up')
            bench.check_nodes(s, self.nodes)
            if self.stage == 'e4':
                s.wait_until(lambda: all(h.newest() is not None for h in s.histories.values()),
                             timeout=120.0, description='all four robot truth streams')
            if self.stage in ('e3', 'e4'):
                from rclpy.time import Time
                s.wait_until(lambda: s._tf.can_transform('map', 'root', Time()),
                             timeout=30.0, description='localized map->root chain')
            s.wait_until(lambda: s._target is not None,
                         timeout=60.0, description='a /cv/target aim point')

    def start_match(self):
        client = self.node.create_client(SetParameters, '/match_driver/set_parameters')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('match_driver parameter service missing')
        req = SetParameters.Request(parameters=[
            Parameter('active', value=True).to_parameter_msg()])
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
        result = future.result()
        if result is None or not all(r.successful for r in result.results):
            raise RuntimeError('match_driver rejected scenario start')
        self.scorer.match_start = self.scorer.now_s()

    def set_target(self, speed, spin_hz, path):
        if not self._set_params.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/target_driver/set_parameters not available')
        params = {'target_speed': speed, 'spin_hz': spin_hz, **TARGET_PATHS[path]}
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
        suite_timing.set_sim_clock(None)
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


def run_match(stack):
    """Score approach and center maneuvers in one physics session."""
    stack.scorer.reset()
    stack.start_match()
    stack.scorer.score_until = stack.scorer.match_start + match_duration(stack.stage)
    stack.scorer.spin_for(match_duration(stack.stage) + RESOLVE_AFTER_S + FIRE_LATENCY_S + 0.1)
    stack.scorer.scoring = False
    bench.check_nodes(stack.scorer, stack.nodes)
    with open(os.path.join(stack.log_dir, 'route.jsonl'), 'w') as stream:
        for record in stack.scorer.route_records:
            stream.write(json.dumps(record) + '\n')
    return stack.scorer.shots


def segment_diagnostics(stack, segment, shots):
    """Name the measured hop responsible for a low score; accuracy stays in the report."""
    records = [r for r in stack.scorer.route_records if r.get('segment') == segment]
    localization = [r['localization_error_m'] for r in records
                    if r.get('localization_error_m') is not None]
    head = [r['head_tf_error_deg'] for r in records if r.get('head_tf_error_deg') is not None]
    if not shots:
        return 'target_tracker/point_to_cv_target: no firmware shots during this segment'
    if localization and np.percentile(localization, 95) > 0.40:
        return 'localization: map->root p95 error exceeds 0.40 m'
    if head and np.percentile(head, 95) > 1.0:
        return 'pose/TF stamps: head_pitch p95 attitude error exceeds 1 degree'
    aim = [s['aim_off_panel_m'] for s in shots if 'aim_off_panel_m' in s]
    if aim and np.median(aim) > 0.10:
        return 'target_tracker/point_to_cv_target: median aim error exceeds 0.10 m'
    barrel = [s['barrel_off_aim_deg'] for s in shots if 'barrel_off_aim_deg' in s]
    if barrel and np.median(barrel) > 1.0:
        return 'MCB/gimbal: median barrel error exceeds 1 degree'
    return None
