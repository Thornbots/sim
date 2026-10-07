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

"""Ballistic first-impact scoring against field triangles, robot hulls and armor faces."""
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.80665])
MUZZLE_SPEED = 25.0
STEP_S = 0.01  # ballistic chords; maximum sag error 0.13 mm
FLIGHT_S = 0.5
PANEL_SIZE = (0.135, 0.125)


def pose_matrix(pose):
    """Homogeneous transform from a standard pose quaternion and translation."""
    q, p = pose.orientation, pose.position
    w, x, y, z = q.w, q.x, q.y, q.z
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                   [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                   [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def armor_offsets():
    """Load the four fixed armor transforms from the expanded sentry URDF."""
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('sim'))
    xml = subprocess.run(['xacro', str(share / 'urdf/sentry_v2.urdf.xacro')],
                         check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(xml)
    offsets = []
    for index in range(4):
        joint = next(j for j in root.iter('joint')
                     if j.find('child').get('link') == f'armor_{index}')
        origin = joint.find('origin')
        x, y, z = [float(v) for v in origin.get('xyz').split()]
        r, p, yaw = [float(v) for v in origin.get('rpy').split()]
        cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                                  math.cos(yaw), math.sin(yaw))
        matrix = np.eye(4)
        matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                          [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
        matrix[:3, 3] = [x, y, z]
        offsets.append(matrix)
    return offsets


def load_stl(path):
    """Read the binary STL triangles shipped with the field and robot."""
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    return np.fromfile(path, dtype=dtype, offset=84)['vertices'].astype(float)


def segment_mesh_hit(start, end, triangles):
    """Earliest two-sided triangle intersection along a finite segment, or None."""
    if not len(triangles):
        return None
    low, high = np.minimum(start, end), np.maximum(start, end)
    candidates = triangles[(triangles.min(axis=1) <= high + 1e-9).all(axis=1)
                           & (triangles.max(axis=1) >= low - 1e-9).all(axis=1)]
    if not len(candidates):
        return None
    a, b, c = candidates[:, 0], candidates[:, 1], candidates[:, 2]
    direction, e1, e2 = end - start, b - a, c - a
    p = np.cross(direction, e2)
    det = (e1 * p).sum(axis=1)
    valid = abs(det) > 1e-10
    inverse = np.divide(1.0, det, out=np.zeros_like(det), where=valid)
    offset = start - a
    u = (offset * p).sum(axis=1) * inverse
    q = np.cross(offset, e1)
    v = (q * direction).sum(axis=1) * inverse
    distance = (e2 * q).sum(axis=1) * inverse
    valid &= (u >= 0) & (v >= 0) & (u + v <= 1) & (distance >= 0) & (distance <= 1)
    return float(distance[valid].min()) if valid.any() else None


def segment_panel_hit(start, end, panel):
    """Intersection with the finite canted face, before damage eligibility is applied."""
    inverse = np.linalg.inv(panel)
    a = (inverse @ np.append(start, 1.0))[:3]
    b = (inverse @ np.append(end, 1.0))[:3]
    if abs(b[0] - a[0]) < 1e-12:
        return None
    fraction = -a[0] / (b[0] - a[0])
    if not 0 <= fraction <= 1:
        return None
    hit = a + fraction * (b - a)
    if abs(hit[1]) <= PANEL_SIZE[0] / 2 and abs(hit[2]) <= PANEL_SIZE[1] / 2:
        return fraction
    return None


class ShotResolver:
    """Use the displayed ghost hulls and the real chassis hull, with moving armor truth."""

    def __init__(self, share, armors):
        share = Path(share)
        model = ET.parse(share / 'world/ARCC_Field_2026.sdf').find('.//model')
        collision = model.find('.//collision')
        field = load_stl(share / 'world/composite_part_1.stl')
        field += [float(v) for v in collision.findtext('pose').split()[:3]]
        pose = [float(v) for v in model.findtext('pose').split()]
        c, s = math.cos(pose[5]), math.sin(pose[5])
        self.field = field @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) + pose[:3]
        self.hull = load_stl(share / 'urdf/sentry_v2/meshes/collision/root.stl')
        self.ghost_hull = self.hull * [0.75, 0.75, 1.0]
        self.armors = armors

    def resolve(self, shooter, origin, barrel, launch, histories, until=None, start_step=0):
        """Return the first physical impact; a blocked/slow/back-face hit absorbs the shot."""
        velocity = MUZZLE_SPEED * barrel
        horizon = np.array([origin, origin + velocity * FLIGHT_S + GRAVITY * FLIGHT_S**2 / 2])
        low, high = horizon.min(axis=0) - 0.4, horizon.max(axis=0) + 0.4
        field = self.field[(self.field.min(axis=1) <= high).all(axis=1)
                           & (self.field.max(axis=1) >= low).all(axis=1)]
        elapsed = min(FLIGHT_S, max(0.0, until - launch)) if until is not None else FLIGHT_S
        steps = int((elapsed + 1e-9) / STEP_S)
        for step in range(start_step, steps):
            t0, t1 = step * STEP_S, (step + 1) * STEP_S
            a = origin + velocity * t0 + GRAVITY * t0**2 / 2
            b = origin + velocity * t1 + GRAVITY * t1**2 / 2
            candidates = []
            wall = segment_mesh_hit(a, b, field)
            if wall is not None:
                candidates.append((wall, 'field', None, None, False, 0.0))
            for name, history in histories.items():
                if name == shooter:
                    continue
                pose = history.at(launch + (t0 + t1) / 2)
                if pose is None:
                    raise RuntimeError(f'missing impact-time truth for {name}')
                root = pose[0]
                if np.linalg.norm(root[:2, 3] - (a[:2] + b[:2]) / 2) > 0.7:
                    continue
                inverse = np.linalg.inv(root)
                aa, bb = (inverse @ np.append(a, 1))[:3], (inverse @ np.append(b, 1))[:3]
                hull = self.hull if name == 'sentry' else self.ghost_hull
                body = segment_mesh_hit(aa, bb, hull)
                if body is not None:
                    candidates.append((body, 'hull', name, None, False, 0.0))
                for index, offset in enumerate(self.armors):
                    panel = root @ offset
                    fraction = segment_panel_hit(a, b, panel)
                    if fraction is None:
                        continue
                    impact_t = t0 + fraction * STEP_S
                    before, after = history.at(launch + t0), history.at(launch + t1)
                    if before is None or after is None:
                        raise RuntimeError(f'missing panel velocity truth for {name}')
                    panel_velocity = ((after[0] @ offset)[:3, 3]
                                      - (before[0] @ offset)[:3, 3]) / STEP_S
                    relative = velocity + GRAVITY * impact_t - panel_velocity
                    normal_speed = -float(np.dot(relative, panel[:3, 0]))
                    facing = normal_speed / max(np.linalg.norm(relative), 1e-9)
                    eligible = bool(normal_speed > 12.0
                                    and facing >= math.cos(math.radians(72.5)))
                    candidates.append((fraction, 'panel', name, index, eligible, normal_speed))
            if candidates:
                fraction, kind, victim, panel, eligible, normal_speed = min(
                    candidates, key=lambda candidate: candidate[0])
                return {'impact_t': launch + t0 + fraction * STEP_S, 'impact': kind,
                        'victim': victim, 'panel': panel, 'eligible': eligible,
                        'normal_speed': normal_speed}
        if elapsed < FLIGHT_S:
            return {'pending': True, 'next_step': steps}
        return {'impact_t': launch + FLIGHT_S, 'impact': 'miss',
                'victim': None, 'panel': None, 'eligible': False, 'normal_speed': 0.0}


class Referee:
    """Track HP and each armor module's 50 ms damage dead time."""

    def __init__(self, teams):
        self.teams = teams
        self.hp = {robot: 400 for robot in teams}
        self.last_hit = {}

    def apply(self, impact):
        victim, panel = impact['victim'], impact['panel']
        if not impact['eligible'] or victim is None or self.hp[victim] == 0:
            return False
        key = (victim, panel)
        if impact['impact_t'] - self.last_hit.get(key, -math.inf) < 0.05:
            return False
        self.last_hit[key] = impact['impact_t']
        self.hp[victim] = max(0, self.hp[victim] - 20)
        return True
