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

"""Check swept robot footprints against the actual field collision mesh."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from sim.auto_explore import SPAWN_YAW
from sim.match_scenario import (
    match_duration, PARKED_PATHS, route_duration, ROUTES, sample_match, sample_robot, sample_route,
)

RADIUS = 0.40  # chassis rotation envelope plus clearance, not just the panel radius


@pytest.fixture(scope='module')
def obstacles():
    world = Path(__file__).parents[1] / 'world'
    model = ET.parse(world / 'ARCC_Field_2026.sdf').find('.//model')
    collision = model.find('.//collision')
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.fromfile(world / 'composite_part_1.stl', dtype=dtype, offset=84)
    points = triangles['vertices'].astype(float)
    points += np.array([float(v) for v in collision.findtext('pose').split()[:3]])
    pose = [float(v) for v in model.findtext('pose').split()]
    c, s = math.cos(pose[5]), math.sin(pose[5])
    points = points @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    points += pose[:3]
    # Low bumps are drivable terrain; raised walls and platforms are obstacles.
    return points[points[:, :, 2].max(axis=1) > 0.1, :, :2]


def assert_clear(points, triangles):
    for p in points:
        near = triangles[(triangles.min(axis=1) <= p + RADIUS).all(axis=1)
                         & (triangles.max(axis=1) >= p - RADIUS).all(axis=1)]
        if not len(near):
            continue
        a, b = near, np.roll(near, -1, axis=1)
        edge = b - a
        length2 = (edge * edge).sum(axis=2)
        ratio = np.divide(((p - a) * edge).sum(axis=2), length2,
                          out=np.zeros_like(length2), where=length2 > 0)
        closest = a + np.clip(ratio, 0, 1)[:, :, None] * edge
        distance = np.linalg.norm(closest - p, axis=2).min()
        cross = edge[:, :, 0] * (p - a)[:, :, 1] - edge[:, :, 1] * (p - a)[:, :, 0]
        area = abs((near[:, 1, 0] - near[:, 0, 0]) * (near[:, 2, 1] - near[:, 0, 1])
                   - (near[:, 1, 1] - near[:, 0, 1]) * (near[:, 2, 0] - near[:, 0, 0]))
        inside = ((cross >= 0).all(axis=1) | (cross <= 0).all(axis=1)) & (area > 1e-8)
        assert distance >= RADIUS and not inside.any(), f'blocked footprint at {p}'


@pytest.mark.parametrize('path', PARKED_PATHS)
def test_diagnostic_paths_clear_the_field(path, obstacles):
    params = PARKED_PATHS[path]
    a = math.radians(params['path_angle_deg'])
    offset = np.linspace(-params['half_width'], params['half_width'], 161)
    points = np.column_stack((params['center_x'] + offset * math.sin(a), offset * math.cos(a)))
    c, s = math.cos(SPAWN_YAW), math.sin(SPAWN_YAW)
    points = points @ np.array([[c, s], [-s, c]])
    assert_clear(points, obstacles)
    assert np.linalg.norm(points, axis=1).min() > 2 * RADIUS


@pytest.mark.parametrize('robot', ROUTES)
def test_spawn_routes_clear_the_field(robot, obstacles):
    route = ROUTES[robot]
    points = np.array([sample_route(route, t)[0]
                       for t in np.arange(0, route_duration(route) + .025, .025)])
    assert_clear(points, obstacles)


def test_center_maneuvers_clear_the_field(obstacles):
    positions = np.array([sample_match(t)[0] for t in np.arange(0, match_duration(), .025)])
    assert_clear(positions, obstacles)


def test_four_robot_references_do_not_cross_each_other():
    for t in np.arange(0, match_duration('mcb_match'), .025):
        positions = {'sentry': np.array(sample_match(t, 'mcb_match')[0])}
        positions.update({name: np.array(sample_robot(name, t)[0])
                          for name in ROUTES if name != 'sentry'})
        names = list(positions)
        for index, first in enumerate(names):
            for second in names[index + 1:]:
                assert np.linalg.norm(positions[first] - positions[second]) > 2 * RADIUS, (
                    f'{first}/{second} overlap at {t:.3f}')
