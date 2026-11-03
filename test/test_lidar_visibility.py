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

"""Check expanded render masks and the CAD head section without launching gz."""

import math
from pathlib import Path
import statistics
import struct
import xml.etree.ElementTree as ET

import pytest
import xacro

SOURCE = Path(__file__).resolve().parents[1]


def model(opponent):
    """Expand the real wrapper, resolving only the local package path on host."""
    text = (SOURCE / 'urdf/sentry_v2.urdf.xacro').read_text()
    document = xacro.parse(text.replace('$(find sim)', str(SOURCE)))
    xacro.process_doc(document, mappings={'opponent': opponent})
    return ET.fromstring(document.toxml())


@pytest.mark.parametrize('opponent', ['false', 'true'])
def test_visibility(opponent):
    """Every rendered body excludes self or admits lidar on opponent copies."""
    robot = model(opponent)
    visuals = {link.get('name') for link in robot.findall('link')
               if link.find('visual') is not None}
    flags = {gazebo.get('reference'): int(flag.text, 0)
             for gazebo in robot.findall('gazebo')
             if (flag := gazebo.find('visual/visibility_flags')) is not None}
    assert set(flags) == visuals
    assert all(bool(value & 1) == (opponent == 'true') for value in flags.values())
    sensors = robot.findall('gazebo/sensor')
    if opponent == 'true':
        assert not sensors
    else:
        assert len(sensors) == 1
        assert int(sensors[0].find('lidar/visibility_mask').text, 0) == 1


def head_section(height):
    """Intersect CAD triangles with z, clipping segments to r <= 0.13 m."""
    robot = ET.parse(SOURCE / 'urdf/sentry_v2/sentry_v2.urdf').getroot()
    offsets = {'root': (0., 0., 0.)}
    for name in ['fastened_2', 'headlink', 'headpitch']:
        joint = robot.find(f"joint[@name='{name}']")
        assert joint.find('origin').get('rpy', '0 0 0') == '0 0 0'
        parent = offsets[joint.find('parent').get('link')]
        xyz = map(float, joint.find('origin').get('xyz').split())
        offsets[joint.find('child').get('link')] = tuple(a + b for a, b in zip(parent, xyz))
    points = []
    for link in ['root', 'head', 'head_pitch']:
        data = (SOURCE / f'urdf/sentry_v2/meshes/{link}.stl').read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        assert len(data) == 84 + 50 * count
        for index in range(count):
            vertices = struct.unpack_from('<12fH', data, 84 + 50 * index)[3:12]
            triangle = [tuple(vertices[k + j] + offsets[link][j] for j in range(3))
                        for k in (0, 3, 6)]
            cuts = []
            for a, b in zip(triangle, triangle[1:] + triangle[:1]):
                if min(a[2], b[2]) <= height < max(a[2], b[2]):
                    fraction = (height - a[2]) / (b[2] - a[2])
                    cuts.append(tuple(a[j] + fraction * (b[j] - a[j]) - offsets['head'][j]
                                      for j in range(2)))
            if len(cuts) == 2:
                points.extend(clip_segment(*cuts))
    return points


def clip_segment(a, b):
    """Clip a planar section segment to the yaw-axis neighbourhood."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    aa = dx * dx + dy * dy
    bb = 2 * (a[0] * dx + a[1] * dy)
    cc = a[0] ** 2 + a[1] ** 2 - 0.13 ** 2
    discriminant = bb * bb - 4 * aa * cc
    if aa == 0 or discriminant < 0:
        return []
    lo = max(0, (-bb - math.sqrt(discriminant)) / (2 * aa))
    hi = min(1, (-bb + math.sqrt(discriminant)) / (2 * aa))
    return [(a[0] + t * dx, a[1] + t * dy) for t in (lo, hi)] if lo <= hi else []


@pytest.mark.parametrize('height', [0.275, 0.325, 0.355, 0.375, 0.425])
def test_head_section(height):
    """Pin the small central head silhouette used to choose tracker gates."""
    points = head_section(height)
    widths = []
    for degrees in range(180):
        angle = math.radians(degrees)
        projected = [x * math.cos(angle) + y * math.sin(angle) for x, y in points]
        widths.append(max(projected) - min(projected))
    print(f'z={height:.3f} m: min/median/max = '
          f'{min(widths):.4f}/{statistics.median(widths):.4f}/{max(widths):.4f} m')
    assert 0.14 < min(widths) < 0.18
    assert 0.22 < max(widths) < 0.26
    if height == 0.355:
        assert 0.22 < statistics.median(widths) < 0.25
