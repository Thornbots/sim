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
Unit test for shot_hit_harness.off_face: a hit crosses the canted face.

Horizontal shots at the front panel of a target at the origin, yaw 0. The
face is 0.135 m wide and 0.125 m tall along its 15 deg cant, so a
horizontal ray clears its top edge at 0.0625 * cos(15 deg) above centre.
"""
import math

import numpy as np
import pytest
from shot_hit_harness import _panel_poses, off_face, PANEL_HEIGHT, PANEL_WIDTH

FRONT = _panel_poses(np.zeros(3), np.eye(3))[0]
TOWARD = np.array([-1.0, 0.0, 0.0])  # from +x, at the front panel


def _shot(right=0.0, up=0.0):
    # FRONT's right_dir is +y.
    return off_face(np.array([3.0, right, up]), TOWARD, FRONT)


def test_centre_hits():
    assert _shot() == 0.0


def test_width_not_circle():
    # Past the old 0.0625 m circle, still on the 0.0675 m half-width.
    assert _shot(right=0.066) == 0.0
    assert _shot(right=0.070) == pytest.approx(0.070 - PANEL_WIDTH / 2.0)


def test_cant_shortens_height():
    top = PANEL_HEIGHT / 2.0 * math.cos(math.radians(15.0))
    assert _shot(up=top - 0.001) == 0.0
    assert _shot(up=-(top - 0.001)) == 0.0
    # Inside the old circle, off the canted face.
    assert _shot(up=0.062) > 0.0


def test_back_and_behind_miss():
    assert off_face(np.array([3.0, 0.0, 0.0]), -TOWARD, FRONT) == math.inf
    assert off_face(np.array([-3.0, 0.0, 0.0]), -TOWARD, FRONT) == math.inf
