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

"""Reticle::solveForPitch as the MCB emulator ports it."""
import math

import pytest
from sim.mcb_emulator import ballistics as b

# The bisection's last step is pi / (4 << 9); it lands within one of them.
STEP = math.pi / (4 << (b.MAX_NUM_ITERATIONS - 1))


@pytest.mark.parametrize('distance,z', [(1.0, 0.0), (3.0, 0.0), (3.0, -0.2), (6.0, 0.3)])
def test_lands_at_the_target_height(distance, z):
    pitch = b.solve_for_pitch(distance, z)
    slope = abs(b.landing_height(distance, pitch + STEP) - b.landing_height(distance, pitch))
    assert b.landing_height(distance, pitch) == pytest.approx(z, abs=slope + 1e-6)


def test_pitch_is_positive_down():
    level = b.solve_for_pitch(3.0, 0.0)
    assert b.solve_for_pitch(3.0, 0.5) < level < b.solve_for_pitch(3.0, -0.5)
    assert level < 0.0  # gravity: a level target needs the barrel slightly up


def test_level_shot_matches_the_range_equation():
    """With the barrel offset ignored, d = v^2 sin(2 theta) / g for the low arc."""
    d = 4.0
    expected = -0.5 * math.asin(b.ACCELERATION_GRAVITY * d / b.INITIAL_SHOT_VELOCITY ** 2)
    assert b.solve_for_pitch(d, 0.0) == pytest.approx(expected, abs=0.01)


def test_target_z_is_taken_from_the_pitch_pivot():
    """Reticle compares pivot-relative landing heights to targetZ as given (Reticle.hpp:370)."""
    pivot = b.OFFSET_Z_ROBOT_TO_PITCH_PIVOT
    panel_off_ground = 0.18
    pitch = b.solve_for_pitch(3.0, panel_off_ground)
    lands_off_ground = pivot + b.landing_height(3.0, pitch)
    assert lands_off_ground == pytest.approx(panel_off_ground + pivot, abs=0.01)


def test_rotate_pitch_turns_y_toward_z():
    x, y, z = b.rotate_pitch((0.5, 1.0, 0.0), math.pi / 2)
    assert (x, y, z) == (0.5, pytest.approx(0.0, abs=1e-12), pytest.approx(1.0))
