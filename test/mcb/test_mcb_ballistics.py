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

"""taproot's ballistics as the MCB emulator ports it."""
import math

import pytest
from sim.mcb_emulator.ballistics import (ACCELERATION_GRAVITY, compute_travel_time,
                                         find_target_projectile_intersection,
                                         SecondOrderKinematicState)

V = 24.0  # the sentry's initialShotVelocity


def _solve(pos, vel=(0.0, 0.0, 0.0)):
    return find_target_projectile_intersection(
        SecondOrderKinematicState(pos, vel, (0.0, 0.0, 0.0)), V, 3)


def test_level_target_matches_closed_form():
    d = 3.0
    ok, pitch, yaw, travel = _solve((d, 0.0, 0.0))
    assert ok
    assert yaw == 0.0
    # Low-arc range equation: d = v^2 sin(2 theta) / g; pitch is negative to aim up.
    assert pitch == pytest.approx(-0.5 * math.asin(ACCELERATION_GRAVITY * d / V ** 2), abs=1e-9)
    assert travel == pytest.approx(d / (V * math.cos(pitch)), rel=1e-9)


def test_higher_target_aims_further_up():
    _, level, _, _ = _solve((4.0, 0.0, 0.0))
    _, up, _, _ = _solve((4.0, 0.0, 0.5))
    assert up < level < 0.0


def test_yaw_is_bearing_ccw_from_downrange():
    ok, _, yaw, _ = _solve((2.0, 2.0, 0.0))
    assert ok
    assert yaw == pytest.approx(math.pi / 4)
    _, _, yaw_right, _ = _solve((2.0, -1.0, 0.0))
    assert yaw_right == pytest.approx(math.atan2(-1.0, 2.0))


def test_moving_target_is_led_by_its_travel_time():
    pos, vel = (5.0, 0.0, 0.0), (0.0, 2.0, 0.0)
    ok, _, yaw, travel = _solve(pos, vel)
    assert ok
    assert yaw == pytest.approx(math.atan2(vel[1] * travel, pos[0]), rel=1e-6)
    assert travel == pytest.approx(math.hypot(5.0, 2.0 * travel) / V, rel=1e-2)


def test_out_of_range_and_origin_fail():
    ok, *_ = _solve((V ** 2 / ACCELERATION_GRAVITY + 1.0, 0.0, 0.0))
    assert not ok
    ok, *_ = _solve((0.0, 0.0, 0.0))
    assert not ok


def test_near_flat_shots_take_the_vertical_time():
    """ballistics.cpp: within 0.01 rad of level, travel time is a vertical shot's to height z."""
    ok, travel, pitch = compute_travel_time((1.0, 0.0, 0.0), V)
    assert ok
    assert abs(pitch) < 1e-2
    assert travel == 0.0  # so no lead inside ~1.2 m at 24 m/s
