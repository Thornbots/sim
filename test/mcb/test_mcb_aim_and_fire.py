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

"""AutoAimAndFireCommand's aim, fire and patrol states, on the gz-free IdealHardware."""
import math

import pytest
from sim.mcb_emulator import aim_and_fire as aaf
from sim.mcb_emulator import ballistics
from sim.mcb_emulator import protocol as p
from sim.mcb_emulator.sentry import DRIVE_STOP, IdealHardware, Sentry
from sim.mcb_emulator.subsystems import COUNTDOWN, IN_GAME, MAX_PITCH_UP, PREMATCH, RefSerial

FIRE = p.CV_TARGET_FLAG_FIRE


def _sentry(firmware_fixes=True, **ref):
    hw = IdealHardware()
    return hw, Sentry(hw, RefSerial(**ref), drive=DRIVE_STOP, firmware_fixes=firmware_fixes)


def _frame(x, y, z=0.0, delay_ms=0, flags=0):
    """Frame a CV_TARGET in the MCB's odometry frame (x right, y forward)."""
    return p.UARTCommunication.frame(p.CvTarget(x, y, z, delay_ms, flags))


def _run(hw, sentry, ms, frame=b'', period=25):
    """Step ms cycles, sending frame every period ms (40 Hz, point_to_cv_target's rate)."""
    for i in range(ms):
        sentry.run(1, frame if frame and i % period == 0 else b'')
        hw.advance(0.001)


def test_aims_at_the_odom_origin_before_any_frame():
    """Both timeouts start stopped and a stopped one never expires: CvTarget{} is aimed at."""
    hw, s = _sentry()
    _run(hw, s, 100)
    assert s.auto_fire.targeting
    assert hw.yaw_target == pytest.approx(-math.pi / 2)  # Vector2d(0, 0).angle() is 0
    assert hw.shots == []


@pytest.mark.parametrize('x,y', [(0.0, 3.0), (-1.0, 3.0), (2.0, 2.0)])
def test_aims_at_the_odom_point(x, y):
    hw, s = _sentry()
    _run(hw, s, 300, _frame(x, y, 0.2))
    assert hw.yaw_target == pytest.approx(math.atan2(-x, y))  # 0 is +y, CCW positive
    assert hw.pitch_target == pytest.approx(
        ballistics.solve_for_pitch(math.hypot(x, y), 0.2))


def test_aim_follows_odometry_not_the_frame_age():
    hw, s = _sentry()
    hw.x = 1.0
    _run(hw, s, 100, _frame(1.0, 3.0))
    assert hw.yaw_target == pytest.approx(0.0)
    hw.x = 0.0
    _run(hw, s, 1)
    assert hw.yaw_target == pytest.approx(math.atan2(-1.0, 3.0))


def test_pitch_clamps_for_a_close_low_point():
    """Down is positive; GimbalSubsystem.cpp:57 clamps to [-MAX_PITCH_DOWN, MAX_PITCH_UP]."""
    hw, s = _sentry()
    _run(hw, s, 100, _frame(0.0, 0.5, -1.0))
    assert hw.pitch_target == pytest.approx(MAX_PITCH_UP)


def test_fires_once_per_fire_frame_after_delay_minus_latency():
    hw, s = _sentry()
    start = s.drivers.time_ms
    _run(hw, s, 300, _frame(0.0, 3.0, delay_ms=20, flags=FIRE), period=100)
    assert hw.shots == [start + 15, start + 115, start + 215]


def test_forty_hz_fire_frames_shoot_at_twenty():
    """The indexer's MIN_SHOT_FREQ (50 ms) drops every other frame's shot."""
    hw, s = _sentry()
    _run(hw, s, 1000, _frame(0.0, 3.0, delay_ms=10, flags=FIRE))
    assert len(hw.shots) == 20


def test_no_fire_bit_no_shots():
    hw, s = _sentry()
    _run(hw, s, 500, _frame(0.0, 3.0, delay_ms=10))
    assert hw.shots == []


@pytest.mark.parametrize('fixes,fires', [(True, True), (False, False)])
def test_delay_under_the_latency_wraps_without_the_fix(fixes, fires):
    """delay_ms - FIRING_LATENCY_TIME < 0 becomes a ~49-day uint32 timeout in the firmware."""
    hw, s = _sentry(firmware_fixes=fixes)
    _run(hw, s, 500, _frame(0.0, 3.0, delay_ms=3, flags=FIRE))
    assert bool(hw.shots) == fires


def test_bridge_frames_fit_without_the_fixes():
    """position-based-cv's CvTarget is 15 bytes, as the bridge sends it."""
    hw, s = _sentry(firmware_fixes=False)
    _run(hw, s, 500, _frame(0.0, 3.0, delay_ms=10, flags=FIRE))
    assert not s.drivers.uart.size_mismatch
    assert s.drivers.uart.consumed[p.CV_TARGET] > 0
    assert hw.shots


@pytest.mark.parametrize('flags,sweeps',
                         [(0, False), (p.CV_TARGET_FLAG_TYPE_C_BASED_PATROL, True)])
def test_patrols_after_target_valid_time_only_if_flagged(flags, sweeps):
    hw, s = _sentry()
    _run(hw, s, 1, _frame(0.0, 3.0, flags=flags))
    _run(hw, s, aaf.TARGET_VALID_TIME + 1)
    assert not s.auto_fire.targeting
    before = s.gimbal.target_yaw_angle_world
    _run(hw, s, 100)
    moved = s.gimbal.target_yaw_angle_world - before
    assert moved == (pytest.approx(100 * aaf.PATROL_SPEED) if sweeps else 0.0)


def test_faces_a_hit_while_patrolling():
    hw, s = _sentry()
    _run(hw, s, 1, _frame(0.0, 3.0))
    _run(hw, s, aaf.TARGET_VALID_TIME + 1)
    s.jetson.angle_to_turn_for_sentry = 0.5
    yaw = s.gimbal.get_yaw_angle_relative_world()
    _run(hw, s, 1)
    assert s.auto_fire.turning_to_hit
    assert s.gimbal.target_yaw_angle_world == pytest.approx(yaw - 0.5)
    _run(hw, s, aaf.HIT_TURN_DURATION + 1)
    assert not s.auto_fire.turning_to_hit


@pytest.mark.parametrize('stage,gimbal,fires', [
    (PREMATCH, False, False), (COUNTDOWN, True, False), (IN_GAME, True, True)])
def test_3v3_stage_gates_gimbal_and_shooting(stage, gimbal, fires):
    hw, s = _sentry(game_stage=stage)
    _run(hw, s, 300, _frame(0.0, 3.0, delay_ms=10, flags=FIRE))
    assert bool(hw.shots) == fires
    assert (hw.yaw_target is not None) == gimbal


def test_no_gating_outside_a_3v3_game():
    hw, s = _sentry(game_stage=PREMATCH, game_type=1)
    _run(hw, s, 300, _frame(0.0, 3.0, delay_ms=10, flags=FIRE))
    assert hw.shots
