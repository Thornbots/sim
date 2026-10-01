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
from sim.mcb_emulator import jetson
from sim.mcb_emulator.protocol import CV_MSG, CVData, encode_frame, UARTCommunication
from sim.mcb_emulator.sentry import DRIVE_STOP, IdealHardware, Sentry
from sim.mcb_emulator.subsystems import (COUNTDOWN, IN_GAME, PREMATCH, RefSerial)

CV_PERIOD_MS = 33  # ~30 Hz, the camera's rate


def _sentry(**ref):
    hw = IdealHardware()
    return hw, Sentry(hw, RefSerial(**ref), drive=DRIVE_STOP)


def _run(hw, sentry, ms, target=None, period=CV_PERIOD_MS):
    """Step ms cycles; target(hw) gives a frame every period ms (bytes are sent as is)."""
    for i in range(ms):
        frame = b''
        if target is not None and i % period == 0:
            frame = target if isinstance(target, bytes) else target(hw)
        sentry.run(1, frame)
        hw.advance(0.001)


def _camera_point(hw, fwd, left, up):
    """
    Render a world point (from the pitch axis, metres) as CVData from the turret now.

    Inverts JetsonSubsystem.cpp:197-220 at the turret's current yaw and pitch.
    """
    c3, s3 = math.cos(hw.imu_yaw()), math.sin(hw.imu_yaw())
    c4, s4 = math.cos(-hw.pitch), math.sin(-hw.pitch)
    px, py = -left, fwd
    x4, f = c3 * px + s3 * py, -s3 * px + c3 * py
    y4, z4 = c4 * f + s4 * up, -s4 * f + c4 * up
    return x4 - jetson.CAMERA_X_OFFSET, z4 - jetson.CAMERA_Z_OFFSET, y4 - jetson.CAMERA_Y_OFFSET


def _at(fwd, left=0.0, up=0.0, confidence=0.9):
    """Make a target fixed in the world, rendered into the camera each frame."""
    def frame(hw):
        x, y, z = _camera_point(hw, fwd, left, up)
        return UARTCommunication.frame(CVData(x=x, y=y, z=z, confidence=confidence))
    return frame


def _ahead(distance=2.0, confidence=0.9):
    return _at(distance, confidence=confidence)


def _boot(hw, s):
    """Fill the orientation delay line before the first frame (it starts at zeros)."""
    _run(hw, s, jetson.ORIENTATION_QUEUE_SIZE + 2)


def test_holds_then_patrols_without_a_target():
    hw, s = _sentry()
    _run(hw, s, aaf.PERSISTANCE)
    assert hw.yaw == 0.0  # lastSeenTime starts at 0: PERSISTANCE ms of holding first
    _run(hw, s, 1000)
    assert s.gimbal.prev_target_pitch == pytest.approx(0.05)
    # PATROL_SPEED a cycle, less the BURST_AMOUNT (0) cycle every CYCLES_UNTIL_BURST.
    assert s.gimbal.target_yaw_angle_world == pytest.approx(-0.002 * (1000 - 2), abs=1e-9)
    assert hw.shots == []


def test_fires_at_ten_hz_while_holding_then_persists():
    hw, s = _sentry()
    _boot(hw, s)
    start = s.drivers.time_ms
    _run(hw, s, 300, _ahead())
    assert hw.shots == [start, start + 100, start + 200]
    _run(hw, s, 400)  # no frames: PERSISTANCE ms more, then patrol stops it
    last_frame = start + 9 * CV_PERIOD_MS
    assert hw.shots[-1] <= last_frame + aaf.PERSISTANCE
    assert not s.auto_fire.is_shooting


def test_confidence_cutoff_is_inclusive():
    hw, s = _sentry()
    _boot(hw, s)
    _run(hw, s, 300, _ahead(confidence=0.75))
    assert hw.shots == []
    hw, s = _sentry()
    _boot(hw, s)
    _run(hw, s, 300, _ahead(confidence=0.76))
    assert hw.shots


def test_far_panels_still_fire():
    """MAX_SHOOT_DIST (3 m) is defined but its check is commented out, JetsonSubsystem.cpp:207."""
    hw, s = _sentry()
    _boot(hw, s)
    _run(hw, s, 300, _ahead(distance=6.0))
    assert hw.shots


def test_steps_toward_a_target_off_centre_and_pitches_between_frames():
    hw, s = _sentry()
    _boot(hw, s)
    _run(hw, s, 2000, _at(3.0, left=-1.0, up=0.3))
    bearing = math.atan2(-1.0, 3.0)
    # A frame moves the setpoint dyaw * min(0.05 |dyaw|, 0.015): 18 deg is half
    # closed after 2 s of 30 Hz frames.
    assert bearing < hw.yaw < 0.4 * bearing
    # Frames barely move pitch (PITCH_MULTIPLY_MAX 5e-6); the hold branch between
    # them sets the ballistic pitch outright (AutoAimAndFireCommand.cpp:71).
    assert hw.pitch < -0.05
    assert hw.shots


def test_target_past_sixty_degrees_aims_without_firing():
    hw, s = _sentry()
    _boot(hw, s)
    _run(hw, s, 300, _at(1.0, left=-3.0))
    assert s.auto_fire.shoot in (0, -1)
    assert hw.shots == []
    assert s.gimbal.target_yaw_angle_world < 0.0


@pytest.mark.parametrize('turret_yaw,fires', [(1.0, True), (-1.0, False)])
def test_fires_only_while_turret_faces_the_lower_half_turn(turret_yaw, fires):
    """Gimbal yaw is [0, 2pi), targetYaw (-pi, pi]: their raw difference fails."""
    hw, s = _sentry()
    hw.yaw = turret_yaw
    s.gimbal.target_yaw_angle_world = turret_yaw
    _boot(hw, s)
    _run(hw, s, 300, _at(2.0 * math.cos(turret_yaw), left=2.0 * math.sin(turret_yaw)))
    assert bool(hw.shots) == fires


def test_bridge_sized_cv_frames_never_aim():
    """The bridge's 23-byte CVDataPayload fails getMsg's size check (JetsonSubsystem.hpp:204)."""
    hw, s = _sentry()
    _run(hw, s, 1000, encode_frame(CV_MSG, b'\x00' * 23))
    assert hw.shots == []
    assert s.drivers.uart.size_mismatch[(CV_MSG, 23)] > 0
    assert s.gimbal.target_yaw_angle_world < 0.0  # patrolling


@pytest.mark.parametrize('stage,gimbal,fires', [
    (PREMATCH, False, False), (COUNTDOWN, True, False), (IN_GAME, True, True)])
def test_3v3_stage_gates_gimbal_and_shooting(stage, gimbal, fires):
    hw, s = _sentry(game_stage=stage)
    _boot(hw, s)
    _run(hw, s, 300, _ahead())
    assert bool(hw.shots) == fires
    assert (hw.yaw_target is not None) == gimbal


def test_no_gating_outside_a_3v3_game():
    hw, s = _sentry(game_stage=PREMATCH, game_type=1)
    _boot(hw, s)
    _run(hw, s, 300, _ahead())
    assert hw.shots
    hw, s = _sentry(game_stage=PREMATCH, receiving=False)
    _boot(hw, s)
    _run(hw, s, 300, _ahead())
    assert hw.shots


def test_resupply_zone_centres_the_gimbal():
    hw, s = _sentry(restoration_zone=True)
    hw.yaw = 0.5
    _run(hw, s, 300, _ahead())
    assert s.gimbal.target_yaw_angle_world == 0.0
    assert s.gimbal.prev_target_pitch == 0.0
