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

"""The MCB emulator's outbound schedule, RELOCALIZE and auto drive, on IdealHardware."""
import math

import pytest
from sim.mcb_emulator import drive
from sim.mcb_emulator import protocol as p
from sim.mcb_emulator.sentry import DRIVE_AUTO, DRIVE_SIMPLE, DRIVE_STOP, IdealHardware, Sentry
from sim.mcb_emulator.subsystems import PREMATCH, RefSerial


def _run(hw, s, ms, rx=b''):
    for i in range(ms):
        s.run(1, rx if i == 0 else b'')
        hw.advance(0.001)


def _frames(data):
    return p.DJISerial().feed(data)


def test_nine_poses_then_one_ref_every_ten_ms():
    hw = IdealHardware()
    s = Sentry(hw, RefSerial(robot_id=107, current_hp=350, restoration_zone=True),
               auto_fire=False, drive=DRIVE_STOP)
    hw.x, hw.y = 1.5, -0.5
    _run(hw, s, 1000)
    frames = _frames(s.drain_tx())
    types = [t for t, _ in frames]
    assert types.count(p.POSE_MSG) == 90  # not UART_PROTOCOL.md's 100 Hz
    assert types.count(p.REF_SYS_MSG) == 9  # 10 Hz; the 200 ms timer is unused
    assert types[:10] == [p.POSE_MSG] * 9 + [p.REF_SYS_MSG]
    pose = p.unpack(p.PoseData, frames[0][1])
    assert (pose.x, pose.y, pose.error_code) == (1.5, -0.5, p.ODOM_PODS)
    ref = p.unpack(p.RefSysMsg, frames[9][1])
    assert (ref.robotID, ref.robotHp) == (7, 350)
    assert ref.booleans == 0b10100011  # blue, reload zone, chassis and gimbal power
    assert ref.deltaAngleGotHitIn == 123.0  # HitRing's "not hit" placeholder


def test_pose_head_yaw_is_zero_to_two_pi():
    hw = IdealHardware()
    s = Sentry(hw, auto_fire=False, drive=DRIVE_STOP)
    hw.yaw = -0.5
    _run(hw, s, 20)
    pose = p.unpack(p.PoseData, _frames(s.drain_tx())[-1][1])
    assert pose.head_yaw == pytest.approx(2 * math.pi - 0.5, abs=1e-6)


def test_relocalize_needs_twelve_bytes_and_only_reaches_simple_auto_drive():
    hw = IdealHardware()
    s = Sentry(hw, auto_fire=False, drive=DRIVE_STOP)
    _run(hw, s, 5, p.encode_frame(p.RELOCALIZE, p.pack(p.ROSData(3.0, 4.0))))  # 8 bytes
    assert not s.simple_auto_drive.set_localization
    _run(hw, s, 5, p.UARTCommunication.frame(p.Relocalize(3.0, 4.0)))
    assert s.simple_auto_drive.set_localization
    assert (s.simple_auto_drive.x_for_localization, s.simple_auto_drive.y_for_localization) \
        == (3.0, 4.0)
    assert s.odo.get_x() == 0.0  # odometry untouched


def _drive_route(hw, s, ms):
    path = []
    for i in range(ms):
        s.run(1)
        hw.advance(0.001)
        if i % 100 == 0:
            path.append((hw.x, hw.y, s.simple_auto_drive.target_index))
    return path


def test_simple_auto_drive_runs_arcc_rough_path_spinning():
    hw = IdealHardware()
    s = Sentry(hw, RefSerial(robot_id=7), auto_fire=False, drive=DRIVE_SIMPLE)
    _drive_route(hw, s, 15000)
    # Red: right 2.236, past the wall, then left-forward to the centre zone.
    assert s.simple_auto_drive.target_index == 3
    assert math.hypot(hw.x - -0.5, hw.y - 4.625) < 0.5
    assert hw.drive[2] == drive.SPIN_VELOCITY  # -12 rad/s at the end point
    assert s.simple_auto_drive.targets[0][0] == (-0.5, -0.5)  # changedInitialPoint


def test_simple_auto_drive_mirrors_x_for_blue_and_waits_for_the_game():
    hw = IdealHardware()
    s = Sentry(hw, RefSerial(robot_id=107, game_stage=PREMATCH), auto_fire=False,
               drive=DRIVE_SIMPLE)
    _drive_route(hw, s, 3000)
    assert s.simple_auto_drive.target_index == 0
    assert math.hypot(hw.x, hw.y) < 0.05
    assert hw.drive[2] == drive.MOVE_TO_POS_SPIN_VELO  # spins in place even before the game
    s.drivers.ref_serial.game_stage = 4
    path = _drive_route(hw, s, 3000)
    assert s.simple_auto_drive.targets[1][0] == (-2.236, 0.5)
    assert min(x for x, _, _ in path) < -1.5  # within MoveToPosition's 0.5 m, then on


def test_simple_auto_drive_heads_home_at_low_hp():
    hw = IdealHardware()
    ref = RefSerial(robot_id=7)
    s = Sentry(hw, ref, auto_fire=False, drive=DRIVE_SIMPLE)
    _drive_route(hw, s, 15000)
    ref.current_hp = 200
    _drive_route(hw, s, 15000)
    assert s.simple_auto_drive.direction == -1
    assert s.simple_auto_drive.target_index == 0
    assert math.hypot(hw.x - -0.5, hw.y - -0.5) < 0.5  # changedInitialPoint, red


def test_simple_auto_drive_applies_relocalize_only_full_hp_in_resupply():
    hw = IdealHardware()
    ref = RefSerial(robot_id=107)
    s = Sentry(hw, ref, auto_fire=False, drive=DRIVE_SIMPLE)
    _run(hw, s, 200, p.UARTCommunication.frame(p.Relocalize(1.0, 2.0)))
    assert s.odo.offset_x == 0.0  # not in a resupply zone
    ref.restoration_zone = True
    _run(hw, s, 1)
    # Read after one more cycle of driving, hence the tolerance.
    assert s.odo.get_x() == pytest.approx(1.0 + drive.RELOCALIZE_X_OFFSET, abs=0.01)
    assert s.odo.get_y() == pytest.approx(2.0 + drive.RELOCALIZE_Y_OFFSET, abs=0.01)


def test_auto_drive_spins_at_nine_and_takes_ros_goals():
    hw = IdealHardware()
    s = Sentry(hw, auto_fire=False, drive=DRIVE_AUTO)
    _run(hw, s, 100)
    assert hw.drive[2] == drive.AUTO_DRIVE_SPIN
    _run(hw, s, 3000, p.UARTCommunication.frame(p.ROSData(1.0, 0.5)))
    assert math.hypot(hw.x - 1.0, hw.y - 0.5) < 0.05
