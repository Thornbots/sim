# Copyright 2026 Thornbots
# SPDX-License-Identifier: Apache-2.0
"""Exercise compiled MCB control code over its real UART parser, with fake hardware."""
import math
import os
from types import SimpleNamespace

import pytest

from sim.mcb_emulator import protocol as wire
from sim.mcb_emulator.pty_link import PtyLink
from sim.mcb_firmware import default_binary, Firmware

pytestmark = pytest.mark.skipif(
    not os.access(default_binary(), os.X_OK), reason='build the hosted MCB firmware first')


@pytest.fixture
def board(tmp_path):
    link = PtyLink(str(tmp_path / 'mcb'))
    firmware = Firmware(link.master)
    os.set_blocking(link.slave, False)
    ref = SimpleNamespace(game_type=4, game_stage=4, stage_time_remaining=300,
                          robot_id=7, current_hp=400, max_hp=400, shooter_power=True,
                          restoration_zone=False, exchange_zone=False, central_buff_zone=False)
    try:
        yield link, firmware, ref
    finally:
        firmware.close()
        link.close()


def run(board, ms, frame=b''):
    link, firmware, ref = board
    parser = wire.DJISerial()
    frames, shots, outputs = [], [], []
    for tick in range(0, ms, 5):
        if frame and tick % 100 == 0:
            os.write(link.slave, frame)
        output = firmware.step(5, [0.0] * 10, ref)
        shots.extend(output[3])
        outputs.append(output)
        while True:
            try:
                data = os.read(link.slave, 4096)
            except BlockingIOError:
                break
            if not data:
                break
            frames.extend(parser.feed(data))
    return frames, shots, outputs


def test_actual_firmware_sends_pose_referee_and_echoes_ping(board):
    frames, _, _ = run(board, 1000, wire.frame(wire.Ping(42)))
    poses = [wire.unpack(wire.Pose, p) for t, p in frames if t == wire.POSE]
    refs = [wire.unpack(wire.RefSys, p) for t, p in frames if t == wire.REF_SYS]
    assert len(poses) >= 85 and len(refs) >= 9
    assert poses[-1].x == pytest.approx(-board[1].start_x)
    assert refs[-1].robotID == 7 and refs[-1].robotHp == 400
    assert any(t == wire.PING for t, _ in frames)



def test_actual_firmware_consumes_fake_sensor_readings(board):
    link, firmware, ref = board
    parser = wire.DJISerial()
    frames = []
    for _ in range(40):
        firmware.step(5, [0.25, 0.5, 0.1, 0.2, 0.7, 0.3, 0.4, 0.1, 0.2, 0.0], ref)
        while True:
            try:
                frames.extend(parser.feed(os.read(link.slave, 4096)))
            except BlockingIOError:
                break
    poses = [wire.unpack(wire.Pose, p) for t, p in frames if t == wire.POSE]
    assert poses[-1].x == pytest.approx(-firmware.start_x + 0.5)
    assert poses[-1].y == pytest.approx(firmware.start_y - 0.25)
    assert poses[-1].vel_x == pytest.approx(0.2)
    assert poses[-1].vel_y == pytest.approx(-0.1)
    assert poses[-1].head_yaw == pytest.approx(0.7, abs=0.01)  # native yaw estimator


def test_actual_firmware_aims_and_fires(board):
    _, firmware, _ = board
    frame = wire.frame(wire.CvTarget(
        -firmware.start_x + 3, 1, 0.2, 100, wire.CV_TARGET_FLAG_FIRE))
    _, shots, outputs = run(board, 1000, frame)
    assert outputs[-1][0] == pytest.approx(math.atan2(1, 3), abs=1e-5)
    assert math.isfinite(outputs[-1][1])
    assert len(shots) >= 8


def test_actual_firmware_relocalizes(board):
    frame = wire.frame(wire.Relocalize(1.25, -0.75))
    frames, _, _ = run(board, 200, frame)
    poses = [wire.unpack(wire.Pose, p) for t, p in frames if t == wire.POSE]
    assert poses[-1].x == pytest.approx(1.25)
    assert poses[-1].y == pytest.approx(-0.75)


def test_actual_firmware_rejects_corrupt_cv_frames(board):
    frame = bytearray(wire.frame(wire.CvTarget(
        2, 0, 0.2, 100, wire.CV_TARGET_FLAG_FIRE)))
    frame[-1] ^= 0xff
    _, shots, _ = run(board, 500, frame)
    assert not shots


def test_actual_firmware_rejects_wrong_sized_cv_frames(board):
    frame = wire.encode_frame(wire.CV_TARGET, wire.pack(wire.CvTarget(
        2, 0, 0.2, 100, wire.CV_TARGET_FLAG_FIRE)) + b'\x00' * 4)
    _, shots, _ = run(board, 500, frame)
    assert not shots


def test_actual_firmware_reports_fake_referee_zones(board):
    board[2].restoration_zone = True
    board[2].central_buff_zone = True
    frames, _, _ = run(board, 200)
    refs = [wire.unpack(wire.RefSys, p) for t, p in frames if t == wire.REF_SYS]
    assert refs[-1].booleans & 0b00110000 == 0b00110000


def test_actual_firmware_referee_stage_blocks_fire(board):
    board[2].game_stage = 0
    frame = wire.frame(wire.CvTarget(
        2, 0, 0.2, 100, wire.CV_TARGET_FLAG_FIRE))
    _, shots, _ = run(board, 500, frame)
    assert not shots


def test_actual_firmware_simple_drive_uses_its_route_and_stage_gate(board):
    board[1].mode = 1
    board[2].game_stage = 0
    _, _, outputs = run(board, 200)
    right, forward, spin = outputs[-1][2]
    assert math.hypot(right, forward) < 0.05
    assert spin < -1
    board[2].game_stage = 4
    _, _, outputs = run(board, 300)
    right, forward, spin = outputs[-1][2]
    assert math.hypot(right, forward) > 0.1
    assert spin < -1


def test_actual_firmware_auto_drive_accepts_nav_goal(board):
    board[1].mode = 2
    frame = wire.frame(wire.NavGoal(-board[1].start_x + 1, -1))
    _, _, outputs = run(board, 300, frame)
    right, forward, spin = outputs[-1][2]
    assert right > 0 and forward > 0
    assert spin > 1
