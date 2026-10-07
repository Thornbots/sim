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

"""Wire helper checks: payload sizes, CRCs and diagnostic frame decoding."""
import struct
import zlib
from types import SimpleNamespace

import pytest

from sim.mcb_emulator import protocol as p
from sim.mcb_firmware import referee_frames


@pytest.mark.parametrize('schema,bits', [('legacy', (13, 18, 19)), ('2025', (19, 20, 23))])
@pytest.mark.parametrize('zone', range(3))
def test_referee_rfid_schema_bits(schema, bits, zone):
    ref = SimpleNamespace(game_type=4, game_stage=4, stage_time_remaining=300,
                          robot_id=7, current_hp=400, max_hp=400, shooter_power=True,
                          restoration_zone=zone == 0, exchange_zone=zone == 1,
                          central_buff_zone=zone == 2)
    frames = p.DJISerial().feed(referee_frames(ref, schema))
    payload = next(payload for kind, payload in frames if kind == 0x0209)
    assert struct.unpack('<I', payload)[0] == 1 << bits[zone]


def test_referee_rfid_schema_rejects_unknown():
    ref = SimpleNamespace(game_type=4, game_stage=4, stage_time_remaining=300,
                          robot_id=7, current_hp=400, max_hp=400, shooter_power=True)
    with pytest.raises(ValueError, match='unknown referee RFID schema'):
        referee_frames(ref, 'unknown')


def test_struct_sizes_are_the_firmwares():
    assert p.size_of(p.CvTarget) == 15  # the bridge's CvTargetPayload
    assert p.size_of(p.Relocalize) == 8
    assert p.size_of(p.NavGoal) == 8
    assert p.size_of(p.Pose) == 25
    assert p.size_of(p.RefSys) == 11


def test_crc_tables_match_taproot():
    # zlib.crc32 of crc.cpp's CRC8Table and CRC16Table (LE), checked entry by entry once.
    assert zlib.crc32(bytes(p.CRC8_TABLE)) == 0x0d53eb11
    assert zlib.crc32(struct.pack('<256H', *p.CRC16_TABLE)) == 0x0917cc20


def test_frame_layout():
    pose = p.Pose(1.0, 2.0, 0.5, -0.5, 0.05, 3.0, p.ODOM_PODS)
    frame = p.frame(pose)
    assert len(frame) == 25 + 9
    head, length, seq, crc8, msg_type = struct.unpack_from('<BHBBH', frame)
    assert (head, length, seq, msg_type) == (0xA5, 25, 0, p.POSE)
    assert crc8 == p.crc8(frame[:4])
    assert struct.unpack_from('<H', frame, 32)[0] == p.crc16(frame[:32])
    assert p.unpack(p.Pose, frame[7:32]) == p.Pose(*struct.unpack(
        '<6fB', struct.pack('<6fB', 1.0, 2.0, 0.5, -0.5, 0.05, 3.0, 0)))


def test_parser_finds_frames_in_noise_and_split_reads():
    a = p.encode_frame(p.CV_TARGET, p.pack(p.CvTarget(z=2.0, flags=p.CV_TARGET_FLAG_FIRE)))
    b = p.encode_frame(p.RELOCALIZE, p.pack(p.Relocalize(1.0, 2.0)))
    stream = b'\x00\x13' + a + b'\xff' + b
    parser = p.DJISerial()
    frames = []
    for i in range(0, len(stream), 5):
        frames += parser.feed(stream[i:i + 5])
    assert [f[0] for f in frames] == [p.CV_TARGET, p.RELOCALIZE]
    assert p.unpack(p.CvTarget, frames[0][1]).z == 2.0


def test_parser_drops_bad_crc_and_resyncs():
    good = p.encode_frame(p.NAV_GOAL, p.pack(p.NavGoal(1.0, 2.0)))
    bad16 = bytearray(good)
    bad16[-1] ^= 0xFF
    bad8 = bytearray(good)
    bad8[4] ^= 0xFF
    parser = p.DJISerial()
    frames = parser.feed(bytes(bad16) + bytes(bad8) + good)
    assert len(frames) == 1
    assert parser.errors['crc16'] == 1
    assert parser.errors['crc8'] == 1
