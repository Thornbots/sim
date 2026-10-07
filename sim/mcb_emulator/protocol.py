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
Wire structs and framing for fake peripherals and diagnostic test packets.

Structs are JetsonSubsystem.hpp's. CvTarget is 15 bytes there, as the
bridge sends it (no stamp_ms since 2026-10-03). Framing is taproot's
DJISerial (dji_serial.cpp updateSerial) and UARTCommunication's sendMsg
(seq always 0).
"""
from collections import Counter
from dataclasses import astuple, dataclass
import struct

# enum UartMessage, JetsonSubsystem.hpp:26-39
NAV_GOAL, CV_TARGET, POSE, REF_SYS, RELOCALIZE, PING = 0, 1, 2, 3, 4, 5
MSG_NAMES = {NAV_GOAL: 'NAV_GOAL', CV_TARGET: 'CV_TARGET', POSE: 'POSE',
             REF_SYS: 'REF_SYS', RELOCALIZE: 'RELOCALIZE', PING: 'PING'}
# enum OdomStatus, JetsonSubsystem.hpp:39-44
ODOM_PODS = 0

SERIAL_HEAD_BYTE = 0xA5  # dji_serial.hpp:132
SERIAL_RX_BUFF_SIZE = 1024  # dji_serial.hpp:131
CRC8_INIT, CRC16_INIT = 0xFF, 0xFFFF  # crc.hpp:33-34


def _table(poly, width):
    """Reflected CRC table; equals taproot's CRC8Table / CRC16Table (crc.cpp)."""
    out = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ poly if c & 1 else c >> 1
        out.append(c & ((1 << width) - 1))
    return out


CRC8_TABLE = _table(0x8C, 8)
CRC16_TABLE = _table(0x8408, 16)


def crc8(data, init=CRC8_INIT):
    """calculateCRC8, crc.cpp:73-85."""
    c = init
    for b in data:
        c = CRC8_TABLE[c ^ b]
    return c


def crc16(data, init=CRC16_INIT):
    """calculateCRC16, crc.cpp:87-100."""
    c = init
    for b in data:
        c = (c >> 8) ^ CRC16_TABLE[(c ^ b) & 0xFF]
    return c


@dataclass
class NavGoal:
    """JetsonSubsystem.hpp:49-53, where the sentry wants to go."""

    FORMAT = '<2f'
    TYPE = NAV_GOAL
    x: float = 0.0
    y: float = 0.0


# CvTarget.flags, JetsonSubsystem.hpp:62-67
CV_TARGET_FLAG_FIRE = 0x01
CV_TARGET_FLAG_TYPE_C_BASED_PATROL = 0x02
CV_TARGET_FLAG_TURN_TO_HIT = 0x04
CV_TARGET_FLAGS_DEFAULT = CV_TARGET_FLAG_TURN_TO_HIT | CV_TARGET_FLAG_TYPE_C_BASED_PATROL


@dataclass
class CvTarget:
    """JetsonSubsystem.hpp:68-75, modm_packed: a field-frame point, no stamp_ms (15 bytes)."""

    FORMAT = '<3fHB'
    TYPE = CV_TARGET
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    delay_ms: int = 0
    flags: int = CV_TARGET_FLAGS_DEFAULT


@dataclass
class Relocalize:
    """JetsonSubsystem.hpp:70-74, modm_packed: where the lidar thinks the robot is."""

    FORMAT = '<2f'
    TYPE = RELOCALIZE
    x: float = 0.0
    y: float = 0.0


@dataclass
class Pose:
    """JetsonSubsystem.hpp:79-88, modm_packed."""

    FORMAT = '<6fB'
    TYPE = POSE
    x: float = 0.0
    y: float = 0.0
    vel_x: float = 0.0
    vel_y: float = 0.0
    head_pitch: float = 0.0
    head_yaw: float = 0.0
    odom_status: int = ODOM_PODS


@dataclass
class RefSys:
    """JetsonSubsystem.hpp:90-107, modm_packed."""

    FORMAT = '<BHHBfB'
    TYPE = REF_SYS
    gameStage: int = 0
    stageTimeRemaining: int = 0
    robotHp: int = 0
    robotID: int = 0
    deltaAngleGotHitIn: float = 0.0
    booleans: int = 0


@dataclass
class Ping:
    """JetsonSubsystem.hpp:117-120, modm_packed: the MCB echoes it back."""

    FORMAT = '<B'
    TYPE = PING
    number: int = 0


def size_of(cls):
    return struct.calcsize(cls.FORMAT)


def pack(msg):
    return struct.pack(msg.FORMAT, *astuple(msg))


def unpack(cls, data):
    return cls(*struct.unpack(cls.FORMAT, bytes(data)))


def encode_frame(msg_type, payload, seq=0):
    """outgoingDataFrame, UARTCommunication.cpp:17-35: seq is always 0 there."""
    head = struct.pack('<BHB', SERIAL_HEAD_BYTE, len(payload), seq & 0xFF)
    head += bytes([crc8(head)]) + struct.pack('<H', msg_type)
    body = head + bytes(payload)
    return body + struct.pack('<H', crc16(body))


class DJISerial:
    """
    DJISerial::updateSerial (dji_serial.cpp), fed bytes; returns whole frames.

    Same three states, CRC checks and length cap. A failed check drops back to
    the head search without rescanning the bytes it consumed, as there.
    """

    SEARCH, HEADER, DATA = range(3)

    def __init__(self, crc_enabled=True):
        self.crc_enabled = crc_enabled
        self.state = self.SEARCH
        self.buf = bytearray()
        self.errors = Counter()

    def feed(self, data):
        """Return (msg_type, payload) per complete frame in data, in order."""
        frames = []
        for b in data:
            frame = self._byte(b)
            if frame is not None:
                frames.append(frame)
        return frames

    def _byte(self, b):
        if self.state == self.SEARCH:
            if b == SERIAL_HEAD_BYTE:
                self.buf = bytearray([b])
                self.state = self.HEADER
            return None
        self.buf.append(b)
        if self.state == self.HEADER:
            if len(self.buf) < 5:
                return None
            if self.crc_enabled and crc8(self.buf[:4]) != self.buf[4]:
                self.errors['crc8'] += 1
                self.state = self.SEARCH
                return None
            if struct.unpack_from('<H', self.buf, 1)[0] >= SERIAL_RX_BUFF_SIZE:
                self.errors['length'] += 1
                self.state = self.SEARCH
                return None
            self.state = self.DATA
            return None
        length = struct.unpack_from('<H', self.buf, 1)[0]
        if len(self.buf) < 5 + 2 + length + (2 if self.crc_enabled else 0):
            return None
        self.state = self.SEARCH
        if self.crc_enabled:
            got = struct.unpack_from('<H', self.buf, 7 + length)[0]
            if got != crc16(self.buf[:7 + length]):
                self.errors['crc16'] += 1
                return None
        return struct.unpack_from('<H', self.buf, 5)[0], bytes(self.buf[7:7 + length])


def frame(msg):
    """Serialize a hardware/test packet using the UART wire format."""
    return encode_frame(msg.TYPE, pack(msg))
