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
The MCB's side of the UART: structs, DJI framing, and its one-slot mailbox.

Structs are JetsonSubsystem.hpp's, which differ from UART_PROTOCOL.md: CVData
is 40 bytes and Relocalize 12, so the bridge's 23- and 8-byte frames never
pass getMsg's size check. Framing is taproot's DJISerial (dji_serial.cpp
updateSerial) and UARTCommunication's sendMsg (seq always 0).
"""
from collections import Counter
from dataclasses import astuple, dataclass
import struct

# enum UartMessage, JetsonSubsystem.hpp:24-35
ROS_MSG, CV_MSG, POSE_MSG, REF_SYS_MSG, RELOCALIZE = 0, 1, 2, 3, 4
MSG_NAMES = {ROS_MSG: 'ROS_MSG', CV_MSG: 'CV_MSG', POSE_MSG: 'POSE_MSG',
             REF_SYS_MSG: 'REF_SYS_MSG', RELOCALIZE: 'RELOCALIZE'}
# enum OdomStatus, JetsonSubsystem.hpp:38-43
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
class ROSData:
    """JetsonSubsystem.hpp:47-51."""

    FORMAT = '<2f'
    TYPE = ROS_MSG
    targetX: float = 0.0
    targetY: float = 0.0


@dataclass
class Relocalize:
    """JetsonSubsystem.hpp:53-58; expectedZ is the 4 bytes the bridge doesn't send."""

    FORMAT = '<3f'
    TYPE = RELOCALIZE
    expectedX: float = 0.0
    expectedY: float = 0.0
    expectedZ: float = 0.0


@dataclass
class CVData:
    """JetsonSubsystem.hpp:60-73: a camera-frame point; no stamp, fire or delay."""

    FORMAT = '<10f'
    TYPE = CV_MSG
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    v_x: float = 0.0
    v_y: float = 0.0
    v_z: float = 0.0
    a_x: float = 0.0
    a_y: float = 0.0
    a_z: float = 0.0
    confidence: float = 0.0


@dataclass
class PoseData:
    """JetsonSubsystem.hpp:77-86, modm_packed."""

    FORMAT = '<6fB'
    TYPE = POSE_MSG
    x: float = 0.0
    y: float = 0.0
    vel_x: float = 0.0
    vel_y: float = 0.0
    head_pitch: float = 0.0
    head_yaw: float = 0.0
    error_code: int = ODOM_PODS


@dataclass
class RefSysMsg:
    """JetsonSubsystem.hpp:89-106, modm_packed."""

    FORMAT = '<BHHBfB'
    TYPE = REF_SYS_MSG
    gameStage: int = 0
    stageTimeRemaining: int = 0
    robotHp: int = 0
    robotID: int = 0
    deltaAngleGotHitIn: float = 0.0
    booleans: int = 0


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


class UARTCommunication:
    """
    The MCB's mailbox, UARTCommunication.cpp:37-79 and JetsonSubsystem.hpp:199-217.

    One slot: each frame overwrites the last (the TODO at
    UARTCommunication.hpp:61). get_msg consumes it only if both type and
    size match; otherwise the slot stays new and the frame is never used.
    Counters say which frames died that way.
    """

    def __init__(self):
        self.has_new_data = False
        self.message_type = 0
        self.data = b''
        self.received = Counter()  # (type, length) -> frames
        self.consumed = Counter()  # type -> frames a getMsg took
        self.size_mismatch = Counter()  # (type, length) -> frames getMsg refused on size
        self.overwritten = Counter()  # (type, length) -> frames lost unread
        self._refused = False

    def message_receive_callback(self, msg_type, payload):
        """UARTCommunication.cpp:37-44."""
        if len(payload) <= 0:
            return
        if self.has_new_data:
            self.overwritten[(self.message_type, len(self.data))] += 1
        self.message_type, self.data = msg_type, payload
        self.has_new_data, self._refused = True, False
        self.received[(msg_type, len(payload))] += 1

    def get_msg(self, cls):
        """JetsonSubsystem::getMsg, JetsonSubsystem.hpp:199-209: a struct or None."""
        if not self.has_new_data:
            return None
        if self.message_type != cls.TYPE:
            return None
        if len(self.data) != size_of(cls):
            if not self._refused:
                self.size_mismatch[(self.message_type, len(self.data))] += 1
                self._refused = True
            return None
        self.has_new_data = False
        self.consumed[cls.TYPE] += 1
        return unpack(cls, self.data)

    @staticmethod
    def frame(msg):
        """Frame a struct as sendMsg does, UARTCommunication.cpp:57-75."""
        return encode_frame(msg.TYPE, pack(msg))
