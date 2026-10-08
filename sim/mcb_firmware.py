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

"""Lockstep hardware transport to the compiled MCB firmware; no control logic."""
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import time

from sim.mcb_emulator.protocol import encode_frame

INPUT = struct.Struct('<I10fII')
OUTPUT = struct.Struct('<5fII')
MODES = {'stop': 0, 'simple': 1, 'auto': 2}


def default_binary():
    override = os.environ.get('MCB_FIRMWARE_BINARY')
    if override:
        return override
    root = Path(__file__).resolve().parents[2]
    return str(root / 'firmware/MCBV3/MCB-project/build/sim/scons-release/MCB-project.elf')


def referee_frames(ref):
    """Physical referee UART packets, decoded by the firmware's RefSerial."""
    game = struct.pack('<BHQ', ref.game_type | ref.game_stage << 4,
                       ref.stage_time_remaining, 0)
    robot = struct.pack('<BB5HB', ref.robot_id, 1, ref.current_hp, ref.max_hp,
                        100, 1000, 240, 3 | int(ref.shooter_power) << 2)
    power = struct.pack('<HHf4H', 24000, 0, 0.0, 60, 0, 0, 0)
    # RFID bits as Taproot's RFIDActivationStatus numbers them: restoration is
    # the resupply zone outside the exchange, exchange the one inside it.
    zones = (int(ref.restoration_zone) << 19 | int(ref.exchange_zone) << 20
             | int(ref.central_buff_zone) << 23)
    return b''.join(encode_frame(kind, payload) for kind, payload in (
        (0x0001, game), (0x0206, bytes([getattr(ref, 'hurt_armor_id', 0) & 3])),
        (0x0201, robot), (0x0202, power),
        (0x0209, struct.pack('<I', zones))))


class Firmware:
    """Own a hosted firmware process attached to the master end of the bridge PTY."""

    def __init__(self, master_fd, binary='', drive='stop', auto_fire=True):
        self.binary = binary or default_binary()
        if drive not in MODES:
            raise ValueError(f'unknown MCB drive mode: {drive}')
        if not os.access(self.binary, os.X_OK):
            raise RuntimeError(f'MCB firmware binary not built: {self.binary}. '
                               'Run src/sim/tools/build_mcb_firmware.sh first, or set '
                               'firmware_binary / MCB_FIRMWARE_BINARY.')
        self.mode = MODES[drive] | (0 if auto_fire else 4)
        self.process = subprocess.Popen(
            [self.binary], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            env=dict(os.environ, MCB_UART_FD=str(master_fd)), pass_fds=(master_fd,))
        self.time_ms = 0
        try:
            if self._read(4) != b'MCB1':
                raise RuntimeError('MCB firmware hardware interface version mismatch')
            self.start_x, self.start_y = struct.unpack('<2f', self._read(8))
        except BaseException:
            self.close()
            raise

    def _read(self, count):
        data = bytearray()
        deadline = time.monotonic() + 5.0
        fd = self.process.stdout.fileno()
        while len(data) < count:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([fd], [], [], max(remaining, 0))[0]:
                raise RuntimeError('MCB firmware stopped responding to hardware ticks')
            chunk = os.read(fd, count - len(data))
            if not chunk:
                raise RuntimeError(f'MCB firmware exited ({self.process.poll()})')
            data.extend(chunk)
        return bytes(data)

    def step(self, cycles, readings, ref):
        """Step with pods x/y/vx/vy, IMU yaw/rate, and joint yaw/rate/pitch/rate."""
        if not 1 <= cycles <= 100:
            raise ValueError('MCB hardware ticks must contain 1..100 cycles')
        packets = referee_frames(ref) if self.time_ms == 0 or self.time_ms // 100 != (
            self.time_ms + cycles) // 100 else b''
        self.process.stdin.write(INPUT.pack(cycles, *readings, len(packets), self.mode) + packets)
        self.process.stdin.flush()
        yaw, pitch, right, forward, spin, count, self.time_ms = OUTPUT.unpack(
            self._read(OUTPUT.size))
        shots = struct.unpack(f'<{count}I', self._read(count * 4)) if count else ()
        return yaw, pitch, (right, forward, spin), shots

    def gz_readings(self, hw, ref):
        """Map Gazebo truth into the pods and IMU's boot frame using firmware constants."""
        heading = math.pi if ref.robot_id > 100 else 0.0
        sx = self.start_x if ref.robot_id > 100 else -self.start_x
        c, s = math.cos(heading), math.sin(heading)
        dx, dy = hw.pos[0] - sx, hw.pos[1] - self.start_y
        forward, left = c * dx + s * dy, -s * dx + c * dy
        vx, vy = hw.vel
        vf, vl = c * vx + s * vy, -s * vx + c * vy
        return (-left, forward, -vl, vf, hw.turret_world_yaw() - heading,
                hw.imu_gz(), *hw.joint['headlink'], *hw.joint['headpitch'])

    def close(self):
        if self.process.stdin:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process.stdout.close()
