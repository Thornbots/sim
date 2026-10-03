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
SentryControl (robots/sentry/SentryControl.hpp) and main.cpp's 1 kHz loop.

The remote is taken as connected with both switches up: left runs
AutoAimAndFireCommand, right SimpleAutoDriveCommand. Hardware is the
boundary: everything the firmware reads from or writes to a motor or sensor.
firmware_fixes applies the two fixes asked of MCBV3 position-based-cv:
CvTarget keeps stamp_ms (19 bytes), and delay_ms - 5 clamps at 0.
"""
import math

from sim.mcb_emulator.aim_and_fire import AutoAimAndFireCommand
from sim.mcb_emulator.drive import (AutoDriveCommand, DrivetrainSubsystem, rotate,
                                    SimpleAutoDriveCommand)
from sim.mcb_emulator.jetson import JetsonSubsystem
from sim.mcb_emulator.protocol import (CvTarget, CvTargetStamped, DJISerial, MSG_NAMES,
                                       UARTCommunication)
from sim.mcb_emulator.subsystems import (FlywheelSubsystem, GimbalSubsystem, IndexerSubsystem,
                                         OdometrySubsystem, RefSerial)

DRIVE_SIMPLE, DRIVE_AUTO, DRIVE_STOP = 'simple', 'auto', 'stop'


class Hardware:
    """
    What the firmware senses and drives. Angles CCW from above, radians.

    imu_yaw is the turret's world yaw in (-pi, pi], zero at boot; odom is the
    Pico's (x right, y forward, vx, vy) in the boot heading's frame.
    """

    def imu_yaw(self): raise NotImplementedError  # noqa: E704
    def imu_gz(self): raise NotImplementedError  # noqa: E704
    def yaw_encoder(self): raise NotImplementedError  # noqa: E704, turret minus chassis
    def yaw_encoder_rate(self): raise NotImplementedError  # noqa: E704
    def pitch_encoder(self): raise NotImplementedError  # noqa: E704, positive is down
    def pitch_rate(self): raise NotImplementedError  # noqa: E704
    def odom(self): raise NotImplementedError  # noqa: E704

    def set_gimbal(self, world_yaw, pitch):
        """Gimbal setpoints; world_yaw None means motors off."""
        raise NotImplementedError

    def set_chassis(self, drive):
        """(x right, y forward, CCW rad/s) in the chassis frame; None means stopped."""
        raise NotImplementedError

    def shoot(self):
        raise NotImplementedError


class IdealHardware(Hardware):
    """A gz-free robot for tests: setpoints reached at slew limits, no dynamics."""

    def __init__(self, max_yaw_rate=10.0, max_pitch_rate=6.0):
        self.max_yaw_rate, self.max_pitch_rate = max_yaw_rate, max_pitch_rate
        self.x = self.y = self.vx = self.vy = 0.0
        self.heading = self.heading_rate = 0.0  # chassis, odom frame
        self.yaw = self.yaw_rate = 0.0  # turret, world
        self.pitch = self.pitch_vel = 0.0
        self.yaw_target, self.pitch_target, self.drive = None, 0.0, None
        self.shots = []
        self.time_ms = 0

    def imu_yaw(self):
        return math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    def imu_gz(self):
        return self.yaw_rate

    def yaw_encoder(self):
        return self.yaw - self.heading

    def yaw_encoder_rate(self):
        return self.yaw_rate - self.heading_rate

    def pitch_encoder(self):
        return self.pitch

    def pitch_rate(self):
        return self.pitch_vel

    def odom(self):
        return self.x, self.y, self.vx, self.vy

    def set_gimbal(self, world_yaw, pitch):
        self.yaw_target, self.pitch_target = world_yaw, pitch

    def set_chassis(self, drive):
        self.drive = drive

    def shoot(self):
        self.shots.append(self.time_ms)

    def advance(self, dt):
        """One control period of motion."""
        lx, ly, w = self.drive if self.drive is not None else (0.0, 0.0, 0.0)
        self.vx, self.vy = rotate(lx, ly, self.heading)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.heading_rate = w
        self.heading += w * dt
        old_yaw, old_pitch = self.yaw, self.pitch
        if self.yaw_target is not None:
            err = math.atan2(math.sin(self.yaw_target - self.yaw),
                             math.cos(self.yaw_target - self.yaw))
            step = self.max_yaw_rate * dt
            self.yaw += min(max(err, -step), step)
        else:
            self.yaw += w * dt  # motors off: the turret turns with the chassis
        step = self.max_pitch_rate * dt
        self.pitch += min(max(self.pitch_target - self.pitch, -step), step)
        self.yaw_rate = (self.yaw - old_yaw) / dt
        self.pitch_vel = (self.pitch - old_pitch) / dt


class Drivers:
    """The parts of src::Drivers the ported code reads."""

    def __init__(self, hw, ref_serial):
        self.hw = hw
        self.ref_serial = ref_serial
        self.uart = UARTCommunication()
        self.time_ms = 0

    def clock(self):
        return self.time_ms  # tap::arch::clock::getTimeMilliseconds


class Sentry:
    """
    The sentry's firmware, one call to step() per 1 ms (main.cpp:123-180).

    Frames from the Jetson go in through receive(); frames out collect in tx.
    """

    def __init__(self, hw, ref_serial=None, auto_fire=True, drive=DRIVE_SIMPLE,
                 firmware_fixes=True):
        self.hw = hw
        self.drivers = Drivers(hw, ref_serial or RefSerial())
        self.serial = DJISerial(crc_enabled=True)  # drivers.hpp:140
        self.tx = bytearray()
        self.pending = []  # (cycle, msg_type, payload) not yet in the mailbox
        self.auto_fire_enabled, self.drive_mode = auto_fire, drive
        d = self.drivers
        # SentryControl.hpp:177-194, constructed and registered in this order.
        self.gimbal = GimbalSubsystem(hw)
        self.flywheel = FlywheelSubsystem()
        self.indexer = IndexerSubsystem(hw, d.ref_serial, d.clock)
        self.drivetrain = DrivetrainSubsystem(hw)
        self.odo = OdometrySubsystem(hw)
        self.jetson = JetsonSubsystem(d, self.gimbal, self.odo, self._send,
                                      CvTargetStamped if firmware_fixes else CvTarget)
        self.auto_drive = AutoDriveCommand(d, self.drivetrain, self.gimbal, self.jetson,
                                           self.odo)
        self.simple_auto_drive = SimpleAutoDriveCommand(d, self.drivetrain, self.gimbal,
                                                        self.odo)
        self.auto_fire = AutoAimAndFireCommand(d, self.gimbal, self.indexer, self.flywheel,
                                               self.jetson, self.odo,
                                               lambda: self.auto_drive.is_scheduled,
                                               fix_delay=firmware_fixes)
        self._started = False

    def _send(self, msg):
        self.tx += UARTCommunication.frame(msg)

    def receive(self, data, cycles=1):
        """Bytes from the Jetson that arrived over the next `cycles` ms, spread evenly."""
        frames = self.serial.feed(data)
        for i, (msg_type, payload) in enumerate(frames):
            self.pending.append((i * cycles // max(len(frames), 1), msg_type, payload))

    def _start(self):
        """Schedule what the switch triggers' onTrue would, SentryControl.hpp:58-63."""
        if self.auto_fire_enabled:
            self.auto_fire.initialize()
        if self.drive_mode == DRIVE_SIMPLE:
            self.simple_auto_drive.initialize()
        elif self.drive_mode == DRIVE_AUTO:
            self.auto_drive.initialize()
        self._started = True

    def step(self):
        """One commandScheduler.run(): commands, then subsystems (command_scheduler.cpp)."""
        d = self.drivers
        self.hw.time_ms = d.time_ms
        due = [p for p in self.pending if p[0] <= 0]
        self.pending = [(c - 1, t, p) for c, t, p in self.pending if c > 0]
        for _, msg_type, payload in due:
            d.uart.message_receive_callback(msg_type, payload)
        if not self._started:
            self._start()
        if self.auto_fire_enabled:
            self.auto_fire.execute()
        else:  # default commands: GimbalStopCommand, IndexerIdleCommand
            self.gimbal.stop_motors()
            self.indexer.stop_indexing_at_rate()
        if self.drive_mode == DRIVE_SIMPLE:
            self.simple_auto_drive.execute()
        elif self.drive_mode == DRIVE_AUTO:
            self.auto_drive.execute()
        else:
            self.drivetrain.stop_motors()  # DrivetrainStopCommand
        self.jetson.refresh()
        self.gimbal.refresh()
        self.indexer.refresh()
        d.time_ms += 1

    def run(self, cycles, rx=b''):
        """Run `cycles` ms with rx arriving across them; hw must be advanced by the caller."""
        self.receive(rx, cycles)
        for _ in range(cycles):
            self.step()

    def drain_tx(self):
        out, self.tx = bytes(self.tx), bytearray()
        return out

    def stats(self):
        """Frames per type and what became of them, for logs and tests."""
        u = self.drivers.uart
        parts = []
        for (msg_type, length), n in sorted(u.received.items()):
            name = MSG_NAMES.get(msg_type, str(msg_type))
            refused = u.size_mismatch.get((msg_type, length), 0)
            lost = u.overwritten.get((msg_type, length), 0)
            parts.append(f'{name} {length} B: {n} in, {refused} refused on size, '
                         f'{lost} overwritten unread')
        errors = ', '.join(f'{k} {v}' for k, v in self.serial.errors.items())
        return '; '.join(parts) + (f'; errors {errors}' if errors else '')
