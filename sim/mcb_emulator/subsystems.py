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
The firmware's drivers and the logic halves of its subsystems.

Each subsystem keeps the firmware's setpoint bookkeeping and hands the
setpoint to a Hardware object (sentry.py), which owns everything below the
motor controllers. Units as in the firmware: radians, metres, milliseconds.
"""
from dataclasses import dataclass
import math

PI = math.pi

# GameType and GameStage, taproot ref_serial_data.hpp:90-109
ROBOMASTER_RMUL_3V3 = 4
PREMATCH, SETUP, INITIALIZATION, COUNTDOWN, IN_GAME, END_GAME = range(6)

# GimbalSubsystemConstants.hpp, SENTRY
MAX_PITCH_UP = PI / 180 * 20
MAX_PITCH_DOWN = PI / 180 * 21
PITCH_RATIO = 3.0
MIN_SHOT_FREQ = 50  # ms, IndexerSubsystem.hpp:25


class Timeout:
    """tap::arch::MilliTimeout (timeout.hpp): starts stopped unless given a time."""

    def __init__(self, clock, timeout=None):
        self.clock = clock
        self.running = False
        self.executed = False
        self.expire_time = 0
        if timeout is not None:
            self.restart(timeout)

    def restart(self, timeout):
        self.running, self.executed = True, False
        self.expire_time = self.clock() + int(timeout)

    def stop(self):
        self.running, self.executed = False, False

    def is_stopped(self):
        return not self.running

    def is_expired(self):
        return self.running and self.clock() >= self.expire_time

    def execute(self):
        if not self.executed and self.is_expired():
            self.executed = True
            return True
        return False


class PeriodicMilliTimer:
    """tap::arch::PeriodicMilliTimer (periodic_timer.hpp): fires once per period, no drift."""

    def __init__(self, clock, period):
        self.period = int(period)
        self.timeout = Timeout(clock, period)

    def restart(self, period):
        self.period = int(period)
        self.timeout.restart(period)

    def stop(self):
        self.timeout.stop()

    def is_stopped(self):
        return self.timeout.is_stopped()

    def execute(self):
        if not self.timeout.execute():
            return False
        now = self.timeout.clock()
        while True:
            self.timeout.expire_time += self.period
            if self.timeout.expire_time > now:
                break
        self.timeout.executed = False
        return True


@dataclass
class RefSerial:
    """The referee data the sentry reads (taproot RefSerial::Rx), set by the emulator."""

    receiving: bool = True
    game_type: int = ROBOMASTER_RMUL_3V3
    game_stage: int = IN_GAME
    stage_time_remaining: int = 300
    robot_id: int = 107  # blue sentry; red is 7
    current_hp: int = 400
    max_hp: int = 400
    restoration_zone: bool = False
    exchange_zone: bool = False
    central_buff_zone: bool = False
    recovery_buff: bool = False
    team_occupies_center: bool = False
    opponent_occupies_center: bool = False
    chassis_power: bool = True
    gimbal_power: bool = True
    shooter_power: bool = True

    @staticmethod
    def is_blue_team(robot_id):
        return robot_id > 100  # ref_serial.hpp isBlueTeam: blue ids are 101-111

    def in_3v3(self):
        return self.receiving and self.game_type == ROBOMASTER_RMUL_3V3


class GimbalSubsystem:
    """
    GimbalSubsystem.cpp's setpoint logic; YawController and PitchController are gz's PD.

    Dropped with the controllers: the yaw velocity feed-forward
    (updateMotorsAndVelocity's targetYawVel) and the IMU-latency estimate in
    YawController::estimateYawPos.
    """

    def __init__(self, hw):
        self.hw = hw
        self.target_yaw_angle_world = 0.0
        self.prev_target_pitch = 0.0
        self.yaw_angle_relative_world = 0.0
        self.motors_on = False

    def refresh(self):
        """GimbalSubsystem.cpp:25-51; getYaw() is [0, 360) deg, MahonyAHRS.h:75-78."""
        self.yaw_angle_relative_world = math.fmod(self.hw.imu_yaw() + 2 * PI, 2 * PI)
        self.hw.set_gimbal(self.target_yaw_angle_world if self.motors_on else None,
                           self.prev_target_pitch)

    def update_motors(self, change_in_target_yaw, target_pitch):
        """GimbalSubsystem.cpp:53-74 (the sentry has no chicken-mode pitch)."""
        self.prev_target_pitch = min(max(target_pitch, -MAX_PITCH_DOWN), MAX_PITCH_UP)
        self.target_yaw_angle_world += change_in_target_yaw
        self.motors_on = True

    def update_motors_and_velocity(self, change_in_target_yaw, target_pitch, yaw_vel, pitch_vel):
        """GimbalSubsystem.cpp:81-93; the velocities fed YawController only."""
        self.update_motors(change_in_target_yaw, target_pitch)

    def set_angles(self, yaw_angle, pitch_angle):
        """GimbalSubsystem.cpp:135-151."""
        self.prev_target_pitch = min(max(pitch_angle, -MAX_PITCH_DOWN), MAX_PITCH_UP)
        self.target_yaw_angle_world = yaw_angle
        self.motors_on = True

    def stop_motors(self):
        """GimbalSubsystem.cpp:112-122."""
        self.target_yaw_angle_world = self.yaw_angle_relative_world
        self.motors_on = False

    def get_yaw_angle_relative_world(self):
        return self.yaw_angle_relative_world  # yawController.estimatedPosition, :202

    def get_yaw_encoder_value(self):
        return math.fmod(self.hw.yaw_encoder(), 2 * PI)  # :188, offset calibrated to 0

    def get_pitch_encoder_value(self):
        return self.hw.pitch_encoder()

    def get_pitch_vel(self):
        """GimbalSubsystem.cpp:201: shaft rpm, never divided by PITCH_RATIO."""
        return self.hw.pitch_rate() * PITCH_RATIO

    def get_yaw_vel(self):
        return self.hw.yaw_encoder_rate()  # :200, divided by YAW_TOTAL_RATIO


class OdometrySubsystem:
    """OdometrySubsystem.cpp:64-87: the Pico's pods plus relocalizeTo's offset."""

    def __init__(self, hw):
        self.hw = hw
        self.offset_x = 0.0
        self.offset_y = 0.0

    def relocalize_to(self, new_x, new_y):
        self.offset_x = new_x - self.hw.odom()[0]
        self.offset_y = new_y - self.hw.odom()[1]

    def get_x(self):
        return self.offset_x + self.hw.odom()[0]

    def get_y(self):
        return self.offset_y + self.hw.odom()[1]

    def get_x_vel(self):
        return self.hw.odom()[2]

    def get_y_vel(self):
        return self.hw.odom()[3]


class IndexerSubsystem:
    """
    IndexerSubsystem.cpp's rate timer and canShoot; a shot is hw.shoot().

    Not modelled: heat (ShotCounter), homing and jams; the indexer counts as
    online with a projectile at the beam.
    """

    def __init__(self, hw, ref, clock):
        self.hw, self.ref, self.clock = hw, ref, clock
        self.timer = PeriodicMilliTimer(clock, 1)
        self.timer.stop()
        self.shots_per_second = 0
        self.num_shots_remaining = 0
        self.last_shot_time = -MIN_SHOT_FREQ
        self.is_stopped = False

    def refresh(self):
        """IndexerSubsystem.cpp:21-31."""
        if self.timer.execute():
            self.try_shoot_once()
            if self.num_shots_remaining > 0:
                self.num_shots_remaining -= 1
            if self.num_shots_remaining == 0:
                self.stop_indexing_at_rate()

    def index_at_rate(self, input_shots_per_second):
        """IndexerSubsystem.cpp:34-56."""
        if input_shots_per_second <= 0:
            self.stop_indexing_at_rate()
            return False
        self.is_stopped = False
        shoot_immediately = self.timer.is_stopped()
        if shoot_immediately:
            self.try_shoot_once()
        if self.shots_per_second != input_shots_per_second:
            self.timer.restart(1000 / input_shots_per_second)
        self.shots_per_second = input_shots_per_second
        self.num_shots_remaining = -1
        return shoot_immediately

    def stop_indexing_at_rate(self):
        self.shots_per_second = 0
        self.num_shots_remaining = 0
        self.timer.stop()

    def stop_index(self):
        self.stop_indexing_at_rate()
        self.is_stopped = True

    def can_shoot(self):
        """IndexerSubsystem.cpp canShoot, less heat and the motor check."""
        powered = not self.ref.receiving or self.ref.shooter_power
        return powered and self.clock() - self.last_shot_time >= MIN_SHOT_FREQ

    def try_shoot_once(self):
        """SingleIndexerSubsystem.cpp:63-67."""
        if not self.can_shoot():
            return False
        self.last_shot_time = self.clock()
        self.hw.shoot()
        return True


class FlywheelSubsystem:
    """Keeps the commanded rpm only; a shot leaves at the scorer's muzzle speed."""

    def __init__(self):
        self.target_velocity = 0.0

    def set_target_velocity(self, rpm):
        self.target_velocity = rpm
