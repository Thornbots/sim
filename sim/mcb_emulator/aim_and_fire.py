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
AutoAimAndFireCommand (subsystems/jetson/AutoAimAndFireCommand.{hpp,cpp}).

Per 1 ms cycle: aim on a fresh CVData, hold for PERSISTANCE ms after the
last, else patrol. Fires at indexer rate 10 from the first action-1 frame
until the patrol branch clears it. In a 3v3 game it shoots only IN_GAME.
"""
import math

from sim.mcb_emulator.jetson import (PITCH_MULTIPLY_MAX, PITCH_MULTIPLY_MIN,
                                     PITCH_MULTIPLY_SCALE, PLACEHOLDER_ANGLE, YAW_MULTIPLY_MAX,
                                     YAW_MULTIPLY_MIN, YAW_MULTIPLY_SCALE)
from sim.mcb_emulator.subsystems import COUNTDOWN, IN_GAME, PI

# AutoAimAndFireCommand.hpp:68-71, :86
CYCLES_UNTIL_BURST = 380
BURST_AMOUNT = 0.0  # rad/cycle
PATROL_SPEED = -0.002  # rad/cycle: -2 rad/s at 1 kHz
PERSISTANCE = 200  # ms
HIT_TURN_DURATION = 500  # ms
INDEX_RATE = 10  # shots/s, AutoAimAndFireCommand.cpp:112
FLYWHEEL_MOTOR_MAX_RPM = 1.0  # FlywheelSubsystemConstants.hpp; only its fraction matters here


def _clamp(v, lo, hi):
    return min(max(v, lo), hi)


class AutoAimAndFireCommand:

    def __init__(self, drivers, gimbal, indexer, flywheel, cv, adc_scheduled):
        """adc_scheduled(): AutoDriveCommand::getIsScheduled, for the idle flywheel."""
        self.drivers, self.gimbal, self.indexer = drivers, gimbal, indexer
        self.flywheel, self.cv, self.adc_scheduled = flywheel, cv, adc_scheduled
        self.is_shooting = False
        self.shoot = 0
        self.num_cycles_for_burst = 0
        self.pitch = 0.0
        self.yawvel = 0.0
        self.pitchvel = 0.0
        self.last_seen_time = 0
        self.turning_to_hit = False
        self.hit_target_yaw = 0.0
        self.hit_turn_start_time = 0
        self.is_scheduled = False

    def initialize(self):
        """AutoAimAndFireCommand.cpp:8-11."""
        self.shoot = -1
        self.is_scheduled = True

    def end(self):
        self.pitch = 0.0
        self.is_scheduled = False

    def execute(self):
        """AutoAimAndFireCommand.cpp:12-129."""
        ref, now = self.drivers.ref_serial, self.drivers.clock()
        allow_shooting = allow_gimbal = True
        if ref.in_3v3():
            allow_shooting = allow_gimbal = ref.game_stage == IN_GAME
            if ref.game_stage == COUNTDOWN:
                allow_gimbal = True

        current_yaw = self.gimbal.get_yaw_angle_relative_world()
        current_pitch = self.gimbal.get_pitch_encoder_value()
        aim = self.cv.update(current_yaw, current_pitch, self.yawvel, self.gimbal.get_pitch_vel())
        self.shoot = aim.action
        dyaw = aim.yaw_out if aim.action != -1 else 0.0
        if aim.action != -1:
            self.pitch, self.yawvel, self.pitchvel = (aim.pitch_out, aim.yaw_vel_out,
                                                      aim.pitch_vel_out)

        in_rfid = ref.restoration_zone or ref.exchange_zone  # rfidStatus.all(), :44
        if in_rfid and allow_gimbal:
            self.gimbal.set_angles(0.0, 0.0)
        elif self.shoot != -1:
            dyaw = math.fmod(dyaw, 2 * PI)
            dyaw = dyaw - 2 * PI if dyaw > PI else dyaw + 2 * PI if dyaw < -PI else dyaw
            self.last_seen_time = now
            dpitch = self.pitch - current_pitch
            new_pitch = current_pitch + dpitch * _clamp(
                abs(dpitch) * PITCH_MULTIPLY_SCALE, PITCH_MULTIPLY_MIN, PITCH_MULTIPLY_MAX)
            dyaw *= _clamp(abs(dyaw) * YAW_MULTIPLY_SCALE, YAW_MULTIPLY_MIN, YAW_MULTIPLY_MAX)
            if allow_gimbal:
                self.gimbal.update_motors_and_velocity(dyaw, new_pitch, self.yawvel,
                                                       self.pitchvel)
            if self.shoot == 1:
                self.is_shooting = True
        elif now - self.last_seen_time < PERSISTANCE:
            if allow_gimbal:
                self.gimbal.update_motors(0.0, self.pitch)
        else:
            self._patrol(allow_gimbal, current_yaw, now)

        if allow_shooting and self.is_shooting:
            self.indexer.index_at_rate(INDEX_RATE)
        else:
            self.indexer.stop_index()
        if allow_gimbal:
            self.flywheel.set_target_velocity(FLYWHEEL_MOTOR_MAX_RPM)
        else:
            self.gimbal.stop_motors()
            if self.adc_scheduled():
                self.flywheel.set_target_velocity(FLYWHEEL_MOTOR_MAX_RPM / 4)

    def _patrol(self, allow_gimbal, current_yaw, now):
        """AutoAimAndFireCommand.cpp:72-106: sweep, or face a hit for HIT_TURN_DURATION."""
        self.is_shooting = False
        self.pitch = 0.05
        self.num_cycles_for_burst += 1
        if not allow_gimbal:
            return
        angle = self.cv.get_angle_to_turn_for_sentry()
        if angle != PLACEHOLDER_ANGLE:
            self.hit_target_yaw = current_yaw - angle
            self.turning_to_hit = True
            self.hit_turn_start_time = now
        if self.turning_to_hit and now - self.hit_turn_start_time < HIT_TURN_DURATION:
            self.gimbal.set_angles(self.hit_target_yaw, self.pitch)
            return
        self.turning_to_hit = False
        if self.num_cycles_for_burst == CYCLES_UNTIL_BURST:
            self.gimbal.update_motors(BURST_AMOUNT, self.pitch)
            self.num_cycles_for_burst = 0
        else:
            self.gimbal.update_motors(PATROL_SPEED, self.pitch)
