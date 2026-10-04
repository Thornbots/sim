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
AutoAimAndFireCommand (subsystems/jetson/AutoAimAndFireCommand.{hpp,cpp}), automatic mode.

Per 1 ms cycle: aim at the latest CvTarget's field point for TARGET_VALID_TIME
after it arrives, one tryShootOnce per fire frame after delay_ms less
FIRING_LATENCY_TIME, else face a hit or patrol. Under the latency the
firmware's uint32 timeout wraps to the past and fires next cycle, as a signed
restart does here. In a 3v3 game it shoots only IN_GAME. fix_pivot_z solves
pitch for z above the pitch pivot, not the ground (asked of the firmware).
"""
import math

from sim.mcb_emulator import ballistics
from sim.mcb_emulator.jetson import PLACEHOLDER_ANGLE
from sim.mcb_emulator.protocol import (CV_TARGET_FLAG_FIRE, CV_TARGET_FLAG_TURN_TO_HIT,
                                       CV_TARGET_FLAG_TYPE_C_BASED_PATROL, CvTarget)
from sim.mcb_emulator.subsystems import COUNTDOWN, IN_GAME, Timeout

# AutoAimAndFireCommand.hpp:68-79
CYCLES_UNTIL_BURST = 380
PATROL_SPEED = -0.0002  # rad/cycle: -0.2 rad/s at 1 kHz
BURST_AMOUNT = PATROL_SPEED  # rad/cycle, burst mode off
PATROL_PITCH = 0.05  # rad, down; also turning to a hit
TARGET_VALID_TIME = 200  # ms
FIRING_LATENCY_TIME = 80  # ms
HIT_TURN_DURATION = 500  # ms
FLYWHEEL_MOTOR_MAX_RPM = 1.0  # FlywheelSubsystemConstants.hpp; only its fraction matters here


class AutoAimAndFireCommand:

    def __init__(self, drivers, gimbal, indexer, flywheel, jetson, odo, adc_scheduled,
                 fix_pivot_z=False):
        """adc_scheduled(): AutoDriveCommand::getIsScheduled, for the idle flywheel."""
        self.drivers, self.gimbal, self.indexer = drivers, gimbal, indexer
        self.flywheel, self.jetson, self.odo = flywheel, jetson, odo
        self.adc_scheduled, self.fix_pivot_z = adc_scheduled, fix_pivot_z
        self.cv_target = CvTarget()  # flags default to patrol and turn-to-hit
        # Both start stopped: no aiming until the first frame.
        self.cv_target_valid_timeout = Timeout(drivers.clock)
        self.start_shot_timeout = Timeout(drivers.clock)
        self.received_cv_target_ever = False
        self.num_cycles_for_burst = 0
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.targeting = False
        self.turning_to_hit = False
        self.hit_target_yaw = 0.0
        self.hit_turn_start_time = 0
        self.is_scheduled = False

    def initialize(self):
        """AutoAimAndFireCommand.cpp:25-27."""
        self.is_scheduled = True

    def end(self):
        self.is_scheduled = False

    def execute(self):
        """AutoAimAndFireCommand.cpp:28-140, isManualControl false."""
        ref, now = self.drivers.ref_serial, self.drivers.clock()
        allow_shooting = allow_gimbal = True
        if ref.in_3v3():
            allow_shooting = allow_gimbal = ref.game_stage == IN_GAME
            if ref.game_stage == COUNTDOWN:
                allow_gimbal = True

        msg = self.jetson.get_cv_target()
        if msg is not None:
            self.cv_target = msg
            self.received_cv_target_ever = True
            self.cv_target_valid_timeout.restart(TARGET_VALID_TIME)
            self.start_shot_timeout.restart(msg.delay_ms - FIRING_LATENCY_TIME)
        if self.cv_target_valid_timeout.is_expired():
            self.cv_target_valid_timeout.stop()
        angle = self.jetson.get_angle_to_turn_for_sentry()
        flags = self.cv_target.flags
        turn_to_hit = bool(flags & CV_TARGET_FLAG_TURN_TO_HIT)
        patrol = bool(flags & CV_TARGET_FLAG_TYPE_C_BASED_PATROL)
        shoot = bool(flags & CV_TARGET_FLAG_FIRE)
        need_to_turn_to_hit = turn_to_hit and angle != PLACEHOLDER_ANGLE
        self.turning_to_hit = self.turning_to_hit and turn_to_hit

        self.targeting = (allow_gimbal and not self.cv_target_valid_timeout.is_stopped()
                          and not (need_to_turn_to_hit or self.turning_to_hit))
        # The field bearing less the start's yaw: the gimbal's world yaw is the IMU's.
        dx, dy = self.cv_target.x - self.odo.get_x(), self.cv_target.y - self.odo.get_y()
        bearing = 0.0 if dx == 0 and dy == 0 else math.atan2(dy, dx)  # Vector2d::angle
        self.target_yaw = bearing - self.odo.get_start_yaw()
        z = self.cv_target.z
        if self.fix_pivot_z:
            z -= ballistics.OFFSET_Z_ROBOT_TO_PITCH_PIVOT
        self.target_pitch = ballistics.solve_for_pitch(math.hypot(dx, dy), z)

        if self.targeting:
            self.gimbal.set_angles(self.target_yaw, self.target_pitch)
            if self.start_shot_timeout.execute() and allow_shooting and shoot:
                self.indexer.try_shoot_once()
        elif allow_gimbal:
            self._face_hit_or_patrol(need_to_turn_to_hit, angle, patrol, now)

        if not allow_shooting:
            self.indexer.stop_index()
        if allow_gimbal:
            self.flywheel.set_target_velocity(FLYWHEEL_MOTOR_MAX_RPM)
        else:
            self.gimbal.stop_motors()
            if self.adc_scheduled():
                self.flywheel.set_target_velocity(FLYWHEEL_MOTOR_MAX_RPM / 4)

    def _face_hit_or_patrol(self, need_to_turn_to_hit, angle, patrol, now):
        """AutoAimAndFireCommand.cpp:91-125: hold a hit's heading 500 ms, else sweep."""
        if need_to_turn_to_hit:
            self.hit_target_yaw = self.gimbal.get_yaw_angle_relative_world() - angle
            self.turning_to_hit = True
            self.hit_turn_start_time = now
        if self.turning_to_hit and now - self.hit_turn_start_time < HIT_TURN_DURATION:
            self.gimbal.set_angles(self.hit_target_yaw, PATROL_PITCH)
            return
        self.turning_to_hit = False
        yaw_change = 0.0
        if patrol:
            self.num_cycles_for_burst += 1
            if self.num_cycles_for_burst == CYCLES_UNTIL_BURST:
                yaw_change = BURST_AMOUNT
                self.num_cycles_for_burst = 0
            else:
                yaw_change = PATROL_SPEED
        self.gimbal.update_motors(yaw_change, PATROL_PITCH)
