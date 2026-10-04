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

Per 1 ms cycle: aim at the latest CvTarget's odom point for TARGET_VALID_TIME
after it arrives, one tryShootOnce per fire frame after delay_ms, else
patrol. In a 3v3 game it shoots only IN_GAME. fix_delay clamps
delay_ms - FIRING_LATENCY_TIME at 0; the firmware's uint32 wraps it.
fix_pivot_z solves pitch for z above the pitch pivot, not the ground.
"""
import math

from sim.mcb_emulator import ballistics
from sim.mcb_emulator.jetson import PLACEHOLDER_ANGLE
from sim.mcb_emulator.protocol import (CV_TARGET_FLAG_FIRE, CV_TARGET_FLAG_TYPE_C_BASED_PATROL,
                                       CvTarget)
from sim.mcb_emulator.subsystems import COUNTDOWN, IN_GAME, PI, Timeout

# AutoAimAndFireCommand.hpp:71-91
CYCLES_UNTIL_BURST = 380
PATROL_SPEED = -0.002  # rad/cycle: -2 rad/s at 1 kHz
BURST_AMOUNT = PATROL_SPEED  # rad/cycle, burst mode off
TARGET_VALID_TIME = 200  # ms
FIRING_LATENCY_TIME = 5  # ms
HIT_TURN_DURATION = 500  # ms
PATROL_PITCH = 0.05  # rad, down
FLYWHEEL_MOTOR_MAX_RPM = 1.0  # FlywheelSubsystemConstants.hpp; only its fraction matters here


class AutoAimAndFireCommand:

    def __init__(self, drivers, gimbal, indexer, flywheel, jetson, odo, adc_scheduled,
                 fix_delay=False, fix_pivot_z=False):
        """adc_scheduled(): AutoDriveCommand::getIsScheduled, for the idle flywheel."""
        self.drivers, self.gimbal, self.indexer = drivers, gimbal, indexer
        self.flywheel, self.jetson, self.odo = flywheel, jetson, odo
        self.adc_scheduled, self.fix_delay = adc_scheduled, fix_delay
        self.fix_pivot_z = fix_pivot_z
        self.cv_target = CvTarget()
        # Both start stopped, and a stopped timer is never expired: until the first
        # frame, the aim branch runs on CvTarget{}, the odom origin.
        self.cv_target_valid_timeout = Timeout(drivers.clock)
        self.start_shot_timeout = Timeout(drivers.clock)
        self.num_cycles_for_burst = 0
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.targeting = False
        self.turning_to_hit = False
        self.hit_target_yaw = 0.0
        self.hit_turn_start_time = 0
        self.is_scheduled = False

    def initialize(self):
        """AutoAimAndFireCommand.cpp:8-10."""
        self.is_scheduled = True

    def end(self):
        self.is_scheduled = False

    def execute(self):
        """AutoAimAndFireCommand.cpp:11-115."""
        ref, now = self.drivers.ref_serial, self.drivers.clock()
        allow_shooting = allow_gimbal = True
        if ref.in_3v3():
            allow_shooting = allow_gimbal = ref.game_stage == IN_GAME
            if ref.game_stage == COUNTDOWN:
                allow_gimbal = True

        msg = self.jetson.get_cv_target()
        if msg is not None:
            self.cv_target = msg
            self.cv_target_valid_timeout.restart(TARGET_VALID_TIME)
            delay = msg.delay_ms - FIRING_LATENCY_TIME
            self.start_shot_timeout.restart(max(0, delay) if self.fix_delay else delay % 2 ** 32)
        if allow_gimbal and not self.cv_target_valid_timeout.is_expired():
            self._aim(allow_shooting)
        else:
            self._patrol(allow_gimbal, now)

        if not allow_shooting:
            self.indexer.stop_index()
        if allow_gimbal:
            self.flywheel.set_target_velocity(FLYWHEEL_MOTOR_MAX_RPM)
        else:
            self.gimbal.stop_motors()
            if self.adc_scheduled():
                self.flywheel.set_target_velocity(FLYWHEEL_MOTOR_MAX_RPM / 4)

    def _aim(self, allow_shooting):
        """AutoAimAndFireCommand.cpp:41-62: the odom point from odometry, no lead."""
        dx, dy = self.cv_target.x - self.odo.get_x(), self.cv_target.y - self.odo.get_y()
        angle = 0.0 if dx == 0 and dy == 0 else math.atan2(dy, dx)  # Vector2d::angle
        self.target_yaw = angle - PI / 2  # gimbal yaw 0 is odometry's +y
        z = self.cv_target.z
        if self.fix_pivot_z:
            z -= ballistics.OFFSET_Z_ROBOT_TO_PITCH_PIVOT
        self.target_pitch = ballistics.solve_for_pitch(math.hypot(dx, dy), z)
        self.gimbal.set_angles(self.target_yaw, self.target_pitch)
        self.targeting = True
        shoot = self.cv_target.flags & CV_TARGET_FLAG_FIRE
        if allow_shooting and shoot and self.start_shot_timeout.execute():
            self.indexer.try_shoot_once()

    def _patrol(self, allow_gimbal, now):
        """AutoAimAndFireCommand.cpp:63-103: sweep if the flag allows, or face a hit."""
        self.targeting = False
        self.num_cycles_for_burst += 1
        if not allow_gimbal:
            return
        angle = self.jetson.get_angle_to_turn_for_sentry()
        # The TURN_TO_HIT flag is read there but never used.
        if angle != PLACEHOLDER_ANGLE and angle:
            self.target_pitch = PATROL_PITCH
            self.hit_target_yaw = self.gimbal.get_yaw_angle_relative_world() - angle
            self.turning_to_hit = True
            self.hit_turn_start_time = now
        if self.turning_to_hit and now - self.hit_turn_start_time < HIT_TURN_DURATION:
            self.gimbal.set_angles(self.hit_target_yaw, self.target_pitch)
            return
        self.turning_to_hit = False
        type_c_based_patrol = self.cv_target.flags & CV_TARGET_FLAG_TYPE_C_BASED_PATROL
        if type_c_based_patrol:
            self.target_pitch = PATROL_PITCH
        yaw_change = 0.0
        if self.num_cycles_for_burst == CYCLES_UNTIL_BURST:
            if type_c_based_patrol:
                yaw_change = BURST_AMOUNT
            self.num_cycles_for_burst = 0
        elif type_c_based_patrol:
            yaw_change = PATROL_SPEED
        self.gimbal.update_motors(yaw_change, self.target_pitch)
