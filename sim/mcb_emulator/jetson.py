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
JetsonSubsystem (subsystems/jetson/JetsonSubsystem.{hpp,cpp}).

Sends 9 POSE then 1 REF_SYS on a 10 ms timer (90 Hz and 10 Hz), applies
RELOCALIZE to odometry, and hands CV_TARGET to AutoAimAndFireCommand as is.
"""
from sim.mcb_emulator.protocol import NavGoal, Pose, RefSys, Relocalize
from sim.mcb_emulator.subsystems import PeriodicMilliTimer

PLACEHOLDER_ANGLE = 123.0  # HitRing.hpp:99, "not hit"


class JetsonSubsystem:

    def __init__(self, drivers, gimbal, odo, send, cv_target_cls):
        """send(struct): one UART frame out; cv_target_cls: the CvTarget layout to read."""
        self.drivers, self.gimbal, self.odo, self.send = drivers, gimbal, odo, send
        self.cv_target_cls = cv_target_cls
        self.pose_data_timeout = PeriodicMilliTimer(drivers.clock, 10)  # .hpp:128
        self.message_count = 0
        self.angle_to_turn_for_sentry = PLACEHOLDER_ANGLE

    def refresh(self):
        """JetsonSubsystem.cpp:15-63; uart.updateSerial runs in Sentry.step."""
        if self.odo is not None:
            self.check_apply_relocalize()
        if not self.pose_data_timeout.execute():
            return
        self.message_count += 1
        if self.message_count < 10 and self.odo is not None:
            self.send(Pose(self.odo.get_x(), self.odo.get_y(), self.odo.get_x_vel(),
                           self.odo.get_y_vel(), self.gimbal.get_pitch_encoder_value(),
                           self.gimbal.get_yaw_angle_relative_world()))
        else:
            self.angle_to_turn_for_sentry = PLACEHOLDER_ANGLE  # HitRing: no referee hits yet
            self.send(self.ref_sys_msg())
            self.message_count = 0

    def ref_sys_msg(self):
        """JetsonSubsystem.cpp:39-59."""
        r = self.drivers.ref_serial
        bits = (r.is_blue_team(r.robot_id) << 7 | r.recovery_buff << 6
                | (r.restoration_zone or r.exchange_zone) << 5 | r.central_buff_zone << 4
                | r.team_occupies_center << 3 | r.opponent_occupies_center << 2
                | r.chassis_power << 1 | r.gimbal_power)
        return RefSys(r.game_stage, r.stage_time_remaining, r.current_hp,
                      r.robot_id % 100, self.angle_to_turn_for_sentry, bits)

    def get_angle_to_turn_for_sentry(self):
        """JetsonSubsystem.cpp:65-69."""
        r = self.angle_to_turn_for_sentry
        self.angle_to_turn_for_sentry = PLACEHOLDER_ANGLE
        return r

    def check_apply_relocalize(self):
        """JetsonSubsystem.cpp:71-76: odometry jumps to the Jetson's point."""
        msg = self.drivers.uart.get_msg(Relocalize)
        if msg is not None:
            self.odo.relocalize_to(msg.x, msg.y)

    def update_ros(self):
        """JetsonSubsystem.cpp:79-86: the NAV_GOAL (x, y), or None."""
        goal = self.drivers.uart.get_msg(NavGoal)
        return None if goal is None else (goal.x, goal.y)

    def get_cv_target(self):
        """JetsonSubsystem.cpp:89-91: a new CvTarget, or None."""
        return self.drivers.uart.get_msg(self.cv_target_cls)
