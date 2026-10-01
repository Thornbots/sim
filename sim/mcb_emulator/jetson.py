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
JetsonSubsystem (subsystems/jetson/JetsonSubsystem.{hpp,cpp}) and its SENTRY constants.

Sends 9 POSE_MSG then 1 REF_SYS_MSG on a 10 ms timer (90 Hz and 10 Hz),
reads CVData as a camera-frame point and solves its own ballistics.
"""
from dataclasses import dataclass
import math

from sim.mcb_emulator import ballistics
from sim.mcb_emulator.protocol import CVData, PoseData, RefSysMsg, Relocalize, ROSData
from sim.mcb_emulator.subsystems import PeriodicMilliTimer, PI

# JetsonSubsystemConstants.hpp:6-12 and the SENTRY block, :40-63
MSG_CONFIDENCE_CUTOFF = 0.75
YAW_OUT_SHOOT_THRESH = PI / 3
MAX_SHOOT_DIST = 3.0  # defined, but its only use is commented out (JetsonSubsystem.cpp:207)
CAMERA_X_OFFSET = -0.0325
CAMERA_Y_OFFSET = 0.1
CAMERA_Z_OFFSET = 54.791 / 1000
INITIAL_SHOT_VELOCITY = 24.0
PITCH_MULTIPLY_MIN, PITCH_MULTIPLY_MAX, PITCH_MULTIPLY_SCALE = 0.0, 0.000005, 0.00000005
YAW_MULTIPLY_MIN, YAW_MULTIPLY_MAX, YAW_MULTIPLY_SCALE = 0.0, 0.015, 0.05
ORIENTATION_QUEUE_SIZE = 23  # JetsonSubsystem.hpp:167, cycles of 1 ms
PLACEHOLDER_ANGLE = 123.0  # HitRing.hpp:99, "not hit"


@dataclass
class OrientationSample:
    """JetsonSubsystem.hpp:127-132."""

    cv_yaw: float = 0.0
    cv_pitch: float = 0.0
    cv_yaw_vel: float = 0.0
    cv_pitch_vel: float = 0.0


@dataclass
class AimResult:
    """update()'s out-parameters; action -1 none, 0 aim, 1 aim and allow shooting."""

    action: int = -1
    yaw_out: float = 0.0
    pitch_out: float = 0.0
    yaw_vel_out: float = 0.0
    pitch_vel_out: float = 0.0


class JetsonSubsystem:

    def __init__(self, drivers, gimbal, odo, send):
        """send(struct): one UART frame out; drivers has uart, ref_serial, hw, clock."""
        self.drivers, self.gimbal, self.odo, self.send = drivers, gimbal, odo, send
        self.pose_data_timeout = PeriodicMilliTimer(drivers.clock, 10)  # .hpp:143
        self.message_count = 0
        self.orientation_queue = [OrientationSample() for _ in range(ORIENTATION_QUEUE_SIZE)]
        self.orientation_queue_head = 0
        self.angle_to_turn_for_sentry = PLACEHOLDER_ANGLE
        self.relocalize_sink = None  # SimpleAutoDriveCommand's statics, set by Sentry

    def refresh(self):
        """JetsonSubsystem.cpp:30-82; uart.updateSerial runs in Sentry.step."""
        if self.odo is not None:
            self.check_apply_relocalize()
        self.record_orientation_sample()
        if not self.pose_data_timeout.execute():
            return
        self.message_count += 1
        if self.message_count < 10 and self.odo is not None:
            self.send(PoseData(self.odo.get_x(), self.odo.get_y(), self.odo.get_x_vel(),
                               self.odo.get_y_vel(), self.gimbal.get_pitch_encoder_value(),
                               self.gimbal.get_yaw_angle_relative_world()))
        else:
            self.angle_to_turn_for_sentry = PLACEHOLDER_ANGLE  # HitRing: no referee hits yet
            self.send(self.ref_sys_msg())
            self.message_count = 0

    def ref_sys_msg(self):
        """JetsonSubsystem.cpp:58-76."""
        r = self.drivers.ref_serial
        bits = (r.is_blue_team(r.robot_id) << 7 | r.recovery_buff << 6
                | (r.restoration_zone or r.exchange_zone) << 5 | r.central_buff_zone << 4
                | r.team_occupies_center << 3 | r.opponent_occupies_center << 2
                | r.chassis_power << 1 | r.gimbal_power)
        return RefSysMsg(r.game_stage, r.stage_time_remaining, r.current_hp,
                         r.robot_id % 100, self.angle_to_turn_for_sentry, bits)

    def record_orientation_sample(self):
        """JetsonSubsystem.cpp:84-102: world turret yaw from the IMU, pitch from the gimbal."""
        hw = self.drivers.hw
        sample = OrientationSample(hw.imu_yaw(), -self.gimbal.get_pitch_encoder_value(),
                                   hw.imu_gz(), -self.gimbal.get_pitch_vel())
        self.orientation_queue[self.orientation_queue_head] = sample
        self.orientation_queue_head = (self.orientation_queue_head + 1) % ORIENTATION_QUEUE_SIZE

    def get_delayed_orientation(self):
        return self.orientation_queue[self.orientation_queue_head]  # :104-108, the oldest

    def get_angle_to_turn_for_sentry(self):
        """JetsonSubsystem.cpp:110-114."""
        r = self.angle_to_turn_for_sentry
        self.angle_to_turn_for_sentry = PLACEHOLDER_ANGLE
        return r

    def check_apply_relocalize(self):
        """JetsonSubsystem.cpp:116-124: hands the point to SimpleAutoDriveCommand, no more."""
        msg = self.drivers.uart.get_msg(Relocalize)
        if msg is not None and self.relocalize_sink is not None:
            self.relocalize_sink(msg.expectedX, msg.expectedY)

    def update_ros(self):
        """JetsonSubsystem.cpp:126-141: (target (x, y) or None, expected position or None)."""
        expected = None
        relocalize = self.drivers.uart.get_msg(Relocalize)
        if relocalize is not None:
            expected = (relocalize.expectedX, relocalize.expectedY)
        ros = self.drivers.uart.get_msg(ROSData)
        if ros is None:
            return None, expected
        return (ros.targetX, ros.targetY), expected

    def update(self, current_yaw, current_pitch, current_yaw_velo, current_pitch_velo):
        """JetsonSubsystem.cpp:143-274: one CVData to a yaw step, a pitch and an action."""
        out = AimResult()
        msg = self.drivers.uart.get_msg(CVData)
        if msg is None:
            return out
        if msg.confidence <= MSG_CONFIDENCE_CUTOFF:
            return out
        d = self.get_delayed_orientation()
        cv_yaw, cv_pitch, cv_yaw_vel, cv_pitch_vel = (d.cv_yaw, d.cv_pitch, d.cv_yaw_vel,
                                                      d.cv_pitch_vel)
        # Camera to frame 4, the shooter axis: y and z swapped, :197-205.
        x4 = msg.x + CAMERA_X_OFFSET
        y4 = msg.z + CAMERA_Y_OFFSET
        z4 = msg.y + CAMERA_Z_OFFSET
        vx4, vy4, vz4 = msg.v_x, msg.v_z, msg.v_y
        c3, s3, c4, s4 = math.cos(cv_yaw), math.sin(cv_yaw), math.cos(cv_pitch), math.sin(cv_pitch)
        # Frame 4 to frame 2 (world yaw, at the pitch axis), :218-229.
        px = c3 * x4 + s3 * (-c4 * y4 + s4 * z4)
        py = s3 * x4 + c3 * (c4 * y4 - s4 * z4)
        pz = s4 * y4 + c4 * z4
        vx = ((-s3 * x4 - c3 * (c4 * y4 - s4 * z4)) * cv_yaw_vel
              + s3 * (s4 * y4 + c4 * z4) * cv_pitch_vel + c3 * vx4 - s3 * (c4 * vy4 - s4 * vz4))
        vy = ((c3 * x4 - s3 * (c4 * y4 - s4 * z4)) * cv_yaw_vel
              - c3 * (s4 * y4 + c4 * z4) * cv_pitch_vel + s3 * vx4 + c3 * (c4 * vy4 - s4 * vz4))
        vz = cv_pitch_vel * c4 * y4 + s4 * vy4 - cv_pitch_vel * s4 * z4 + c4 * vz4
        # X down range, :233-237; velocity scaled down "until latency is reduced".
        state = ballistics.SecondOrderKinematicState(
            (py, -px, pz), (vy / 4.0, -vx / 4.0, vz / 100.0), (msg.a_x, msg.a_z, -msg.a_y))
        valid, target_pitch, target_yaw, _ = ballistics.find_target_projectile_intersection(
            state, INITIAL_SHOT_VELOCITY, 3)
        if not valid:
            return out
        out.yaw_out = target_yaw - current_yaw  # unwrapped; current_yaw is [0, 2pi)
        out.pitch_out = target_pitch
        out.yaw_vel_out = (-c3 * vx - s3 * vy + vx4) / (c4 * y4 - s4 * z4)
        out.pitch_vel_out = 0.0
        # abs() of a float at :268; std::abs is assumed (an int abs would allow < 2 rad).
        out.action = 1 if abs(out.yaw_out) < YAW_OUT_SHOOT_THRESH else 0
        return out
