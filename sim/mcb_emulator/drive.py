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
The sentry's auto drive: SimpleAutoDriveCommand (what the right switch runs), AutoDriveCommand.

Positions are the field frame (OdometrySubsystem); waypoints are (forward,
left) from the start, turned into it by at()/vel(). The chassis frame is x
right, y forward. ChassisController's position loop is ported; its velocity
loop and power limiting are gz's VelocityControl.
"""
import math

from sim.mcb_emulator.subsystems import COUNTDOWN, IN_GAME, Timeout

# ChassisControllerConstants.hpp: :14 and the SENTRY block
MAX_POS_VEL = 1.7
KP = 2.0
KI = 0.01
DT = 0.001
SPIN_VELOCITY = -12.0  # DrivetrainSubsystemConstants.hpp:32
MOVE_TO_POS_SPIN_VELO = -8.0  # MoveToPositionCommand.hpp:48
AUTO_DRIVE_SPIN = 9.0  # AutoDriveCommand.cpp:22
TOWARDS_ZONE_OFFSET = 0.5  # SimpleAutoDriveCommand.hpp:71
STUCK_TIMER_AMOUNT = 15 * 1000  # ms, SimpleAutoDriveCommand.hpp:74
ARCC_ROUGH_PATH = 'arcc_rough_path'


def rotate(x, y, amt):
    """Turn the vector by amt, CCW: Pose2d::rotate, Pose2d.hpp:26."""
    c, s = math.cos(amt), math.sin(amt)
    return c * x - s * y, s * x + c * y


class DrivetrainSubsystem:
    """DrivetrainSubsystem.cpp:71-109 down to ChassisController's local velocity."""

    def __init__(self, hw):
        self.hw = hw
        self.position_integral = (0.0, 0.0)

    def set_target_position(self, target, current, input_velocity):
        """ChassisController::followPosition, ChassisController.cpp:179-194."""
        ex, ey = target[0] - current[0], target[1] - current[1]
        self.position_integral = (self.position_integral[0] + ex * KI * DT,
                                  self.position_integral[1] + ey * KI * DT)
        cx = ex * KP + self.position_integral[0]
        cy = ey * KP + self.position_integral[1]
        norm = max(math.hypot(cx, cy) / MAX_POS_VEL, 1.0)
        vx, vy = input_velocity[0] + cx / norm, input_velocity[1] + cy / norm
        vx, vy = -vy, vx  # field (x forward, y left) -> the chassis frame's x right, y forward
        lx, ly = rotate(vx, vy, current[2])
        self.set_target_translation((lx, ly, input_velocity[2]))

    def set_target_translation(self, drive):
        """Local (x right, y forward, CCW rad/s); calculateBeybladeVelocity passes < 17.5."""
        self.hw.set_chassis(drive)

    def stop_motors(self):
        self.hw.set_chassis(None)


class MoveToPositionCommand:
    """MoveToPositionCommand.{hpp,cpp}."""

    def __init__(self, drivers, drivetrain, gimbal, odo, tolerance=0.2):
        self.drivers, self.drivetrain, self.gimbal, self.odo = drivers, drivetrain, gimbal, odo
        self.tolerance = tolerance
        self.target_position = (0.0, 0.0, 0.0)
        self.input_velocity = (0.0, 0.0, 0.0)
        self.target_velocity = (0.0, 0.0, MOVE_TO_POS_SPIN_VELO)
        self.current_position = (0.0, 0.0, 0.0)

    def execute(self):
        """MoveToPositionCommand.cpp:13-42."""
        ref = self.drivers.ref_serial
        if ref.in_3v3() and ref.game_stage == COUNTDOWN:
            self.target_velocity = (0.0, 0.0, MOVE_TO_POS_SPIN_VELO)
        else:
            self.target_velocity = self.input_velocity
        reference_angle = (self.gimbal.get_yaw_encoder_value()
                           - self.gimbal.get_yaw_angle_relative_world()
                           - self.odo.get_start_yaw())  # the IMU's zero is the start's heading
        self.current_position = (self.odo.get_x(), self.odo.get_y(), reference_angle)
        self.drivetrain.set_target_position(self.target_position[:2], self.current_position,
                                            self.target_velocity)

    def is_finished(self):
        """MoveToPositionCommand.cpp:44, less the remote check (Sentry's)."""
        return math.hypot(self.target_position[0] - self.current_position[0],
                          self.target_position[1] - self.current_position[1]) < self.tolerance


class SimpleAutoDriveCommand:
    """
    SimpleAutoDriveCommand.cpp: waypoints out to the centre, back to resupply on low HP.

    RELOCALIZE moves odometry directly (JetsonSubsystem), so nothing here
    relocalizes. Stuck for STUCK_TIMER_AMOUNT, it stops driving and spins in
    place. The sentry runs ARCC_ROUGH_PATH (SentryControl.hpp:61, :191).
    """

    def __init__(self, drivers, drivetrain, gimbal, odo, mode=ARCC_ROUGH_PATH):
        self.drivers, self.odo, self.mode = drivers, odo, mode
        self.position_command = MoveToPositionCommand(drivers, drivetrain, gimbal, odo, 0.5)
        self.drivetrain = drivetrain
        self.targets = [((0.0, 0.0), (0.0, 0.0))]
        self.stuck_timer = Timeout(drivers.clock)
        self.target_index = 0
        self.direction = 1
        self.is_scheduled = False
        self.need_to_apply_initial_point_change = True
        self.changed_initial_point = (0.0, 0.0)

    def initialize(self):
        self.setup_map()
        self.is_scheduled = True

    def setup_map(self):
        """SimpleAutoDriveCommand.cpp:102-174, the ARCC_ROUGH_PATH case only."""
        if self.mode != ARCC_ROUGH_PATH:
            raise ValueError(f'only {ARCC_ROUGH_PATH} is ported, not {self.mode}')
        ref = self.drivers.ref_serial
        m = -1 if ref.is_blue_team(ref.robot_id) else 1

        def at(forward, left):
            return self.odo.from_start(forward, left)

        def vel(forward, left):
            return rotate(forward, left, self.odo.get_start_yaw())

        self.targets[0] = (at(0.0, 0.0), self.targets[0][1])  # the start
        self.changed_initial_point = at(-TOWARDS_ZONE_OFFSET, m * TOWARDS_ZONE_OFFSET)
        self.targets.append((at(0.5, -m * 2.236), vel(0.0, 0.0)))
        self.targets.append((at(1.224, -m * 2.236), vel(0.0, 0.0)))
        self.targets.append((at(4.125 + TOWARDS_ZONE_OFFSET, m * TOWARDS_ZONE_OFFSET),
                             vel(0.0, 0.0)))

    def set_direction(self):
        """SimpleAutoDriveCommand.cpp:176-195, the default (non-TEST) branch."""
        ref = self.drivers.ref_serial
        if ref.receiving:
            ratio = ref.current_hp / ref.max_hp
            if ratio > 0.99:
                self.direction = 1
            if ratio <= 0.5525:
                self.direction = -1

    def execute(self):
        """SimpleAutoDriveCommand.cpp:23-94."""
        if not self.is_scheduled:
            self.drivetrain.set_target_translation((0.0, 0.0, SPIN_VELOCITY))
            return
        self.set_direction()
        faster_spinning = False
        if self.position_command.is_finished():
            faster_spinning = self._advance(self.drivers.ref_serial)
        spin = SPIN_VELOCITY if faster_spinning else MOVE_TO_POS_SPIN_VELO
        (px, py), (vx, vy) = self.targets[self.target_index]
        self.position_command.target_position = (px, py, 0.0)
        self.position_command.input_velocity = (self.direction * vx, self.direction * vy, spin)
        self.position_command.execute()
        if self.stuck_timer.execute():
            self.is_scheduled = False

    def _advance(self, ref):
        """SimpleAutoDriveCommand.cpp:34-74: next waypoint; True at either end."""
        allow_advancing = not ref.in_3v3() or ref.game_stage == IN_GAME
        if not allow_advancing:
            self.stuck_timer.restart(STUCK_TIMER_AMOUNT)
            return False
        self.stuck_timer.restart(STUCK_TIMER_AMOUNT)
        if self.direction == 1:
            if self.target_index < len(self.targets) - 1:
                if ref.game_stage == IN_GAME:
                    self.target_index += 1
                if self.need_to_apply_initial_point_change:
                    self.need_to_apply_initial_point_change = False
                    self.targets[0] = (self.changed_initial_point, self.targets[0][1])
                return False
            return True
        if self.target_index > 0:
            self.target_index -= 1
            return False
        return True


class AutoDriveCommand:
    """
    AutoDriveCommand.{hpp,cpp}: holds a NAV_GOAL and spins at 9 rad/s.

    Built but never scheduled on the sentry (SentryControl.hpp:61).
    """

    def __init__(self, drivers, drivetrain, gimbal, jetson, odo):
        self.drivers, self.drivetrain, self.gimbal = drivers, drivetrain, gimbal
        self.jetson, self.odo = jetson, odo
        self.target_position = (0.0, 0.0)
        self.target_velocity = (0.0, 0.0, 0.0)
        self.is_scheduled = False

    def initialize(self):
        """AutoDriveCommand.cpp:13-19: the goal starts where the robot is."""
        self.is_scheduled = True
        self.target_position = (self.odo.get_x(), self.odo.get_y())

    def end(self):
        self.is_scheduled = False

    def execute(self):
        """AutoDriveCommand.cpp:21-95; the relocalize branch there is all comments."""
        self.target_velocity = (0.0, 0.0, AUTO_DRIVE_SPIN)
        allow_spinning = allow_moving = True
        ref = self.drivers.ref_serial
        if ref.in_3v3():
            allow_spinning = allow_moving = ref.game_stage == IN_GAME
            if ref.game_stage == COUNTDOWN:
                allow_spinning = True
        reference_angle = (self.gimbal.get_yaw_encoder_value()
                           - self.gimbal.get_yaw_angle_relative_world()
                           - self.odo.get_start_yaw())  # the IMU's zero is the start's heading
        goal = self.jetson.update_ros()
        if goal is not None:  # Pose2d(Vector2d) zeroes rotation: no spin this cycle
            self.target_position, self.target_velocity = goal, (0.0, 0.0, 0.0)
        current = (self.odo.get_x(), self.odo.get_y(), reference_angle)
        px, py = self.target_position
        vx, vy, vr = self.target_velocity
        if not allow_moving:
            vx = vy = 0.0
            px, py = current[0], current[1]
        if not allow_spinning:
            vr = 0.0
        self.target_position = (px, py)
        self.target_velocity = (vx, vy, vr)
        self.drivetrain.set_target_position(self.target_position, current, self.target_velocity)
