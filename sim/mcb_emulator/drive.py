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

Odometry frame: x right, y forward of the heading at boot, CCW rotation
(SimpleAutoDriveCommand.hpp:188-189). ChassisController's position loop is
ported; its velocity loop and power limiting are gz's VelocityControl.
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
AUTO_DRIVE_SPIN = 9.0  # AutoDriveCommand.cpp:20
TOWARDS_ZONE_OFFSET = 0.5  # SimpleAutoDriveCommand.hpp:279
STUCK_TIMER_AMOUNT = 15 * 1000  # ms, :285
MAX_RELOCALIZE_TRIES = 3  # :305
RELOCALIZE_X_OFFSET = 0.688  # :93, blue adds it, red subtracts
RELOCALIZE_Y_OFFSET = -0.05
ARCC_ROUGH_PATH = 'arcc_rough_path'

# RfidRelocalizeState, SimpleAutoDriveCommand.hpp:295-299
WAITING_FOR_CENTER, WAITING_FOR_RESUPPLY, STUCK_WAITING_FOR_RESUPPLY = range(3)


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
        """ChassisController::followPosition, ChassisController.cpp:179-191."""
        ex, ey = target[0] - current[0], target[1] - current[1]
        self.position_integral = (self.position_integral[0] + ex * KI * DT,
                                  self.position_integral[1] + ey * KI * DT)
        cx = ex * KP + self.position_integral[0]
        cy = ey * KP + self.position_integral[1]
        norm = max(math.hypot(cx, cy) / MAX_POS_VEL, 1.0)
        vx, vy = input_velocity[0] + cx / norm, input_velocity[1] + cy / norm
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
        """MoveToPositionCommand.cpp:13-41."""
        ref = self.drivers.ref_serial
        if ref.in_3v3() and ref.game_stage == COUNTDOWN:
            self.target_velocity = (0.0, 0.0, MOVE_TO_POS_SPIN_VELO)
        else:
            self.target_velocity = self.input_velocity
        reference_angle = (self.gimbal.get_yaw_encoder_value()
                           - self.gimbal.get_yaw_angle_relative_world())
        self.current_position = (self.odo.get_x(), self.odo.get_y(), reference_angle)
        self.drivetrain.set_target_position(self.target_position[:2], self.current_position,
                                            self.target_velocity)

    def is_finished(self):
        """MoveToPositionCommand.cpp:43, less the remote check (Sentry's)."""
        return math.hypot(self.target_position[0] - self.current_position[0],
                          self.target_position[1] - self.current_position[1]) < self.tolerance


class SimpleAutoDriveCommand:
    """
    SimpleAutoDriveCommand.hpp: waypoints out to the centre, back to resupply on low HP.

    Relocalize statics (:176-178) are instance fields here; JetsonSubsystem
    writes them through set_localization. The sentry runs ARCC_ROUGH_PATH
    (SentryControl.hpp:192).
    """

    def __init__(self, drivers, drivetrain, gimbal, odo, mode=ARCC_ROUGH_PATH):
        self.drivers, self.odo, self.mode = drivers, odo, mode
        self.position_command = MoveToPositionCommand(drivers, drivetrain, gimbal, odo, 0.5)
        self.drivetrain = drivetrain
        self.targets = [((0.0, 0.0), (0.0, 0.0))]
        self.stuck_timer = Timeout(drivers.clock)
        self.x_for_localization = self.y_for_localization = 0.0
        self.set_localization = False
        self.target_index = 0
        self.direction = 1
        self.is_scheduled = False
        self.need_to_apply_initial_point_change = True
        self.changed_initial_point = (0.0, 0.0)
        self.relocalize_state = WAITING_FOR_CENTER
        self.times_retried_relocalize = 0

    def set_localization_point(self, x, y):
        """JetsonSubsystem.cpp:120-122."""
        self.x_for_localization, self.y_for_localization = x, y
        self.set_localization = True

    def initialize(self):
        self.setup_map()
        self.is_scheduled = True

    def setup_map(self):
        """SimpleAutoDriveCommand.hpp:182-247, the ARCC_ROUGH_PATH case only."""
        if self.mode != ARCC_ROUGH_PATH:
            raise ValueError(f'only {ARCC_ROUGH_PATH} is ported, not {self.mode}')
        ref = self.drivers.ref_serial
        m = -1 if ref.is_blue_team(ref.robot_id) else 1
        self.changed_initial_point = (m * -TOWARDS_ZONE_OFFSET, -TOWARDS_ZONE_OFFSET)
        self.targets.append(((m * 2.236, 0.5), (0.0, 0.0)))
        self.targets.append(((m * 2.236, 1.224), (0.0, 0.0)))
        self.targets.append(((m * -TOWARDS_ZONE_OFFSET, 4.125 + TOWARDS_ZONE_OFFSET), (0.0, 0.0)))

    def set_direction(self):
        """SimpleAutoDriveCommand.hpp:249-270, the default (non-TEST) branch."""
        if self.relocalize_state == STUCK_WAITING_FOR_RESUPPLY:
            return
        ref = self.drivers.ref_serial
        if ref.receiving:
            ratio = ref.current_hp / ref.max_hp
            if ratio > 0.99:
                self.direction = 1
            if ratio <= 0.5525:
                self.direction = -1

    def execute(self):
        """SimpleAutoDriveCommand.hpp:64-167."""
        if not self.is_scheduled:
            self.drivetrain.set_target_translation((0.0, 0.0, SPIN_VELOCITY))
            return
        self.set_direction()
        faster_spinning = False
        ref = self.drivers.ref_serial
        is_blue = ref.is_blue_team(ref.robot_id)
        heal_zone = ref.restoration_zone or ref.exchange_zone
        if self.relocalize_state in (STUCK_WAITING_FOR_RESUPPLY, WAITING_FOR_RESUPPLY):
            if heal_zone:
                self.relocalize_state = WAITING_FOR_CENTER
                self.odo.relocalize_to(0.0, self.odo.get_y())
                if self.times_retried_relocalize >= MAX_RELOCALIZE_TRIES:
                    self.is_scheduled = False
        if (not self.need_to_apply_initial_point_change and ref.current_hp == ref.max_hp
                and self.set_localization and heal_zone):
            dx = RELOCALIZE_X_OFFSET if is_blue else -RELOCALIZE_X_OFFSET
            self.odo.relocalize_to(self.x_for_localization + dx,
                                   self.y_for_localization + RELOCALIZE_Y_OFFSET)
        if self.relocalize_state == WAITING_FOR_CENTER and ref.central_buff_zone:
            self.relocalize_state = WAITING_FOR_RESUPPLY
            self.times_retried_relocalize = 0

        if self.position_command.is_finished():
            faster_spinning = self._advance(ref)

        spin = SPIN_VELOCITY if faster_spinning else MOVE_TO_POS_SPIN_VELO
        (px, py), (vx, vy) = self.targets[self.target_index]
        self.position_command.target_position = (px, py, 0.0)
        self.position_command.input_velocity = (self.direction * vx, self.direction * vy, spin)
        self.position_command.execute()
        if self.stuck_timer.execute():
            self.times_retried_relocalize += 1
            self.direction = -1
            self.relocalize_state = STUCK_WAITING_FOR_RESUPPLY

    def _advance(self, ref):
        """SimpleAutoDriveCommand.hpp:106-146: next waypoint; True at either end."""
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
    AutoDriveCommand.{hpp,cpp}: holds a ROS_MSG goal and spins at 9 rad/s.

    Built but never scheduled on the sentry (SentryControl.hpp:61, :191).
    Its RELOCALIZE read never sees a frame: JetsonSubsystem::refresh takes it first.
    """

    def __init__(self, drivers, drivetrain, gimbal, jetson, odo):
        self.drivers, self.drivetrain, self.gimbal = drivers, drivetrain, gimbal
        self.jetson, self.odo = jetson, odo
        self.target_position = (0.0, 0.0)
        self.target_velocity = (0.0, 0.0, 0.0)
        self.is_scheduled = False

    def initialize(self):
        """AutoDriveCommand.cpp:11-17: the goal starts where the robot is."""
        self.is_scheduled = True
        self.target_position = (self.odo.get_x(), self.odo.get_y())

    def end(self):
        self.is_scheduled = False

    def execute(self):
        """AutoDriveCommand.cpp:19-93; the relocalize branch there is all comments."""
        self.target_velocity = (0.0, 0.0, AUTO_DRIVE_SPIN)
        allow_spinning = allow_moving = True
        ref = self.drivers.ref_serial
        if ref.in_3v3():
            allow_spinning = allow_moving = ref.game_stage == IN_GAME
            if ref.game_stage == COUNTDOWN:
                allow_spinning = True
        reference_angle = (self.gimbal.get_yaw_encoder_value()
                           - self.gimbal.get_yaw_angle_relative_world())
        goal, _ = self.jetson.update_ros()
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
