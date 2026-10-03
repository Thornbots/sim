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
Reticle::solveForPitch (subsystems/ui/objects/Reticle.hpp:312-381), the sentry's constants.

Robot frame: x right, y forward (downrange), z up, metres. Pitch is
positive down, the gimbal's sense. The landing height is from the pitch
pivot, but it's compared with targetZ as given: that's how the firmware
does it, so a target's height off the ground aims OFFSET_Z_ROBOT_TO_PITCH_PIVOT
too high.
"""
import math

ACCELERATION_GRAVITY = 9.80665  # math_user_utils.hpp:44
INITIAL_SHOT_VELOCITY = 24.0  # JetsonSubsystemConstants.hpp:46, SENTRY
OFFSET_Y_PITCH_PIVOT_TO_BARREL = 0.068  # Projections.hpp:32, SENTRY; X and Z are 0
OFFSET_Z_ROBOT_TO_PITCH_PIVOT = 0.39  # Projections.hpp:25, SENTRY; unused by solveForPitch
MAX_NUM_ITERATIONS = 10  # Reticle.hpp:292


def rotate_pitch(v, amt):
    """Vector3d::rotatePitch (util/Vector3d.hpp:43-46): turn (y, z) by amt about x."""
    x, y, z = v
    mag = math.hypot(y, z)
    angle = 0.0 if y == 0 and z == 0 else math.atan2(z, y)
    return x, mag * math.cos(amt + angle), mag * math.sin(amt + angle)


def get_initials(pitch):
    """Reticle.hpp:312-318: (position, velocity) of a shot leaving the barrel, pivot space."""
    velo = rotate_pitch((0.0, INITIAL_SHOT_VELOCITY, 0.0), -pitch)
    # barrelSpaceToPivotSpace of the barrel origin (Projections.hpp:82-84)
    pos = rotate_pitch((0.0, OFFSET_Y_PITCH_PIVOT_TO_BARREL, 0.0), -pitch)
    return pos, velo


def landing_height(distance_down_range, pitch):
    """calculateLandingSpotForHeightOffGround, Reticle.hpp:350-360: z at that range."""
    pos, velo = get_initials(pitch)
    t = (distance_down_range - pos[1]) / velo[1]
    return pos[2] + velo[2] * t - ACCELERATION_GRAVITY / 2 * t * t


def solve_for_pitch(distance_down_range, target_z):
    """Reticle.hpp:363-381: bisect from 0 in steps of pi/4, pi/8, ..."""
    pitch = 0.0
    for j in range(MAX_NUM_ITERATIONS):
        if landing_height(distance_down_range, pitch) > target_z:
            pitch += math.pi / (4 << j)
        else:
            pitch -= math.pi / (4 << j)
    return pitch
