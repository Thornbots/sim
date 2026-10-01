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
taproot's ballistics (taproot/src/tap/algorithms/ballistics.{hpp,cpp}), no drag.

Frame: x downrange, y left, z up, metres, from the pitch axis. Pitch comes
out negative to aim up, the firmware's encoder sense (positive is down).
"""
import math

ACCELERATION_GRAVITY = 9.80665  # math_user_utils.hpp:44


class SecondOrderKinematicState:
    """ballistics.hpp SecondOrderKinematicState: constant-acceleration projection."""

    def __init__(self, position, velocity, acceleration):
        self.position = tuple(position)
        self.velocity = tuple(velocity)
        self.acceleration = tuple(acceleration)

    def project_forward(self, dt):
        return tuple(s + v * dt + 0.5 * a * dt ** 2
                     for s, v, a in zip(self.position, self.velocity, self.acceleration))


def _close(a, b, eps):
    return abs(a - b) <= eps  # compareFloatClose, math_user_utils.hpp


def compute_travel_time(target, bullet_velocity, pitch_axis_offset=0.0):
    """computeTravelTime, ballistics.cpp: (ok, travel_time, turret_pitch)."""
    horizontal = math.hypot(target[0], target[1]) + pitch_axis_offset
    v2 = bullet_velocity ** 2
    g = ACCELERATION_GRAVITY
    sqrt_term = v2 ** 2 - g * (g * horizontal ** 2 + 2 * target[2] * v2)
    if sqrt_term < 0:
        return False, None, None
    pitch = -math.atan2(v2 - math.sqrt(sqrt_term), g * horizontal)
    if _close(pitch, 0.0, 1e-2):
        sqrt_term = bullet_velocity ** 2 - 2 * g * target[2]
        if sqrt_term < 0:
            return False, None, pitch
        # A ballistics.cpp quirk kept as is: this is the vertical-shot time.
        return True, (bullet_velocity - math.sqrt(sqrt_term)) / g, pitch
    travel = horizontal / (bullet_velocity * math.cos(pitch))
    return not (math.isnan(pitch) or math.isnan(travel)), travel, pitch


def find_target_projectile_intersection(state, bullet_velocity, num_iterations,
                                        pitch_axis_offset=0.0):
    """findTargetProjectileIntersection, ballistics.cpp: (ok, pitch, yaw, travel_time)."""
    projected = state.project_forward(0.0)
    if projected == (0.0, 0.0, 0.0):
        return False, None, None, None
    pitch = travel = None
    for _ in range(num_iterations):
        ok, travel, pitch = compute_travel_time(projected, bullet_velocity, pitch_axis_offset)
        if not ok:
            return False, pitch, None, travel
        projected = state.project_forward(travel)
    yaw = math.atan2(projected[1], projected[0])
    return not (math.isnan(pitch) or math.isnan(yaw)), pitch, yaw, travel
