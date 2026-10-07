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

"""Field-safe diagnostic paths and team routes from spawn to the center fight."""
import math

E1_PATHS = {
    'lateral': {'path_angle_deg': 0.0, 'center_x': 1.3, 'half_width': 1.9},
    'radial': {'path_angle_deg': 90.0, 'center_x': 1.3, 'half_width': 0.4},
    'diagonal': {'path_angle_deg': 45.0, 'center_x': 1.3, 'half_width': 0.65},
}
# Field frame: blue at +x, red at -x. Firmware's start is provisional too.
ROUTES = {
    'sentry': [(4.625, 0.0), (4.625, -2.35), (1.55, -2.35), (1.55, -0.65)],
    'opponent_0': [(-4.625, 0.0), (-4.625, -2.35), (-1.55, -2.35), (-1.55, -0.65)],
    'ally_0': [(5.0, 0.9), (5.0, -2.9), (2.15, -2.9), (2.15, -1.5),
               (0.65, -1.5), (0.65, 1.05)],
    'opponent_1': [(-5.0, 0.9), (-5.0, -2.9), (-2.15, -2.9), (-2.15, -1.5),
                   (-0.65, -1.5), (-0.65, 1.05)],
}
ROUTE_ACCEL = 2.0  # m/s^2
ROUTE_SPEED = 2.0  # m/s, stop at each corner


def leg_duration(length, speed=ROUTE_SPEED, accel=ROUTE_ACCEL):
    """Time for a rest-to-rest leg, triangular when it cannot reach speed."""
    peak = min(speed, math.sqrt(length * accel))
    return length / peak + peak / accel if length else 0.0


def route_duration(points):
    """Duration with a full stop at every waypoint."""
    return sum(leg_duration(math.dist(a, b)) for a, b in zip(points, points[1:]))


def sample_route(points, seconds):
    """Return ((x, y), (vx, vy), finished) with bounded speed and acceleration."""
    seconds = max(seconds, 0.0)
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        duration = leg_duration(length)
        if seconds > duration:
            seconds -= duration
            continue
        peak = min(ROUTE_SPEED, math.sqrt(length * ROUTE_ACCEL))
        ramp = peak / ROUTE_ACCEL
        if seconds < ramp:
            distance, velocity = ROUTE_ACCEL * seconds**2 / 2, ROUTE_ACCEL * seconds
        elif seconds <= duration - ramp:
            distance = peak * (seconds - ramp / 2)
            velocity = peak
        else:
            remaining = duration - seconds
            distance = length - ROUTE_ACCEL * remaining**2 / 2
            velocity = ROUTE_ACCEL * remaining
        direction = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
        return (tuple(a[i] + direction[i] * distance for i in range(2)),
                tuple(direction[i] * velocity for i in range(2)), False)
    return points[-1], (0.0, 0.0), True
