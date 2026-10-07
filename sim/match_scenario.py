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


def route_duration(points, speed=ROUTE_SPEED):
    """Duration with a full stop at every waypoint."""
    return sum(leg_duration(math.dist(a, b), speed) for a, b in zip(points, points[1:]))


def sample_route(points, seconds, speed=ROUTE_SPEED):
    """Return ((x, y), (vx, vy), finished) with bounded speed and acceleration."""
    seconds = max(seconds, 0.0)
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        duration = leg_duration(length, speed)
        if seconds > duration:
            seconds -= duration
            continue
        peak = min(speed, math.sqrt(length * ROUTE_ACCEL))
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


CENTER = ROUTES['sentry'][-1]
CENTER_LEGS = [
    ('parked', [CENTER], 1.0, 5.0),
    ('straight_1', [CENTER, (1.55, -0.05), CENTER], 1.0, None),
    ('turn', [CENTER, (0.65, -0.65), (0.65, -0.05), (1.55, -0.05), CENTER], 1.0, None),
    ('spin', [CENTER], 1.0, 6.0),
]


def match_duration(stage='e3'):
    """Return the duration of team spawn approaches and center maneuvers."""
    return ingress_duration(stage) + sum(
        duration if duration is not None else route_duration(points, speed)
        for _, points, speed, duration in CENTER_LEGS)


def sample_match(seconds, stage='e3'):
    """Return position, velocity, chassis spin and segment for our scripted route."""
    approach = ingress_duration(stage)
    if seconds < approach:
        p, v, _ = sample_route(ROUTES['sentry'], seconds)
        return p, v, 0.0, 'approach_2'
    seconds -= approach
    for name, points, speed, duration in CENTER_LEGS:
        duration = duration if duration is not None else route_duration(points, speed)
        if seconds < duration:
            p, v, _ = sample_route(points, seconds, speed)
            return p, v, 9.0 if name == 'spin' else 0.0, name
        seconds -= duration
    return CENTER, (0.0, 0.0), 0.0, 'finished'


ROUTE_DELAYS = {'sentry': 0.0, 'opponent_0': 0.0, 'ally_0': 4.0, 'opponent_1': 4.0}
TEAMS = {'sentry': 'blue', 'ally_0': 'blue', 'opponent_0': 'red', 'opponent_1': 'red'}


def ingress_duration(stage='e3'):
    """Wait for both lanes to arrive before the center fight in E4."""
    return (max(route_duration(route) + ROUTE_DELAYS[name] for name, route in ROUTES.items())
            if stage == 'e4' else route_duration(ROUTES['sentry']))


def sample_robot(name, seconds, stage='e4'):
    """Return reference pose/twist for a ghost; second lanes leave four seconds later."""
    delay = ROUTE_DELAYS[name] if stage == 'e4' else 0.0
    p, v, finished = sample_route(ROUTES[name], max(0, seconds - delay))
    yaw = math.pi if TEAMS[name] == 'blue' else 0.0
    spin = 0.0
    fight = max(0.0, seconds - ingress_duration(stage))
    if finished and fight > 0:
        omega, accel = 3.0 * math.pi, 20.0
        ramp = omega / accel
        rotation = (accel * fight**2 / 2 if fight < ramp else omega * (fight - ramp / 2))
        yaw += rotation
        spin = min(accel * fight, omega)
        if TEAMS[name] == 'red':
            amplitude, frequency = (0.3, 2.0) if name == 'opponent_0' else (0.2, 1.5)
            p = (p[0], p[1] + amplitude * (1 - math.cos(frequency * fight)))
            v = (0.0, amplitude * frequency * math.sin(frequency * fight))
    return p, v, yaw, spin
