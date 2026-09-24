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
Unit test for cv_head_aim_core.py's closed-form head IK (sentry_v2 chain).

Checked against a from-scratch FK of root -> headlink -> headpitch ->
muzzlelink written with the URDF's literal numbers, not cv_head_aim_core's
constants, so a sign or algebra error in either one shows up. It scores
the ray leaving the MUZZLE, not bearings from root (CV_TEST_GAPS.md gap 7).
No rclpy: `python3 -m pytest test/cv/test_cv_head_aim.py`.
"""
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from sim.cv_head_aim_core import (  # noqa: E402
    MUZZLE_Y, MUZZLE_Z, solve_head_angles,
)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _muzzle_pose(theta_y, theta_p):
    """
    Return the root-frame muzzle position and shot direction, from-scratch FK.

    fastened_2 is identity. headlink: origin (-0.000171242, 9.52126e-05,
    0.248293), axis -z, so Rz(-theta_y). headpitch: origin (-0.00760542,
    -0.100122, 0.14235), axis +y, so Ry(theta_p). muzzlelink: (0, 0.1128, 0).
    The shot leaves along head_pitch +x.
    """
    p_head = np.array([-0.000171242, 9.52126e-05, 0.248293])
    r_head = _rz(-theta_y)
    p_pitch = p_head + r_head @ np.array([-0.00760542, -0.100122, 0.14235])
    r_pitch = r_head @ _ry(theta_p)
    p_muzzle = p_pitch + r_pitch @ np.array([0.0, 0.1128, 0.0])
    return p_muzzle, r_pitch @ np.array([1.0, 0.0, 0.0])


def test_muzzle_constants_match_the_from_scratch_fk():
    for theta_y in (0.0, 0.7, -2.5):
        for theta_p in (0.0, 0.5):
            pos, _ = _muzzle_pose(theta_y, theta_p)
            assert math.isclose(pos[2], MUZZLE_Z, abs_tol=1e-12)
    pos, _ = _muzzle_pose(0.0, 0.0)
    assert math.isclose(pos[1] - 9.52126e-05, MUZZLE_Y, abs_tol=1e-12)


def test_random_angles_round_trip():
    """A point on the ray out of the muzzle must solve back to the angles that made it."""
    rng = random.Random(0)
    for _ in range(200):
        theta_y = rng.uniform(-3.0, 3.0)
        theta_p = rng.uniform(-0.6, 0.6)
        pos, fwd = _muzzle_pose(theta_y, theta_p)
        target = pos + rng.uniform(0.5, 12.0) * fwd
        est_y, est_p = solve_head_angles(tuple(target))
        yaw_err = math.atan2(math.sin(est_y - theta_y), math.cos(est_y - theta_y))
        assert abs(yaw_err) < 1e-9
        assert math.isclose(est_p, theta_p, abs_tol=1e-9)


def test_solved_ray_passes_through_the_target_point():
    """The end-to-end condition the shot-hit bench scores: miss distance, not bearing."""
    for target in [(3.0, 0.0, 0.3), (2.7, 0.0, 0.3), (1.0, -1.0, 0.0),
                   (-4.0, 2.0, 1.2), (0.5, 0.0, 0.3), (0.0, -3.0, 0.1)]:
        theta_y, theta_p = solve_head_angles(target)
        pos, fwd = _muzzle_pose(theta_y, theta_p)
        rel = np.array(target) - pos
        assert np.dot(rel, fwd) > 0.0, f'{target}: target is behind the muzzle'
        miss = np.linalg.norm(rel - np.dot(rel, fwd) * fwd)
        assert miss < 1e-9, f'{target}: ray misses by {miss:.4f} m'


def test_target_level_with_root_is_aimed_downward():
    # The muzzle sits MUZZLE_Z (0.391 m) above root, so a target at root
    # height is below it and the pitch must be positive (nose-down).
    _theta_y, theta_p = solve_head_angles((5.0, 0.0, 0.0))
    assert theta_p > 0.0
    assert math.isclose(theta_p, math.atan2(MUZZLE_Z, 5.0), abs_tol=2e-3)


def test_lateral_muzzle_offset_shifts_the_azimuth():
    # Straight up root +x, the muzzle's MUZZLE_Y (0.013 m) sideways offset
    # moves the azimuth phi = -theta_y off zero by about asin(MUZZLE_Y / range).
    theta_y, _ = solve_head_angles((5.0, 0.0, 0.0))
    phi = -theta_y
    assert not math.isclose(phi, 0.0, abs_tol=1e-3)
    assert math.isclose(phi, -math.asin(MUZZLE_Y / 5.0), abs_tol=1e-4)
