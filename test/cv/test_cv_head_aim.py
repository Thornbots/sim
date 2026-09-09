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
Unit test for cv_head_aim_core.py's closed-form head IK.

It is cross-checked against an independent from-scratch FK
implementation of the same chain, position included: the solve aims the
ray that leaves the MUZZLE, so a test that only checks bearings from the
root origin would pass the pre-2026-09-09 parallax bug (CV_TEST_GAPS.md
gap 7). The chain is sentry.urdf.xacro's
(root -> body -> headlink -> headpitch -> camera) -- not a copy of
cv_target_emulator.py's `_camera_pose` but a from-scratch re-derivation,
so this actually catches a sign/algebra error in either one rather than
just checking self-consistency. No rclpy, no
ROS message packages: run it with
`python3 -m pytest test/cv/test_cv_head_aim.py`.
"""
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from sim.cv_head_aim_core import (  # noqa: E402
    HEADPITCH_ORIGIN_YAW, MUZZLE_RADIUS, MUZZLE_Z, solve_head_angles,
)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _camera_pose(theta_y, theta_p):
    """
    Compute the muzzle position and camera forward direction, from-scratch FK.

    root->body (Rz(pi)) -> headlink origin (0,0,0.252215), Rz(pi) *
    rotate(-z axis, theta_y) -> headpitch origin (0.1,0,0.1218),
    Rz(HEADPITCH_ORIGIN_YAW) * rotate(y axis, theta_p) -> camera
    (identity). rotate about -z by theta_y equals Rz(-theta_y). The
    headpitch joint origin is the muzzle: pitch rotates about it, so
    theta_p does not move it.
    """
    r_body = _rz(math.pi)
    r_head = r_body @ _rz(math.pi) @ _rz(-theta_y)
    p_head = r_body @ np.array([0.0, 0.0, 0.252215])
    p_muzzle = p_head + r_head @ np.array([0.1, 0.0, 0.1218])
    r_cam = r_head @ _rz(HEADPITCH_ORIGIN_YAW) @ _ry(theta_p)
    return p_muzzle, r_cam @ np.array([1.0, 0.0, 0.0])


def test_muzzle_constants_match_the_from_scratch_fk():
    for theta_y in (0.0, 0.7, -2.5):
        pos, _ = _camera_pose(theta_y, 0.0)
        assert math.isclose(pos[2], MUZZLE_Z, abs_tol=1e-12)
        assert math.isclose(math.hypot(pos[0], pos[1]), MUZZLE_RADIUS, abs_tol=1e-12)


def test_random_angles_round_trip():
    """A point on the ray out of the muzzle must solve back to the angles that made it."""
    rng = random.Random(0)
    for _ in range(200):
        theta_y = rng.uniform(-3.0, 3.0)
        theta_p = rng.uniform(-0.6, 0.6)
        pos, fwd = _camera_pose(theta_y, theta_p)
        target = pos + rng.uniform(0.5, 12.0) * fwd
        est_y, est_p = solve_head_angles(tuple(target))
        yaw_err = math.atan2(math.sin(est_y - theta_y), math.cos(est_y - theta_y))
        assert abs(yaw_err) < 1e-9
        assert math.isclose(est_p, theta_p, abs_tol=1e-9)


def test_solved_ray_passes_through_the_target_point():
    """The end-to-end condition the shot-hit bench scores: miss distance, not bearing."""
    for target in [(3.0, 0.0, 0.3), (2.7, 0.0, 0.3), (1.0, -1.0, 0.0),
                   (-4.0, 2.0, 1.2), (0.5, 0.0, 0.3)]:
        theta_y, theta_p = solve_head_angles(target)
        pos, fwd = _camera_pose(theta_y, theta_p)
        rel = np.array(target) - pos
        miss = np.linalg.norm(rel - np.dot(rel, fwd) * fwd)
        assert miss < 1e-9, f'{target}: ray misses by {miss:.4f} m'


def test_target_level_with_root_is_aimed_downward():
    # The muzzle sits MUZZLE_Z above root, so a target at root height is
    # BELOW the muzzle and the pitch must be positive (nose-down): the
    # sign that the parallax is being solved at all, not just the bearing.
    _theta_y, theta_p = solve_head_angles((5.0, 0.0, 0.0))
    assert theta_p > 0.0
    assert math.isclose(theta_p, math.atan2(MUZZLE_Z, 5.0), abs_tol=2e-3)


def test_yaw_ignores_the_lateral_offset_only_when_it_cannot_matter():
    # Straight up the root +x axis the muzzle's own lateral offset still
    # shifts the required azimuth off HEADPITCH_ORIGIN_YAW, by ~atan(the
    # lateral part / range) -- small, real, and the thing that used to be
    # dropped.
    theta_y, _ = solve_head_angles((5.0, 0.0, 0.0))
    assert not math.isclose(theta_y, HEADPITCH_ORIGIN_YAW, abs_tol=1e-4)
    assert math.isclose(theta_y, HEADPITCH_ORIGIN_YAW, abs_tol=0.02)
