"""
Unit test for cv_head_aim_core.py's closed-form head IK, cross-checked
against an independent from-scratch forward-kinematics implementation of
the same sentry.urdf.xacro chain (root -> body -> headlink -> headpitch ->
camera) -- not a copy of cv_target_emulator.py's `_camera_pose`, a
from-scratch re-derivation, so this actually catches a sign/algebra error
in either one rather than just checking self-consistency. No rclpy, no
ROS message packages: run with `python3 -m pytest test/cv/test_cv_head_aim.py`.
"""
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from sim.cv_head_aim_core import HEADPITCH_ORIGIN_YAW, solve_head_angles  # noqa: E402


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _camera_forward(theta_y, theta_p):
    """Independent FK: root->body (Rz(pi)) -> headlink origin (Rz(pi)) *
    rotate(-z axis, theta_y) -> headpitch origin (Rz(HEADPITCH_ORIGIN_YAW))
    * rotate(y axis, theta_p) -> camera (identity). rotate about -z by
    theta_y equals Rz(-theta_y)."""
    R = (_rz(math.pi) @ _rz(math.pi) @ _rz(-theta_y)
         @ _rz(HEADPITCH_ORIGIN_YAW) @ _ry(theta_p))
    return R @ np.array([1.0, 0.0, 0.0])


def test_zero_angles_forward_direction_round_trips():
    fwd = _camera_forward(0.0, 0.0)
    theta_y, theta_p = solve_head_angles(tuple(fwd))
    assert math.isclose(theta_y, 0.0, abs_tol=1e-9)
    assert math.isclose(theta_p, 0.0, abs_tol=1e-9)


def test_random_angles_round_trip():
    rng = random.Random(0)
    for _ in range(200):
        theta_y = rng.uniform(-3.0, 3.0)
        theta_p = rng.uniform(-0.6, 0.6)
        fwd = _camera_forward(theta_y, theta_p)
        est_y, est_p = solve_head_angles(tuple(fwd))
        yaw_err = math.atan2(math.sin(est_y - theta_y), math.cos(est_y - theta_y))
        assert abs(yaw_err) < 1e-9
        assert math.isclose(est_p, theta_p, abs_tol=1e-9)


def test_target_straight_ahead_in_root_frame():
    # A target directly along root +x (straight ahead, root frame): with
    # HEADPITCH_ORIGIN_YAW baked into the mesh alignment, headlink must
    # yaw by -HEADPITCH_ORIGIN_YAW to compensate and point the camera
    # there, not zero.
    theta_y, theta_p = solve_head_angles((5.0, 0.0, 0.0))
    assert math.isclose(theta_y, HEADPITCH_ORIGIN_YAW, abs_tol=1e-9)
    assert math.isclose(theta_p, 0.0, abs_tol=1e-9)
