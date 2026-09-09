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
cv_head_aim_core.py -- pure head-IK math for cv_head_aim.py, no rclpy import.

Unit-tested standalone in test/cv/test_cv_head_aim.py. See README.md's
### cv_head_aim.py Notes for the derivation.
"""
import math

# headpitch joint origin's rpy z component (sentry.urdf.xacro:137-139) -- an
# empirically-tuned mesh-alignment YAW on the joint origin, not a pitch
# bias. See the plan's Phase 2 "Confirm before composing the rotation."
HEADPITCH_ORIGIN_YAW = -0.38885
# Where the shot actually leaves from, in root: the headpitch joint origin
# (== camera, cameralink is identity). root -> headlink is Rz(pi) about a
# zero translation then (0, 0, HEADLINK_ORIGIN_Z), so the muzzle sits
# MUZZLE_Z up and MUZZLE_RADIUS out from the yaw axis. All three pinned
# against the xacro in test/cv/test_urdf_constants.py.
HEADLINK_ORIGIN_Z = 0.252215
HEADPITCH_ORIGIN_X = 0.1
HEADPITCH_ORIGIN_Z = 0.1218
MUZZLE_RADIUS = HEADPITCH_ORIGIN_X          # offset from the yaw axis
MUZZLE_Z = HEADLINK_ORIGIN_Z + HEADPITCH_ORIGIN_Z   # 0.374 m above root
# The muzzle's offset from the yaw axis is carried round by the yaw, so
# only this component of it -- 0.038 m, fixed -- is ever perpendicular to
# the shot. See README.md's ### cv_head_aim.py Notes.
MUZZLE_PERP = MUZZLE_RADIUS * math.sin(HEADPITCH_ORIGIN_YAW)


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def muzzle_offset_root(phi):
    """
    Root-frame position of the muzzle (headpitch joint origin) at camera azimuth phi.

    Depends on the yaw only: the pitch axis passes through this point, so
    pitching does not move it. theta_y = HEADPITCH_ORIGIN_YAW - phi, and
    the head's x offset is carried round by Rz(-theta_y).
    """
    a = phi - HEADPITCH_ORIGIN_YAW
    return (MUZZLE_RADIUS * math.cos(a), MUZZLE_RADIUS * math.sin(a), MUZZLE_Z)


def solve_head_angles(target_root):
    """
    Solve absolute (yaw, pitch) aiming the camera/muzzle at the root-frame POINT target_root.

    The aim axis is the camera's local +X and the shot leaves the muzzle,
    MUZZLE_Z (0.374 m) above the root origin -- so this solves the
    parallax, not just the bearing from root. Aiming from root instead
    missed a stationary panel by a constant ~0.33 m at any range
    (CV_TEST_GAPS.md gap 7). Closed form, no iteration: the muzzle rides
    round with the yaw, leaving MUZZLE_PERP as the only perpendicular
    offset, so the azimuth is just bearing + asin(MUZZLE_PERP / range).
    See README.md's ### cv_head_aim.py Notes for the FK derivation.
    """
    x, y, z = target_root
    horiz = math.hypot(x, y)
    bearing = math.atan2(y, x) if horiz > 0.0 else 0.0
    # Inside MUZZLE_PERP of the yaw axis there is no azimuth that points
    # the muzzle at the target at all; clamp rather than raise, the head
    # cannot usefully aim at its own turret either way.
    ratio = MUZZLE_PERP / horiz if horiz > abs(MUZZLE_PERP) else math.copysign(
        1.0, MUZZLE_PERP)
    phi = bearing + math.asin(ratio)

    mx, my, mz = muzzle_offset_root(phi)
    dx, dy, dz = x - mx, y - my, z - mz
    r = math.hypot(dx, dy)
    theta_p = math.atan2(-dz, r) if (r > 0.0 or dz != 0.0) else 0.0
    theta_y = HEADPITCH_ORIGIN_YAW - phi
    return theta_y, theta_p
