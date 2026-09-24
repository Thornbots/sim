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

Unit-tested standalone in test/cv/test_cv_head_aim.py. see README.md for
design rationale
"""
import math

# sentry_v2's head chain in root, pinned to thornbots_pkg's URDF by
# test/cv/test_urdf_constants.py. root -> headlink origin (no rotation,
# yaw about -z, so azimuth = -theta_y) -> headpitch origin (pitch about +y)
# -> muzzlelink. The muzzle sits on the pitch axis, so pitching never moves
# it; its only offset off the shot line is MUZZLE_Y, sideways.
HEADLINK_ORIGIN_X = -0.000171242
HEADLINK_ORIGIN_Y = 9.52126e-05
HEADLINK_ORIGIN_Z = 0.248293
HEADPITCH_ORIGIN = (-0.00760542, -0.100122, 0.14235)
MUZZLELINK_ORIGIN = (0.0, 0.1128, 0.0)
MUZZLE_X = HEADPITCH_ORIGIN[0] + MUZZLELINK_ORIGIN[0]   # along the shot, in the head frame
MUZZLE_Y = HEADPITCH_ORIGIN[1] + MUZZLELINK_ORIGIN[1]   # 0.013 m left of the yaw axis
MUZZLE_Z = HEADLINK_ORIGIN_Z + HEADPITCH_ORIGIN[2] + MUZZLELINK_ORIGIN[2]  # 0.391 m


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def muzzle_offset_root(phi):
    """Root-frame muzzle position at head azimuth phi; pitch does not move it."""
    c, s = math.cos(phi), math.sin(phi)
    return (HEADLINK_ORIGIN_X + MUZZLE_X * c - MUZZLE_Y * s,
            HEADLINK_ORIGIN_Y + MUZZLE_X * s + MUZZLE_Y * c, MUZZLE_Z)


def solve_head_angles(target_root):
    """
    Solve absolute (yaw, pitch) putting the muzzle's +X ray through root-frame point target_root.

    Solves the parallax from the muzzle, not a bearing from root: aiming
    from root missed a stationary panel by a constant ~0.33 m on the old
    model (CV_TEST_GAPS.md gap 7). Closed form: the ray's lateral offset
    from the yaw axis is MUZZLE_Y whatever the yaw, so the azimuth is the
    bearing minus asin(MUZZLE_Y / range). see README.md for design rationale
    """
    x = target_root[0] - HEADLINK_ORIGIN_X
    y = target_root[1] - HEADLINK_ORIGIN_Y
    horiz = math.hypot(x, y)
    bearing = math.atan2(y, x) if horiz > 0.0 else 0.0
    # Inside MUZZLE_Y of the yaw axis no azimuth reaches the target; clamp.
    ratio = MUZZLE_Y / horiz if horiz > abs(MUZZLE_Y) else math.copysign(1.0, MUZZLE_Y)
    phi = bearing - math.asin(ratio)

    mx, my, mz = muzzle_offset_root(phi)
    dx, dy, dz = target_root[0] - mx, target_root[1] - my, target_root[2] - mz
    r = dx * math.cos(phi) + dy * math.sin(phi)
    theta_p = math.atan2(-dz, r) if (r != 0.0 or dz != 0.0) else 0.0
    return -phi, theta_p
