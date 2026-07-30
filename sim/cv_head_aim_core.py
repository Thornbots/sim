"""
cv_head_aim_core.py -- pure head-IK math for cv_head_aim.py (no rclpy
import), unit-tested standalone in test/test_cv_head_aim.py. See
README.md's ### cv_head_aim.py Notes for the derivation.
"""
import math

# headpitch joint origin's rpy z component (sentry.urdf.xacro:137-139) -- an
# empirically-tuned mesh-alignment YAW on the joint origin, not a pitch
# bias. See the plan's Phase 2 "Confirm before composing the rotation."
HEADPITCH_ORIGIN_YAW = -0.38885


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def solve_head_angles(target_root):
    """Absolute (yaw, pitch) for headlink/headpitch that points the camera's
    local +X axis at target_root (a root-frame point), ignoring the camera's
    small (~0.35m) position offset from the yaw/pitch axes -- same
    simplification the plan applies to Type-C's muzzle offset ("a few cm
    against metres of range doesn't move t"). Derived from the fixed FK
    chain in sentry.urdf.xacro (root -[pi yaw]-> body -[pi yaw]-> headlink(
    theta_y, axis -z) -> head -[HEADPITCH_ORIGIN_YAW yaw]-> headpitch(
    theta_p, axis y) -> head_pitch == camera): the two fixed pi yaws cancel,
    leaving camera_rotation = Rz(HEADPITCH_ORIGIN_YAW - theta_y) @
    Ry(theta_p), so pointing camera +X at unit direction d=(dx,dy,dz)
    solves in closed form here (numerically verified against
    cv_target_emulator's independent from-scratch FK, see
    test/test_cv_head_aim.py)."""
    x, y, z = target_root
    r = math.hypot(x, y)
    theta_p = math.atan2(-z, r) if (r > 0.0 or z != 0.0) else 0.0
    phi = math.atan2(y, x) if (x != 0.0 or y != 0.0) else 0.0
    theta_y = HEADPITCH_ORIGIN_YAW - phi
    return theta_y, theta_p
