# Sentry head section, 2026-10-25

Stage 2 opponent visibility uses the existing rendered opponent geometry.
Only render flags change; our own model stays excluded from lidar, opponents
and allies become visible, and sensor settings and scoring gates stay unchanged.

Measured the checked-in `sentry_v2` visual STL triangles for `root`, `head`
and `head_pitch` at neutral joint angles, transformed through their URDF
origins into root coordinates. Intersected triangles with each horizontal
plane, clipped resulting segments to a 0.13 m radius around the head yaw
axis, then projected segment endpoints onto 180 horizontal axes at 1 degree
spacing. Width is maximum minus minimum projection; the radius excludes the
outboard lidar arm. This measures the central silhouette envelope, not return
count or continuity.

- z=0.275 m: min/median/max 0.1433 / 0.2352 / 0.2470 m.
- z=0.325 m: min/median/max 0.1748 / 0.2366 / 0.2490 m.
- z=0.355 m: min/median/max 0.1517 / 0.2369 / 0.2521 m.
- z=0.375 m: min/median/max 0.1491 / 0.2354 / 0.2502 m.
- z=0.425 m: min/median/max 0.1529 / 0.1795 / 0.2315 m.

Thus approximately 0.15–0.25 m, typically 0.23 m, describes the central head
near z=0.355 m. The top slice is narrower; width is not constant throughout
the entire 15 cm band. No tracker gates were changed.

Reproduce the section calculation and expanded-model render-mask checks with:

```bash
python3 -m pytest -q -s test/test_lidar_visibility.py
```

The tests require pytest and xacro and launch no ROS or gz processes. The
package-path substitution lets the real xacro wrapper expand on a host without
a sourced ROS installation. Registered colcon tests include these checks.

The opponent driver retains its existing faster collision-hull visuals and
75% root-hull x/y scale. The measurement above uses the CAD visual meshes;
full rendered opponents can include off-axis structures and hull inflation.
The nominal 0.355 m plane is also not an exact mount calibration: the current
URDF puts the lidar at root z=0.3450931 m before chassis settling, and the
opponent mover places root at world z=-0.0115 m. Integration validation must
check actual `/scan` returns; expanded masks alone do not prove Gazebo's
URDF-to-SDF conversion and renderer behaviour.
