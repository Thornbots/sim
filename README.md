# sim

ROS 2 package that launches `gz sim` (Ignition/Gazebo Sim) with the
`ARCC_Field_2026` world and spawns the `sentry` robot (defined in
`sentry_urdf.xacro`) into it.

## Package layout

```
sim/
├── launch/sim.launch.py       # main launch file
├── urdf/sentry_urdf.xacro     # robot description (uses package://sim/meshes/*.stl)
├── worlds/ARCC_Field_2026.sdf # world file (uses model://sim/world/*.stl)
├── meshes/                    # <-- put the ROBOT meshes here: Body.stl, Head.stl,
│                               #     Lidar.stl, OdoWheel.stl
└── world/                     # <-- put the WORLD mesh here: composite_part_1.stl
```

**You must drop the mesh files in before building** — they weren't part of
the uploaded files, only referenced by path:

- `meshes/Body.stl`, `meshes/Head.stl`, `meshes/Lidar.stl`, `meshes/OdoWheel.stl`
  (referenced by the xacro as `package://sim/meshes/...`)
- `world/composite_part_1.stl`
  (referenced by the world file as `model://sim/world/...`)

The package name is `sim` deliberately, to match both of these
existing `package://` / `model://` references without editing the source files.

## Build

```bash
cd ~/ros2_ws          # your colcon workspace, with sim under src/
colcon build --packages-select sim
source install/setup.bash
```

## Run

```bash
ros2 launch sim sim.launch.py
```

Useful arguments:

```bash
ros2 launch sim sim.launch.py gui:=false          # headless (server only)
ros2 launch sim sim.launch.py x:=1.0 y:=0.5 z:=0.1
ros2 launch sim sim.launch.py world:=/abs/path/to/other.sdf
```

## What the launch file does

1. Sets `GZ_SIM_RESOURCE_PATH` (and `IGN_GAZEBO_RESOURCE_PATH` for older
   Ignition releases) to the installed `share/` directory so that
   `model://sim/world/composite_part_1.stl` resolves correctly.
2. Includes `ros_gz_sim`'s `gz_sim.launch.py` to start the simulator with
   `worlds/ARCC_Field_2026.sdf` loaded and running (`-r`).
3. Runs `robot_state_publisher`, feeding it the URDF produced by expanding
   `sentry_urdf.xacro` (this is also what lets rviz/tf resolve
   `package://sim/meshes/...` for the robot's own visuals).
4. Spawns the robot into the running world via `ros_gz_sim create`, reading
   the description straight from the `/robot_description` topic.
5. Bridges `/clock` from gz sim to ROS so every node using
   `use_sim_time` stays in sync with the simulation clock.

## Assumptions / notes

- Assumes a ROS 2 distro with `ros_gz_sim` + `ros_gz_bridge` (e.g. Humble,
  Iron, Jazzy) installed and providing `gz sim` (or `ign gazebo` on older
  Fortress-era installs — the launch file sets resource-path env vars for
  both naming schemes, but if your system only has the older
  `ros_ign_gazebo` package instead of `ros_gz_sim`, swap the
  `get_package_share_directory('ros_gz_sim')` calls in `launch/sim.launch.py`
  for `'ros_ign_gazebo'` and the launch filename to `ign_gazebo.launch.py`).
- The world's `<physics type="ode">` and plugin filenames (`gz-sim-*`) and
  the xacro's gazebo plugins (`ignition-gazebo-*`) mix Fortress/Harmonic
  naming — both are typically aliased in current `gz-sim` builds, but if a
  plugin fails to load, check your `gz-sim` version's plugin name for
  odometry/joint-state publishing.
- No `<static>` tag on the robot model is set by this launch file — the
  robot is spawned as a normal dynamic model since the xacro doesn't mark
  it static.

## Testing

`test/localization/run_localization_drift_tests.py` is a standalone
integration suite (not part of `colcon test`) that launches `sim` +
`sentry_pkg` (which includes `sentry_localization`) end to end and
exercises localization drift/jerk-correction behavior against this
package's synthetic odometry noise model (`sim/pose_emulator.py`). Run
after tuning `sentry_localization/config/slam.yaml`,
`config/amcl.yaml`, `config/ekf.yaml`, or `pose_emulator.py`'s noise
model:

```bash
isaac_ros_common/scripts/dexec.sh -- python3 \
  /workspaces/isaac_ros-dev/src/sim/test/localization/run_localization_drift_tests.py \
  --backend slam   # or amcl, ekf
```

**After editing a `sentry_localization/config/*.yaml` file, rebuild
before rerunning the suite** — that package's `data_files`
(config/launch/map) are copied at build time, not live-read from
`src/`, so an edited YAML silently has no effect on the running
container until a rebuild resyncs `install/`:

```bash
isaac_ros_common/scripts/dexec.sh -- colcon build --symlink-install \
  --packages-select sentry_localization sentry_pkg
```

`--symlink-install` makes `install/` a symlink chain back to `src/` (via
`build/`) for both this rebuild and every future one, so subsequent config
edits take effect immediately with no rebuild needed — only the *first*
build (or any build that didn't use `--symlink-install`) leaves a stale
plain-copy trap. If a run's results look implausibly unaffected by a
config change you just made, check `diff install/sentry_localization/
share/sentry_localization/config/amcl.yaml src/sentry_localization/
config/amcl.yaml` before assuming the change itself didn't work.

Scenarios (`--scenario NAME` to run just one; all four run by default, in
this order): `baseline`, `drift_correction`, `drift_correction_obstacle`,
`jerk_with_motion`. Each scenario's exact pass condition and rationale is
documented in the script's own module docstring (`SCENARIOS` section) —
read that before interpreting a failure. `drift_correction` shares its
driving loop and threshold with `drift_correction_obstacle` on purpose —
compare the two directly before attributing a failure on either to the
obstacle specifically.

Other useful flags: `--keep-running` (skip teardown for interactive
follow-up), `--headless` (no gz-sim GUI — GUI is the default per the
project's standing "watch sim live" rule, see `SESSION_NOTES.md`). Full
usage/rationale in the script's own docstring.
