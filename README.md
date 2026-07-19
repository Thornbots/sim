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
