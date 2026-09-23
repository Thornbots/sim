# sim

sim holds the robot's integration tests. The localization drift suite and the
CV shot-hit bench each start a `gz sim` model of the `ARCC_Field_2026` field,
spawn the `sentry` robot from `urdf/sentry.urdf.xacro`, and run the real
`thornbots_pkg` stack against it.

## Run the tests

Build first on a fresh container (see Build). Each test starts its own sim and
`thornbots_pkg` stacks, so stop anything you already have running.

The localization suite runs `amcl` with the EKF, the configuration the robot
is targeting. All six drift scenarios take about three minutes:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim localization_tests.launch.py
```

`suite:=ekf` runs the EKF ground-truth test instead.

The CV bench runs ten shot-hit cases, all with lead on: a stationary target,
then 0.5, 1, 2 and 4 m/s, first with flat panels and then with neighbouring
panels staggered 90% of a panel's height apart (`--panel-layout` picks one).
It launches the stack once and only changes the target between cases; each
case settles for 3s, then scores 30s of sim time. The bench fires at up to
40 Hz, far above the real launcher, and scores each case on hit rate and hits
per expected shot equally (`score()` in `test/cv/shot_hit_harness.py`), so
falling behind 40 Hz costs points.

Every scored shot goes to `shots.jsonl` in `--log-dir`, one JSON object per
line: the case, whether it hit, the miss distance split into the panel's
offset right of, above and ahead of the shot (ahead is along the target's
travel, so positive means the shot trailed), the panel and incidence angle,
the target's velocity, the target rotation it arrived in, and the `CVTarget`
fire fields. Each case also prints the mean of those offsets over its misses.
`panel_hits.jsonl` holds one line per case with the hits on each panel (front,
left, back, right) in each target rotation. It is for reading, not scoring.

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim shot_hit.launch.py
```

The bench is one launch tree: the sim, the CV pipeline and the pytest that
scores it. The localization launch runs pytest, and pytest starts the sim once
(`run_tests:=false part:=sim`) and a fresh robot stack for each scenario
(`part:=robot`). Between scenarios it stops the robot stack, teleports the
robot back to spawn, removes anything a scenario spawned, and resets
`pose_emulator`'s noise state and parameters. `restart_sim:=true` brings the
sim up fresh for every scenario instead, the old behaviour and the control when
a verdict looks off. Both
shut down when the tests finish, and Ctrl-C stops everything, stacks included.

Both launches default to `real_time_factor:=0`, which lets gz run as fast as
the machine allows. Every test times itself in sim seconds, so a faster sim
shortens the wall-clock run without shortening what gets scored. Pass
`real_time_factor:=1` to run in real time. On the dev laptop the GUI runs
2-3x real time, and headless runs 8-10x.

Add an argument to run part of a suite. `scenario:=odom_stuck` runs one drift
scenario and `backend:=slam` or `use_ekf:=false` changes the stack. For the
bench, `only_stationary:=true` runs the stationary case, `speeds:='0.5 1'`
picks the moving cases. `headless:=true` drops the gz GUI and rviz from
either. `--show-args` on either launch lists the rest.

`sim_engine:=sapien` runs either suite on SAPIEN instead of gz (see the
`sapien_sim.py` note). It is new and has not yet run a full suite, so treat its
numbers as unverified until they have been compared with gz:

```bash
ros2 launch sim localization_tests.launch.py sim_engine:=sapien
ros2 launch sim shot_hit.launch.py sim_engine:=sapien
```

## Build

`Dockerfile.thornbots` installs neither `ros-humble-ros-gz` nor this package.
On a fresh container, run `install-sim.sh` once from a container terminal. It
installs the dependencies for both engines (gz from apt, SAPIEN from pip) and
builds `sim`:

```bash
cd /workspaces/isaac_ros-dev
sudo src/isaac_ros_common/docker/scripts/install-sim.sh
```

Rebuild after you edit the package:

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select sim
```

Source the workspace in every new terminal. `sim` has no copy baked into
`/workspaces/ros2_ws`, so without this `ros2` can't find it:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

Keep `--symlink-install`. Without it `install/sim` holds copies, and your edits
under `sim/` do nothing until the next rebuild. If a change seems to have no
effect, check this before anything else, then `rm -rf build/sim install/sim`
and rebuild.

## More on the tests

Everything under `test/` is pytest, and `colcon test` collects it.

| Tier | Files | Needs |
| --- | --- | --- |
| unit | `cv/test_cv_head_aim.py`, `cv/test_urdf_constants.py`, ament copyright/flake8/pep257 | Python + pytest |
| integration | `localization/test_localization_drift.py`, `localization/test_ekf_ground_truth.py`, `cv/test_shot_hit.py` | gz-sim and a launch tree |

`setup.cfg` deselects the `integration` marker, so a plain `colcon test` runs
only the unit tests and finishes in seconds. A `-m` on the command line
overrides that:

```bash
colcon test --packages-select sim
colcon test --packages-select sim --pytest-args ' -m integration'
colcon test-result --verbose
```

Each drift test launches gz-sim and `thornbots_pkg` as one tree, runs for
tens of seconds, and shuts it down before the next test starts. The shot-hit suite launches
them once for all its cases, through `shot_hit.launch.py run_tests:=false`
when pytest starts it. ROS topics are shared
across every process on the machine, so a stack you left running will corrupt
the measurements.

Rerun the drift suite whenever you tune `slam.yaml`, `amcl.yaml`, `ekf.yaml` or
the noise model. `pytest_args:=` passes extra arguments such as `-k` through to
pytest.

`headless:=true` turns off the gz GUI and rviz2, which are on by default.
`speed:=` changes the 4.0 m/s loop speed, but nobody has re-validated the
thresholds at other speeds.
`sentry_localization` copies its config, launch and map files into `install/`
at build time. After you edit its YAML, rebuild with `--symlink-install` so
later edits go straight through:

```bash
colcon build --symlink-install --packages-select sentry_localization thornbots_pkg
```

If a result looks unaffected by your change, `diff` the installed YAML against
the source copy.

Before you interpret a drift failure, read the notes below rather than the
script docstrings. The shot-hit suite runs one test per case and prints a hit
rate for each. Its pass conditions are in the `test/cv/test_shot_hit.py`
docstring and `CV_TEST_GAPS.md`.

## Launch sim by hand

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim sim.launch.py
ros2 launch sim sim.launch.py gui:=false rviz:=false     # server only
ros2 launch sim sim.launch.py x:=1.0 y:=0.5 yaw:=0.0     # spawn pose (z:= too)
ros2 launch sim sim.launch.py world:=/abs/path/to/other.sdf
ros2 launch sim sim.launch.py camera:=true               # bridge /color and /depth
ros2 launch sim sim.launch.py sim_engine:=sapien         # SAPIEN instead of gz
```

The camera is off by default. Its color and depth images are the heaviest
thing the sim publishes, and nothing in `sim` or its tests reads them. Turn it
on to run the YOLO pipeline against sim or to fill rviz's Image panel.

These add synthetic wheel-odometry error. All are off by default; the
`pose_emulator.py` note explains each one:

```bash
odom_noise_enabled:=true    # master switch for drift + jitter
odom_drift_stddev:=         # random-walk step, m/callback (0.0005)
odom_jitter_stddev:=        # per-sample jitter, m (0.001)
odom_jerk_stddev:=          # trigger_jerk size, m (0.2)
odom_jerk_bias_enabled:=true odom_jerk_bias_x:= odom_jerk_bias_y:=
odom_slip_ratio:=           # fraction of each driven metre lost from /pose (0.0)
```

`spawn_target` adds the moving CV target. It is off by default, but when you
turn it on, all three `cv_*` degradations apply at the defaults below. Set them
to zero for a clean run:

```bash
spawn_target:=true            # target_driver, cv_target_emulator, cv_head_aim
target_speed:=2.0 target_spin_hz:=1.5
cv_noise_pos_stddev:=0.005    # Gaussian position noise, m
cv_dropout_probability:=0.1   # per-sample detection drop
cv_publish_latency_s:=0.06    # placeholder, not measured
```

`sim.launch.py` starts gz, spawns the robot, bridges its lidar, joint, odometry,
camera and head-command topics to ROS, and runs `pose_emulator`, which
publishes `/pose` the way the Type-C board does. It runs no
`robot_state_publisher`, so TF comes from `thornbots_pkg`'s `auto.launch.py`.
`/sim/raw_odom` and `/sim/raw_joint_states` are ground truth that real hardware
doesn't have; only sim and its tests should read them.

## Notes

These notes explain why the code looks the way it does, so the in-code
comments can stay short. Each heading names a file.

### sapien_sim.py: the SAPIEN engine

`sim_engine:=sapien` swaps gz, the robot spawn and every bridge for this one
node. Everything downstream (`pose_emulator`, `target_driver`,
`cv_target_emulator`, `cv_head_aim`, `thornbots_pkg`) is unchanged, because the
node publishes the same topics gz does: `/clock`, `/scan_raw`, `/sim/raw_odom`
and `/sim/raw_joint_states`, with gz's frame names and rates. It subscribes to
`/cmd_vel`, `/head_pan_cmd` and `/head_pitch_cmd`. SAPIEN is the engine under
Triton Robotics' ManiSkill sim, which is why it was picked over MuJoCo and
Webots; the comparison against gz has not been run yet.

Why it looks the way it does:

- The chassis is moved kinematically. `sentry.urdf.xacro` has no collision
  and no gravity, so in gz the robot is a ghost that `VelocityControl` moves;
  integrating the `/cmd_vel` body-frame twist each step is the same thing. Like
  `VelocityControl`, the last command holds until a new one arrives.
- The head joints use the xacro's `JointPositionController` gains (p 75,
  d 0.125, effort 50) as SAPIEN force-mode drives, at gz's 1 ms step. On a
  gram-scale head that tracks a command within a few milliseconds, as in gz.
- The lidar is not SAPIEN's own ray cast. That casts one ray per Python call,
  14 ms for the 3000-beam scan, which caps the sim near 7x real time. Instead
  `trimesh` + `embreex` (Embree) cast all 3000 beams against
  `world/composite_part_1.stl` in about 1.3 ms, and spawned boxes are
  intersected analytically. The scan matches gz's `gpu_lidar` spec: frame
  `lidar`, -3.14 to 3.14 rad, 0.2-12 m, 0.03 m Gaussian noise. It does not see
  the robot's own head; `lidar_self_filter` blanks that sector anyway.
- `spawn_box` is a parameter, not a service, because `sim` is an
  `ament_python` package and can't define a `.srv`. `drift_harness.py`'s
  `spawn_box_obstacle` sets it with `ros2 param set` when `SIM_ENGINE=sapien`.
- The engine choice travels as the `SIM_ENGINE` environment variable. The
  test launches set it from `sim_engine:=`, so pytest and every per-scenario
  stack it starts inherit it without a new pytest option.
- `ros_gz_sim`'s launch file is found with `FindPackageShare`, which resolves
  only when the gz group runs, so the sapien engine works without `ros_gz`.
- SAPIEN, `trimesh` and `embreex` are pip-installed with `--no-deps`. SAPIEN's
  pip dependencies include `opencv-python`, which would replace the system
  `cv2`; SAPIEN imports fine without it. No renderer is created, so the node
  needs no GPU or Vulkan.

`world:=` works: the node reads the SDF for its first collision mesh and the
`<pose>` the world gives it, which is what keeps sapien's ranges on top of
gz's rather than 8 cm off. Only that one mesh is read, so a world built from
several models needs `field_mesh:=` or a change here.

Not yet supported under sapien: `gui:=` (no viewer), `camera:=`,
`auto_explore.py`'s teleport and `head_slider_relay.py` (both gz-transport
only).

### tools/simplify_urdf.py and urdf/sentry_v2

The mechanical team's Onshape export of the new sentry has 3782 links, 3781
joints and 581 meshes (220 MB), with no collision geometry. Onshape turns
every assembly mate into joints (a planar mate becomes two prismatics and a
continuous, a cylindrical one a prismatic and a continuous) and closes loops
with `*_loop_closure` dummy links, so almost none of those joints are real
motion. The export itself is not in the repo; regenerate from it with:

```bash
python3 tools/simplify_urdf.py <export_dir> urdf/sentry_v2.yaml urdf/sentry_v2
```

The tool collapses it to ten bodies: chassis (`root`), gimbal yaw (`head`),
gimbal pitch (`head_pitch`), and per corner a sprung carrier and an omni
wheel. Each body's mass and inertia are summed exactly (parallel-axis), so the
total stays 8.92 kg. Assignment, in order:

- `head` and `head_pitch` are the export tree below the yaw slewing bearing
  (`revolute_1_2`) and the horizontal shooter axis (`cylindrical_1_4`).
- A wheel is every part inside its measured cylinder (r 0.0949 m, +-0.02 m
  axially); its carrier is the parts inboard of the wheel within 0.10 m of
  the axle, plus the suspension arms by name.
- Everything else is chassis. Reference geometry (the rules keep-out boxes,
  FOV cones) and two stray parts with no inertia are dropped by name.

Each corner's parallel-arm suspension is one prismatic joint, which is close
to the arms' real arc over a few centimetres of travel. Wheels collide as
spheres (omni rollers can't be modelled with isotropic friction anyway),
other bodies as convex hulls of their parts above 3 cm. The robot's links
must not collide with each other; the chassis hull contains the wheels.
Visuals are one convex hull per part, capped at 120 faces, with parts under
15 mm left out: CAD tessellations are full of T-junctions that quadric
decimation cannot reduce. That keeps the model at 6.4 MB.

The joint and link names match `sentry.urdf.xacro` (`root`, `body`, `head`,
`head_pitch`, `lidar`, `camera`, `headlink`, `headpitch`) and `headlink` keeps
its -z axis, because the CV stack and `test_urdf_constants.py` assume them.
The output frame puts the gun on +x (the export's +y) with the origin on the
ground under the chassis centre.

Placeholders, not from the CAD: pitch limits (the old model's +-0.6 rad),
suspension travel (+-2 cm), spring rate (2200 N/m per corner, about 1 cm of
sag) and damping. The hopper landed on the pitch stage because of how the
export tree runs; confirm with the mechanical team. Nothing uses `sentry_v2`
yet; wiring it in means a driven chassis instead of the kinematic ghost, and
new pinned constants in `test_urdf_constants.py` and `cv_target_emulator.py`.

### sentry.urdf.xacro: the lidar does not scan its own model

Every robot visual carries `visibility_flags=0xFFFFFFFE` against the
`gpu_lidar`'s `visibility_mask=0x01`, so `(mask & flags) == 0` and the sensor
renders none of the sentry. Nothing else in the world sets the flags, so the
field keeps sdformat's default `0xFFFFFFFF` and stays visible.

This was tried in July 2026, reverted as "all-or-nothing per visual", and is
back because all-or-nothing turned out to be the right answer. Measured
2026-09-23, with the exclusion off: the head blanked 118-180 deg of
`/scan_raw`, and a further rear sector out to -45 deg came and went with the
head's pose -- between 863 and 1613 of 3000 beams, reported as `-inf` because
the self-hits land inside `range_min`. `lidar_self_filter` blanks 1.0 rad
(126-183 deg), so up to 140 deg of the scan was being lost to something
nothing in the stack knew about. A bare `gpu_lidar` at the same height in the
same world returns all 3000 beams at 2.22-7.17 m, which is what the sentry's
sensor now returns too.

Hardware is the reason to prefer it: the real RPLIDAR's scanning disk sits
clear of the chassis, and only the head's own footprint blocks it.
`lidar_self_filter` models exactly that, in the one place that also runs on
the robot. Note that the filter's 2.20-3.20 rad came from sim's old self-hit
cluster, which no longer exists -- it now needs a real `/scan_raw` capture to
retune against (see `../thornbots_pkg/README.md`).

### test_localization_drift.py

Integration suite for `sentry_localization`'s drift and jerk correction
against `pose_emulator.py`'s noise model. It mirrors `auto.launch.py`'s two
axes: `--backend slam/amcl/none` (who owns `map->odom`) and `--use-ekf` /
`--no-use-ekf` (whether `odom->root` is EKF-fused; on by default, matching
`auto.launch.py`). For each scenario it resets the shared sim, launches the
robot stack, drives, samples the correction TF, asserts, and stops the robot
stack. The sim itself stops after the last scenario.

`drift_harness.py` holds stack lifecycle, driving and scenarios;
`test_localization_drift.py` is one parametrized test per scenario. That split
lets `ekf_diag_harness.py` reuse `run_stack`/`drive` and puts `Scenario`'s
`details` into the assertion message instead of pytest's capture. The harness
launches the sim and each scenario's robot stack as trees in their own process
groups and won't attach to a running stack. The stack gets SIGINT if pytest dies, so a killed
run still tears it down.

Each scenario watches the edge the backend owns (`BACKEND_FRAMES`):

| Backend | Edge | Gate |
| --- | --- | --- |
| `slam` | `map->odom` | distance since last scan (`minimum_travel_distance`) |
| `amcl` | `map->odom` | `update_min_d`/`update_min_a` |
| `none` | `odom->root` | no map node; `ekf_node` unless `--no-use-ekf`, then raw `/odom` |

`--use-ekf` swaps `odom->root`'s source to `ekf_node` and leaves `map->odom`
alone. `mapping` isn't offered, since it builds a map rather than being scored
against one.

`jerk_with_motion` is skipped for `none`: `ekf_node` fuses `/odom` velocity
only (`odom0_config`), with no travel gate, so neither expectation is defined.
Drift scenarios do run for `none`, since rf2o's `/scan_odom` does real scan
matching. With no map to miss a feature from, `drift_correction` and
`drift_correction_obstacle` should read about the same there.

amcl with and without EKF under slip, measured 2026-07-26 against a 0.30m
bound; verdicts shown against today's 0.40m `MAX_DELTA_THRESHOLD`:

| `odom_slip_ratio` | `amcl` | `amcl` + `use_ekf:=true` |
| --- | --- | --- |
| 0.0 | 0.1478 m (PASS) | 0.2043 m (PASS) |
| 0.25 | 0.4033 m (**FAIL**) | 0.1642 m (PASS) |

At zero slip `/odom` is near perfect and fusing rf2o's noise only hurts.
Under slip the EKF turns a fail into a pass. That was the first sign it helps
a backend that owns a map. 0.25 is harsher than the defaults (0.02, or 0.15 in drift
scenarios). `slam --use-ekf` measured worse than plain `slam`; see
`sentry_localization/README.md`.

#### Scenarios

The suite runs them in this order.

1. `baseline` (noise off) asserts the correction TF settles and stays stable,
   with no ERROR in any log. A steady ~0.1-0.15m offset is normal, because the
   saved ARCC26 map origin (`[-4.3, -6.23, 0]`) doesn't match sim's spawn. A
   growing offset would be a real problem.
2. `noise_correction` drives the 3m square under drift and jitter (no slip or
   jerks) for a fixed 30s (60s before 2026-07-27). The second half's samples
   must stay under 2x the first half's max. The window doesn't exit early, so
   a stalled TF can't cause an open-ended drive.
3. `drift_correction` drives the same square, no obstacle. The instant
   reversals at 4.0 m/s build dead-reckoning error faster than the scan-match
   gate follows, and the wobble is the backend correcting it at each dwell.
4. `drift_correction_obstacle` adds a static box at the loop centre
   mid-scenario, absent from the world and the map. It shares driving and
   threshold with `drift_correction`, so comparing the two isolates the
   obstacle. A pass here means nothing if `drift_correction` failed.
5. `jerk_with_motion` (slam/amcl) models a collision impulse. Each trial fires
   `trigger_jerk`, drives one leg to the next corner, then requires a
   correction proportional to the jerk or an end state within
   `MAX_DELTA_THRESHOLD`. Jerks are biased toward `OBSTACLE_XY` so they don't
   push the robot into nearby walls, and the leg absorbs the actual (dx, dy)
   so the robot still lands on its corner. `JERK_WITH_MOTION_REPEATS` (8)
   trials share one stack (relaunching costs 15-20s each) and all must pass.
   A closing lap follows.
6. `odom_stuck` models a dead encoder: `trigger_odom_stuck` pins `/pose` x/y
   and velocity at zero with fresh timestamps. It checks liveness only, since
   there is no valid odometry to bound drift against: scans keep processing
   and pairwise TF spread exceeds `ODOM_STUCK_MIN_TF_SPREAD` (1cm).
   Measured 2026-07-27: `amcl` fails, stuck at 0.0000m for 30s, because the
   scan-match gate runs on odom-reported travel and frozen odom never reopens
   it. The stack really does depend on odometry to stay live. `amcl --use-ekf`
   passes at 1.3071m, because the EKF keeps reporting travel.

Two checks were removed. `jerk_stationary` (2026-07-23) re-verified a documented limit
of the travel gate instead of testing recovery. A no-leak-before-motion check
in `jerk_with_motion` (2026-07-26) failed for reasons unrelated to the
correction.

#### Geometry constants

`OBSTACLE_XY = (0.0, 0.0)` is the world origin, where the box and robot both
spawn, so loop centre and box coincide by construction.

`OBSTACLE_LOOP_LEGS` is a 3m square there, corners at (+-1.5, +-1.5), widened
from 2m on 2026-07-26 (`4f182e7`). Legs are `(vx, vy, duration)`, 0.75s at
4.0 m/s. Wall clearances are known on y only: north clears `upper_mid`
(y=2.49) by 0.99m, south clears `lower_mid` (y=-2.11) by 0.61m and
`bottom_wall`'s ramp edge (y=-3.35) by 1.85m. `lower_mid` is the tightest, so
start from it if you widen the loop.

`OBSTACLE_LOOP_DWELL_SECONDS = 1.0` lets scan and TF settle after each
reversal. Speed stays at 4.0 m/s, so the dwell is what you can adjust. Nobody
has validated 1.0s; change it if `max_delta` won't get under the threshold.

No scenario drives `PATROL_LEGS` any more; it stays as the geometry the loop
above came from. A 6-leg field tour that cleared every wall's bounding box by
~0.77m still hit a wall after ~10 open-loop laps, as small per-leg errors added
up, which is why the suite switched to the smaller loop.

#### Helpers

`wait_for_scans_flowing` decides when the stack is ready. slam_toolbox and amcl
publish an identity TF at startup before any scan, so waiting on TF can start
assertions on a cold stack. Once, under load, slam_toolbox registered 2 scans
in 30+ seconds.

`call_trigger_jerk_and_get_dxdy` parses the applied (dx, dy) from the
`Trigger` response instead of assuming `odom_jerk_stddev`, since one draw can
land far under its stddev and the corrective leg needs the real vector. It
returns `None` if parsing fails, so a change to the message format weakens the
check instead of crashing it.

`drive()` re-aims every tick at the leg's ground-truth endpoint from
`/sim/raw_odom` until within `WAYPOINT_TOLERANCE` (0.03m). Its ticks, like
`spin_for` and every observe window, count sim seconds; the wall clock only
guards against a stalled `/clock`. A fixed Twist for a
wall-clock duration undershot whenever gz's real-time factor dipped under load;
gating on projected distance fixed undershoot but not lateral drift. Speed is
capped at `dist / CONTROL_PERIOD` (0.1s) so it tapers near the target. At full
4.0 m/s one tick covers 0.4m, which overshot the tolerance and oscillated at
every corner. `duration` remains a safety cap (3x, at least +5s) that logs a
warning when hit.

`spawn_box_obstacle` spawns a `<static>` box with
`ros_gz_sim create -string <inline SDF>` as a subprocess, since it fires
mid-scenario after the pre-spawn baseline. Sim teardown removes it.

#### Thresholds

`MAX_DELTA_THRESHOLD = 0.40` m covers `noise_correction`, both drift scenarios,
and the fallback pass for `jerk_with_motion` (a small jerk can demand an
unrealistically tiny correction). It was 0.20 on 2026-07-26, 0.30 later that
day when no config reached 0.20 at 0.25 slip, and 0.40 on 2026-07-27 once the
chosen config (tuned `slam`, no EKF, 0.15 slip) measured 0.30-0.33m. See
`sentry_localization/README.md`'s tuning history.

`CORRECTION_FRACTION` is 0.3. slam_toolbox plateaus at a partial correction,
typically 40-70% of the jerk, because scan matching corrects the pose graph
incrementally. 0.5 sat at that plateau's edge and flaked; 0.3 keeps margin
and still catches the known-broken case (`minimum_travel_distance` at 0.5,
indistinguishable from zero). It was never checked against amcl's plateau, so
suspect it first if amcl runs flake.

That calibration used 0.15 m/s and `JERK_STDDEV=0.3`. Both have changed: 4.0
m/s, and `JERK_STDDEV` 0.5, then 0.08, now 0.24 for a ~30cm mean jerk
(magnitude of two N(0, stddev) draws is Rayleigh, mean
`stddev * sqrt(pi/2)`). Re-derive the plateau if pass rates look off.

On 2026-07-23 the correction step was a `while` loop driving `PATROL_LEGS`
for up to 60s until the threshold was crossed. The TF stalled for an unrelated
reason, the loop never exited, and open-loop drift took the robot off the field
and crashed gz physics. The step now drives one bounded leg and takes one TF
sample.

CPU contention from a stray rviz2 or other sessions slows scan processing to
about 2 registrations in a ~35s run, so the post-drive `get_correction_tf()`
uses a 5s timeout.

### pose_emulator.py: odom noise model

Sim ground truth has no wheel drift, so nothing would exercise `map->odom`
correction. These params add it, all off by default:

- `odom_drift_stddev`: random-walk step (m/callback) added to a persistent
  offset.
- `odom_jitter_stddev`: independent per-sample jitter, not accumulated.
- `odom_jerk_stddev`: one-off displacement fired by `~/trigger_jerk`, never
  automatically. The 0.2 default is well above a drift step so the correction
  shows as a jump.
- `odom_jerk_bias_enabled`, `odom_jerk_bias_x/y`: bias the jerk direction
  toward a point, for loops with corners near walls.
- `odom_slip_ratio`: drops a fraction of each metre driven (0.5 means `/pose`
  moves 0.5m per real metre), like wheels spinning on the "Bumpy Road" zone.
  It grows with distance, where drift grows with time.

`trigger_jerk()` moves the gz robot by a random (dx, dy) and subtracts the
same (dx, dy) from the drift accumulator, so `/pose` doesn't jump. The encoders
never saw the move. The error appears when the next scan match disagrees and
corrects `map->odom`, and that correction is what the jerk tests. Fire one by
hand with
`ros2 service call /pose_emulator/trigger_jerk std_srvs/srv/Trigger`.

### head_slider_relay.py

The gz GUI slider always publishes to `/model/<model>/joint/<joint>/<axis>/cmd_pos`.
ROS can't bridge that name (`parameter_bridge` raises `InvalidTopicNameError`
on the `0` token), so the xacro's controllers listen on a topic without the
axis segment, which `sim.launch.py` bridges as `/head_pan_cmd` and
`/head_pitch_cmd`. `JointPositionController` takes one `<topic>`, so this
relay forwards the slider to it.

The image has no gz-transport Python bindings. The relay reads with a
long-lived `ign topic -e` and writes with a fresh `ign topic -p` per message,
each costing tens of ms of discovery. When one blocking loop did both, slider
ticks queued and the head crawled toward stale positions. Reader and publisher
are now separate threads sharing the latest value, with an `Event` that drops
values arriving mid-publish.

### auto_explore.py: teleport

Teleport writes the gz world pose through `/world/<world>/set_pose` via
`ign service`; ROS has no equivalent. It works because root is a free 6DOF body
with no parent joint and no collision on any link. gz only honours a pose write
on a link its `FreeGroup` API sees as free (an older URDF with a prismatic chain
ignored it), and without collision nothing it passes through can spin it up.
Nothing enforces zero rotation any more, so each teleport sets orientation to
identity.

Each teleport also fires a `model_only` `WorldReset`, before and after
`set_pose`, to zero the joints. It can't move root, which has no parent joint.
The reset afterwards clears the one-step reaction impulse root's position jump
can put through the body joints. Root's inflated rotational inertia damps its
own angular velocity.

### sim.launch.py: spawn_robot uses -string

`-topic robot_description` makes `create` subscribe over ROS, and it reliably
misses `robot_state_publisher`'s TRANSIENT_LOCAL message. `ros2 topic echo`
got it instantly on the same QoS while a matched `spawn_sentry` waited 30+
seconds. That's a `ros_gz_sim create` bug, not a race, so delays don't help.
`-string` passes the URDF text directly.

### test_ekf_ground_truth.py

This suite (`ekf_diag_harness.py`, run by `localization_tests.launch.py
suite:=ekf`) asks whether fusing `/scan_odom` into `/odom` through `ekf_node` gets closer to
where the robot really is, which the drift suite can't answer. Drift scenarios
run with noise off, so only slip corrupts `/odom`; at zero slip
`pose_emulator` reports exact ground truth and no EKF can beat it. Old "EKF is
worse than raw /odom" numbers came from that setup and say nothing about the
EKF. Those scenarios also score `map->odom`, while the EKF's edge is
`odom->root`, dominated by the robot's own motion.

It turns on drift and continuous slip, drives the same loop, scores both
estimators against `/sim/raw_odom` (mean, RMS, max Euclidean error), and
asserts the EKF's mean error beats raw `/odom`'s.

### target_driver.py / cv_target_emulator.py

The target doesn't exist in gz. `target_driver` integrates `(x, y, z)` on a timer
and publishes `nav_msgs/Odometry` on `/target/ground_truth_odom`, the same
stand-in approach as `pose_emulator`. That skips SDF, spawning and bridges, but
the target is invisible in the gz GUI; check it with topic echoes.

Both stamp from `self.get_clock().now()` (sim `/clock`). `cv_target_emulator`
stamps `panel_detections` headers at sample time and holds messages in a
queue for `publish_latency_s`, so downstream `now - header.stamp` shows the
delay.

The default path runs laterally at `x=3.0m`, `y in [-2.4, 2.4]`, `z=0.3m`.
Visible half-width at 3m is `3.0*tan(1.5184/2)` ~ 2.85m, so the outer panels
(0.3m out) keep ~0.15m margin with the head straight ahead. `max_accel`
(6 m/s^2) brakes the target to a stop at each end and ramps any change of
`target_speed`, and `max_spin_accel` (20 rad/s^2) does the same for `spin_hz`;
both are estimates. It used to reverse and change speed instantly.

`cv_target_emulator` computes camera pose by chaining the xacro's fixed joint
offsets (root -> fastened_2 -> body -> headlink(yaw) -> head ->
headpitch(pitch) -> head_pitch -> cameralink -> camera), since sim runs no
`robot_state_publisher`. It reads joint angles from `/sim/raw_joint_states` by
name. `headlink`'s and `fastened_2`'s pi yaws cancel at `head_yaw=0`, but
`headpitch` carries a fixed -0.38885 rad yaw that never cancels. Mean `pos_err`
was ~2.45m with +0.38885, ~1.22m with 0, and ~0.13m with -0.38885, near the
0.03m noise floor, so -0.38885 is the right sign. Compare vectors rather than
magnitudes, as with rf2o's `angle_min` bug, because `tan(+x)` and `tan(-x)` have
the same magnitude.

`headlink` has been a continuous joint since 2026-07-28, matching the
free-spinning real gimbal. It used to be `revolute` with a +-pi limit;
`cv_head_aim` pegging at exactly +-3.14159 turned out to be its own software
clamp. `headpitch` keeps its real +-0.6 limit.

Target positions use REP-103 (x forward, y left, z up), not the optical frame
a real driver reports. Mislabelling optical as REP-103 would rotate every
detection by a fixed offset, the same bug class as rf2o's `angle_min` (179.81
degrees off, magnitude right). On 2026-07-27, `/cv/target`'s left/right sign
matched an independent bearing from `/sim/raw_odom` and
`/target/ground_truth_odom` in 20/20 samples.

Detections outside the FOV (`horizontal_fov=1.5184`, vertical from 640x480) or
range (0.1-10.0m) aren't published, which exercises `point_to_cv_target`'s
watchdog. Inside, `noise_pos_stddev` (0.005m), `dropout_probability` (0.1) and
`publish_latency_s` (0.06s, a placeholder) all default on.

### cv_head_aim.py

Subscribes `/cv/target` (from `thornbots_pkg`'s `point_to_cv_target`, so run
`auto.launch.py` alongside `sim.launch.py spawn_target:=true`) and
`/sim/raw_joint_states`, and publishes `/head_pan_cmd`/`/head_pitch_cmd`. It is
the only thing moving the head in CV tests. `CVTarget.x/y/z` is a root-frame
position, so the old `atan2(x, z)` bearing controller was replaced.

`cv_head_aim_core.solve_head_angles()` inverts the FK chain `cv_target_emulator`
walks (root -> body -> headlink(yaw) -> headpitch(pitch) -> camera), treating
`HEADPITCH_ORIGIN_YAW` (-0.38885) as a yaw in the joint origin, not a pitch
bias. `test/cv/test_cv_head_aim.py` checks it against a separately written FK,
and `test_urdf_constants.py` pins the duplicated constants to the xacro.

It aims along the ray from the muzzle. Before 2026-09-09 it aimed from root,
assuming a ~0.35m offset wouldn't matter at range. That's true for flight time,
but a parallel offset in direction stays the same size at any range. The
muzzle sits `MUZZLE_Z` = 0.374m above root against a 0.05m hit radius, and
every stationary shot missed by ~0.33m (`CV_TEST_GAPS.md` gap 7).

The fix is closed-form even though muzzle position depends on the yaw being
solved. The muzzle circles the yaw axis at `MUZZLE_RADIUS` = 0.1m, and only
`MUZZLE_PERP` = `MUZZLE_RADIUS * sin(HEADPITCH_ORIGIN_YAW)` = 0.038m sits off
the shot line. Azimuth is `atan2(y, x) + asin(MUZZLE_PERP / horizontal_range)`,
and pitch follows from the elevation at that muzzle point. The pitch axis
passes through the muzzle, so pitch needs no correction.

Type-C probably has the same bug. It receives a root-frame position and runs its
own gimbal solve, and only the firmware knows where the real barrel sits, so
raise it with the firmware team (see `ros2_dji_serial_bridge/README.md`).

Every `control_rate_hz` (30) tick it commands `current + gain *
wrapped_error`. A timer, not `/cv/target` arrival (up to 60Hz), drives it;
per-message updates let the setpoint race ahead of the joint in early tuning.
`gain` is 1.0, the IK angle itself, leaving tracking to gz's joint PID. At the
old 0.3 and 15Hz the head trailed a 0.5 m/s target by 9.4cm against a 5cm hit
radius (2026-09-17), a lag no 1kHz gimbal loop would have. The old `sign_yaw`/`sign_pitch` params are gone, because the IK geometry
sets the sign and the test checks it analytically.

When `/cv/target` confidence reaches 0.0 it stops publishing and holds
position, without re-homing, since a lost target is usually a brief FOV gap.
