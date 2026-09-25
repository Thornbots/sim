# sim

sim holds the robot's integration tests. The localization drift suite starts a
`gz sim` model of the `ARCC_Field_2026` field, spawns the `sentry` robot from
`urdf/sentry_v2.urdf.xacro`, and runs the real `thornbots_pkg` stack against
it. The CV aim bench runs no gz: it scores `point_to_cv_target` against a
perfectly known target. gz on the CV side is for Part 2 only, turning noisy
detections into a target model (`../CV_SPLIT_PLAN.md`, Estimation).

## Run the tests

Build first on a fresh container (see Build). Each test starts its own sim and
`thornbots_pkg` stacks, so stop anything you already have running.

The localization suite runs `amcl` with the EKF, the configuration the robot
is targeting. The six scenarios before `moving_obstacles` took about three
minutes; it adds one more 30s loop:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim localization_tests.launch.py
```

`suite:=ekf` runs the EKF ground-truth test instead.

The CV aim bench runs ten shot-hit cases, all with lead on: a stationary
target, then 0.5, 1, 2 and 4 m/s, first with flat panels and then with
neighbouring panels staggered 90% of a panel's height apart
(`panel_layout:=` picks one). It launches the stack once and only changes the
target between cases; each case settles for 3s, then scores 30s of sim time.
The bench fires at up to 40 Hz, far above the real launcher, and scores each
case on hit rate and hits per expected shot equally (`score()` in
`test/cv/shot_hit_harness.py`), so falling behind 40 Hz costs points.

There is no gz, robot or tracker. `sim_clock` publishes `/clock`,
`point_shooter` puts `root` in `odom` (`POINT_SHOOTER`, 0.4 m up) and
publishes `/pose`, `target_driver` moves the phantom target and
`target_state_truth` publishes its true `TargetState`. Each shot leaves
`root` toward the newest `/cv/target` aim before its exit time, carrying
`root`'s velocity: a perfect gimbal that holds each 40 Hz aim until the
next. So a miss is `point_to_cv_target`'s math and nothing else. It runs at
`real_time_factor` (1 by default; no "unthrottled" without gz) and draws the
target's panels in rviz.

`chase_settle_s` picks `point_to_cv_target`'s spin mode: `>= 0` (the default,
0) chases the facing panel and fires every tick, `< 0` is center aim with
timed fire. `gimbal_lag_s` defaults to 0, the perfect gimbal. Each cell has
its own floor in `FLOORS` (`test/cv/shot_hit_harness.py`), the lowest score
over three runs minus 10 points. The ten lateral, still-shooter cells are
seeded from one run until three exist; any other cell falls back to the
placeholders and says so. `target_path:=radial` or `diagonal`
moves the target along the camera ray instead of across it, and
`shooter_speed:=1.0` bounces our own chassis along y (within 1 m of
`POINT_SHOOTER`) through every case.

Every scored shot goes to `shots.jsonl` in `--log-dir`, one JSON object per
line: the case, whether it hit, the miss distance split into the panel's
offset right of, above and ahead of the shot (ahead is along the target's
travel, so positive means the shot trailed), the panel and incidence angle,
the target's velocity, the target rotation it arrived in, and the `CVTarget`
fire fields. Each case also prints the mean of those offsets over its misses.
`panel_hits.jsonl` holds one line per case with the hits on each panel (front,
left, back, right) in each target rotation. It is for reading, not scoring.
`scores.jsonl` holds each case's score. Three runs' worth make the floors:

```bash
python3 tools/shot_floors.py run1/ run2/ run3/   # each a log_dir:=
```

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim shot_hit.launch.py
```

The CV estimation bench (C2) scores Part 2, not hits. gz runs our
`sentry_v2`, `target_driver` moves the phantom target through the same ten
cells, `cv_target_emulator` turns it into detections off the real head's
camera, and `target_selector` and `target_tracker` build the `TargetState`.
`point_to_cv_target` and `cv_head_aim` keep the head on it; nothing fires.
Each case switches detections off for 1 s so the tracker starts a fresh
track, then scores every state for 3 s + 30 s against `target_driver`'s truth
at the state's own stamp, so a late stamp scores as error:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim estimation.launch.py
```

Per case it prints how long the facing panel's error took to stay under
5 cm, the fraction of states valid, and the mean and p95 over the last 30 s
of: the facing panel's error (the panel Part 1 aims at), all four panels',
center, velocity, yaw (mod a quarter turn), spin rate, radius and height per
pair. `blackout:=true` drops every detection for 0.3 s in each 2 s.
`camera_latency_s:=0.03` stamps detections late and tells the tracker to
undo it (`tracker_camera_latency_s:=0` to leave it undone).
`shooter_speed:=1.0`, `target_path:=`, `speeds:=` and `panel_layout:=` work
as on the aim bench, and `process_noise_accel:=` sets the tracker's. A cell
passes on liveness until `LIMITS` in `test/cv/estimation_harness.py` has
limits for it:

```bash
python3 tools/estimation_limits.py run1/ run2/ run3/   # each a log_dir:=
```

`estimation.jsonl` has one summary line per case and
`estimation_states.jsonl` one line per scored state.

The bench is one launch tree: the stack and the pytest that scores it. The
localization launch runs pytest, and pytest starts the sim once
(`run_tests:=false part:=sim`) and a fresh robot stack for each scenario
(`part:=robot`). Between scenarios it stops the robot stack, teleports the
robot back to spawn, removes anything a scenario spawned, and resets
`pose_emulator`'s noise state and parameters. `restart_sim:=true` brings the
sim up fresh for every scenario instead, the old behaviour and the control when
a verdict looks off. Both
shut down when the tests finish, and Ctrl-C stops everything, stacks included.

The localization launch defaults to `real_time_factor:=0`, which lets gz run
as fast as the machine allows; the aim bench runs at 1. Every test times
itself in sim seconds, so a faster sim shortens the wall-clock run without
shortening what gets scored. On the dev laptop with `sentry_v2`, the drift
suite runs about 1.2x real time, GUI or headless alike.

Add an argument to run part of a suite. `scenario:=odom_stuck` runs one drift
scenario and `backend:=slam` or `use_ekf:=false` changes the stack. For the
bench, `only_stationary:=true` runs the stationary case, `speeds:='0.5 1'`
picks the moving cases. `headless:=true` drops the gz GUI and rviz from
either. `--show-args` on either launch lists the rest.

## Build

`Dockerfile.thornbots` installs neither `ros-humble-ros-gz` nor this package.
On a fresh container, run `install-sim.sh` once from a container terminal. It
installs gz from apt and builds `sim`:

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
| unit | `cv/test_cv_head_aim.py`, `cv/test_urdf_constants.py`, `cv/test_estimation_metrics.py`, ament copyright/flake8/pep257 | Python + pytest |
| integration | `localization/test_localization_drift.py`, `localization/test_ekf_ground_truth.py` | gz-sim and a launch tree |
| integration | `cv/test_shot_hit.py` | a launch tree, no gz |
| integration | `cv/test_estimation.py` | gz-sim and a launch tree |

`setup.cfg` deselects the `integration` marker, so a plain `colcon test` runs
only the unit tests and finishes in seconds. A `-m` on the command line
overrides that:

```bash
colcon test --packages-select sim
colcon test --packages-select sim --pytest-args ' -m integration'
colcon test-result --verbose
```

The drift suite starts gz-sim once and a fresh `thornbots_pkg` stack for each
scenario, resetting the sim between them (see "Run the tests");
`restart_sim:=true` restarts gz per scenario instead. The aim bench launches
its stack once for all its cases, through `shot_hit.launch.py
run_tests:=false` when pytest starts it. ROS topics are shared
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
ros2 launch sim sim.launch.py model:=sentry              # the old collision-free model
```

`model:=` picks the robot: `sentry_v2` (the default, from the CAD) or
`sentry`, the old model.

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

### SAPIEN: tried, not adopted

A SAPIEN engine (`sim_engine:=sapien`) was built as a faster stand-in for gz
and removed on 2026-09-24 before it ever ran a full suite. We are staying on
gz. The removed `sim/sapien_sim.py` is in the commit log if the question
comes back.

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

The joint and link names match the old model's (`root`, `body`, `head`,
`head_pitch`, `lidar`, `camera`, `headlink`, `headpitch`) and `headlink` keeps
its -z axis, because the CV stack and `test_urdf_constants.py` assume them.
The output frame puts the gun on +x (the export's +y) with the origin on the
ground under the chassis centre.

Placeholders, not from the CAD: pitch limits (the old model's +-0.6 rad),
suspension travel (+-2 cm), spring rate (2200 N/m per corner, about 1 cm of
sag) and damping. The hopper landed on the pitch stage because of how the
export tree runs; confirm with the mechanical team.

The export's base link, `root`, is `G_Lidar_Bottom_Guard`: it is the grounded
part, and nothing mates it to the head, so the tool put it on the chassis. It
sits diametrically opposite the lidar at (0.13, 0.14, 0.32-0.35), which pushes
the chassis collision hull up to 0.362 m. Scans don't notice (see the lidar
note below), but the hull is wrong. Fix it in Onshape by mating the guard to
the head, or drop it in `sentry_v2.yaml`'s `drop` list.

`sentry_v2.urdf.xacro` wraps the generated URDF for gz: colours, the lidar and
camera sensors, the plugins, and a `muzzle` frame (see the `cv_head_aim.py`
note). `thornbots_pkg`'s `sentry.urdf.xacro` carries the same frames and
meshes, and `test_urdf_constants.py` checks the two agree.

### sentry_v2.urdf.xacro: motion model

Gravity and collision are on. `VelocityControl` sets the chassis's whole
twist every step: planar velocity from `/cmd_vel`, zero angular velocity (a
hard yaw lock) and zero vertical velocity. The chassis settles onto its
sprung carriers (2200 N/m each) at about 1 cm/s and can't bounce. The wheels
are zero-friction spheres that ride along instead of fighting the commanded
velocity.

The head's PID keeps p 75 but raises d to 2.1 (yaw) and 1.2 (pitch), sized for
the real gimbal's inertia, about 0.031 and 0.009 kg m^2. The old model's
d 0.125 was tuned for a gram-scale head.

Measured on a real-time run: the chassis rests at z -1.0 cm. Over five laps of
the 3 m square at 4 m/s it rode at z -1.2 to -1.8 cm with roll and pitch under
0.2 deg. It picked up about 1 deg of yaw in the first lap's hard corners and
kept it (0.97 deg, then 0.89 deg by the end): the head's reaction torque during
instant 4 m/s velocity steps gets past the yaw lock. The real robot is expected
to drift 1-5 deg in yaw as well; for now the whole stack assumes a fixed
heading.

### Both robot models: the lidar does not scan its own model

Both of sim's models, `urdf/sentry.urdf.xacro` and `urdf/sentry_v2.urdf.xacro`,
do this. Every robot visual carries `visibility_flags=0xFFFFFFFE` against the
`gpu_lidar`'s `visibility_mask=0x01`, so `(mask & flags) == 0` and the sensor
renders none of the sentry. Nothing else in the world sets the flags, so the
field keeps sdformat's default `0xFFFFFFFF` and stays visible.

This was tried in July 2026, reverted as "all-or-nothing per visual", and is
back because all-or-nothing turned out to be the right answer. Measured
2026-09-23 on the old model, with the exclusion off: the head blanked 118-180 deg of
`/scan_raw`, and a further rear sector out to -45 deg came and went with the
head's pose -- between 863 and 1613 of 3000 beams, reported as `-inf` because
the self-hits land inside `range_min`. `lidar_self_filter` then blanked 1.0
rad (126-183 deg), so up to 140 deg of the scan was being lost to something
nothing in the stack knew about. A bare `gpu_lidar` at the same height in the
same world returns all 3000 beams at 2.22-7.17 m, which is what the sentry's
sensor now returns too. Both models have since dropped to the A2M8's 800
beams.

Hardware is the reason to prefer it: the real RPLIDAR's scanning disk sits
clear of the chassis, and only the head's own footprint blocks it.
`lidar_self_filter` models exactly that, in the one place that also runs on
the robot. Its sector (0.09-1.41 rad) now comes from slicing `sentry_v2`'s CAD
at the scan plane, and still needs checking against a real `/scan_raw` (see
`../thornbots_pkg/README.md`).

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
`drift_correction_obstacle` should read about the same there, and so should
`moving_obstacles`.

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
5. `moving_obstacles` drives the same square while `actor_driver` walks three
   unmapped boxes across its south, west and north edges at 1.0, 2.0 and 0.5
   m/s. It scores like `drift_correction`, on `MAX_DELTA_THRESHOLD`, and logs
   each sample's `map->root` error against `/sim/raw_odom`. It also fails if
   `actor_driver` dies mid-loop. Under `slam`, ROADMAP A4 also wants the
   actors' cells checked in `/map` at the end; that check isn't built yet.
6. `jerk_with_motion` (slam/amcl) models a collision impulse. Each trial fires
   `trigger_jerk`, drives one leg to the next corner, then requires a
   correction proportional to the jerk or an end state within
   `MAX_DELTA_THRESHOLD`. Jerks are biased toward `OBSTACLE_XY` so they don't
   push the robot into nearby walls, and the leg absorbs the actual (dx, dy)
   so the robot still lands on its corner. `JERK_WITH_MOTION_REPEATS` (8)
   trials share one stack (relaunching costs 15-20s each) and all must pass.
   A closing lap follows.
7. `odom_stuck` models a dead encoder: `trigger_odom_stuck` pins `/pose` x/y
   and velocity at zero with fresh timestamps. It checks liveness only, since
   there is no valid odometry to bound drift against: scans keep processing
   and pairwise TF spread exceeds `ODOM_STUCK_MIN_TF_SPREAD` (1cm).
   Measured 2026-07-27: `amcl` fails, stuck at 0.0000m for 30s, because the
   scan-match gate runs on odom-reported travel and frozen odom never reopens
   it. The stack really does depend on odometry to stay live. `amcl --use-ekf`
   passes at 1.3071m, because the EKF keeps reporting travel. Passing isn't
   tracking: 2026-09-24 it passed at 1.5073m with ground-truth error cycling
   0.24-4.3m per lap (see `AGENTS.md`).

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

`start_actor_driver` runs `python3 -m sim.actor_driver` (a checkout that
hasn't rebuilt its console scripts still has the module) and waits for its
"all N actors spawned" log line. `_reset_sim` and `teardown_stack` stop it
before anything else, and `_reset_sim` removes its `moving_actor_<i>` boxes.

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
with no parent joint. gz only honours a pose write on a link its `FreeGroup`
API sees as free (an older URDF with a prismatic chain ignored it). Each
teleport sets orientation to identity. On the old model nothing has collision,
so nothing the robot passes through can spin it up; on `sentry_v2`,
`VelocityControl` zeroes root's angular velocity every step.

Each teleport also fires a `model_only` `WorldReset`, before and after
`set_pose`, to zero the joints. It can't move root, which has no parent joint.
The reset afterwards clears the one-step reaction impulse root's position jump
can put through the body joints. On the old model, root's inflated rotational
inertia damps its own angular velocity.

### actor_driver.py: moving boxes

`actor_driver` spawns `count` boxes (0.3 x 0.3 x 0.8 m) with
`ros_gz_sim create` and walks each back and forth along a segment
(`paths`, four numbers per actor) at `speeds` m/s, calling `set_pose` on
every box each tick. Ticks run on a sim-time timer (`rate_hz`, 10), and each
advances the box by speed times the sim time since the last tick, so a slow
tick makes a longer jump and the actor keeps its speed.

The boxes are free bodies with no collision and gravity off. gz only
honours `set_pose` on a free body (see the `auto_explore.py` note), and a
static model is welded to the world. The lidar still sees them, since
`gpu_lidar` renders visuals. With collision, each box sat on the field's STL
mesh and ODE ran box-on-mesh contact every 1 ms step: gz's real-time factor
fell from ~1.0 to 0.24-0.42 and the scenario took twice as long as
`drift_correction`. The robot would drive straight through a box, so the
driver reads `/sim/raw_odom` and keeps every box `robot_radius`
(0.5 m, the barrel's reach) plus the box's half diagonal plus `margin`
(0.3 m) from wherever the robot can be within `lookahead_s` (1 s, or three
tick times if that is longer). That is its velocity swept ahead, plus
`route` ahead of it at `route_speed`: the harness passes the loop's corners
and `DRIVE_SPEED`, because the robot dwells stopped at each corner and its
velocity alone says nothing about the next leg. A blocked box steps to the
nearest clear spot on its path, forward or back.

The first version only swept the velocity 0.5 s ahead and hopped blocked
boxes forward, so on 2026-09-24 they landed on the robot's next leg and it
drove into them. The default paths now run from the loop's middle, 1.1 m
from every edge and always clear, across the south, west and north edges.
`closest box` in the log is the nearest a box centre got to the robot's.

Moves go through `sim.launch.py`'s `set_pose_bridge`, gz's `set_pose` as a
ROS `SetEntityPose` service, one async call per box per tick. The first
version started an `ign service` process (Ruby) per call, and ticks lagged to
~0.19 s of sim time. The default paths clear the ARCC26 map's walls by at
least 0.4 m.

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
stamps `panel_detections` headers at sample time plus `camera_latency_s` (0),
and holds messages in a queue for `publish_latency_s` (at least the camera
latency), so downstream `now - header.stamp` shows the delay. A late stamp is
the camera latency `target_tracker`'s own `camera_latency_s` has to undo.

The default path runs laterally at `x=3.0m`, `y in [-2.4, 2.4]`, `z=0.3m`.
`path_angle_deg` turns it about `(center_x, center_y)`: 90 runs along x, down
the camera ray, and the bench's `TARGET_PATHS` sets a centre and half-width
per path that keeps the near panel past ~1.2 m. Path params are read every
tick, and the twist carries world-frame x and y velocity.
Visible half-width at 3m is `3.0*tan(1.5184/2)` ~ 2.85m, so the outer panels
(0.3m out) keep ~0.15m margin with the head straight ahead. `max_accel`
(6 m/s^2) brakes the target to a stop at each end and ramps any change of
`target_speed`, and `max_spin_accel` (20 rad/s^2) does the same for `spin_hz`;
both are estimates. It used to reverse and change speed instantly.

`cv_target_emulator` computes camera pose by chaining `sentry_v2`'s fixed joint
offsets (root -> fastened_2 -> body -> headlink(yaw) -> head ->
headpitch(pitch) -> head_pitch -> cameralink -> camera), since sim runs no
`robot_state_publisher`. It reads joint angles from `/sim/raw_joint_states` by
name. None of `sentry_v2`'s joint origins rotate, so the head frame has the
gun on +x at zero yaw. `cameralink` puts the camera 9 cm ahead of the pitch
axis, 5.7 cm above it and 1.8 cm right of the muzzle.

The old model's `headpitch` carried a fixed -0.38885 rad yaw, and getting its
sign wrong cost a debugging cycle: mean `pos_err` was ~2.45m with +0.38885 and
~0.13m with -0.38885. Compare vectors rather than magnitudes, as with rf2o's
`angle_min` bug, because `tan(+x)` and `tan(-x)` have the same magnitude.

`headlink` has been a continuous joint since 2026-07-28, matching the
free-spinning real gimbal. It used to be `revolute` with a +-pi limit;
`cv_head_aim` pegging at exactly +-3.14159 turned out to be its own software
clamp. `headpitch` keeps a +-0.6 rad limit, carried over from the old model
as a placeholder.

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

### target_state_truth.py: the aim bench's perfect knowledge

Stands in for the whole of Part 2 on the aim bench (`shot_hit.launch.py`).
For each `/target/ground_truth_odom` sample it
publishes the target's `TargetState` at once, stamped with that sample's time:
the contract `TargetState.msg` sets for Part 2, a state describing the target
at its stamp. It used to publish on its own 60 Hz timer, which ran just ahead
of `target_driver`'s samples and sent each one ~17 ms late.

Panel 0 (front) is always the tracked panel: `yaw` is the chassis yaw,
unwrapped, and `radius` is `[panel_radius_x, panel_radius_y]`. `z_offset`
carries the stagger, `[+stagger/2, -stagger/2]` (front/back above, left/right
below). `acceleration` is the velocity change over the last sample step: exact
on `target_driver`'s constant-acceleration stretches, one step late at each
switch. `valid` is always true, `confidence` 1, `robot_track_id` 1, `variance`
zero. `panel_stagger_m` follows the case's layout (`CvStack.set_target`).

### point_shooter.py

Our chassis on the aim bench. A second `target_driver`, named
`shooter_driver`, with no spin and its output remapped to
`/shooter/ground_truth_odom`, bounces along y at `shooter_speed` with the
same braking; `point_shooter` republishes each sample at once as `odom->root`
TF and `/pose` (`RobotPose`, root-frame velocity), stamped with its sample
time. No noise or latency, so our motion is as perfectly known as the
target's. The harness flies each shot from `root`'s interpolated position at
exit with `root`'s velocity added, as a real projectile would carry it.

### sim_clock.py

`/clock` for stacks with no gz: `rate` sim seconds per wall second, stepped
by measured wall time at 1 kHz.

### cv_head_aim.py

Subscribes `/cv/target` (from `thornbots_pkg`'s `point_to_cv_target`, so run
`auto.launch.py` alongside `sim.launch.py spawn_target:=true`) and
`/sim/raw_joint_states`, and publishes `/head_pan_cmd`/`/head_pitch_cmd`: the
sim's gimbal when the full stack runs in gz. No test scores its shots; the
aim bench has its own perfect gimbal. `CVTarget.x/y/z` is a root-frame
position, so the old `atan2(x, z)` bearing controller was replaced.

`cv_head_aim_core.solve_head_angles()` inverts the FK chain from root to the
`muzzle` frame (root -> body -> headlink(yaw) -> headpitch(pitch) ->
muzzlelink). `test/cv/test_cv_head_aim.py` checks it against a separately
written FK, and `test_urdf_constants.py` pins the duplicated constants to
`thornbots_pkg`'s URDF and checks sim's model against it.

The `muzzle` frame sits on `head_pitch` at (0, 0.1128, 0): on the pitch axis,
between the two stacked flywheels. In the CAD the barrel is 0.3915 m up and
the pitch axis 0.3906 m, so the barrel really does sit on the axis. In the head
frame that puts the muzzle `MUZZLE_Y` = 0.0127 m left of the yaw axis and
`MUZZLE_Z` = 0.391 m above root.

It aims along the ray from the muzzle. Before 2026-09-09 it aimed from root,
assuming a ~0.35m offset wouldn't matter at range. That's true for flight time,
but a parallel offset in direction stays the same size at any range. On the
old model every stationary shot missed by ~0.33m against a 0.05m hit radius
(`CV_TEST_GAPS.md` gap 7).

The solve is closed-form even though the muzzle's position depends on the yaw
being solved. The shot leaves along the head's +x, and whatever the yaw that
line passes `MUZZLE_Y` to the left of the yaw axis. So azimuth is
`bearing - asin(MUZZLE_Y / horizontal_range)`, with bearing and range taken
from the yaw axis, and pitch follows from the elevation seen from the muzzle
at that azimuth. The muzzle is on the pitch axis, so pitching never moves it.
Head yaw is minus the azimuth, because `headlink` turns about -z.

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
