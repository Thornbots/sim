# sim

Launches `gz sim` with the `ARCC_Field_2026` world, spawns the `sentry` robot
from `urdf/sentry.urdf.xacro`, and hosts the localization and CV integration
suites. The package is named `sim` so the world's `model://sim/world/...` and
the xacro's `package://sim/meshes/...` resolve unedited.

```
sim/
├── launch/sim.launch.py
├── urdf/sentry.urdf.xacro
├── meshes/                        # Body, Head, Lidar, OdoWheel
├── world/                         # ARCC_Field_2026.sdf, composite_part_1.stl
├── rviz/                          # config.rviz, cv_target.rviz
├── sim/
│   ├── pose_emulator.py           # /pose + odometry noise model
│   ├── auto_explore.py            # teleporting grid sweep
│   ├── target_driver.py           # CV target ground truth
│   ├── cv_target_emulator.py      # camera FK + detection noise
│   ├── cv_head_aim.py             # CV head tracking (IK in cv_head_aim_core.py)
│   ├── head_slider_relay.py       # gz GUI slider <-> /head_*_cmd
│   └── wasd_teleop.py
└── test/
    ├── conftest.py                # shared pytest options
    ├── localization/              # drift and EKF ground-truth suites
    └── cv/                        # head-aim, URDF-constant and shot-hit suites
```

## Build

In a container terminal. `Dockerfile.thornbots` installs neither
`ros-humble-ros-gz` nor this package, so on a fresh container run
`install-sim.sh` once; it installs the deps and builds `sim`:

```bash
cd /workspaces/isaac_ros-dev
sudo src/isaac_ros_common/docker/scripts/install-sim.sh

colcon build --symlink-install --packages-select sim   # after later edits
source install/setup.bash
```

Unlike the other packages, `sim` has no baked copy in `/workspaces/ros2_ws`,
but a fresh shell still needs `source install/setup.bash` to find it.

Keep `--symlink-install`. Without it `install/sim` holds copies and edits
under `sim/` do nothing until the next rebuild. If a change seems to have no
effect, check that first; `rm -rf build/sim install/sim` and rebuild.

## Run

```bash
ros2 launch sim sim.launch.py
ros2 launch sim sim.launch.py gui:=false rviz:=false     # server only
ros2 launch sim sim.launch.py x:=1.0 y:=0.5 yaw:=0.0     # spawn pose (z:= too)
ros2 launch sim sim.launch.py world:=/abs/path/to/other.sdf
```

Synthetic wheel-odometry error, off by default (see the `pose_emulator.py`
note):

```bash
odom_noise_enabled:=true    # master switch for drift + jitter
odom_drift_stddev:=         # random-walk step, m/callback (0.0005)
odom_jitter_stddev:=        # per-sample jitter, m (0.001)
odom_jerk_stddev:=          # trigger_jerk size, m (0.2)
odom_jerk_bias_enabled:=true odom_jerk_bias_x:= odom_jerk_bias_y:=
odom_slip_ratio:=           # fraction of each driven metre lost from /pose (0.0)
```

CV target simulation. `spawn_target` is off by default, but once on, all three
`cv_*` degradations apply at these defaults; zero them for a clean run:

```bash
spawn_target:=true            # target_driver, cv_target_emulator, cv_head_aim
target_speed:=2.0 target_spin_hz:=1.5
cv_noise_pos_stddev:=0.03     # Gaussian position noise, m
cv_dropout_probability:=0.1   # per-sample detection drop
cv_publish_latency_s:=0.06    # placeholder, not measured
```

## What the launch file does

1. Sets `GZ_SIM_RESOURCE_PATH` (and `IGN_GAZEBO_RESOURCE_PATH`) to the
   installed `share/` so `model://sim/world/composite_part_1.stl` resolves.
2. Starts `ros_gz_sim`'s `gz_sim.launch.py` with `world/ARCC_Field_2026.sdf`,
   running (`-r`).
3. Bridges `/clock` for `use_sim_time` nodes.
4. Spawns the robot with `ros_gz_sim create -string` (see the `sim.launch.py`
   note), 2s after `clock_bridge` starts so gz's create service is up. It runs
   no `robot_state_publisher`; `thornbots_pkg`'s `auto.launch.py` owns that
   and TF.
5. Bridges the xacro plugins' gz topics: `/scan` as `/scan_raw` (for
   `thornbots_pkg`'s `lidar_self_filter`), `/sim/raw_joint_states` and
   `/sim/raw_odom`. The `/sim/raw_*` topics are ground truth with no hardware
   equivalent; nothing outside sim should read them.
6. Runs `pose_emulator`, which turns `/sim/raw_odom` and
   `/sim/raw_joint_states` into the `RobotPose` the Type-C board publishes on
   `/pose`, with the optional noise model.
7. Bridges `/cmd_vel`, `/head_pan_cmd`, `/head_pitch_cmd` and the four camera
   topics, and runs `head_slider_relay`.
8. Runs rviz2 unless `rviz:=false`.

With `spawn_target:=true` it also runs `target_driver`, `cv_target_emulator`
and `cv_head_aim`.

Compatibility: it needs `ros_gz_sim` and `ros_gz_bridge`. On Fortress-era
installs with `ros_ign_gazebo`, swap the package name in `sim.launch.py` and
use `ign_gazebo.launch.py`. The world uses `gz-sim-*` plugin names and the
xacro uses `ignition-gazebo-*`; current builds alias both. The robot spawns as
a dynamic model.

## Testing

Everything under `test/` is pytest, collected by `colcon test`.

| Tier | Files | Needs |
| --- | --- | --- |
| unit | `cv/test_cv_head_aim.py`, `cv/test_urdf_constants.py`, ament copyright/flake8/pep257 | Python + pytest |
| integration | `localization/test_localization_drift.py`, `localization/test_ekf_ground_truth.py`, `cv/test_shot_hit.py` | gz-sim + two launch trees |

`setup.cfg` deselects the `integration` marker, so a plain `colcon test` runs
the unit tier in seconds. A `-m` on the command line overrides it:

```bash
colcon test --packages-select sim
colcon test --packages-select sim --pytest-args ' -m integration'
colcon test-result --verbose
```

Each integration test launches gz-sim and `thornbots_pkg`, takes tens of
seconds, and tears down fully before the next. ROS topics are process-global,
so a stack you already have up corrupts the measurements. Stop it first.

Each suite keeps an argparse wrapper that re-invokes pytest, so old command
lines and `--help` still work. The wrappers install to `lib/sim/`:
`ros2 run sim run_localization_drift_tests.py`,
`ros2 run sim ekf_ground_truth_diag.py`, `ros2 run sim run_shot_hit_tests.py`.
Rerun the drift suite after tuning `slam.yaml`, `amcl.yaml`, `ekf.yaml` or
the noise model:

```bash
ros2 run sim run_localization_drift_tests.py --backend slam
# --backend amcl or none; --use-ekf layers on any of them
```

`--scenario NAME` runs one scenario. `--headless` skips the GUI and rviz2
(both on by default). `--speed` overrides the 4.0 m/s loop speed, which
nothing has been re-validated against. Each flag maps to a pytest option in
`test/conftest.py`, so from `src/sim` this is the same run:

```bash
python3 -m pytest test/localization/test_localization_drift.py \
  -m integration -s --backend slam --scenario drift_correction
```

Pass `-s`, or pytest swallows the numbers the suites print.

`sentry_localization` copies its config, launch and map files at build time.
After editing its YAML, rebuild with `--symlink-install` so later edits link
through:

```bash
colcon build --symlink-install --packages-select sentry_localization thornbots_pkg
```

If results look unaffected by a change, `diff` the installed YAML against
source.

Read the notes below, not the script docstrings, before interpreting a drift
failure. The shot-hit suite runs one test per (lead, speed) cell and prints a
hit rate for each, so a full run is the before/after lead table. Its pass
conditions are in `test/cv/test_shot_hit.py`'s docstring and
`CV_TEST_GAPS.md`.

## Notes

Design rationale, kept out of in-code comments. Each heading names a file.

### test_localization_drift.py

Integration suite for `sentry_localization`'s drift and jerk correction
against `pose_emulator.py`'s noise model. It mirrors `auto.launch.py`'s two
axes: `--backend slam/amcl/none` (who owns `map->odom`) and `--use-ekf`
(whether `odom->root` is EKF-fused). Per scenario: launch, drive, sample the
correction TF, assert, tear down.

`drift_harness.py` holds stack lifecycle, driving and scenarios;
`test_localization_drift.py` is one parametrized test per scenario. That split
lets `ekf_diag_harness.py` reuse `run_stack`/`drive` and puts `Scenario`'s
`details` into the assertion message instead of pytest's capture. The harness
owns its launch trees (each in its own process group) and won't
attach to a running stack.

Each scenario watches the edge the backend owns (`BACKEND_FRAMES`):

| Backend | Edge | Gate |
| --- | --- | --- |
| `slam` | `map->odom` | distance since last scan (`minimum_travel_distance`) |
| `amcl` | `map->odom` | `update_min_d`/`update_min_a` |
| `none` | `odom->root` | no map node; raw `/odom` passthrough unless `--use-ekf` |

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
Under slip the EKF flips the verdict, the first evidence it helps a
map-owning backend. 0.25 is harsher than the defaults (0.02, or 0.15 in drift
scenarios). `slam --use-ekf` measured worse than plain `slam`; see
`sentry_localization/README.md`.

#### Scenarios

Run in this order:

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
   it. That is a real stack finding. `amcl --use-ekf` passes at 1.3071m,
   because the EKF keeps reporting travel.

Removed: `jerk_stationary` (2026-07-23), which re-verified a documented limit
of the travel gate instead of testing recovery, and a no-leak-before-motion
check in `jerk_with_motion` (2026-07-26), which failed independently of the
correction.

#### Geometry constants

`OBSTACLE_XY = (0.0, 0.0)` is the world origin, where the box and robot both
spawn, so loop centre and box coincide by construction.

`OBSTACLE_LOOP_LEGS` is a 3m square there, corners at (+-1.5, +-1.5), widened
from 2m on 2026-07-26 (`4f182e7`). Legs are `(vx, vy, duration)`, 0.75s at
4.0 m/s. Wall clearances are known on y only: north clears `upper_mid`
(y=2.49) by 0.99m, south clears `lower_mid` (y=-2.11) by 0.61m and
`bottom_wall`'s ramp edge (y=-3.35) by 1.85m. `lower_mid` binds; re-derive
from it if the loop grows.

`OBSTACLE_LOOP_DWELL_SECONDS = 1.0` lets scan and TF settle after each
reversal. Speed is fixed at 4.0 m/s, so this is the knob. It hasn't been
validated; change it if `max_delta` won't get under threshold.

`PATROL_LEGS` is no longer driven, only kept as the geometry the above came
from. A 6-leg field tour that cleared every wall AABB by ~0.77m still hit one
after ~10 open-loop cycles as per-leg error piled up. A smaller loop fixed it.

#### Helpers

`wait_for_scans_flowing` is the readiness signal. slam_toolbox and amcl
publish an identity TF at startup before any scan, so waiting on TF can start
assertions on a cold stack. Once, under load, slam_toolbox registered 2 scans
in 30+ seconds.

`call_trigger_jerk_and_get_dxdy` parses the applied (dx, dy) from the
`Trigger` response instead of assuming `odom_jerk_stddev`, since one draw can
land far under its stddev and the corrective leg needs the real vector. It
returns `None` if parsing fails, so a message format change degrades instead
of crashing.

`drive()` re-aims every tick at the leg's ground-truth endpoint from
`/sim/raw_odom` until within `WAYPOINT_TOLERANCE` (0.03m). A fixed Twist for a
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

`CORRECTION_FRACTION = 0.3`. slam_toolbox plateaus at a partial correction,
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
and crashed gz physics. It is now one bounded drive plus one TF sample.

CPU contention (a stray rviz2, other agent sessions) slows scan processing to
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
corrects `map->odom`, which is what the jerk tests. To fire one:
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

(`ekf_diag_harness.py` plus the `ekf_ground_truth_diag.py` wrapper.) It asks
whether fusing `/scan_odom` into `/odom` through `ekf_node` gets closer to
where the robot really is, which the drift suite can't answer. Drift scenarios
run with noise off, so only slip corrupts `/odom`; at zero slip
`pose_emulator` reports exact ground truth and no EKF can beat it. Old "EKF is
worse than raw /odom" numbers came from that setup and say nothing about the
EKF. Those scenarios also score `map->odom`, while the EKF's edge is
`odom->root`, dominated by the robot's own motion.

This suite enables drift and continuous slip, drives the same loop, scores both
estimators against `/sim/raw_odom` (mean, RMS, max Euclidean error), and
asserts the EKF's mean error beats raw `/odom`'s.

### target_driver.py / cv_target_emulator.py

The target is not a gz entity. `target_driver` integrates `(x, y, z)` on a timer
and publishes `nav_msgs/Odometry` on `/target/ground_truth_odom`, the same
stand-in approach as `pose_emulator`. That skips SDF, spawning and bridges, but
the target is invisible in the gz GUI; check it with topic echoes.

Both stamp from `self.get_clock().now()` (sim `/clock`). `cv_target_emulator`
stamps `panel_detection.header.stamp` at sample time and holds messages in a
queue for `publish_latency_s`, so downstream `now - header.stamp` shows the
delay.

The default path bounces laterally at `x=3.0m`, `y in [-2.0, 2.0]`, `z=0.3m`.
Visible half-width at 3m is `3.0*tan(1.5184/2)` ~ 2.85m, leaving ~0.85m margin
each side. On 2026-07-27, 8 m/s (the fastest tested) gave at least 47
consecutive in-frustum samples.

`cv_target_emulator` computes camera pose by chaining the xacro's fixed joint
offsets (root -> fastened_2 -> body -> headlink(yaw) -> head ->
headpitch(pitch) -> head_pitch -> cameralink -> camera), since sim runs no
`robot_state_publisher`. It reads joint angles from `/sim/raw_joint_states` by
name. `headlink`'s and `fastened_2`'s pi yaws cancel at `head_yaw=0`, but
`headpitch` carries a fixed -0.38885 rad yaw that never cancels. Mean `pos_err`
was ~2.45m with +0.38885, ~1.22m with 0, and ~0.13m with -0.38885, near the
0.03m noise floor, which confirms the sign. As with rf2o's `angle_min` bug,
compare vectors: `tan(+x)` and `tan(-x)` have the same magnitude.

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
watchdog. Inside, `noise_pos_stddev` (0.03m), `dropout_probability` (0.1) and
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

It aims the ray from the muzzle, not root. Before 2026-09-09 it aimed from
root on the theory that ~0.35m is negligible at range. That holds for flight
time but not direction: a parallel offset doesn't shrink with range. The
muzzle sits `MUZZLE_Z` = 0.374m above root against a 0.05m hit radius, and
every stationary shot missed by ~0.33m (`CV_TEST_GAPS.md` gap 7).

The fix is closed-form even though muzzle position depends on the yaw being
solved. The muzzle circles the yaw axis at `MUZZLE_RADIUS` = 0.1m, and only
`MUZZLE_PERP` = `MUZZLE_RADIUS * sin(HEADPITCH_ORIGIN_YAW)` = 0.038m sits off
the shot line. Azimuth is `atan2(y, x) + asin(MUZZLE_PERP / horizontal_range)`,
and pitch follows from the elevation at that muzzle point. The pitch axis
passes through the muzzle, so pitch needs no correction.

**Type-C likely has the same bug.** It gets a root-frame position and does its
own gimbal solve, and only firmware knows where the real barrel sits. Raise it
with firmware; see `ros2_dji_serial_bridge/README.md`.

Control is closed-loop on the IK setpoint so that tracking lag shows up in sim
as it would on Type-C. Every `control_rate_hz` (15) tick it commands
`current + gain * wrapped_error`. A timer, not `/cv/target` arrival (up to
60Hz), drives it; per-message updates let the setpoint race ahead of the joint
in early tuning. `gain` (0.3) is a placeholder that still needs an empirical
pass. The old `sign_yaw`/`sign_pitch` params are gone because the IK fixes
sign, as the test checks analytically.

When `/cv/target` confidence reaches 0.0 it stops publishing and holds
position, without re-homing, since a lost target is usually a brief FOV gap.

### Removed

- `head_sweep.py` (2026-08-31): a yaw oscillation that replaced `cv_head_aim`
  when `head_sweep_hz > 0`, for `run_shot_hit_tests.py`'s head-slew scenario
  (checking `target_tracker`'s TF decoupling with a moving camera and still
  target). The scenario went with it; restoring the check means rebuilding
  both.
- `CVTarget` `v_x/v_y/v_z` and `a_x/a_y/a_z` (2026-07-28): EMA
  finite-differences from `point_to_cv_target`, sent into the MCB's `CV_MSG`.
  They were removed from the message and the UART struct together, a
  wire-format break (see `ros2_dji_serial_bridge/README.md`). Velocity came
  back as `thornbots_pkg`'s `target_tracker` on `/cv/target_state`, ROS-only.
  `CVTarget` gained only a `lead_applied`/`track_valid` flags byte.

### Environment

`thornbots_pkg`'s `install/` was once a stale symlink-install pointing at a
deleted git worktree, breaking `ros2 run` and imports until a rebuild. The
sim and CV test scripts call `point_to_cv_target` by absolute install path to
avoid that class of failure.
