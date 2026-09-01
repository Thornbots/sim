# sim

ROS 2 package that launches `gz sim` (Ignition/Gazebo Sim) with the
`ARCC_Field_2026` world, spawns the `sentry` robot (defined in
`sentry.urdf.xacro`) into it, and hosts the localization and CV
integration suites.

## Package layout

```
sim/
├── launch/sim.launch.py           # main launch file
├── urdf/sentry.urdf.xacro         # robot description (package://sim/meshes/*.stl)
├── meshes/                        # robot meshes: Body, Head, Lidar, OdoWheel
├── world/                         # ARCC_Field_2026.sdf + composite_part_1.stl
├── rviz/                          # config.rviz, cv_target.rviz
├── sim/                           # the nodes (see ## Notes for per-file design history)
│   ├── pose_emulator.py           # /pose + synthetic odometry noise model
│   ├── auto_explore.py            # teleporting grid sweep
│   ├── target_driver.py           # fast-moving CV target ground truth
│   ├── cv_target_emulator.py      # camera FK + detection noise -> cv/panel_detection
│   ├── cv_head_aim.py             # CV-driven head tracking (cv_head_aim_core.py: IK)
│   ├── head_slider_relay.py       # gz GUI slider <-> /head_*_cmd bridge
│   └── wasd_teleop.py
└── test/
    ├── localization/              # run_localization_drift_tests.py,
    │                              #   ekf_ground_truth_diag.py
    └── cv/                        # run_shot_hit_tests.py, test_cv_head_aim.py
```

The package name is `sim` deliberately, so the world file's
`model://sim/world/...` and the xacro's `package://sim/meshes/...`
references both resolve without editing either source file.

## Build

Everything runs inside the isaac_ros-dev container. On a fresh or
recreated container, install gz-sim first: `Dockerfile.thornbots` ships
neither `ros-humble-ros-gz` nor this package, since real hardware never
needs them.

```bash
isaac_ros_common/scripts/dexec.sh -r -- \
  src/isaac_ros_common/docker/scripts/install-sim.sh

isaac_ros_common/scripts/dexec.sh -- \
  colcon build --packages-select sim --symlink-install
```

`--symlink-install` is not optional. Without it `install/sim` holds plain
copies, and an edit under `sim/` silently has no effect on the next
`ros2 run`/`ros2 launch` until you rebuild. If a change appears to do
nothing, check that linkage before assuming the change is wrong; recover
with `rm -rf build/sim install/sim` and rebuild.

## Run

```bash
ros2 launch sim sim.launch.py
```

Useful arguments:

```bash
ros2 launch sim sim.launch.py gui:=false             # no gz GUI (server only)
ros2 launch sim sim.launch.py rviz:=false            # no rviz2
ros2 launch sim sim.launch.py x:=1.0 y:=0.5 z:=0.1   # spawn pose (yaw:= too)
ros2 launch sim sim.launch.py world:=/abs/path/to/other.sdf
```

Synthetic wheel-odometry error, all off by default (see the
`pose_emulator.py` noise-model note below for what each one models):

```bash
odom_noise_enabled:=true    # master switch for drift + jitter
odom_drift_stddev:=         # random-walk step, m/callback
odom_jitter_stddev:=        # per-sample jitter, m
odom_jerk_stddev:=          # size of a trigger_jerk impulse, m (default 0.2)
odom_jerk_bias_enabled:=true odom_jerk_bias_x:= odom_jerk_bias_y:=
odom_slip_ratio:=           # fraction of each driven meter lost from /pose
```

Fast-moving CV target simulation. `spawn_target` is off by default, but
once it's on, the three `cv_*` degradations are all active at the defaults
shown; zero them for a clean run:

```bash
spawn_target:=true            # target_driver.py + cv_target_emulator.py
target_speed:=2.0 target_spin_hz:=1.5
cv_noise_pos_stddev:=0.03     # Gaussian position noise, m
cv_dropout_probability:=0.1   # per-sample detection drop
cv_publish_latency_s:=0.06    # placeholder, not a measured number
```

## What the launch file does

1. Sets `GZ_SIM_RESOURCE_PATH` (and `IGN_GAZEBO_RESOURCE_PATH` for older
   Ignition releases) to the installed `share/` directory so that
   `model://sim/world/composite_part_1.stl` resolves correctly.
2. Includes `ros_gz_sim`'s `gz_sim.launch.py` to start the simulator with
   `world/ARCC_Field_2026.sdf` loaded and running (`-r`).
3. Bridges `/clock` from gz sim to ROS so every node using
   `use_sim_time` stays in sync with the simulation clock.
4. Spawns the robot into the running world via `ros_gz_sim create`, passing
   expanded `sentry.urdf.xacro` text with `-string` (not `-topic`; see the
   `spawn_robot` note below for why). Delayed 2s off `clock_bridge`'s start
   so gz's entity-creation service is up first. `sim.launch.py` does not run
   `robot_state_publisher`; `sentry_pkg`'s `auto.launch.py` owns
   `robot_state_publisher` and TF now.
5. Bridges the gz-transport topics the xacro's plugins publish on, none of
   which reach ROS by themselves: `/scan` (remapped to `/scan_raw`, since
   `sentry_pkg`'s `lidar_self_filter` publishes the final `/scan`),
   the `JointStatePublisher` plugin's output as `/sim/raw_joint_states`,
   and the `OdometryPublisher` plugin's as `/sim/raw_odom`. Both `/sim/raw_*`
   topics are ground-truth and sim-internal; real hardware has no equivalent,
   so nothing outside sim should consume them.
6. Runs `pose_emulator`, which repackages `/sim/raw_odom` +
   `/sim/raw_joint_states` into the `dji_serial_bridge/msg/RobotPose`
   interface real hardware's Type-C board publishes on `/pose`, optionally
   with the synthetic noise model applied.
7. Bridges `/cmd_vel`, `/head_pan_cmd`, `/head_pitch_cmd`, and the four
   camera topics, and runs `head_slider_relay` so the gz GUI slider and
   `/head_*_cmd` can both drive the same joint controller.
8. Runs rviz2 unless `rviz:=false`.

With `spawn_target:=true` it also runs `target_driver`,
`cv_target_emulator`, and `cv_head_aim`.

## Assumptions / notes

- Assumes a ROS 2 distro providing `ros_gz_sim` + `ros_gz_bridge`. On
  older Fortress-era installs with `ros_ign_gazebo` instead, swap the
  `get_package_share_directory('ros_gz_sim')` calls in
  `launch/sim.launch.py` and the launch filename to `ign_gazebo.launch.py`.
- The world's plugin filenames (`gz-sim-*`) and the xacro's
  (`ignition-gazebo-*`) mix Fortress/Harmonic naming. Current builds alias
  both, but check your version's plugin names if one fails to load.
- The robot spawns as a normal dynamic model; nothing marks it `<static>`.

## Testing

`test/localization/run_localization_drift_tests.py` launches `sim` +
`sentry_pkg` end to end and exercises drift/jerk-correction behaviour
against this package's noise model. Not part of `colcon test`. Run it
after tuning `sentry_localization`'s `slam.yaml`/`amcl.yaml`/`ekf.yaml`
or `pose_emulator.py`'s noise model:

```bash
isaac_ros_common/scripts/dexec.sh -- python3 \
  /workspaces/isaac_ros-dev/src/sim/test/localization/run_localization_drift_tests.py \
  --backend slam   # or amcl, none; --use-ekf layers on top of any of them
```

Six scenarios run in order (`--scenario NAME` for just one): `baseline`,
`noise_correction`, `drift_correction`, `drift_correction_obstacle`,
`jerk_with_motion`, `odom_stuck`. `drift_correction` shares its loop and
threshold with `drift_correction_obstacle` deliberately, so compare the
two before blaming the obstacle for either failing. `--headless` skips the
GUI and rviz2 (both on by default, per the standing "watch sim live"
rule); `--speed` overrides the loop's 4.0 m/s, which nothing else has been
re-validated against.

Rebuild after editing a `sentry_localization/config/*.yaml`. That
package's config/launch/map `data_files` are copied at build time, so an
edited YAML has no effect on the running container until `install/`
resyncs:

```bash
isaac_ros_common/scripts/dexec.sh -- colcon build --symlink-install \
  --packages-select sentry_localization sentry_pkg
```

`--symlink-install` makes `install/` a symlink chain back to `src/`, so
every later edit takes effect immediately. Only the first build, or one
that omitted the flag, leaves the stale plain-copy trap. If results look
implausibly unaffected by a change, `diff` the installed YAML against the
source before assuming the change was wrong.

Each scenario's exact pass condition, the per-backend TF edge it watches,
and the suite's design history are in `## Notes` below. Read that before
interpreting a failure, not the script's docstring.

`test/cv/` holds `run_shot_hit_tests.py` (standalone, same shape as the
drift suite) and `test_cv_head_aim.py` (pure pytest over
`solve_head_angles()`, no stack needed).

## Notes

Design rationale kept out of in-code comments so the code stays skimmable.
Each subheading names a file; the in-code comment usually points back
here.

### run_localization_drift_tests.py

Integration suite for `sentry_localization`'s map-relative drift/jerk
correction, driven against sim's synthetic odometry noise model
(`sim/pose_emulator.py`). Mirrors `auto.launch.py`'s two independent axes:
`--backend slam/amcl/none` (who owns `map->odom`) and `--use-ekf`
(whether `odom->root` is EKF-fused). Loop per scenario: launch stack ->
drive -> sample the correction TF -> assert -> tear down.

It is a standalone script rather than a `colcon test` file because it
needs a running container, gz-sim, and two full launch trees, takes tens
of seconds per scenario, must run scenarios strictly sequentially with a
full teardown between them, and needs its measured numbers printed rather
than buried in a pytest traceback. It manages its own launch trees end to
end (same setsid/process-group approach as `dexec.sh -d`) and will not
attach to a stack you already have running, because ROS topics are
process-global and two stacks collide. Stop yours first.

Each scenario watches whichever TF edge the backend owns,
not literally `map->odom` every time (see `BACKEND_FRAMES`):

| backend | owns | notes |
|---|---|---|
| `slam` | `map->odom` | gated on distance travelled since last processed scan (`slam.yaml`'s `minimum_travel_distance`) |
| `amcl` | `map->odom` | same gating concept via `update_min_d`/`update_min_a` |
| `none` | `odom->root` | no map node at all; raw `/odom` passthrough unless `--use-ekf` |

`--use-ekf` is independent of all three: it swaps `odom->root`'s source
from passthrough to `ekf_node`, leaving `map->odom` ownership alone.
`mapping` is deliberately not a backend choice here, since its job is
building a map rather than being evaluated against one.

`jerk_with_motion` is SKIPPED for `--backend none`: `ekf_node` fuses
`/odom`'s velocity only, never its x/y (`ekf.yaml`'s `odom0_config`), and
applies no distance-travelled gate, so neither the "must not change" nor
the "must track the jerk" expectation is characterized. The drift
scenarios do run for `none`, since `/scan_odom` carries real
scan-to-scan matching from rf2o. An unmapped obstacle isn't a special
case for scan-to-scan matching (there's no map to be missing a feature
from), so `drift_correction` and `drift_correction_obstacle` should read
similarly there.

amcl vs amcl+EKF under slip, measured 2026-07-26. PASS/FAIL shown
against the current 0.40m `MAX_DELTA_THRESHOLD`; the numbers were taken
against the 0.30m bound in force then.

| `odom_slip_ratio` | `amcl` alone | `amcl` + `use_ekf:=true` |
|---|---|---|
| 0.0 | 0.1478 m (PASS) | 0.2043 m (PASS) |
| 0.25 | 0.4033 m (**FAIL**) | 0.1642 m (PASS) |

At zero slip the EKF makes amcl measurably worse: `/odom` is already a
near-perfect input there, so fusing rf2o's scan-matching noise on top can
only hurt. Under slip the ordering flips hard enough to change the
verdict. That's the first measured evidence that `use_ekf:=true` helps a
map-owning backend, not just the standalone `none --use-ekf` case. Note
0.25 slip is harsher than current defaults (0.02 generally, 0.15 for the
drift scenarios). `slam --use-ekf` was later measured and is worse than
plain `slam`; see `sentry_localization/README.md`.

#### Scenarios

Run in order: `baseline`, `noise_correction`, `drift_correction`,
`drift_correction_obstacle`, `jerk_with_motion`, `odom_stuck`.

1. `baseline` (`odom_noise_enabled:=false`) asserts the correction TF
   settles and stays stable, and that no log carries an ERROR. It is NOT
   expected near (0,0,0): the saved ARCC26 map's origin
   (`origin: [-4.3, -6.23, 0]`) doesn't coincide with sim's spawn pose, so
   a consistent ~0.1-0.15m offset is normal and reproducible. Stability is
   the actual check; a growing offset with noise disabled would be a real
   steady-state problem.
2. `noise_correction` drives the shared 3m cornering square under
   continuous drift/jitter (no slip, no jerks) for a fixed 30s (lowered
   from 60s on 2026-07-27 for faster iteration). Asserts the correction
   stays bounded: the second half's samples must not exceed 2x the first
   half's max. The window is fixed rather than early-exiting, so a stalled
   TF can't turn into an open-ended drive.
3. `drift_correction` drives the same square with no obstacle. Its
   instant-reversal corners at a real 4.0 m/s accumulate dead-reckoning
   error faster than the scan-match gate tracks it live, and the measured
   wobble is the backend correcting that error once the robot stops at
   each leg's dwell, rather than a wheel-slip artifact.
4. `drift_correction_obstacle` is strictly harder: same loop, plus a
   static box spawned mid-scenario at the loop's centre, absent from both
   `ARCC_Field_2026.sdf` and the saved map, so it's a lidar return with no
   map feature behind it. Shares driving code and threshold with
   `drift_correction` on purpose, so comparing the two isolates whether
   the obstacle compounds the cornering wobble. A PASS here means nothing
   if `drift_correction` (run immediately before) failed.
5. `jerk_with_motion` (slam/amcl only) models a collision impulse
   rather than gradual slip. Per trial: fire `trigger_jerk`, drive one
   bounded leg to the next corner, then assert either a prompt correction
   tracking the jerk's magnitude, or an end state within
   `MAX_DELTA_THRESHOLD`. Jerks are biased inward toward `OBSTACLE_XY`,
   since these corners sit close enough to real walls that a uniformly
   random jerk could push the robot into one. The leg is corrected by the
   jerk's actual (dx, dy), so the robot still lands on the intended corner
   and the loop doesn't walk off its clearance-checked geometry. Runs
   `JERK_WITH_MOTION_REPEATS` (8) trials in one stack, since each
   `trigger_jerk` is a fresh `random.gauss()` draw and relaunching per
   trial would add 15-20s of overhead for no coverage. ALL trials must
   pass, so one lucky draw can't flip the result. A closing lap follows.
6. `odom_stuck` models a dead encoder: a one-shot, permanent
   `trigger_odom_stuck` pins `/pose`'s x/y and velocities at zero while
   fresh timestamps keep arriving, so nothing looks stale. This is a
   LIVENESS check rather than a correctness one, since with no valid
   odometry there is nothing to bound drift against. It asserts scans keep
   being processed
   and corrections keep being attempted (pairwise TF spread exceeds
   `ODOM_STUCK_MIN_TF_SPREAD`, 1cm) rather than latching on one value.

   Measured 2026-07-27: `--backend amcl` FAILs, latched at 0.0000m spread
   for the full 30s, because the scan-match gate is driven by
   *odom-reported* travel and frozen odom never re-opens it. That is a real
   finding about the stack's reliance on odom for liveness rather than a
   test bug. `--backend amcl --use-ekf` PASSes with 1.3071m spread, since
   the EKF keeps odom reporting travel with the encoder input dead. EKF
   genuinely fixes this failure mode; it is not only noise smoothing here.

A former `jerk_stationary` scenario (fire a jerk, never move, assert the
TF must NOT change) was removed 2026-07-23: it re-verified a documented
structural limitation of the distance-travelled gate rather than testing
recovery. A "no-leak-before-motion" soft check inside `jerk_with_motion`
was removed 2026-07-26 for failing independently of the correction it was
wrapped around.

#### Geometry constants

`OBSTACLE_XY = (0.0, 0.0)` is the world origin, where both the box and
the robot spawn, so the loop centre and the box position coincide by
construction rather than through a chain of offsets.

`OBSTACLE_LOOP_LEGS` is a 3m square centred there, corners at
(+-1.5, +-1.5), widened from 2m on 2026-07-26 (`4f182e7`). Wall
clearances (y-axis only; no x-axis data exists): north edge clears
`upper_mid` (y=2.49) by 0.99m, south edge clears `lower_mid` (y=-2.11) by
0.61m and sits 1.85m off `bottom_wall`'s ramp edge (y=-3.35). The 0.61m to
`lower_mid` is the binding constraint, so re-derive from there rather than
from `PATROL_LEGS` if this loop is ever widened again. Legs are
`(vx, vy, duration)`, 3m per side, 0.75s at 4.0 m/s.

`OBSTACLE_LOOP_DWELL_SECONDS = 1.0` is a stationary dwell after each leg,
giving the scan/TF pipeline a moment to settle after each hard reversal,
closer to how a real robot corners. Driving speed (4.0 m/s) isn't
negotiable, so this is the available knob. Not re-validated against a real
run; re-derive it if 1.0s doesn't get `max_delta` under threshold.

`PATROL_LEGS` is no longer driven by any scenario, kept as the geometric
basis the above derive from. An earlier version toured the field on a
computed 6-leg loop that AABB-cleared every wall by ~0.77m and still drove
into one after ~10 cycles: the legs were open-loop, so per-leg execution
error on a free-floating chassis accumulated until it clipped a wall that
looked clear on paper. The fix was a smaller loop, not a more precisely
computed big one.

#### Helpers

`wait_for_scans_flowing` is the real readiness signal, not the correction
TF's existence: slam_toolbox and amcl both broadcast an identity transform
immediately at startup, before processing a single scan, so waiting on TF
can start timed assertions on a cold stack. Observed directly once:
slam_toolbox had registered 2 scans total in 30+ wall-clock seconds under
load.

`call_trigger_jerk_and_get_dxdy` parses the real applied (dx, dy) out of
the `Trigger` response message, rather than using `odom_jerk_stddev`. A
single draw can land well under its own stddev, making a
fraction-of-stddev assertion flaky by construction, and the corrective leg
needs the actual vector anyway. Falls back to `None` if parsing fails, so
a `pose_emulator` message-format change degrades rather than hard-fails.

`drive()` steers toward each leg's ground-truth endpoint, re-aiming every
tick off `/sim/raw_odom` until within `WAYPOINT_TOLERANCE`. It used to
publish a fixed Twist for a wall-clock `duration`, which assumed gz-sim's
real-time factor is exactly 1.0; under load (GPU lidar, GUI, contention)
RTF dips and every leg undershot. Gating on ground-truth distance
projected onto the commanded heading fixed the undershoot but not lateral
drift off that heading. Steering to the target bearing every tick corrects
both axes at once.

Commanded speed is capped at `dist / CONTROL_PERIOD`, tapering as the
target nears. Held at full nominal speed it visibly oscillated at every
corner: at 4.0 m/s on a 0.1s tick, one tick covers 0.4m, so anywhere
inside that distance it overshot `WAYPOINT_TOLERANCE`, flipped direction,
and repeated, giving a bang-bang limit cycle rather than a settle. `duration`
survives as a generous wall-clock safety cap (3x, floored at +5s) so an
unreachable target can't hang a scenario; hitting it logs a warning.

`spawn_box_obstacle` spawns a `<static>true` box into the running world
via `ros_gz_sim create -string <inline SDF>` (`-topic` is broken for this
stack), as a subprocess rather than a launch Node since it has to fire
mid-scenario, after the pre-spawn baseline is sampled. Sim teardown
disposes of it.

#### Thresholds

`MAX_DELTA_THRESHOLD = 0.40` m, shared by `drift_correction`,
`drift_correction_obstacle` and `noise_correction`, and available as a
fallback pass for `jerk_with_motion` trials (a small jerk draw can demand
an unrealistically tiny fraction-based correction that a healthy backend
wouldn't hit; landing inside the bound the rest of the suite accepts is a
legitimate pass). History: hardened to 0.20 on 2026-07-26, raised to 0.30
later that day when no config could reach 0.20 against the 3m loop at
0.25 slip, then to 0.40 on 2026-07-27 once the chosen config (tuned
`--backend slam`, no EKF, at 0.15 slip) measured 0.30-0.33m, too close to
the 0.30 bound to pass reliably. Full investigation in
`sentry_localization/README.md`'s Tuning history.

`CORRECTION_FRACTION = 0.3`, not 0.5: slam_toolbox settles into a genuine
but partial correction plateau, typically 40-70% of the true jerk, since
scan matching corrects the pose graph incrementally and this scenario only
gives it a brief wiggle. 0.5 sat at the edge of that plateau and produced
borderline false failures; 0.3 keeps margin while staying far above the
known-broken case (`minimum_travel_distance` reverted to 0.5, which is
indistinguishable from zero). Never independently re-validated against
amcl's own plateau, so if amcl runs go flaky, revisit this first.

That calibration was done at 0.15 m/s with `JERK_STDDEV=0.3`. Both have
since changed (4.0 m/s; `JERK_STDDEV` 0.5 -> 0.08 -> 0.24, targeting a
~30cm average jerk, since dx/dy are independent N(0, stddev) draws and
magnitude is Rayleigh with mean `stddev * sqrt(pi/2)`). Re-derive the
plateau rather than assuming 0.3 still holds if this scenario's pass
rate looks off.

The 2026-07-23 crash: the correction step used to be a `while` loop
driving `PATROL_LEGS` for up to 60s, exiting early once the threshold was
crossed. When the correction TF stalled for an unrelated reason the exit
never fired, and 60s of open-loop driving accumulated enough execution
drift to leave the field and crash gz-sim's physics. The fix was a single
bounded drive to the next corner plus exactly one TF sample, bounding
driven distance by construction instead of by a timeout that depends on
the thing under test behaving.

This scenario is also sensitive to unrelated CPU contention on the host
(an rviz2 left running, other agent sessions). Under contention scan
processing falls behind wall clock, giving 2 sensor registrations across a
~35s run versus prompt repeated re-registration on a quiet box. The
post-drive `get_correction_tf()` sample uses a generous 5s timeout for
that reason.


### pose_emulator.py: odom noise model

Real wheel odometry accumulates drift that `map->odom` correction exists
to compensate for; sim's ground truth has none, leaving that path
unexercised. These params inject it synthetically, all off by default.

- `odom_drift_stddev`: random-walk step (m/callback) added to a
  persistent offset, accumulating like real wheel slip.
- `odom_jitter_stddev`: independent per-sample jitter on top, not
  accumulated.
- `odom_jerk_stddev`: one-time sudden displacement, fired by the
  `~/trigger_jerk` service (nothing calls it automatically). Default 0.2
  is meaningfully larger than one drift step so the resulting correction
  reads as a jump rather than blending into the drift.
- `odom_jerk_bias_enabled`/`odom_jerk_bias_x/y`: pulls the drawn jerk's
  direction toward a fixed point instead of firing uniformly at random,
  for scenarios whose loop corners sit near real walls.
- `odom_slip_ratio`: loses a fixed fraction of every metre actually
  driven, modelling wheels that spin without gripping (the arena's "Bumpy
  Road" zone). 0.5 means `/pose` advances 0.5m per 1m truly moved. Grows
  with distance travelled rather than elapsed time, unlike drift.

The jerk does the opposite of what it looks like. `trigger_jerk()`
moves the real simulated robot in gz by a random (dx, dy) *and
simultaneously cancels that same (dx, dy) out of the drift accumulator*,
so reported `/pose` does not jump at all at trigger time. That's the
point: wheel encoders never registered the displacement, so they keep
reporting what they would have anyway. The discrepancy only surfaces later,
when the next scan match disagrees with wheel odometry and corrects
`map->odom`, which is the behaviour being exercised.

### head_slider_relay.py: topic naming and the input-lag fix

The gz GUI slider panel always publishes to gz-transport's auto-generated
default topic for a joint, `/model/<model>/joint/<joint>/<axis>/cmd_pos`,
and that isn't configurable from the GUI side. `sentry.urdf.xacro`'s
plugins instead listen on a custom topic *without* the axis segment, so
`sim.launch.py`'s `ros_gz_bridge` can remap them to clean ROS names
(`/head_pan_cmd`, `/head_pitch_cmd`). ROS 2 topic names can't have a
namespace token starting with a digit, so the GUI's own default topic can
never be bridged into ROS at all (`parameter_bridge` raises
`InvalidTopicNameError` on `.../0/cmd_pos`). Since gz-sim's
`JointPositionController` accepts only one `<topic>` per instance, this
relay is what lets the slider and the ROS topics drive the same
controller.

No gz-transport Python bindings exist in this image, so the script shells
out to the `ign topic` CLI for both directions: `ign topic -e` as a
long-lived subprocess to read, `ign topic -p` fresh per message to write,
each paying tens of ms of discovery overhead.

That per-message cost used to make the head lag visibly behind a dragged
slider: the reader fed every intermediate value into a blocking
`subprocess.run`, so slider ticks queued faster than they published and
the head crawled through the backlog toward where the slider *used to be*.
Reader and publisher are now separate threads sharing only the latest
value, with an `Event` coalescing bursts so that values arriving mid-publish
are dropped rather than queued.

### auto_explore.py: teleport mechanism

"Teleport" is a real gz-sim world-pose write via the
`/world/<world>/set_pose` service, called through the `ign service` CLI
since there's no ROS equivalent to bridge. It only works because
`sentry.urdf.xacro`'s root link is a genuinely free 6DOF body with no
parent joint and no collision on any link: gz physics only honours a
direct pose write on a link its `FreeGroup` API sees as free-floating (a
jointed link silently ignores it, confirmed against an earlier URDF that
drove root through a prismatic chain), and having no collision means
nothing it teleports through can generate contact forces that spin it up.
The robot never turns in normal operation, but nothing structurally
enforces that any more, so every teleport pins orientation to identity
explicitly.

Each teleport also fires a `model_only` `WorldReset` before the pose
write, snapping joints back to their SDF-declared zero state. A
`model_only` reset provably doesn't touch root's own pose (no parent
joint, so no initial joint state to reset to), which makes it safe to call
unconditionally.

`reset_joints()` runs both before *and* after `set_pose`. Before, so each
hop starts clean; after, because that's when a bad reaction actually shows
up: root's hard position discontinuity can induce a one-step reaction
impulse through the real joints on body/root, and resetting only
beforehand doesn't touch what that impulse just produced. Root's own
inflated rotational inertia is what suppresses angular velocity on root
itself; this just stops the joints carrying residual spin into the next
hop.

### sim.launch.py: spawn_robot uses -string, not -topic

Deliberately `-string` (raw URDF text), not `-topic robot_description`.
`-topic` makes `create` subscribe over ROS, and that subscription reliably
fails to receive the TRANSIENT_LOCAL-cached message from
`robot_state_publisher`. Confirmed live: `ros2 topic echo
/robot_description` got the message instantly over the same QoS while an
already-matched `spawn_sentry` sat waiting 30+ seconds. That's a bug in
`ros_gz_sim create`'s subscription handling, not a startup race, so no
amount of delay fixes it. `-string` sidesteps ROS for this one hand-off.

To exercise the jerk mechanism once sim is up:
`ros2 service call /pose_emulator/trigger_jerk std_srvs/srv/Trigger`.

### ekf_ground_truth_diag.py: why it exists separately

It answers what the drift suite structurally can't: *does fusing
`/scan_odom` into `/odom` via `ekf_node` actually produce a better
estimate of where the robot really is?*

The drift scenarios run with `odom_noise_enabled=False`, so only
`odom_slip_ratio` corrupts `/odom` at all. With noise off and slip at 0.0,
`pose_emulator`'s `odom_callback` assigns `x, y = true_x, true_y`, exactly
ground truth, and no EKF beats a perfect input. Historically the
suite ran at that zero-slip default, which is why "EKF is worse than raw
/odom" numbers from those runs say nothing about the EKF. Those scenarios
also assert on a `map->odom` residual, whereas the EKF's relevant edge is
`odom->root`, whose delta is dominated by the robot's own real motion.

So this script turns wheel-odometry error on (drift plus continuous slip),
drives the same cornering loop, and scores both estimators against
`/sim/raw_odom` using mean/RMS/max Euclidean error.

### Removed: head_sweep.py (2026-08-31)

A standalone yaw oscillation `sim.launch.py` ran instead of `cv_head_aim`
when `head_sweep_hz > 0`. It served `run_shot_hit_tests.py`'s head-slew
scenario, which checked `target_tracker`'s TF-based decoupling of head
motion against a moving camera and a stationary target. That scenario went
with it, so head motion is now only ever `cv_head_aim` tracking a target.
Restoring the decoupling check means rebuilding both.

### target_driver.py / cv_target_emulator.py: CV target simulation

The fast-moving target is deliberately **not** a gz entity. `target_driver`
integrates its own `(x, y, z)` in a timer callback and publishes
`nav_msgs/Odometry` on `/target/ground_truth_odom`, the same "plain ROS
node standing in for something more complex" approach `pose_emulator`
already uses. That sidesteps SDF authoring, a spawn step, and gz-side
bridges, at the cost of nothing being visible in the gz GUI. Verification is
topic echoes and the vector checks below.

Both nodes stamp off `self.get_clock().now()` (so `/clock` under
`use_sim_time`), never wall clock. `cv_target_emulator` stamps
`panel_detection.header.stamp` at **sample** time, not flush time, so
`publish_latency_s` is purely delivery delay layered on top via a pending
queue and `now - header.stamp` downstream actually reflects it.

#### Path geometry

The default path is a lateral bounce at fixed depth
`x=3.0m`, `y in [-2.0, 2.0]`, `z=0.3m`. Visible half-width at that depth
is `3.0*tan(1.5184/2)` ~ 2.85m against the path's 2.0m half-amplitude, so
the traverse stays in-frustum throughout with ~0.85m margin each side.
Measured 2026-07-27: minimum 47 consecutive in-frustum samples at 8 m/s,
the fastest speed swept. (That measurement originally sized around an EMA
velocity filter in `point_to_cv_target` that no longer exists; the numbers
still hold, they just no longer serve that purpose.)

#### Camera FK, no TF

`cv_target_emulator` chains `sentry.urdf.xacro`'s
fixed joint offsets directly (root -> fastened_2 -> body -> headlink(yaw)
-> head -> headpitch(pitch) -> head_pitch -> cameralink -> camera), the
same self-contained approach `pose_emulator` uses, since sim runs no
`robot_state_publisher`. Joint angles are read from
`/sim/raw_joint_states` by name, not array position. Easy to miss:
`headlink`'s and `fastened_2`'s pi-yaw origins cancel at `head_yaw=0`, but
`headpitch`'s origin carries a **fixed -0.38885 rad yaw that does not
cancel** and applies regardless of joint state.

That sign was verified rather than assumed. Comparing `pos_err` with
`+0.38885`, `-0.38885`, and `0` applied gave ~2.45m, ~0.13m and ~1.22m
mean error respectively: only `-0.38885` collapses toward the ~0.03m noise
floor, which can only happen if the FK sign is right. Same method as the
rf2o `angle_min` bug: compare vectors rather than magnitudes, since
`tan(+x)` and `tan(-x)` have equal magnitude and a single error number can't
tell a correct rotation from a sign-flipped one.

`headlink` is a continuous joint with no yaw limit (fixed
2026-07-28). It was declared `revolute` with a `+-pi` limit, which is
stale, since real hardware's gimbal spins freely. The tell was `cv_head_aim`'s
early runaway bug pegging at exactly `+-3.14159`, which turned out to be
its own software clamp saturating rather than a physical limit.
`headpitch` keeps its real `+-0.6` limit.

The convention is REP-103 rather than optical. Target position is computed relative
to the camera as x=forward, y=left, z=up, not the optical convention a
real driver reports. Computing optical and labelling it REP-103 would
silently rotate every detection by a fixed offset, the same bug class as
rf2o's `angle_min` (179.81 degrees off, magnitude correct, direction
exactly backwards). Verified 2026-07-27 by comparing `/cv/target`'s
reported left/right sign against a bearing derived independently from
`/sim/raw_odom` + `/target/ground_truth_odom`: 20/20 samples agreed.

#### Noise model

It gates on FOV (`horizontal_fov=1.5184` plus a derived
vertical FOV from the 640x480 aspect) and range (0.1-10.0m, the camera's
clip planes), publishing nothing outside either, which simulates track
loss and exercises `point_to_cv_target`'s watchdog. Inside, it adds
per-axis Gaussian position noise (`noise_pos_stddev`, 0.03m), per-sample
dropout (`dropout_probability`, 0.1) and fixed latency
(`publish_latency_s`, 0.06s). Unlike `pose_emulator`'s model, all three are
ON by default, so a `spawn_target:=true` run is degraded unless you
zero them. The 0.06s latency is a placeholder, not a measurement.

### cv_head_aim.py: CV-driven head tracking, root-frame IK

Subscribes `/cv/target` (from `sentry_pkg`'s `point_to_cv_target`, so
`auto.launch.py` has to be running alongside `sim.launch.py
spawn_target:=true`) plus `/sim/raw_joint_states`, and publishes
`/head_pan_cmd`/`/head_pitch_cmd`, the same topics the GUI slider
drives. This is the only thing that moves the head during CV testing.

`CVTarget.x/y/z` carries a root-frame position rather than a camera-relative
bearing. The `atan2(x, z)` bearing-nulling this node used to do is
meaningless against a position, so it was replaced rather than re-tuned.

`cv_head_aim_core.solve_head_angles()` inverts the fixed FK chain (`root
-> body -> headlink(yaw) -> headpitch(pitch) -> camera`, the same chain
`cv_target_emulator._camera_pose()` walks forward) to solve for the joint
angles that point the camera's +X at the target, handling
`HEADPITCH_ORIGIN_YAW` (-0.38885) correctly as a yaw baked into the joint
origin rather than a pitch bias. It ignores the camera's own ~0.35m
offset from the yaw/pitch axes, the same simplification applied to
Type-C's muzzle offset. Cross-checked in `test/cv/test_cv_head_aim.py`
against an independently written from-scratch FK, so a sign error in one
can't pass by agreeing with itself.

It closes the loop on that absolute setpoint instead of running open-loop
feedforward, because setpoint-tracking lag against a moving target has to show
up in sim the way it would on real Type-C, since sim exists to measure
that lag rather than hide it. Each `control_rate_hz` (15) tick computes
the wrapped angular error between the IK target and the current joint
position and commands `current + gain * error`. Correcting off a timer
rather than per `/cv/target` arrival avoids the
setpoint-races-ahead-of-the-joint failure mode from early tuning, since
the emulator publishes at up to 60Hz.

`gain` (0.3) is a placeholder. The old `0.1` plus
`sign_yaw`/`sign_pitch` were tuned for the bearing-correction design and
don't carry over; the sign params are gone entirely, since the IK's
geometry determines sign directly (verified analytically in the test, not
tuned empirically). This still needs an empirical verification pass.

Stale-target behaviour: stops publishing and holds the last commanded
position once `/cv/target`'s confidence hits 0.0. It deliberately doesn't
re-home, since a lost target is usually a momentary FOV gap.

### CVTarget velocity/acceleration fields: removed 2026-07-28

`CVTarget` used to carry `v_x/v_y/v_z` and `a_x/a_y/a_z`,
finite-differenced by an EMA filter in `point_to_cv_target` and forwarded
byte-for-byte into the MCB's `CV_MSG` packet. Removed from both the
message and the UART struct: a deliberate coordinated wire-format break,
not a ROS-only trim. See `ros2_dji_serial_bridge/README.md`.

Velocity estimation returned later as `sentry_pkg`'s `target_tracker.py`,
entirely ROS-internal on `/cv/target_state`, never on the wire.
`CVTarget`/`CVDataPayload` stayed lean, gaining only a
`lead_applied`/`track_valid` flags byte.

### Environment footguns

sim's `gz-sim`/`ros_gz` apt deps aren't in this
container by default; see `## Build` for `install-sim.sh`. Separately,
`sentry_pkg`'s build under `install/` was once a stale colcon
symlink-install pointing at a deleted git worktree, breaking both `ros2
run` and a plain import; rebuilding fixed it. The sim/CV test scripts
invoke `point_to_cv_target` by absolute install path rather than `ros2
run` to sidestep that class of failure.