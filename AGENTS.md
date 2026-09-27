# sim: agent notes

gz-sim simulation of the `ARCC_Field_2026` world plus the spawned `sentry`
robot (`urdf/sentry_v2.urdf.xacro` by default), and the home of the localization integration suite. **Reference docs live
in `README.md`**, in particular its `## Notes` section, which holds the
drift-suite design history and per-scenario pass conditions. This file is only
the operating contract for working here.

`README.md` commands are written for a human in a container terminal. You run
them from the host through `../isaac_ros_common/scripts/dexec.sh` (load the
`isaac-ros-docker` skill first); the equivalents are below.

**On a fresh or recreated container, run `install-sim.sh` before the first sim
launch.** `Dockerfile.thornbots` deliberately installs neither
`ros_gz` nor this package (real hardware never needs gz-sim):

```bash
../isaac_ros_common/scripts/dexec.sh -r -- \
  src/isaac_ros_common/docker/scripts/install-sim.sh
```

**`sim` is the one package with no `/workspaces/ros2_ws` shadow copy.** It isn't
copied in during the Docker build, so `src/sim` edits are live immediately,
including from the user's terminal. Don't apply the `ros2_ws` shadowing
workaround here; it applies to every _other_ first-party package.

## Testing

Everything under `test/` is pytest, collected by `colcon test`. The suites
that launch `sim` + `thornbots_pkg` end to end carry the `integration` marker and
are deselected by `setup.cfg`, so a plain `colcon test --packages-select sim`
runs the unit tests only. `sim` is `ament_cmake` now, so `--pytest-args`
doesn't reach it; run pytest directly for the integration tier:

```bash
../isaac_ros_common/scripts/dexec.sh -- colcon test --packages-select sim
../isaac_ros_common/scripts/dexec.sh -- bash -c 'cd src/sim && python3 -m pytest test -m integration'
```

A new Python node needs a `scripts/<name>` wrapper (copy one) and a rebuild;
edits to existing modules are live through the symlink install.

Both suites run from a launch file, so `kill_launch.sh <pid>` on the outer
launch stops everything, per-scenario stacks included (`--show-args` lists
each one's args):

```bash
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch sim localization_tests.launch.py
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch sim shot_hit.launch.py
```

Both time everything in sim seconds. The drift suite defaults to
`real_time_factor:=0`, as fast as the machine allows: gz can't step past
its own physics, and the gz-free aiming bench's `sim_clock` waits for each
stage's output. `real_time_factor:=1` is the control when a result looks
off.

Every scenario failing "stack NOT ready" means the container has the old
discovery-server DDS profile; see the `isaac-ros-docker` skill. The
target configs are `--backend amcl --use-ekf` for drift and the defaults for
shot-hit; `README.md`'s "Running the current target tests" lists the commands.

`--backend` is `slam`, `amcl`, or `none` (who owns `map->odom`). `--use-ekf` is
a separate axis and layers EKF fusion of `odom->root` on top of any of them;
there is no `ekf` backend. It is on by default, matching `auto.launch.py`;
`--no-use-ekf` is the way back to raw `/odom` passthrough.

`test/localization/` is `test_localization_drift.py` (one test per scenario) and
`test_ekf_ground_truth.py`, both over `drift_harness.py`/`ekf_diag_harness.py`.
`test/cv/` is `test_shot_hit.py` (integration, one test per layout/speed case,
over `shot_hit_harness.py`) and `test_cv_head_aim.py` (plain pytest, no stack
needed). Pass `-s` when running pytest directly, or the measured numbers these
suites print get captured.

Every integration run ends with a `suite timing` table (`sim/suite_timing.py`):
wall seconds per case split into sim start, bring-up, reset, settle, scored
and teardown, plus the RTF over the spans with a sim clock. Harnesses wrap
their overhead in `suite_timing.phase()`; anything else in a case counts as
scored. Read it before and after any speed change.

**Check every run before trusting it** (the user's rule, 2026-09-27): a
bench can pass with rviz dead or a stack node crashed.
`tools/check_bench_log.py` on the `dexec.sh -d` log, plus
`/tmp/localization_drift_tests/*.log` for drift and EKF, prints the table
and exits 1 on a mid-run crash, a missing display or a wait that gave up.
`dexec.sh` doesn't forward `DISPLAY`; if the container has none, prefix the
launch with `env DISPLAY=:2` (`ls /tmp/.X11-unix`), or pytest's preflight
stops a GUI run at once.

Before launching anything, check for a live session:

```bash
ps aux | grep -E 'gz sim|slam_toolbox|amcl|ros2 launch'
```

A colliding stack silently corrupts measurements. If something is running, ask
before killing it; it may be the user's own work. That grep detects a live
session only. To get a PID to kill, use `kill_launch.sh -l`, since the grep also
matches `dexec.sh`'s own bash wrapper. Clean up anything _you_ started, in a
`finally` block.

## Standing rules

- **gz is the only engine. We are not switching to SAPIEN**; it was tried and
  removed (README.md).
- **Ask before starting any sim test run**, the aiming bench or the drift
  suite, even when the next run seems the obvious step. The user may have
  tuning to do first.
- **GUI on, not headless**, for both `sim` and the drift suite; the user watches
  the gz-sim window during testing. Pass `--headless` only when asked (e.g. a
  quick unattended run). Launch through `dexec.sh -d`, which execs as `admin`,
  never a bare `docker exec -d`, or the window fails to open; see the
  `isaac-ros-docker` skill's "Launching GUI apps".
- **Always fully restart `sim` (fresh spawn) before restarting SLAM/explorer.**
  Partial restarts leave stale TF/pose state ("pos desync"). The drift
  suite's shared sim is the one exception: `drift_harness._reset_sim`
  teleports to spawn, removes spawned models and resets `pose_emulator`
  before each robot stack starts, and sim time only runs forward.
- **If an edit under `sim/` doesn't take effect** for a `ros2 run` or
  `ros2 launch` node, suspect `install/sim` losing its `--symlink-install`
  linkage (stale copies instead of symlinks) before assuming the edit is wrong.
  Fix with `rm -rf build/sim install/sim`, then
  `colcon build --packages-select sim --symlink-install`. The `test/` suites run
  against `src/` directly and are unaffected, which makes this easy to misread.

## Scope

- Owns the world, the sim URDF, `pose_emulator.py`'s noise model, and the
  localization test suite.
- Localization backends belong to `../sentry_localization`; hardware interface
  and CV target selection to `../thornbots_pkg`.

## Open

- **Sim speed: the full stack caps at RTF ~1.55, cause unknown.** Idle
  `sim.launch.py` sits there with GUI or headless, rviz or not, and with the
  field collision simplified, so neither physics nor rendering sets it.
  Bisecting the stack's nodes and bridges next. Known costs: the field
  mesh's collision (its sub-3 cm floor triangles under the wheels) is
  ~0.4 ms of each 1 ms step, and a subscribed 60 Hz RGB-D camera caps a
  bare server near RTF 2.2 (gz skips rendering it with no subscriber).
  Many separate box collisions cost more than the mesh: ~2 us/shape/step.
- **Unthrottled runs score like real time now**, localization included, with
  rf2o's `fixed_heading` and `/odom` prior. Run at the default
  `real_time_factor:=0`; keep `:=1` as the control. rf2o now matches every
  scan in its callback (`thornbots_workspace#11`); its `ros2_ws` copy is
  shadowed, so rebuild it in `isaac_ros-dev` before trusting a run.
- **`odom_stuck` passes but localization is lost, and that is accepted**
  (the user's call, 2026-09-25). Its check stays liveness only. rf2o's
  `/odom` seed freezes with `/odom`, and 0.4 m between 10 Hz scans at
  4 m/s is too far to match unseeded, so `odom->root` stays inside ~1 m
  and amcl rotates its estimate (up to 0.66 rad) to fit the scan.
  README.md's scenario list has the details.
- **A robot bring-up can stall for good on a lost lifecycle reply.** A
  node's reply to `change_state` times out in DDS (`failed to send
  response`), Nav2's lifecycle manager waits on it with no timeout, and
  nothing activates. Seen on `amcl` and `map_server`, on Humble and Jazzy.
  The drift harness waits for `parent->root` after each robot start and
  restarts the stack once if it never comes (`_wait_for_root_chain`). The
  robot can hit the same race at boot; unfixed there.
- **The drift suite passes 7/7 on `sentry_v2` at `--backend amcl --use-ekf`,**
  unthrottled with the A2M8 lidar and per-scan rf2o, GUI on, 227 s
  (2026-09-26: drift_correction 0.16 m, with obstacle 0.15 m,
  moving_obstacles 0.17 m, against 0.40 m).
  Anything spawned into the world must clear the robot, which now collides:
  a box spawned inside the chassis stalls gz's contact solver and `/clock`.
- **`sentry_v2` spawns by default; `model:=sentry` is the old model.** The
  TF tree (`thornbots_pkg`'s URDF) and the CV chain (`cv_head_aim_core`,
  `cv_target_emulator`, `shot_hit_harness`) moved to it too. Pitch limits
  (+-0.6 rad), suspension travel, spring rate and damping are placeholders,
  not CAD values.
- **The estimation bench (`estimation.launch.py`) runs on `bench_world`, not gz** (the
  user's call, 2026-09-25): it fakes detections, so it needs no physics.
  One C++ loop steps target, chassis and head every 1 ms, so its rates are
  exact at any speed, and holds sim time for the nodes under test, up to
  `pace_slack_s` past each period: `target_tracker`'s input
  (`/cv/tracker/measurement`, plus the detections' delivery delay),
  `/cv/target`, and the scorer's `/bench/progress`. `/cv/target_state`
  can't be the gate: it is stamped at publish time, and pacing on it let
  the tracker fall 0.12-0.21 s behind capture (2026-09-26). The tracker
  gate waits only on frames that carried a detection, and a gate with no
  publisher left stops holding at once. The ten cells take ~53 s with rviz,
  90% of it scoring at ~6.5x; `target_tracker` is the ceiling. Root
  sits at z 0 and yaw 0, and the head is gz's PD on the arm inertias.
  `LIMITS` (`test/cv/estimation_limits_data.py`, generated by
  `tools/estimation_limits.py`) covers all 60 cells at 2x the worst Jazzy
  run; about a quarter of runs trip one limit on a spin-rate or radius
  outlier.
- **`bench_world` duplicates the estimation bench's share of `target_driver`,
  `cv_target_emulator`, `cv_head_aim` and `pose_emulator`**, which the gz
  sim and the aiming bench still use. A change to one of those that should
  reach the estimation bench has to be made in `src/bench_world.cpp` too.
- **`/cv/target` is an `odom` point since 2026-09-27.** `cv_head_aim.py`,
  `bench_world.cpp` and `shot_hit_harness.py` aim from our current pose, as
  the MCB should; `../CV_SPLIT_PLAN.md` W.3.
- **The head controller holds the head when there's no target**, so a case can
  start with the target out of view. `estimation_harness` aims the head at
  the truth during each case's reset; before that, staggered stationary
  after 4 m/s scored nothing.
- **Every CV test runs with ROS** (the user's rule, 2026-09-25): tune and
  score Part 2 on the estimation bench, never on a copy of the nodes outside ROS. The
  offline estimator (`tools/estimation_offline.py`) was removed for that:
  it left out the head slewing and its tracker defaults drifted from the
  node's. Pure-numpy unit tests of the `*_core.py` modules stay.
- **`cv_target_emulator` adds D435-like ray noise** (depth 0.0036 r^2,
  bearing 0.003 r) on top of the 5 mm. Datasheet estimates, not measured.
- **Launch long runs with `dexec.sh -d`, not a foreground `dexec.sh`.** On
  2026-09-25 a foreground queue lost its host shell: each queued `ros2
  launch` died at once and left its nodes orphaned (parent 1, invisible to
  `kill_launch.sh -l`), nine stacks all publishing `/clock`. Before a run,
  check `ps -eo pid,ppid,cmd | grep install/` for such orphans too.
- **Panels are canted 15 deg in the game (S122), and not everywhere here.**
  The emulator, the scorer's facing test and both rviz views keep the cant;
  a hit is still scored as distance to the panel centre, not a crossing of
  the canted square (`../ROADMAP.md` Caveats).
- **The estimation bench's target is a phantom** with exact truth. E2E adds a gz model with
  armor plates for `roi_depth_node`'s depth; no sim test runs YOLO
  (`../E2E_PLAN.md`).
- **`sentry_v2`'s chassis picks up ~1 deg of yaw** in the first hard
  corners at 4 m/s and keeps it: the head's reaction torque gets past the
  yaw lock. The real robot is expected to drift 1-5 deg too. Noted, not
  acted on; the stack assumes a fixed heading for now.
- **The lidar's bottom guard sits on the chassis in `sentry_v2`.** It is the
  export's grounded part and isn't mated to the head, so the chassis hull
  reaches 0.362 m. Harmless for scans; fix it in Onshape (mate it) or in
  `sentry_v2.yaml`'s `drop` list.
- **`moving_obstacles` (ROADMAP A4) runs and passes.** Its boxes have no
  collision (the lidar sees visuals; box-on-field-mesh contact cost ~3x
  sim speed), so they never touch the robot and `actor_driver` keeps them
  >= 1 m from it along the loop's route. Seen crossing the loop in gz. The
  `slam` occupancy-grid check is still a TODO in
  `_run_cornering_loop_scenario`.
- **`ros_gz_sim create` ignores the SDF `<pose>`**; pass `-x/-y/-z` or the
  model lands at the origin. `spawn_box_obstacle` buried half its box this way
  until 2026-09-24.
- **Wanted: a localization scenario with finite acceleration.** `drive()`
  steps `/cmd_vel` to 4 m/s and stops within one 0.1 s tick.
- **The EKF beats raw `/odom` by 95%** (`suite:=ekf`, unthrottled,
  2026-09-26: 0.0075 m fused mean against 0.1415 m), since rf2o got
  `fixed_heading` and an `/odom` prior (`../sentry_localization`).
  Both were needed: without the prior rf2o undershot legs that start from
  rest, whatever the blind sector. rf2o's sign is right; don't invert its
  warping again.
- **Under `--backend none` the drift scenarios score ground-truth error**
  (`_truth_error`, ROADMAP A3), since `odom->root` there is the robot's
  own motion. Built 2026-09-25, not yet run.
- **`noise_correction`'s growth_ratio compares the two halves of a run**, so
  a run that starts clean fails hardest. Don't read its verdict as absolute
  accuracy.
- **The aiming bench (`shot_hit.launch.py`) is gz-free and passes 10/10**
  on every path and at `shooter_speed:=1.0`. A point shooter with a perfect
  gimbal; README.md has the setup. `FLOORS` holds 40 cells from three runs
  each on sentry_v2's armor panels (2026-09-27, chase, unthrottled at ~8x),
  through `tools/shot_floors.py`. Radial or
  diagonal with a moving shooter has no floor yet.
- **Shot-hit results are optimistic.** Detection noise (0.005 m) is far
  cleaner than a D435, and slew limits and target accelerations are
  estimates.
- **`lidar_self_filter`'s sector comes from the CAD**, not a scan. Check it
  against a hardware `/scan_raw` (`../thornbots_pkg/AGENTS.md`).
- **Keep the physics step at 1 ms.** 2-4 ms sent the head's PID unstable.
- **`sentry_v2` collides, and `VelocityControl` drives it anyway.** It sets
  the chassis's whole twist every step (planar velocity from `/cmd_vel`,
  zero angular, zero vertical), so a wall hit can't flip or spin it. What
  it does when driven into a wall hasn't been checked. `root` still has no parent joint, so `set_pose` teleports work.
  The old model has no collision and drives through everything.
- **Never validate odometry on magnitude alone.** Compare displacement
  vectors; a backwards rf2o once scored 1% error on magnitude.
- ARCC zone coordinates (`../ARCC_2026_SENTRY_CONTEXT.md` Figures 3-1 to
  3-9) aren't in the world yet.
- **`sim` runs on gz Harmonic (gz-sim 8) since the Jazzy move.** Plugins are
  `gz-sim-*-system`, and every CLI call is `gz topic`/`gz service` with
  `gz.msgs.*` types; Harmonic ships no `ign`. On the laptop (2026-09-26)
  the drift suite, `suite:=ekf`, the aiming and estimation benches give Humble's results.
- **robot_localization 3.8 logs "Failed to meet update rate!" at ERROR**
  (Humble's printed it untagged), unthrottled, with no effect on the pose.
  `scan_log_for_errors` skips that line.
- **`trimesh` isn't in `package.xml`**: noble has no apt `python3-trimesh`
  and rosdep only a pip key. `install-sim.sh` pips it; only
  `tools/simplify_urdf.py` uses it, and regenerating `urdf/sentry_v2` also
  needs `fast_simplification`, `scipy==1.13.1` and `networkx==3.3`, pipped
  by hand with `--no-deps` so the system numpy survives. The export
  (`/home/tmp/sentry_export` on the laptop) isn't mounted in the container;
  copy it under the workspace root for the run and delete it after.
- **`sentry*.urdf.xacro`'s `<gz_frame_id>` warns "not defined in SDF"**
  under Harmonic's sdformat, but gz-sensors still reads it (`/scan_raw`
  arrives with `frame_id: lidar`).

## Committing

This package is a submodule of `thornbots_workspace`, on branch `main`. Commit
and push here first, then bump this gitlink in `../` — one logical change, one
bump, never a gitlink pointing at an unpushed commit. Full rule in
`../CLAUDE.md` § Packages.
