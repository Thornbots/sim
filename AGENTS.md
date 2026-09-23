# sim: agent notes

gz-sim simulation of the `ARCC_Field_2026` world plus the spawned `sentry`
robot, and the home of the localization integration suite. **Reference docs live
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
runs the unit tests only:

```bash
../isaac_ros_common/scripts/dexec.sh -- colcon test --packages-select sim
../isaac_ros_common/scripts/dexec.sh -- \
  colcon test --packages-select sim --pytest-args ' -m integration'
```

Both suites run from a launch file, so `kill_launch.sh <pid>` on the outer
launch stops everything, per-scenario stacks included (`--show-args` lists
each one's args):

```bash
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch sim localization_tests.launch.py
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch sim shot_hit.launch.py
```

Both default to `real_time_factor:=0` (unthrottled) and time everything in sim
seconds; `real_time_factor:=1` is the control when a result looks off.

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

- **Ask before starting any sim test run**, the shot-hit bench or the drift
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

- **The drift suite shares one sim across scenarios (2026-09-23), not yet
  run.** pytest starts `part:=sim` once; each scenario resets it
  (`_reset_sim`) and launches `part:=robot`. `--restart-sim` /
  `restart_sim:=true` is the old fresh-sim-per-scenario path. Done when
  all six scenarios run against one gz process and each gives the same
  verdict alone, sixth, and under `restart_sim:=true`. Watch for state
  the reset misses: the head joints (`reset_joints` zeroes them), rviz
  holding old TF, and `amcl`'s initial pose (0, 0) against the teleport.
- **Wanted: move the sim to the new 3D model, `urdf/sentry_v2`.** It is
  generated by `tools/simplify_urdf.py` (README.md) and nothing uses it
  yet; `sentry.urdf.xacro` is still what spawns. Unlike the old model it
  has collision shapes, which reopens the free-floating `root` question
  below. Its pitch limits, suspension and spring values are placeholders.

- **sapien vs gz on the drift suite (2026-09-23).** Same amcl/EKF settings,
  unthrottled, after the lidar visibility fix. Five of six scenarios agree:
  `baseline`, `jerk_with_motion`, `odom_stuck` pass on both;
  `drift_correction` (gz 4.02 m, sapien 3.39 m) and
  `drift_correction_obstacle` (gz 3.0-3.5 m, sapien 5.5-8.7 m) fail on
  both, as they did before any of this.
- **`noise_correction`'s growth_ratio cannot tell the engines apart
  (2026-09-23).** Over three runs each it read 1.40, 1.47, 3.00 on gz and
  2.69, 3.31, 4.26 on sapien, against a 2.0 threshold -- gz passes twice
  and fails once, sapien fails three times, and the latest pass of each
  gives the same verdict (3 pass, 3 fail). It is a ratio of
  second_half_max to first_half_max, so a run that starts clean fails
  hardest: sapien's worst ratio (4.26) came from its *lowest* absolute
  error (max|xy| 4.45 m, against gz's 9.62 m in the run scored 3.00).
  Do not read a pass or fail here as an engine property until the metric
  scores absolute error. Wall clock for the six scenarios: gz 106 and
  122 s, sapien 156 and 160 s.
  sapien's `spawn_box` was verified by hand: 87 beams shortened to
  2.09-2.38 m against an expected 2.12 m, at the right bearing.
- **The lidar stopped scanning its own model (2026-09-23).** Every robot
  visual now carries `visibility_flags` against the sensor's
  `visibility_mask`, restoring the July approach that was reverted. Before
  it, the head blanked 118-180 deg of `/scan_raw` plus a rear sector that
  moved with the head's pose -- 863 to 1613 of 3000 beams as `-inf`, against
  `lidar_self_filter`'s 1.0 rad. A bare `gpu_lidar` in the same world returns
  all 3000 beams, so it was the robot, not the world or the sensor. Every
  drift and shot-hit number recorded before this date was measured on the
  blanked scan. `lidar_self_filter`'s sector was tuned to the self-hit
  cluster that no longer exists; it needs a hardware capture to retune.
  The drift suite (amcl, EKF on, unthrottled) run either side of the change
  gives the same verdicts -- 4 pass, 2 fail -- with smaller errors on the
  scan-driven scenarios: `noise_correction` max|xy| 7.32 -> 5.72 m,
  `drift_correction` 5.16 -> 4.02 m, `drift_correction_obstacle`
  3.87 -> 3.45 m. Both `drift_correction` failures predate the change.
- **Test results at `real_time_factor:=0` vs 1 (2026-09-21).** Both suites
  run 2-3x real time with the GUI. `drift_correction` (amcl, no EKF, 3 runs
  each) read 0.348 m mean unthrottled vs 0.338 m at 1x. The CV pipeline adds
  36 ms of sim-time latency unthrottled vs 26 ms at 1x, on top of the
  emulator's 60 ms. Rerun at 1x before trusting a small CV score change.
- **rf2o is not backwards; the EKF's remaining error is rf2o yaw drift
  (2026-09-23).** `suite:=ekf` now prints each source's per-leg
  displacement against truth as an angle and a length ratio. At
  `real_time_factor:=1`, 4 m/s: `/scan_odom` reads x0.94-1.00 on every
  leg but its direction drifts steadily, -1.3 deg to -5.1 deg over 45 s,
  on a chassis that never rotates. The EKF takes rf2o's x/y at 0.02^2, so
  it inherits the rotation: 0.205 m mean error against raw `/odom`'s
  0.169 m (-21%). At 1 m/s the same run gave the EKF +93%. The 2026-09-21
  "x runs backwards" reading and rf2o `73744a3`'s inverted warping (the
  inversion gave 49.6 m mean error; reverted) both came from the next item.
- **Unthrottled runs corrupt every rf2o/EKF number.** rf2o's main loop
  runs at `freq` (20 Hz) on the wall clock and keeps only the latest scan.
  At `real_time_factor:=0` the sim runs 2-3x real time, so rf2o skips
  scans and matches pairs 0.8-1.2 m apart at 4 m/s. Same code, same
  4 m/s loop: unthrottled `/scan_odom` read x0.40-0.77 on eastward legs and
  drifted -19 deg; at 1x it read x0.94-1.00 and -5 deg. Run anything with
  `use_ekf` at `real_time_factor:=1` until rf2o processes by scan stamp.
- **Shot-hit stationary cases are bimodal (2026-09-21).** Two runs swapped
  results: flat 98%/1% and staggered 2%/84%. The bad mode misses by
  0.149 m every time, the panel 15-20 cm right of the shot.
- **The aim bench (C1) is built, not run (2026-09-23).**
  `shot_hit.launch.py target_state:=truth` swaps `target_tracker` for
  `target_state_truth`, which publishes the true `TargetState` per
  `/cv/robot_panels` message. `TargetState` grew `z_offset` /
  `other_z_offset` for it (`dji_serial_bridge`); `point_to_cv_target`
  doesn't read them yet, so staggered cases on the aim bench still aim at
  one height. First run sets real floors for `MOVING_MIN_HIT_RATE`; run it
  at `real_time_factor:=1` too, since results are bimodal (below).
- **Shot-hit bench state (2026-09-21).** One stack per run, 10 cases (flat and
  staggered x stationary, 0.5, 1, 2, 4 m/s), 3s settle + 30s scored each, ~5.5
  min. Results and the stack's failures are in `../thornbots_pkg/AGENTS.md`.
  Caveats a reader of those numbers needs:
  - Detection noise is 0.005 m, far cleaner than a real D435, so rates run
    optimistic. The tracker still assumes ~3 cm (`meas_noise_base_m`).
  - `cv_head_aim`'s slew limits (10/6 rad/s) and `target_driver`'s
    `max_accel` (6 m/s^2) and `max_spin_accel` (20 rad/s^2) are estimates.
  - `MOVING_MIN_HIT_RATE` (0.25) is still a placeholder, now applied to the
    score, not the hit rate.
  - `panel_hits.jsonl` was only checked offline; no live run has written it.
- **Keep the physics step at 1 ms.** A 2 ms or 4 ms step (tried 2026-09-21)
  sent the head's PID unstable and threw the free-floating robot metres. With
  the camera bridges off (`camera:=false`, the default) the laptop holds real
  time at 1 ms anyway.
- **The chassis has zero collision geometry, deliberately.** No link carries any
  `<collision>`, so the robot drives straight through walls and through
  `drift_correction_obstacle`'s spawned box. This was the price of making `root`
  free-floating so `set_pose` teleporting works for `auto_explore.py`'s grid
  sweep. The box itself has real collision geometry, so lidar/SLAM still see it;
  only the body passes through. Undecided whether to add a collision-only proxy
  that doesn't feed torque back into `root`. Revisit if obstacle _avoidance_
  (not just mapping) becomes something to demonstrate.
- **The rotation lock is soft (inertia-based only).** A real wall collision can
  tumble the robot to extreme angles (~86° roll observed). The user chose to
  accept occasional flips rather than reintroduce a kinematic constraint.
  Revisit only if flips start blocking exploration in practice.
- **Slip now corrupts `/pose` velocity as well as position (2026-09-23).**
  `odom_slip_ratio` scales `vel_x`/`vel_y` by the same `(1 - ratio)`, as
  slipping encoders would. Every EKF number recorded before this was
  measured with true velocity under slip; the drift suite has not been
  rerun since.
- **`drift_correction`/`drift_correction_obstacle` need an ekf-appropriate
  metric.** Their `MAX_DELTA_THRESHOLD` is calibrated for `map->odom`'s
  residual-correction semantics. Under `--backend none`, where the watched edge
  is `odom->root`, the delta is mostly the robot's own motion around the loop,
  so both reliably FAIL without indicating a problem.
  `test/localization/test_ekf_ground_truth.py` scores against `/sim/raw_odom`
  with slip enabled and should probably become the assertion for
  `--backend none` (EKF on by default).
- **Never validate odometry on magnitude alone.** An early speed sweep compared
  `|displacement|` and scored rf2o healthy at ~1% error while it was pointing
  exactly backwards. Compare displacement _vectors_; the angle between them is
  what caught it. Applies to any odometry or detection source.
- ARCC Battlefield zone coordinates (Figures 3-1 through 3-9 in
  `../ARCC_2026_SENTRY_CONTEXT.md`) aren't pulled into the world yet, if a
  precise arena map is ever needed.

## Committing

This package is a submodule of `thornbots_workspace`, on branch `main`. Commit
and push here first, then bump this gitlink in `../` — one logical change, one
bump, never a gitlink pointing at an unpushed commit. Full rule in
`../CLAUDE.md` § Packages.
