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

- **Run anything localization-related at `real_time_factor:=1`.**
  Unthrottled runs score far worse than real time even though every suite
  times itself in sim seconds; find out why. rf2o is one cause: its 20 Hz
  wall-clock loop keeps only the newest scan, so a faster sim makes it skip
  scans. Check every node for wall-clock timers, rates and timeouts.
- **The shared sim isn't stable yet.** One scenario's robot stack came up
  without `/scan`; its `robot_<n>.log` will say why next time. Done when
  each scenario gives the same verdict alone, sixth, and under
  `restart_sim:=true`.
- **Wanted: switch to `urdf/sentry_v2`** for its collision and suspension
  (README.md). Its pitch limits and suspension values are placeholders, and
  collision reopens the free-floating `root` item below.
- **Wanted: a localization scenario with finite acceleration.** `drive()`
  steps `/cmd_vel` to 4 m/s and stops within one 0.1 s tick.
- **The EKF is worse than raw `/odom` at 4 m/s.** rf2o's heading drifts on
  a chassis that never rotates, and the EKF trusts rf2o's x/y. rf2o's sign
  is right; don't invert its warping again.
- **`drift_correction`/`drift_correction_obstacle` have no metric for
  `--backend none`.** `MAX_DELTA_THRESHOLD` assumes `map->odom`; under
  `none` it measures the robot's own motion. `test_ekf_ground_truth.py`
  should become that assertion.
- **`noise_correction`'s growth_ratio compares the two halves of a run**, so
  a run that starts clean fails hardest. Don't read its verdict as absolute
  accuracy.
- **The aim bench (`target_state:=truth`) hasn't run.** `point_to_cv_target`
  ignores `TargetState`'s `z_offset` fields, and `MOVING_MIN_HIT_RATE`
  (0.25) stays a placeholder until the bench measures a real floor.
- **Shot-hit results are bimodal and optimistic.** Runs have swapped 98% and
  1% on the same case, so don't trust one run. Detection noise (0.005 m) is
  far cleaner than a D435, and slew limits and target accelerations are
  estimates.
- **`lidar_self_filter`'s sector is tuned to a self-hit cluster that no
  longer exists**, since the lidar stopped scanning the robot. Retune from a
  hardware capture.
- **Keep the physics step at 1 ms.** 2-4 ms sent the head's PID unstable.
- **The chassis has no collision geometry, deliberately,** so `root` stays
  free-floating for `set_pose` teleports. It drives through walls and
  obstacles; lidar still sees them.
- **The rotation lock is soft.** A wall hit can flip the robot; accepted
  until flips block exploration.
- **Never validate odometry on magnitude alone.** Compare displacement
  vectors; a backwards rf2o once scored 1% error on magnitude.
- ARCC zone coordinates (`../ARCC_2026_SENTRY_CONTEXT.md` Figures 3-1 to
  3-9) aren't in the world yet.

## Committing

This package is a submodule of `thornbots_workspace`, on branch `main`. Commit
and push here first, then bump this gitlink in `../` — one logical change, one
bump, never a gitlink pointing at an unpushed commit. Full rule in
`../CLAUDE.md` § Packages.
