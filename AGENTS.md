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

The drift suite keeps an argparse wrapper that re-invokes pytest. The
shot-hit bench is a launch file that runs the stack and pytest as one tree, so
`kill_launch.sh <pid>` on it stops everything (`--show-args` lists its flags):

```bash
../isaac_ros_common/scripts/dexec.sh -- \
  ros2 run sim run_localization_drift_tests.py --backend amcl --use-ekf
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch sim shot_hit.launch.py
```

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
  Partial restarts leave stale TF/pose state ("pos desync").
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
- **The slip model corrupts position but not velocity.** `pose_emulator.py`
  applies `odom_slip_ratio` to `_slipped_x/y` only, while `vel_x`/`vel_y` pass
  through as true twist. Velocity-only wheel fusion therefore looks better in
  sim than it will on hardware, where encoder velocity is also wrong during a
  slip. Make slip corrupt velocity before trusting the +89% EKF number as a
  hardware prediction.
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
