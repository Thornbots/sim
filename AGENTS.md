# sim: agent notes

gz-sim simulation of the `ARCC_Field_2026` world plus the spawned `sentry`
robot, and the home of the localization integration suite. **Reference docs
live in `README.md`**, in particular its `## Notes` section, which holds the
drift-suite design history and per-scenario pass conditions. This file is only
the operating contract for working here.

Parent conventions in `../CLAUDE.md` apply, notably: **in-code
comments/docstrings under 10 lines**; longer prose goes to a `## Notes`
subheading in `README.md`.

## Running anything

Never hand-roll `docker exec`. Use `../isaac_ros_common/scripts/dexec.sh`, the
only path with correct env parity (ROS_DOMAIN_ID, FastDDS profile, both
workspace installs, `-u admin` for GUI). Load the `isaac-ros-docker` skill
before your first container command. `README.md`'s Build section (`cd
~/ros2_ws`) is stale; use these:

```bash
# all paths below are relative to this package dir
../isaac_ros_common/scripts/dexec.sh -- colcon build --symlink-install --packages-select sim
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch sim sim.launch.py
../isaac_ros_common/scripts/kill_launch.sh -l              # list launch trees;  kill_launch.sh <pid> to stop
```

**On a fresh or recreated container, run `install-sim.sh` before the first
sim launch.** `Dockerfile.thornbots` deliberately installs neither
`ros-humble-ros-gz` nor this package (real hardware never needs gz-sim):

```bash
../isaac_ros_common/scripts/dexec.sh -r -- src/isaac_ros_common/docker/scripts/install-sim.sh
```

**`sim` is the one package with no `/workspaces/ros2_ws` shadow copy.** It
isn't cloned during the Docker build, so `src/sim` edits are live immediately,
including from the user's terminal. Don't apply the `ros2_ws` shadowing
workaround here; it applies to every *other* first-party package.

## Testing

`test/localization/run_localization_drift_tests.py` is standalone, not part of
`colcon test`. It launches `sim` + `sentry_pkg` end to end:

```bash
../isaac_ros_common/scripts/dexec.sh -d -- python3 \
  /workspaces/isaac_ros-dev/src/sim/test/localization/run_localization_drift_tests.py \
  --backend slam        # --use-ekf layers EKF on top; --headless, --keep-running
```

Before launching anything, check for a live session (`ps aux | grep -E 'gz
sim|slam_toolbox|amcl|ros2 launch'`), since a colliding stack silently
corrupts measurements. If something is running, ask before killing it; it may
be the user's own work. That grep detects a live session only. To get a PID to
kill, use `kill_launch.sh -l`, since the grep also matches `dexec.sh`'s own
bash wrapper. Clean up anything *you* started, in a `finally` block.

## Standing rules

- **GUI on, not headless**, for both `sim` and the drift suite; the user
  watches the gz-sim window during testing. Pass `--headless` only when asked
  (e.g. a quick unattended run). Launch as `-u admin` via `dexec.sh -d`, never
  a bare `docker exec -d`, or the window fails to open; see the
  `isaac-ros-docker` skill's "Launching GUI apps".
- **Always fully restart `sim` (fresh spawn) before restarting SLAM/explorer.**
  Partial restarts leave stale TF/pose state ("pos desync").
- **If an edit under `sim/` doesn't take effect** for a `ros2 run`/`ros2
  launch` node, suspect `install/sim` losing its `--symlink-install` linkage
  (stale copies instead of symlinks) before assuming the edit is wrong. Fix:
  `rm -rf build/sim install/sim && colcon build --packages-select sim
  --symlink-install`. Raw scripts like `run_shot_hit_tests.py` run against
  `src/` directly and are unaffected, which makes this easy to misread.

## Scope

- Owns the world, the sim URDF, `pose_emulator.py`'s noise model, and the
  localization test suite.
- Localization backends belong to `../sentry_localization`; hardware
  interface and CV target selection to `../sentry_pkg`.
- Its own git repo (`Thornbots/sim`), so commits here are separate from the
  workspace.

## Open

- **The chassis has zero collision geometry, deliberately** — no link carries
  any `<collision>`, so the robot drives straight through walls and through
  `drift_correction_obstacle`'s spawned box. This was the price of making
  `root` free-floating so `set_pose` teleporting works for `auto_explore.py`'s
  grid sweep. The box itself has real collision geometry, so lidar/SLAM still
  see it; only the body passes through. Undecided whether to add a
  collision-only proxy that doesn't feed torque back into `root`. Revisit if
  obstacle *avoidance* (not just mapping) becomes something to demonstrate.
- **The rotation lock is soft (inertia-based only).** A real wall collision can
  tumble the robot to extreme angles (~86° roll observed). The user chose to
  accept occasional flips rather than reintroduce a kinematic constraint.
  Revisit only if flips start blocking exploration in practice.
- **The slip model corrupts position but not velocity** — `pose_emulator.py`
  applies `odom_slip_ratio` to `_slipped_x/y` only, while `vel_x`/`vel_y` pass
  through as true twist. Velocity-only wheel fusion therefore looks better in
  sim than it will on hardware, where encoder velocity is also wrong during a
  slip. Make slip corrupt velocity before trusting the +89% EKF number as a
  hardware prediction.
- **`drift_correction`/`drift_correction_obstacle` need an ekf-appropriate
  metric.** Their `MAX_DELTA_THRESHOLD` is calibrated for `map->odom`'s
  residual-correction semantics; under `odom->root` the delta is mostly the
  robot's own motion around the loop, so both reliably FAIL without indicating
  a problem. `test/localization/ekf_ground_truth_diag.py` scores against
  `/sim/raw_odom` with slip enabled and should probably become the assertion
  for `--backend ekf`.
- **The suite drives at 4.0 m/s, where rf2o degrades** (see
  `../sentry_localization/AGENTS.md`). Either cap the legs' speed or raise the
  sim lidar's 10 Hz update rate if scan-matcher accuracy starts mattering to a
  scenario's pass condition.
- **Never validate odometry on magnitude alone.** An early speed sweep compared
  `|displacement|` and scored rf2o healthy at ~1% error while it was pointing
  exactly backwards. Compare displacement *vectors* — the angle between them is
  what caught it. Applies to any odometry or detection source.
- ARCC Battlefield zone coordinates (Figures 3-1–3-9 in
  `../ARCC_2026_SENTRY_CONTEXT.md`) aren't pulled into the world yet, if a
  precise arena map is ever needed.
