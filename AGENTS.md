# sim — agent notes

gz-sim simulation of the `ARCC_Field_2026` world plus the spawned `sentry`
robot, and the home of the localization integration suite. **Reference docs
live in `README.md`** — in particular its `## Notes` section, which holds the
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
~/ros2_ws`) is stale — use these:

```bash
# all paths below are relative to this package dir
../isaac_ros_common/scripts/dexec.sh -- colcon build --symlink-install --packages-select sim
../isaac_ros_common/scripts/dexec.sh -d -- ros2 launch sim sim.launch.py
../isaac_ros_common/scripts/kill_launch.sh -l              # list launch trees;  kill_launch.sh <pid> to stop
```

**On a fresh or recreated container, run `install-sim.sh` before the first
sim launch** — `Dockerfile.thornbots` deliberately installs neither
`ros-humble-ros-gz` nor this package (real hardware never needs gz-sim):

```bash
../isaac_ros_common/scripts/dexec.sh -r -- src/isaac_ros_common/docker/scripts/install-sim.sh
```

**`sim` is the one package with no `/workspaces/ros2_ws` shadow copy** — it
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
sim|slam_toolbox|amcl|ros2 launch'`) — a colliding stack silently corrupts
measurements. If something is running, ask before killing it; it may be the
user's own work. That grep is for *detecting* a live session only — to get a
PID to kill, use `kill_launch.sh -l`, since the grep also matches `dexec.sh`'s
own bash wrapper. Clean up anything *you* started, in a `finally` block.

GUI is on by default for sim and for the drift suite — a standing rule in
`../SESSION_NOTES.md`. Use `--headless` only when asked.

## Scope

- Owns the world, the sim URDF, `pose_emulator.py`'s noise model, and the
  localization test suite.
- Localization backends belong to `../sentry_localization`; hardware
  interface and CV target selection to `../sentry_pkg`.
- Its own git repo (`Thornbots/sim`) — commits here are separate from the
  workspace.
