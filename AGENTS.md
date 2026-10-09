# sim

Follow [workspace rules](../AGENTS.md) and [CI](../docs/CI.md).
Read [tests](README.md#run-the-tests), [build](README.md#build) and
[design](README.md#notes); host execution uses the Docker skill.
`sim` has no image-baked overlay; check `--symlink-install` if edits appear stale.

## Standing rules

- [Run approval and live-session rules](../AGENTS.md#containers-and-runs)
  cover integration suites and benches; registered unit/lint tests launch no stack.
- Keep GUI enabled unless headless was requested; Foxglove is the main viewer.
  Launch long runs through `dexec.sh -d`; use `kill_launch.sh`, never `pkill`.
- Also check for orphaned installed nodes (`ps -eo pid,ppid,cmd | grep install/`).
  Keep gz stacks in their own process group, outside the GTest launch, and
  kill the whole group in `finally`.
- Fully restart sim before restarting SLAM/explorer. The drift suite's reset
  between scenarios is the exception; simulated time must keep moving forward.
- Trust a result only after the native log checker accepts the run:
  [log checks](README.md#more-on-the-tests). Check displays as well as test verdicts.
- CV integration tests run through ROS; use the estimation bench for tracker
  tuning, never an offline copy. Pure-logic unit tests are separate.
- Keep gz as the engine and the physics step at 1 ms; 2-4 ms destabilised the head PID.
- Validate odometry displacement vectors, not only magnitude.
- Add runtime nodes as CMake C++ executables; Python stays launch adapters/lint.

## Scope

Own worlds, sim URDF, sensor/pose noise models and integration suites.
Backends belong to `sentry_localization`, robot/CV behavior to `thornbots_pkg`.
Every `e2e.launch.py` stage runs checked-out hosted MCB firmware and the real
UART bridge; build instructions and fake-hardware limits are in
[MCB emulator](README.md#mcb-emulator). Never downgrade its wire contract.
Keep field-safe match paths separate from bench paths; no blanket skips/xfail.
Keep approach firing disabled until all routes reach center.

## Open

- Current failures and repeatability work: [ROADMAP status](../ROADMAP.md#where-we-actually-are),
  [match track](../ROADMAP.md#a-the-match-test), [tracker track](../ROADMAP.md#g-estimation-accuracy).
  Repeat accuracy changes three times; reject lockstep timeouts.
- Driving/match passes establish diagnostic completion; combat accuracy needs
  repeated calibrated floors: [drive](README.md#driving-and-combat-diagnostics-mcb_drive),
  [match](README.md#four-robot-center-fight-mcb_match).
- `odom_stuck` intentionally checks liveness, not recovered localization:
  [scenarios](README.md#scenarios). Never invert rf2o's warping to retune a regression.
- No scored stage runs YOLO or camera occlusion; detections are truth stand-ins.
  Noise, gimbal dynamics, suspension and spawn coordinates remain estimates;
  [model limits](README.md#sentry_v2urdfxacro-motion-model).
- gz quirks: `spawn_model` can report failure after a slow `sentry_v2` spawn
  succeeds (check `model_names()`); spawning inside the colliding chassis
  stalls `/clock`; only the `sim.launch.py` robot obeys VelocityControl.
- Changes shared with `target_driver`, `cv_target_emulator`, `cv_head_aim` or
  `pose_emulator` may also need `src/bench_world.cpp`; it has its own lockstep implementation.
- Lidar/CAD and carried-map/zone gaps: [track J](../ROADMAP.md#j-lidar-and-camera),
  [track H](../ROADMAP.md#h-slam-at-amcls-level), [URDF export](README.md#toolssimplify_urdfcpp-and-urdfsentry_v2).
