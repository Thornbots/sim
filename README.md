# sim

sim holds the robot's integration tests. The localization drift suite starts a
`gz sim` model of the `ARCC_Field_2026` field, spawns the `sentry` robot from
`urdf/sentry_v2.urdf.xacro`, and runs the real `thornbots_pkg` stack against
it. The two CV benches run no gz. The aiming bench scores `point_to_cv_target`
against a perfectly known target; the estimation bench scores how well Part 2
turns noisy detections into a target model. The split is documented in the
[CV interface](../thornbots_pkg/README.md#cv-interface); historical results
are in [CV bench observations](docs/cv-bench-results-2026-09-28.md).

`ament_cmake` builds the runtime C++ nodes, Gazebo components and `bench_world`.
All ROS runtime nodes are C++. Python remains for launch files, pytest
harnesses and tools. `auto_explore.py` supplies shared Gazebo helpers;
`combat.py` and `match_scenario.py` serve the E2E score harness; and
`mcb_firmware.py`, `mcb_emulator/pty_link.py` and `mcb_emulator/protocol.py`
support the hosted firmware and its tests. `cv_target_emulator.py` keeps only
geometry constants used by the URDF consistency test.

`/cmd_vel` remains a bare `Twist` because gz's diff-drive interface and the
harnesses expect it; this is the workspace timestamp rule's standard-interface exception.

## Run the tests

Build first on a fresh container (see Build). Each test starts its own sim and
`thornbots_pkg` stacks, so stop anything you already have running.

The localization suite runs `amcl` with the EKF by default (`backend:=` picks
another; `auto.launch.py`'s own default is now `mapping`). Its nine scenarios
are listed under Notes:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim localization_tests.launch.py
```

`suite:=ekf` runs the EKF ground-truth test instead. It is on demand, not part
of the standard run: run it after changing the EKF or rf2o config.

The CV aiming bench runs ten shot-hit cases, all with lead on: a stationary
target, then 0.5, 1, 2 and 4 m/s, first with flat panels and then with
neighbouring panels staggered 90% of a panel's height apart
(`panel_layout:=` picks one). It launches the stack once and only changes the
target between cases; each case settles for 3s, then scores 30s of sim time.
The bench fires at up to 40 Hz, far above the real launcher, and scores each
case on hit rate and hits per expected shot equally (`score()` in
`test/cv/shot_hit_harness.py`), so falling behind 40 Hz costs points.

There is no gz, robot or tracker. `sim_clock` publishes `/clock`,
`point_shooter` puts `root` in `odom` (`POINT_SHOOTER`, 0.4 m up) and
publishes `/dji_serial_bridge/pose`, `target_driver` moves the phantom target and
`target_state_truth` publishes its true `TargetState`. Each shot leaves
`root` toward the newest `/cv/target` aim (an `odom` point) before its exit time, carrying
`root`'s velocity: a perfect gimbal that holds each 40 Hz aim until the
next. So a miss is `point_to_cv_target`'s math and nothing else. It draws the
target's panels in rviz.

`real_time_factor:=0` (the default) runs it as fast as the stack keeps up.
`sim_clock` steps sim time 2 ms at a time and stops exactly one period past
the newest stamp on `/cv/target_state`, `/dji_serial_bridge/pose`, `/cv/target` and the
scorer's `/bench/progress` until that topic publishes, so no node falls
behind and each 40 Hz tick is stamped on its deadline; a topic with
no publisher left (the scorer between cases) stops holding it. A fixed
`real_time_factor` above 0 doesn't wait: 8x lost 2-5 points. On 2026-09-25
the paced clock ran 6.6-6.8x and scored every cell within half a point of
three 4x runs, `keep_up` 1.00. Use `:=1` as the control.

`chase_settle_s` picks `point_to_cv_target`'s spin mode: `>= 0` (the default,
0) chases the facing panel and fires every tick, `< 0` is shotgating: hold
the center line and time the fire. `gimbal_lag_s` defaults to 0, the perfect gimbal. Each cell has
its own floor in `FLOORS` (`test/cv/shot_hit_harness.py`), the lowest score
over three runs minus 10 points. Forty cells are measured: both layouts on
lateral, radial and diagonal with a still shooter, and lateral at
`shooter_speed:=1.0`. Any other cell falls back to the placeholders and says so. `target_path:=radial` or `diagonal`
moves the target along the camera ray instead of across it, and
`shooter_speed:=1.0` bounces our own chassis along y (within 1 m of
`POINT_SHOOTER`) through every case.

Every scored shot goes to `shots.jsonl` in `--log-dir`, one JSON object per
line: the case, whether it hit (the ray crosses a facing panel's canted
0.135 x 0.125 m face), how far outside that face it crossed
(`off_face_m`), the miss distance from the panel centre split into its
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

The suite launch commands return a nonzero exit status when pytest fails,
including collection errors, after shutting down their stack. Check the
pytest summary as well as the measured hit rates: diagnostic-tier completion
is not an accuracy guarantee. `tools/check_bench_log.py` also rejects failed
pytest summaries and clock or lockstep stalls.

The CV estimation bench scores Part 2, not hits. `bench_world` (C++)
is the whole world in one lockstep loop: `/clock`, the phantom target through
the aiming bench's ten cells plus a still one at 45 deg (`stationary45`, two
panels in view), our chassis and head, `/dji_serial_bridge/pose`, the head controller and
the detections off our head's camera. `target_selector` and `target_tracker`
build the `TargetState`, and `point_to_cv_target` aims the head; nothing
fires. Each case starts with one parameter set (`case_seed`, the cell's
CRC): the target at its path start and its yaw (0, a panel square to us,
except `stationary45`), our chassis at the origin, every schedule and the
blackout restarted, and detections off for 1 s (`case_hold_s`) while the
head turns to the truth, so the tracker starts a fresh track. Then it
scores every state for 3 s + 30 s against the truth at the state's own
stamp, so a late stamp scores as error. Detection noise is drawn per
(`seed`, `case_seed`, frame, panel), not from one stream, so a cell sees
the same noise on every run whatever ran before it or where the head
pointed; `seed:=` samples another draw. At `real_time_factor:=0` the loop
waits on the nodes under test. Historical five-run p95 comparisons were
within 1.03x (Mac, 2026-10-01), but moving cells can still vary with the
same seed: zero timeouts alone does not prove repeatability. `/clock` holds at 0 until
`point_to_cv_target` and `target_tracker` are up, so the aim node's 40 Hz
timer starts at 0 and each case starts on a 0.1 s boundary (`case_align_s`).
A step's detections go out only once the tracker's `/cv/tracker/clock_ack`
shows it reads the last `/clock`, since it stamps `TargetState` with `now()`;
`/clock` then waits for the tracker to echo each one on
`/cv/tracker/measurement`, and for `/cv/target` on each aim tick. The scorer
only paces, up to `pace_slack_s` behind. The loop also waits for
`point_to_cv_target` to acknowledge model consumption on `/cv/target/state_ack`
and for a publish tick's indicated aim point to arrive. Separate DDS topics
can arrive in either order; a publication acknowledgment alone does not
prove that the next consumer has processed its input. A wait that passes `max_wait_s`
(0.5 s wall) is logged as a lockstep timeout, and that run may not repeat.
The 12 cells take ~30 s on the Mac (`real_time_factor:=1` for real time):

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim estimation.launch.py
```

Per case it prints how long the facing panel's error took to stay under
5 cm, the fraction of states valid, and the mean and p95 over the last 30 s
of: the facing panel's error (the panel Part 1 aims at), all four panels',
center (also split along and across the ray from us), velocity, yaw (mod a
quarter turn), spin rate, radius and height per pair. `blackout:=true` drops every detection for 0.3 s in each 2 s.
`camera_latency_s:=0.03` stamps detections late and tells the tracker to
undo it (`tracker_camera_latency_s:=0` to leave it undone).
`shooter_speed:=1.0`, `target_path:=`, `speeds:=` and `panel_layout:=` work
as on the aiming bench, and `process_noise_accel:=` sets the tracker's.
`chassis_spin:=9` spins our chassis (rad/s, CCW, the firmware's rate) under
the head, which holds world yaw as the MCB's IMU loop does;
`yaw_bearing_damping:=` (N m s/rad, default 0, unmeasured) lets the spin drag
the head. A cell
passes on liveness until `LIMITS` in `test/cv/estimation_limits_data.py` has
limits for it. The table has 72 keyed cells, including 12 `chassis_spin:=9`
cells. The two default `stationary45` cells (`chassis_spin:=0`) still have
no p95 limits and pass only state-presence/valid-fraction checks. A 12/12
default run therefore does not establish accuracy for those two cells:

```bash
python3 tools/estimation_limits.py run1/ run2/ run3/   # each a log_dir:=
```

`--keep test/cv/estimation_limits_data.py` keeps every cell the new runs
don't score, so one axis can get limits without rerunning the rest.

`estimation.jsonl` has one summary line per case and
`estimation_states.jsonl` one line per scored state.

The bench is one launch tree: the stack and the pytest that scores it. The
localization launch runs pytest, and pytest starts the sim once
(`run_tests:=false part:=sim`) and a fresh robot stack for each scenario
(`part:=robot`). Between scenarios it stops the robot stack, teleports the
robot back to spawn, removes anything a scenario spawned, and resets
`pose_emulator`'s noise state and parameters. `restart_sim:=true` brings the
sim up fresh for every scenario instead, the old behaviour and the control when
a verdict looks off. `spawn_yaw_deg:=5` turns the chassis that far off the spawn heading at every
reset; the real robot drifts 1-5 deg. Both
shut down when the tests finish, and Ctrl-C stops everything, stacks included.

The localization launch defaults to `real_time_factor:=0`, which lets gz run
as fast as the machine allows, and so does the aiming bench. Every test times
itself in sim seconds, so a faster sim shortens the wall-clock run without
shortening what gets scored. On the dev laptop with `sentry_v2`, the drift
suite runs about 1.2x real time, GUI or headless alike.

Add an argument to run part of a suite. `scenario:=odom_stuck` runs one drift
scenario; `backend:=mapping` builds a blank map, and `use_rf2o:=false`
uses raw odometry. This suite rejects `backend:=slam`: no pose graph ships,
and its launch has no saved-map override. Use `auto.launch.py` with an
explicit saved `map_file` for localization-mode SLAM outside this suite. For
the bench, `only_stationary:=true` runs the stationary case, `speeds:='0.5 1'`
picks the moving cases. `headless:=true` drops the gz GUI and rviz from
either. `--show-args` on either launch lists the rest.

`tools/run_suite.sh drift|ekf|shot_hit|estimation [args]` wraps any of these
launches: it refuses to start over a running stack, tees the output to
`/tmp/sim_suite_*.log`, then runs `tools/check_bench_log.py` on it. It exits 0
only when pytest passed and the checker trusts the run:

```bash
src/sim/tools/run_suite.sh drift scenario:=odom_stuck
```

## Build

`Dockerfile.thornbots` installs neither `ros-jazzy-ros-gz` nor this package.
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

`colcon test` collects the Python unit and integration suites plus the C++ GTests.

| Tier | Files | Needs |
| --- | --- | --- |
| unit | `cpp/test_cv_head_aim.cpp`, `cpp/test_mcb_protocol.cpp`, `cpp/test_combat.cpp`, `cpp/test_target_odometry.cpp`, `cpp/test_depth_mm.cpp`, `cpp/test_panel_view.cpp` | GTest |
| unit | `cv/test_urdf_constants.py`, `cv/test_estimation_metrics.py`, `test_suite_timing.py`, ament copyright/flake8/pep257 | Python + pytest |
| integration | `localization/test_localization_drift.py` | gz-sim and a launch tree |
| integration | `cv/test_shot_hit.py` | a launch tree, no gz |
| integration | `cv/test_estimation.py` | a launch tree, no gz |
| on demand | `localization/test_ekf_ground_truth.py` | gz-sim and a launch tree |

`setup.cfg` deselects the `integration` marker, so a plain `colcon test` runs
only the unit tests and finishes in seconds. pytest's own `-m` overrides that
(`colcon test`'s `--pytest-args` only reaches `ament_python` packages):

```bash
colcon test --packages-select sim
colcon test-result --test-result-base build/sim --verbose
cd src/sim && python3 -m pytest test -m integration
```

The on-demand tier is specialized checks run only when their subject
changes, such as the EKF ground-truth test after an `ekf.yaml` edit. It
carries `integration` too, but is skipped without `--run-on-demand`;
`suite:=ekf` passes it.

An integration run ends with a `suite timing` table: wall seconds per case
in sim start, bring-up, reset, settle, scored and teardown, and the RTF.
Every bench serves Foxglove on port 8765 (`foxglove:=false` to skip). A GUI
run also opens the gz and rviz windows where a display opens, and runs
without them where none does. The CV benches fail a case whose stack lost
a node.
Before trusting a run's numbers, check its launch log, which `ros2 launch`
names at the top of its output:

```bash
python3 tools/check_bench_log.py ~/.ros/log/<run>/launch.log
python3 tools/check_bench_log.py /tmp/localization_drift_tests/*.log  # drift and EKF suites
```

It prints the result and the timing table, and exits 1 when a node crashed
mid-run, a GUI couldn't open or a wait gave up.

rf2o's match grades (`/scan_odom/quality`), for setting
`sentry_localization/config/rf2o.yaml`: record beside a suite, then
summarise.

```bash
python3 tools/rf2o_quality.py record /tmp/q.jsonl   # Ctrl-C when the suite ends
python3 tools/rf2o_quality.py summary /tmp/q.jsonl
```

The drift suite starts gz-sim once and a fresh `thornbots_pkg` stack for each
scenario, resetting the sim between them (see "Run the tests");
`restart_sim:=true` restarts gz per scenario instead. The aiming bench launches
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
docstring.

## Launch sim by hand

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim sim.launch.py
ros2 launch sim sim.launch.py gui:=false rviz:=false     # server only
ros2 launch sim sim.launch.py x:=1.0 y:=0.5 yaw:=0.0     # spawn pose (z:= too)
ros2 launch sim sim.launch.py world:=/abs/path/to/other.sdf
ros2 launch sim sim.launch.py camera:=true               # add the depth camera, as the D435's topics
ros2 launch sim sim.launch.py model:=sentry              # the old collision-free model
ros2 launch sim sim.launch.py foxglove:=false            # no Foxglove bridge on :8765
```

`model:=` picks the robot: `sentry_v2` (the default, from the CAD) or
`sentry`, the old model.

The camera is off by default, sensor and all: without `camera:=true` the
robot spawns with no camera sensor, so gz renders nothing for it. The
`camera` link and its TF stay, since the emulators and tracker place
detections through that frame.

The optional `sentry_v2` camera is depth only, 640x480 at 60 Hz. It is
available for manual depth-pipeline checks; no current suite enables it.
The match stages instead feed 3D truth through `detector_standin`. The optional
camera publishes the robot's D435 topics: `/depth/image_rect_raw` (16UC1 millimetres, 0 for no
data), `/depth/camera_info` and `/color/camera_info` (one lens, one set of
intrinsics), and a latched identity `/extrinsics/depth_to_color`. The gz
bridge and `depth_camera_emulator`, which converts gz's 32FC1 metres, run
in `camera_container`. A manual ROI-depth check must load `roi_depth_node`
into that container separately; `sim.launch.py` does not load it. Over DDS a
640x480 frame is past Fast DDS's 512 KB shared-memory segment, and most frames dropped: 14-18 of 60
Hz arrived (2026-09-29). Depth costs sim speed: the Mac's bare
`sim.launch.py` runs at RTF 2.66 without it and 1.1 with it (llvmpipe).

These add synthetic wheel-odometry error. All are off by default; the
The pose emulator's odometry noise note explains each one:

```bash
odom_noise_enabled:=true    # master switch for drift + jitter
odom_drift_stddev:=         # random-walk step, m/callback (0.0005)
odom_jitter_stddev:=        # per-sample jitter, m (0.001)
odom_jerk_stddev:=          # trigger_jerk size, m (0.2)
odom_jerk_bias_enabled:=true odom_jerk_bias_x:= odom_jerk_bias_y:=
odom_slip_ratio:=           # fraction of each driven metre lost from /dji_serial_bridge/pose (0.0)
```

`spawn_target` adds the moving CV target. It is off by default, but when you
turn it on, all three `cv_*` degradations apply at the defaults below. Set them
to zero for a clean run:

```bash
spawn_target:=true            # target_driver, cv_target_emulator, cv_head_aim
target_speed:=2.0 target_spin_hz:=1.5
cv_noise_pos_stddev:=0.005    # Gaussian position noise, m, plus D435-like ray noise
cv_dropout_probability:=0.03  # per-sample detection drop, placeholder
cv_publish_latency_s:=0.06    # placeholder, not measured
```

`e2e.launch.py` is the match test (`../E2E_PLAN.md`). Every stage runs the
MCB emulator: the compiled sentry firmware (see Notes "MCB emulator") on a
pty, with `dji_serial_bridge` and `mcb_relay` on the other end, the real CV
chain from `target_selector` to `point_to_cv_target`, and no camera:
`detector_standin` feeds gz truth in place of YOLO and `roi_depth_node`.
Shots are the ones the firmware fires, each falling under gravity since the
firmware pitches up for it. `point_to_cv_target` patrols with no target, as
on the robot, so it can sweep for an opponent that starts out of view or
whose track was lost; patrol frames clear `fire`. Reacquisition and moving
hit rates still vary (ROADMAP T17).

| Stage | Test | What it scores |
|---|---|---|
| `mcb_parked` (default) | `test/e2e/test_mcb_parked.py` | our robot parked, one red opponent riding `target_driver`'s path; one test per speed and path |
| `mcb_drive` | `test/e2e/test_mcb_drive.py` | our robot drives spawn to center against one red sentry |
| `mcb_match` | `test/e2e/test_mcb_match.py` | the 2v2 center fight with ballistic shots and HP |

`mcb_parked`'s field-safe paths are separate from the gz-free aiming bench:
lateral at world y=-1.3 with x ±1.9 m, radial y=-0.9 to -1.7 m, diagonal
centered at (0, -1.3), half-length 0.65 m. Mesh regression checks include
a 0.40 m robot footprint. Short paths brake before reaching high requested
speeds; labels are speed limits. Each cell fails on zero firmware shots or
a hit rate under its floor. Shots `CVTarget.fire` asks for are logged
beside. `firmware_fixes` is deprecated; the hosted build runs this
checkout's code without overlays.

```bash
ros2 launch sim e2e.launch.py                                 # mcb_parked, all 12 cells
ros2 launch sim e2e.launch.py speeds:='0 2' paths:=lateral duration:=15
ros2 launch sim e2e.launch.py run_tests:=false target_speed:=2.0 target_spin_hz:=1.5
```

`mcb_parked`'s test-owned stack brings up a parked, non-spinning opponent
and waits for a valid `/cv/target_state` before requesting the first cell
(patrol keeps `/cv/target` fresh without one). Starting it at the launch's
moving defaults left unscored motion dependent on wall-time bring-up speed.
Each requested cell sets its speed/spin and settles for three sim seconds
before scoring.

`shots.jsonl` in `log_dir` (`/tmp/e2e_test_logs`) splits every miss into
the barrel's angle off the aim and the aim's distance from the panel, and
each case prints TargetState's centre, velocity and spin error against
truth. `pytest_args:='--e2e-spin 0'` holds the spin for every cell.
`states.jsonl` records stamped TargetState errors by case. `mcb_parked` shot
and state records also include head-TF and map-localization errors against gz
truth, and `odom_disagreement_m`: the MCB's POSE against TF `odom->root` at
that POSE's stamp. That agreement is what a hit depends on; every aiming
hop uses `odom`, so `map->root` error is logged but never blamed. Missing
stamped TF is reported as unavailable; latest TF is never substituted to
calculate these errors. `python3 tools/compare_runs.py LOG_DIR_A LOG_DIR_B`
prints the first record where two runs' logs diverge.
`poses.jsonl` samples head-TF and localization at 20 Hz even when tracking
is lost and no shots fire. `real_time_factor` reaches the test-owned stack;
use `real_time_factor:=1` for a paced control against the unthrottled default.
The E2E wait checks clock progress: five wall seconds without progress fails,
backward time fails, and a window has a budget of at least 30 wall seconds
or 20 times its sim duration. This permits deliberately slow controls;
it does not shorten their scoring windows.

- `opponent_driver` spawns `opponent_0`, a `sentry_v2` with no sensors,
  gravity or contacts, drawn from its collision shapes (the CAD visuals
  cost RTF 1.10 to 0.61 with the depth camera on). Its root hull is drawn at
  75% in x and y: at full size it stood 3-7 cm proud of every panel.
- Its `OpponentMover` gz system (`src/opponent_mover.cpp`) sets its pose
  every physics step from `target_driver`'s path, bridged to
  `/model/opponent_0/path`. VelocityControl moved only the robot
  `sim.launch.py` spawns, and teleporting from Python jumped 4-36 cm.
- `detector_standin` (`src/detector_standin.cpp`) publishes
  `/cv/panel_detections` at 60 Hz of sim time, stamped with the tick, as
  `roi_depth_node` would: every panel that faces the camera within
  72.5 deg and whose centre lands in the D435's 640x480 image (horizontal
  FOV 1.5184, 10 m), its true centre and corners in `camera`. No
  occlusion. `noise_depth_range_coeff` and `noise_lateral_rad` add
  `cv_target_emulator`'s ray noise, 0 by default. Truth is gz's
  `/model/<name>/pose` at the tick; panel and camera offsets come from
  the URDF.
- Our team comes from the MCB emulator's `REF_SYS`, as on the robot.

`sim.launch.py` and every test launch start a Foxglove bridge on port 8765
(`foxglove:=false` turns it off). To run one next to anything else:

```bash
ros2 launch sim foxglove.launch.py   # port:=8765
```

In Foxglove (desktop or browser, any OS), choose "Open connection", then
"Foxglove WebSocket", then `ws://<this host's tailscale IP>:8765`. It sees
every topic in the container's DDS domain, and sends only the ones a panel
subscribes to, so leave image panels closed over a slow link.

A fresh Foxglove 3D panel shows none of our displays. `foxglove/` has one
layout per rviz config, the same displays, fixed frame and view: `config.json`
for `sim.launch.py` and the localization suite, `estimation.json` and
`cv_target.json` for the two CV benches. Import one (the layout menu, then
"Import from file"). After editing an rviz config, regenerate them:

```bash
python3 tools/rviz_to_foxglove.py   # --check: exit 1 if one is stale
```

`sim.launch.py` starts gz, spawns the robot, bridges its lidar, joint, odometry,
camera and head-command topics to ROS, and runs `pose_emulator`, which
publishes `/dji_serial_bridge/pose` the way the Type-C board does. It runs no
`robot_state_publisher`, so TF comes from `thornbots_pkg`'s `auto.launch.py`.
`/sim/raw_odom` and `/sim/raw_joint_states` are ground truth that real hardware
doesn't have; only sim and its tests should read them.

## Notes

These notes explain why the code looks the way it does, so the in-code
comments can stay short. Each heading names a file.

### The world frame

The gz world is the field frame (REP-105, (0, 0) at the field centre, x
along the 12 m toward blue's base), as are `odom`, `map` and the saved maps.
The field mesh is turned -90 deg in `ARCC_Field_2026.sdf` to get there
(2026-10-04); before, the world's x ran across the field. Everything placed
in world coordinates turned with it, so each suite drives the same physical
paths: the robot spawns facing -y (`auto_explore.SPAWN_YAW`), the drift
loop, actors and map-sweep grid are turned, and `target_driver`'s
`origin_yaw` keeps the target path in front of the robot.

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
`head_pitch`, `lidar`, `camera`, `headlink`, `headpitch`) and `headlink` turns about
+z, CCW like the MCB's `head_yaw` (it was -z until 2026-10-03, which mirrored
the camera on the sentry), and `test_urdf_constants.py` pins it.
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

The export has all four armor modules, and the tool emits one fixed link per
module face, `armor_0` to `armor_3`, found by the face part's mesh file
(`armor.face_mesh`). Each frame sits at the face centre with +x along the
outward normal and +y horizontal, and carries a box of the face's size as
visual and collision, so depth sees it and shots can hit it. From the CAD
(2026-09-27):

- The faces sit on the diagonals, between the wheels: `armor_0` at +45 deg
  from the gun, then every 90 deg anticlockwise. Face centres are 0.252 m
  from the chassis axis.
- `armor_0` and `armor_2` sit at 0.230 m, `armor_1` and `armor_3` at
  0.136 m, so the lower edges are 94 mm apart (S126 allows 100).
- Every normal is 75.0 deg from straight up (S122's 15 deg cant).
- The face is 135 x 125 mm, the Small Armor Module's, the only size in
  ARCC 2026.

The regenerated meshes aren't byte-identical across trimesh versions, so when
only the URDF text should change, keep the committed meshes
(`git checkout -- urdf/sentry_v2/meshes`).

`sentry_v2.urdf.xacro` wraps the generated URDF for gz: colours, the lidar and
camera sensors, the plugins, and a `muzzle` frame (see the CV head aim node
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
against the pose emulator's noise model. It mirrors `auto.launch.py`'s two
axes: `--backend slam/mapping/amcl/none` (who owns `map->odom`) and `--use-rf2o` /
`--no-use-rf2o` (whether `odom->root` is EKF-fused; on by default, matching
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
| `mapping` | `map->odom` | as `slam`, on a map it builds from blank |
| `amcl` | `map->odom` | `update_min_d`/`update_min_a` |
| `none` | `odom->root` | no map node; `ekf_node` unless `--no-use-rf2o`, then raw `/odom` |

`--use-rf2o` swaps `odom->root`'s source to `ekf_node` and leaves `map->odom`
alone.

`mapping` starts slam_toolbox on a blank map (`load_map:=false`) at spawn,
the world origin, so its `map` frame is the world's. Like `none`, it scores
`map->root` against `/sim/raw_odom` in `noise_correction` and the
cornering-loop scenarios; the rest watch `map->odom` as under `slam`. Each
scenario's stack builds a fresh map: nothing carries between scenarios.
`odom_stuck` drives two mapping laps first (`ODOM_STUCK_MAPPING_LAPS`).

`mapping --use-rf2o` passes 9/9 (2026-10-02, archlinux, unthrottled):
cornering loops 0.02-0.09 m truth error, `noise_correction` 0.04 m,
`scan_degraded` 0.43 m during and 0.07 m after. `--no-use-rf2o` fails 5/9
(Mac): the three 4 m/s loops at 0.41-0.42 m, `scan_degraded` 0.42 m after,
and `odom_stuck`, whose `map->odom` freezes as amcl's does without rf2o
(below), mapping laps or not. Under rf2o, `odom_stuck` passes its liveness
check while lost (1-5 m truth error), and mapping keeps writing to the map
the whole time.

Under `none`, `odom->root` is the robot's own position, so its change says
nothing about error. `noise_correction` and the three cornering-loop scenarios
score `odom->root`'s distance from `/sim/raw_odom` there instead, against the
same thresholds. `jerk_with_motion` is skipped for `none`: `ekf_node` fuses
`/odom` velocity only (`odom0_config`), with no travel gate, so neither
expectation is defined. With no map to miss a feature from, `drift_correction`
and `drift_correction_obstacle` should read about the same under `none`, and
so should `moving_obstacles`. They do: 0.022, 0.037 and 0.020 m, with
`noise_correction` at 0.040 m (2026-09-28, `--use-rf2o`, unthrottled).
`scan_degraded` fails under `none` (0.63 m during, 0.64 m after): with no
map layer nothing pulls the error back once the scan recovers.

amcl with and without EKF under slip, measured 2026-07-26 against a 0.30m
bound; verdicts shown against today's 0.40m `MAX_DELTA_THRESHOLD`:

| `odom_slip_ratio` | `amcl` | `amcl` + `use_rf2o:=true` |
| --- | --- | --- |
| 0.0 | 0.1478 m (PASS) | 0.2043 m (PASS) |
| 0.25 | 0.4033 m (**FAIL**) | 0.1642 m (PASS) |

At zero slip `/odom` is near perfect and fusing rf2o's noise only hurts.
Under slip the EKF turns a fail into a pass. That was the first sign it helps
a backend that owns a map. 0.25 is harsher than the defaults (0.02, or 0.15 in drift
scenarios). `slam --use-rf2o` measured worse than plain `slam`; see
`sentry_localization/README.md`.

#### Scenarios

The suite runs them in this order.

1. `baseline` (noise off) asserts the correction TF settles and stays stable,
   with no ERROR in any log. A steady ~0.1-0.15m offset is normal, because the
   saved maps sit that far off the gz world (measured before both were turned
   into the field frame, the same turn for each). A
   growing offset would be a real problem.
2. `noise_correction` drives the 3m square under drift and jitter (no slip or
   jerks) for a fixed 30s (60s before 2026-07-27). The second half's samples
   must stay under 2x the first half's max. The window doesn't exit early, so
   a stalled TF can't cause an open-ended drive.
3. `drift_correction` drives the same square, no obstacle. The hard
   reversals at 4.0 m/s (20 m/s^2 ramps) build dead-reckoning error faster
   than the scan-match gate follows, and the wobble is the backend
   correcting it at each dwell.
4. `drift_correction_obstacle` adds a static box at the loop centre
   mid-scenario, absent from the world and the map. It shares driving and
   threshold with `drift_correction`, so comparing the two isolates the
   obstacle. A pass here means nothing if `drift_correction` failed.
5. `moving_obstacles` drives the same square while `actor_driver` walks three
   unmapped boxes across its -x, +y and +x edges at 1.0, 2.0 and 0.5
   m/s. It scores like `drift_correction`, on `MAX_DELTA_THRESHOLD`, and logs
   each sample's `map->root` error against `/sim/raw_odom`. It also fails if
   `actor_driver` dies mid-loop. Under `slam`, ROADMAP.md T3 also wants the
   actors' cells checked in `/map` at the end; that check isn't built yet.
   Under `none` it scores ground-truth error, like `drift_correction`.
6. `real_accel` drives the `drift_correction` square with every leg ramped
   at `REAL_ACCEL` (1.2 m/s^2, close to the real chassis) instead of
   `DRIVE_ACCEL` (20), so a 3 m side peaks near 1.9 m/s. Same metric and
   threshold as `drift_correction`.
7. `jerk_with_motion` (slam/amcl) models a collision impulse. Each trial fires
   `trigger_jerk`, drives one leg to the next corner, then requires a
   correction proportional to the jerk or an end state within
   `MAX_DELTA_THRESHOLD`. Jerks are biased toward `OBSTACLE_XY` so they don't
   push the robot into nearby walls, and the leg absorbs the actual (dx, dy)
   so the robot still lands on its corner. `JERK_WITH_MOTION_REPEATS` (8)
   trials share one stack (relaunching costs 15-20s each) and all must pass.
   A closing lap follows.
8. `odom_stuck` models a dead encoder: `trigger_odom_stuck` pins `/dji_serial_bridge/pose` x/y
   and velocity at zero with fresh timestamps. It checks liveness only, since
   there is no valid odometry to bound drift against: scans keep processing
   and pairwise TF spread exceeds `ODOM_STUCK_MIN_TF_SPREAD` (1cm).
   Measured 2026-07-27: `amcl` fails, stuck at 0.0000m for 30s, because the
   scan-match gate runs on odom-reported travel and frozen odom never reopens
   it. The stack really does depend on odometry to stay live. `amcl --use-rf2o`
   passes at 1.3071m, because the EKF keeps reporting travel. Passing isn't
   tracking: 2026-09-25 it passed at 1.35m while `odom->root` stayed inside
   about 1m and ground-truth error cycled 0.2-3.9m per lap. rf2o seeds each
   match from `/odom`, so a frozen `/odom` pins it too, and at 4 m/s the 10 Hz
   lidar moves 0.4m between scans, too far for rf2o to match without a good
   seed. Matching from rf2o's own last motion as well didn't help, since that
   motion was already near zero. amcl has no motion to spread its particles
   and turns its estimate (up to 0.66 rad) to fit the scan. Losing the robot
   here is a known limit of this sensor set, not a defect; the scenario stays
   a liveness check. The gz robot itself doesn't turn: over a 120 s run
   (2026-09-28) every `/sim/raw_odom` message held true yaw within
   -0.15..0.00 deg and the head within -0.28..0.07 deg, while `map->odom`
   yaw sat at -0.3 to -0.58 rad. A turning robot in rviz is amcl's
   estimate. The scenario logs both ranges at the end.
9. `scan_degraded` is the one scenario that breaks rf2o instead of `/odom`.
   After one lap of the cornering loop at 0.15 slip it sets
   `lidar_self_filter`'s blind sector to 300 deg for two legs, leaving a 60
   deg arc (a robot parked against the lidar), then restores it for two more
   laps. It scores ground-truth error of `parent->root` against
   `/sim/raw_odom` after every leg, and logs `/scan_odom/quality`'s grades
   per phase. The blackout legs pass under `SCAN_BLACKOUT_MAX_ERROR` (0.5
   m), the rest under `MAX_DELTA_THRESHOLD` (0.4 m). Across four runs
   (2026-09-27) the blackout peaked at 0.398-0.432 m, failing three against
   0.4 m, while before and after stayed under 0.154 m.

Two checks were removed. `jerk_stationary` (2026-07-23) re-verified a documented limit
of the travel gate instead of testing recovery. A no-leak-before-motion check
in `jerk_with_motion` (2026-07-26) failed for reasons unrelated to the
correction.

#### Geometry constants

`OBSTACLE_XY = (0.0, 0.0)` is the world origin, where the box and robot both
spawn, so loop centre and box coincide by construction.

`OBSTACLE_LOOP_LEGS` is a 3m square there, corners at (+-1.5, +-1.5), widened
from 2m on 2026-07-26 (`4f182e7`). Legs are `(vx, vy, duration)`, 0.75s at
4.0 m/s. Wall clearances are known on x only: +x clears `upper_mid`
(x=2.49) by 0.99m, -x clears `lower_mid` (x=-2.11) by 0.61m and
`bottom_wall`'s ramp edge (x=-3.35) by 1.85m. `lower_mid` is the tightest, so
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

`start_actor_driver` runs `ros2 run sim actor_driver` and waits for its
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

### Pose emulator: odom noise model

It sends what the MCB sends: `head_yaw` in the world (gz's joint is relative
to the chassis), velocity in the world (gz's twist is in the chassis frame),
and the chassis's heading as `chassis_yaw`, which the firmware doesn't send
yet. `sentry_v2` picks up ~1 deg of chassis yaw in hard corners; before
2026-09-29 `/dji_serial_bridge/pose` passed the relative joint and chassis-frame velocity
through, so the lidar's TF heading was off by that yaw.

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
- `odom_slip_ratio`: drops a fraction of each metre driven (0.5 means `/dji_serial_bridge/pose`
  moves 0.5m per real metre), like wheels spinning on the "Bumpy Road" zone.
  It grows with distance, where drift grows with time.

`trigger_jerk()` moves the gz robot by a random (dx, dy) and subtracts the
same (dx, dy) from the drift accumulator, so `/dji_serial_bridge/pose` doesn't jump. The encoders
never saw the move. The error appears when the next scan match disagrees and
corrects `map->odom`, and that correction is what the jerk tests. Fire one by
hand with
`ros2 service call /pose_emulator/trigger_jerk std_srvs/srv/Trigger`.

### Head slider relay

The gz GUI slider always publishes to `/model/<model>/joint/<joint>/<axis>/cmd_pos`.
ROS can't bridge that name (`parameter_bridge` raises `InvalidTopicNameError`
on the `0` token), so the xacro's controllers listen on a topic without the
axis segment, which `sim.launch.py` bridges as `/head_pan_cmd` and
`/head_pitch_cmd`. `JointPositionController` takes one `<topic>`, so this
relay forwards the slider to it.

The image has no gz-transport Python bindings. The relay reads with a
long-lived `gz topic -e` and writes with a fresh `gz topic -p` per message,
each costing tens of ms of discovery. When one blocking loop did both, slider
ticks queued and the head crawled toward stale positions. Reader and publisher
are now separate threads sharing the latest value, with an `Event` that drops
values arriving mid-publish.

### Auto explore: teleport

Teleport writes the gz world pose through `/world/<world>/set_pose` via
`gz service`; ROS has no equivalent. It works because root is a free 6DOF body
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

### actor_driver: moving boxes

`actor_driver` spawns `count` boxes (0.3 x 0.3 x 0.8 m) with
`ros_gz_sim create` and walks each back and forth along a segment
(`paths`, four numbers per actor) at `speeds` m/s, calling `set_pose` on
every box each tick. Ticks run on a sim-time timer (`rate_hz`, 10), and each
advances the box by speed times the sim time since the last tick, so a slow
tick makes a longer jump and the actor keeps its speed.

The boxes are free bodies with no collision and gravity off. gz only
honours `set_pose` on a free body (see the Auto explore note), and a
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
from every edge and always clear, across the -x, +y and +x edges.
`closest box` in the log is the nearest a box centre got to the robot's.

Moves go through `sim.launch.py`'s `set_pose_bridge`, gz's `set_pose` as a
ROS `SetEntityPose` service, one async call per box per tick. The first
version started a `gz service` process (Ruby) per call, and ticks lagged to
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

### Target driver / CV target emulator

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
tick. The twist is expressed in the `target` child frame, as ROS Odometry
requires; consumers rotate it by the pose quaternion for world-frame
velocity. `bench_world` publishes the same convention.
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
watchdog. Inside, `noise_pos_stddev` (0.005m), `dropout_probability` (0.03, not measured) and
`publish_latency_s` (0.06s, a placeholder) all default on.

On top of the isotropic noise sits a D435-shaped ray model, since the tracker
has to be tuned on the noise it will meet: depth std `noise_depth_range_coeff *
range^2` (0.0036, 3 cm at 3 m, from the stereo baseline and a 0.08 px subpixel)
and bearing std `noise_lateral_rad * range` (0.003, about a pixel). Both are
datasheet estimates; measure them on the robot. Set both to 0 for the old
5 mm-only noise.

### target_state_truth: the aiming bench's perfect knowledge

Stands in for the whole of Part 2 on the aiming bench (`shot_hit.launch.py`).
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

### point_shooter

Our chassis on the aiming bench. A second `target_driver`, named
`shooter_driver`, with no spin and its output remapped to
`/shooter/ground_truth_odom`, bounces along y at `shooter_speed` with the
same braking; `point_shooter` republishes each sample at once as `odom->root`
TF and `/dji_serial_bridge/pose` (`RobotPose`, root-frame velocity), stamped with its sample
time. No noise or latency, so our motion is as perfectly known as the
target's. It has no chassis spin: `root` is heading-fixed and the point
shooter's gimbal is perfect, so a spin changes nothing it scores. The
estimation bench's `chassis_spin:=` is where spin is tested. The harness flies each shot from `root`'s interpolated position at
exit with `root`'s velocity added, as a real projectile would carry it.

### Sim clock

`/clock` for stacks with no gz: `rate` sim seconds per wall second, stepped
by measured wall time at 1 kHz.

### CV head aim

Subscribes `/cv/target` (from `thornbots_pkg`'s `point_to_cv_target`, so run
`auto.launch.py` alongside `sim.launch.py spawn_target:=true`) and
`/sim/raw_joint_states`, and publishes `/head_pan_cmd`/`/head_pitch_cmd`: the
sim's gimbal when the full stack runs in gz. No test scores its shots; the
aiming bench has its own perfect gimbal. `CVTarget.x/y/z` is an `odom`
position; each tick it goes into root at the newest `odom->root`, the way
the MCB holds it, then through the solve below.

`cv_head_aim_core::solve_head_angles()` inverts the FK chain from root to the
`muzzle` frame (root -> body -> headlink(yaw) -> headpitch(pitch) ->
muzzlelink). `test/cpp/test_cv_head_aim.cpp` checks it against a separately
written FK, and `test_urdf_constants.py` pins the C++ core header's constants to
`thornbots_pkg`'s URDF and checks sim's model against it.

The `muzzle` frame sits on `head_pitch` at (0, 0.1128, 0): on the pitch axis,
between the two stacked flywheels. In the CAD the barrel is 0.3915 m up and
the pitch axis 0.3906 m, so the barrel really does sit on the axis. In the head
frame that puts the muzzle `MUZZLE_Y` = 0.0127 m left of the yaw axis and
`MUZZLE_Z` = 0.391 m above root.

It aims along the ray from the muzzle. Before 2026-09-09 it aimed from root,
assuming a ~0.35m offset wouldn't matter at range. That's true for flight time,
but a parallel offset in direction stays the same size at any range. On the
old model every stationary shot missed by ~0.33m against a 0.05m hit radius.

The solve is closed-form even though the muzzle's position depends on the yaw
being solved. The shot leaves along the head's +x, and whatever the yaw that
line passes `MUZZLE_Y` to the left of the yaw axis. So azimuth is
`bearing - asin(MUZZLE_Y / horizontal_range)`, with bearing and range taken
from the yaw axis, and pitch follows from the elevation seen from the muzzle
at that azimuth. The muzzle is on the pitch axis, so pitching never moves it.
Head yaw is the azimuth, because `headlink` turns about +z, CCW like the MCB's
`head_yaw`.

Type-C probably has the same bug. It receives a world-frame position and runs its
own gimbal solve, and only the firmware knows where the real barrel sits, so
raise it with the firmware team (see `ros2_dji_serial_bridge/README.md`).

Every `control_rate_hz` (30) tick it commands `current + gain *
wrapped_error`. A timer, not `/cv/target` arrival (40Hz), drives it;
per-message updates let the setpoint race ahead of the joint in early tuning.
`gain` is 1.0, the IK angle itself, leaving tracking to gz's joint PID. At the
old 0.3 and 15Hz the head trailed a 0.5 m/s target by 9.4cm against a 5cm hit
radius (2026-09-17), a lag no 1kHz gimbal loop would have. The old `sign_yaw`/`sign_pitch` params are gone, because the IK geometry
sets the sign and the test checks it analytically.

With no `/cv/target` it stops publishing and holds position, without
re-homing, since a lost target is usually a brief FOV gap. `CVTarget` has no
confidence; every message is an aim point, patrol points included, and the
head follows each.

### MCB emulator

The emulator compiles and runs the checked-out `firmware/MCBV3` C++ sources.
The Python control port has been removed. `mcb_firmware.py` transports hardware
readings to `MCB-project/src/hosted/main.cpp`, which instantiates the actual
`SentryControl` and runs its command scheduler in 1 ms steps. UART1 uses the
PTY attached to the real `dji_serial_bridge`; MCBV3's own DJISerial parser,
mailbox, JetsonSubsystem, ballistics, aim/fire and drive commands run unchanged.

From the workspace root, in the ROS container:

```sh
git -C src submodule update --init --checkout firmware/MCBV3
git -C src/firmware/MCBV3 submodule update --init taproot-scripts
apt-get install -y scons g++-11
src/sim/tools/build_mcb_firmware.sh
colcon build --packages-select sim --symlink-install
ros2 launch sim mcb.launch.py
```

Use `drive:=simple` for the firmware's waypoint route or `drive:=auto` for
NAV_GOAL; the default `stop` parks the robot. Foxglove serves on :8765, and
gz/rviz open where a display is available. This launch uses the actual MCB
and bridge without an opponent; the `e2e.launch.py` stages score it.

`firmware_binary:=/path/to/MCB-project.elf` selects another hosted build;
`MCB_FIRMWARE_BINARY` supplies the default for both the node and tests.
The normal default is the firmware's `build/sim/scons-release/MCB-project.elf`.
Re-run the build after firmware edits: SCons tracks the original sources and
headers. There is no Python `firmware_fixes` overlay; the firmware under test
is exactly what the checked-out C++ implements. This is a host executable,
not the STM32 ELF flashed to the board.

The fake interfaces are calibrated IMU and joint-encoder readings, Pico
odometry, centred DBUS input with both switches up, referee UART frames,
ADC and CAN motor feedback. The DBUS and referee packets go through the
real parsers. Referee state remains live node parameters. The firmware's
start constants are read from the executable and used to map gz truth into
the pods' frame, so its field-frame POSE agrees with the gz spawn.

The fixture replaces boot calibration and hardware polling. CAN feedback
currently keeps motors online with zero shaft speed/current; ADC returns 0.
gz's joint position and chassis velocity controllers apply the firmware's
setpoints, so motor dynamics, CAN timing, MCU interrupts, calibration and
physical homing are not validated. A shot records a successful native
indexer request, not measured projectile exit. `/mcb_emulator/shot` is a
`std_msgs/Header` stamped with that request's sim time.

Run the native control and real-bridge checks without gz:

```sh
cd src/sim
python3 -m pytest test/mcb -v
```

The native tests skip if the binary is absent; build it first to test the MCB.
The bridge test also needs ROS. Pure wire-format behavior is covered by GTests.

#### Firmware behaviour and historical results (then stage E2, now `mcb_parked`)

Where the firmware and `UART_PROTOCOL.md` disagree is listed, with
MCBV3 file:line, in `../ros2_dji_serial_bridge/README.md` "Where the
firmware stands". Behaviour that isn't a wire gap but changes what `mcb_parked` can
score (paths under `MCB-project/src/`):

- It aims at the latest `CvTarget` for 200 ms after it arrives: the field
  bearing from its odometry less the start's yaw (the IMU's zero), pitch
  from `Reticle::solveForPitch`, no lead (`AutoAimAndFireCommand.cpp:66-90`).
  Before the first frame it doesn't aim; `CvTarget{}`'s default flags patrol.
- Current field-frame aim and pitch-height handling are documented in
  [firmware coordination](../ros2_dji_serial_bridge/README.md#where-the-firmware-stands).
  The old ground-z/pivot-z mismatch is fixed in pinned MCBV3 `nightly`.
- One `tryShootOnce` per fire frame, `delay_ms - 80` after it arrives; under
  80 the `uint32` timeout lands in the past and it fires at once. The
  indexer's 50 ms minimum caps that at 20 Hz, so 40 Hz frames fire every
  other one.
- `RELOCALIZE` moves odometry at once, anywhere, any HP
  (`JetsonSubsystem.cpp:76-81`). The RFID relocalize in
  `SimpleAutoDriveCommand` is gone; stuck 15 s, it spins in place.
- With an expired aim it patrols only if the last frame set
  `TYPE_C_BASED_PATROL`. A hit turn with `TURN_TO_HIT` set can override a
  live CV aim; see [firmware coordination](../ros2_dji_serial_bridge/README.md#where-the-firmware-stands).
- The sentry drives `SimpleAutoDriveCommand`'s ARCC route, spinning -8 rad/s
  moving and -12 at either end. The normal switch schedules this command,
  not the NAV_GOAL reader; `mcb.launch.py drive:=auto` selects that reader
  explicitly. RMUL 3v3 waypoint advancement waits for IN_GAME, while the
  simple route can still spin before the game. The `mcb_drive` and
  `mcb_match` stages park firmware drive and use `match_driver` for chassis
  movement; their results do not validate the firmware route.

E2 on `position-based-cv` with only the `delay_ms` fix (2026-10-03, container,
lateral, still and 1 m/s, 15 s each): `CV_TARGET` and `RELOCALIZE` frames
are read, none refused, but the gun turns away. One `RELOCALIZE` moved the
parked MCB's odometry to (0.74, 1.86), our `odom`'s numbers, and the head
sat at `head_yaw` 4.13 rad with the opponent straight ahead. Still: no
valid `TargetState`, no shots. 1 m/s: the tracked centre 3.17 m off, 0 of
6 shots hit. `pose_translator` reads POSE's x right, y forward as REP-105,
so our `odom` and the MCB's odometry differ by a turn.

With all three fixes (2026-10-03, container, still lateral cell): the
aim lands within 3 mm of the panel and the barrel 1.55 deg above it, a
24 m/s shot's drop over 3.2 m, so the stages now score shots under gravity.
Then 28 of 40 hit (70%, barrel 0.36 deg off the aim, miss 0.020 m)
on a clean track (centre 0.097 m), 2 of 40 and 1 of 30 on runs where
the tracker read 1.6-1.9 m/s for the still target (T17).


### Driving and combat diagnostics (`mcb_drive`)

```bash
ros2 launch sim e2e.launch.py stage:=mcb_drive
```

Our sentry starts at blue's provisional firmware spawn `(4.625, 0)`,
facing red; the opponent starts at `(-4.625, 0)`. Both travel around the
south ends of the side walls and stop in the center. Our chassis then
parks, translates at a 1 m/s cap, makes a 90-degree corner, and spins at
9 rad/s. The approach reaches a 2 m/s cap; acceleration is 2 m/s².
These are scripted sim routes, pending a real navigation publisher.
`match_driver` owns `/cmd_vel`; firmware drive output is remapped to
`/mcb_emulator/cmd_vel`. The real firmware retains aim/fire, UART, pose and
referee output, and the real AMCL + rf2o + EKF stack remains in the loop.

`segments.json` reports hit rates and the measured failing hop per segment,
checked down the hit path: no shots, aim off the panel, head-TF stamps, then
the barrel off the aim. Localization (`map->root`) is never the diagnosis.
`shots.jsonl` includes route, localization, head-TF and aim errors at fire
time. `route.jsonl` captures those diagnostics even when no shots fire.
A diagnostic-tier pass means nodes and clock remained live, every route
segment completed within 0.40 m p95, stamped localization data existed,
and a low hit rate named a measured failing hop, as allowed in E2E_PLAN.
It does **not** mean combat accuracy passed: floors need repeated valid
runs, and current moving runs expose localization and tracking failures.
The spawn coordinates are firmware defaults, not surveyed starting zones.

### Four-robot center fight (`mcb_match`)

```bash
ros2 launch sim e2e.launch.py stage:=mcb_match headless:=true
```

Two blue and two red sentries leave their respective spawns on separate
field-safe routes. The second lane leaves four seconds later; firing stays
disabled until every route reaches center. Our robot runs `mcb_drive`'s center
maneuvers; the three ghosts spin and use a truth-fed aimer with seed 2026,
0.015 rad Gaussian aim noise, and a 2 Hz fire rate. The selector sees both
red opponents and the blue ally. Only our robot runs the real CV stack.

The scorer advances 25 m/s ballistic shots under gravity against the field
mesh, chassis hulls and canted armor. The first impact absorbs the shot;
damage requires more than 12 m/s relative normal speed, the exposure cone,
and the panel's 50 ms dead time. Each robot starts with 400 HP; hits cost
20 HP. Defeated robots stop moving and shooting. Referee HP, team, stage,
time and hurt panel feed the hosted firmware's physical referee parser;
the test checks HP and team return through `REF_SYS` on the Jetson UART.

`match.json` reports per-segment route and localization error, enemy hit
rates, HP, friendly intersections, damage taken and shot counts per robot.
`shots.jsonl` records first impacts and HP after each resolved impact.
The approach checks routing with firing disabled; center segments use the
same diagnostic completion criterion as `mcb_drive`. Accuracy floors, repeated-run
spread and standalone-versus-sequence equivalence remain uncalibrated.
Detector occlusion, non-chassis appendage blockers, heat, ammo and respawn
are not modeled. Hurt-panel feedback is supplied, but the firmware's
`delta_angle_got_hit_in` response still needs separate verification.
