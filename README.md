# sim

sim holds the robot's integration tests. The localization drift suite starts a
`gz sim` model of the `ARCC_Field_2026` field, spawns the `sentry` robot from
`urdf/sentry_v2.urdf.xacro`, and runs the real `thornbots_pkg` stack against
it. The two CV benches run no gz. The aiming bench scores `point_to_cv_target`
against a perfectly known target; the estimation bench scores how well Part 2
turns noisy detections into a target model. The split is documented in the
[CV interface](../thornbots_pkg/README.md#cv-interface); historical results
are in [CV bench observations](docs/cv-bench-results-2026-09-28.md).

`ament_cmake` builds the runtime nodes, Gazebo components, GTest harnesses,
unit tests and offline CLI tools in C++17. Python remains for ROS launch
files, their small display/event/constant adapters, and ament lint tests.
The [native port map](#native-port-map) identifies the shared implementations.

`/cmd_vel` remains a bare `Twist` because gz's diff-drive interface and the
harnesses expect it; this is the workspace timestamp rule's standard-interface exception.

## Run the tests

The hosted MMM end-to-end suite runs compiled sentry firmware, the real UART
bridge and `auto.launch.py`'s `mmm`, SLAM reset and map save service. Initialize
MCBV3 and its build tools, build the [hosted firmware](#mcb-emulator), then
build `sim` with the matching firmware checkout present. Run in a fresh,
isolated ROS domain with no other stack:

```sh
MMM_EVIDENCE_DIR="$HOME/mmm-$(date +%Y%m%d-%H%M%S)" ros2 run sim mmm_suite
```

Use a Linux filesystem for this directory; Mac shared mounts can reject the
standard library's file copy during map finalization. Archive the directory
before removing a disposable container. The suite keeps frame traces, process
logs, measurements and map bundles. It covers telemetry load, bidirectional
drop/CRC faults, UART backpressure, bridge/firmware restarts, the sole PING
publisher, mapping restrictions, setup/save failures and validated saving.
The private hosted control socket supplies start/save and DBUS sticks without
operator bindings. The hardware build excludes it; firmware unit tests check
that boundary. LED indication is inferred from Ready state, while beep timing
comes from the firmware's Ready timer. Hosted loop execution time does not
represent board scheduling. This suite applies firmware velocity setpoints to
Gazebo's velocity controller; it does not measure the CAN motor plant or board
drive dynamics. Motor-force tests remain in the MCB physics suite.

During bridge restarts the harness keeps stepping firmware and servicing ROS
while the owned process shuts down. This preserves control and callback timing
through the disconnect instead of introducing a shutdown-length relay pause.
The boot snapshot records chassis heading independently of turret motion.

The `BothTeams/TeamBoot.*` cases use the real firmware/bridge and Gazebo scans
with REF_SYS held at the UART relay. They assert AMCL is absent through missing
and invalid team data, then release the real team frame and check raw `/odom`
near zero, `/localization/odom` near the red/blue field start, and the absolute
wire RELOCALIZE within the relay's 0.05 m threshold with rf2o/EKF enabled.
They drop the first RELOCALIZE and require a later absolute retry to unlock
actual firmware navigation. A goal at that fix must command no translation;
a displaced goal must drive. Separate both-team mapping cases check the first
published map pose is `(0, 0)` while navigation keeps its field offset. Run these cases
with a fresh evidence directory:

```sh
MMM_EVIDENCE_DIR=/tmp/team-boot-evidence ros2 run sim mmm_suite --gtest_filter='BothTeams/TeamBoot.*'
```

No scored stage runs YOLO or camera occlusion; detections are truth stand-ins.

Build first on a fresh container (see Build). Each test starts its own sim and
`thornbots_pkg` stacks, so stop anything you already have running.

The localization suite runs `amcl` with the EKF by default, like
`auto.launch.py` (`backend:=` picks another). Its ten scenarios
are listed under Notes:

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim localization_tests.launch.py
```

`suite:=ekf` runs the EKF ground-truth test instead. It is on demand, not part
of the standard run: run it after changing the EKF or rf2o config.

The CV aiming bench runs ten shot-hit cases, all with lead on: a stationary
target, then 0.5, 1, 2 and 4 m/s, first with flat panels and then with
neighbouring panels staggered 10.39 cm apart vertically (about 83% of a
panel's 12.5 cm height, with even-numbered panels lower)
(`panel_layout:=` picks one). It launches the stack once and only changes the
target between cases; each case settles for 3s, then scores 30s of sim time.
The bench fires at up to 40 Hz, far above the real launcher, and scores each
case on hit rate and hits per expected shot equally (the `total`
blend in `test/cv/shot_hit_suite.cpp`), so falling behind 40 Hz costs points.

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
the newest stamp on `/cv/target_state`, `/dji_serial_bridge/pose`, `/cv/target/tick` and the
scorer's `/bench/progress` until that topic publishes, so no node falls
behind and each 40 Hz tick is stamped on its deadline; a topic with
no publisher left (the scorer between cases) stops holding it. A fixed
`real_time_factor` above 0 doesn't wait: 8x lost 2-5 points. On 2026-09-25
the paced clock ran 6.6-6.8x and scored every cell within half a point of
three 4x runs, `keep_up` 1.00. Use `:=1` as the control.

`chase_settle_s` picks `point_to_cv_target`'s spin mode: `>= 0` (the default,
0) chases the facing panel and fires every tick, `< 0` is shotgating: hold
the center line and time the fire. `gimbal_lag_s` defaults to 0, the perfect gimbal. Each cell has
its own floor in `FLOORS` (`include/sim/shot_floors_data.hpp`), the lowest score
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
ros2 run sim shot_floors run1/ run2/ run3/   # each a log_dir:=
```

```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch sim shot_hit.launch.py
```

To compare panel switching with a stationary target initially facing 45 deg,
set `only_stationary:=true target_yaw_deg:=45`; for a slow turn at 0.2 rad/s,
use `target_spin_hz:=0.0318309886`. `panel_switch_margin_rad:=` optionally
overrides the aim node's margin; empty leaves its own default in force. For
three runs at each margin for both scenarios, use distinct log directories:

```bash
for repeat in 1 2 3; do
  for margin in 0 0.2 0.3; do
    ros2 launch sim shot_hit.launch.py only_stationary:=true target_yaw_deg:=45 panel_switch_margin_rad:=$margin log_dir:=/tmp/shot45-$margin-$repeat
    ros2 launch sim shot_hit.launch.py only_stationary:=true target_yaw_deg:=45 target_spin_hz:=0.0318309886 panel_switch_margin_rad:=$margin log_dir:=/tmp/slow-spin-$margin-$repeat
  done
done
```

`model_yaw_jitter_rad` adds a synthetic 4 Hz sinusoid to the published target
model's yaw only. Its default is zero, which preserves exact truth; true target
odometry and panel-hit scoring stay unchanged. `scores.jsonl` records the
scenario settings, aim yaw and pitch total variation, and selected-panel flips.
Angles are measured from each scored CV aim to the newest shooter position;
total variation sums shortest angular differences. A selected panel is the
nearest predicted panel center to the aim point at the aim stamp plus a rough
flight time (aim range divided by muzzle speed); the first sample establishes
the panel and is not counted as a flip. Future panel centers use current
truth velocity and angular rate beyond the newest odometry sample. The
[2026-10-11 measurement](docs/panel-choice-2026-10-11.md) compares exact-yaw
controls and synthetic jitter; no measured hit-rate floors are assigned to
these custom cells.

The suite launch commands return a nonzero exit status when GTest fails,
including collection errors, after shutting down their stack. Check the
GTest summary as well as the measured hit rates: diagnostic-tier completion
is not an accuracy guarantee. `tools/check_bench_log.cpp` also rejects failed
GTest summaries and clock or lockstep stalls.

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
`process_noise_radius:=`, `panel_radius_m:=` and `radius_prior_std:=` override
the tracker's radius noise, initial radius and radius prior standard deviation;
empty values use the node defaults declared in `target_tracker.cpp`.
`log_innovations:=true` opts into the tracker's innovation diagnostics.
The [G.1 investigation](docs/cv-estimation-g1-2026-10-15.md#matched-radius-and-prior-trials)
records the radius/prior screens and their full-matrix failures; no tuning
default or estimation limit is changed by that checkpoint.
`chassis_spin:=9` spins our chassis (rad/s, CCW) as a stand-in under
the head; firmware sentry spin is clockwise. The head holds world yaw as
the MCB's IMU loop does;
`yaw_bearing_damping:=` (N m s/rad, default 0, unmeasured) lets the spin drag
the head. A cell
passes on liveness until `LIMITS` in `include/sim/estimation_limits_data.hpp` has
limits for it. The table has 72 keyed cells, including 12 `chassis_spin:=9`
cells. The two default `stationary45` cells (`chassis_spin:=0`) still have
no p95 limits and pass only state-presence/valid-fraction checks. A 12/12
default run therefore does not establish accuracy for those two cells:

```bash
ros2 run sim estimation_limits run1/ run2/ run3/   # each a log_dir:=
```

`--keep include/sim/estimation_limits_data.hpp` keeps every cell the new runs
don't score, so one axis can get limits without rerunning the rest.

`estimation.jsonl` has one summary line per case and
`estimation_states.jsonl` one line per scored state.

The bench is one launch tree: the stack and the GTest that scores it. The
localization launch runs GTest, and GTest starts the sim once
(`run_tests:=false part:=sim`) and a fresh robot stack for each scenario
(`part:=robot`). Between scenarios it stops the robot stack, teleports the
robot back to spawn, removes anything a scenario spawned, and resets
`pose_emulator`'s noise state and parameters. `restart_sim:=true` brings the
sim up fresh for every scenario instead, the old behaviour and the control when
a verdict looks off. `spawn_yaw_deg:=5` turns the chassis that far off the spawn heading at every
reset; the real robot drifts 1-5 deg. Both
shut down when the tests finish, and Ctrl-C stops everything, stacks included.
Each shared-sim robot startup gets one 30-second wait for its complete TF chain;
a failure ends that case without restarting the robot. Startup diagnostics name
the raw `odom->root`, navigation `nav_odom->odom` and backend `map->nav_odom`
links separately. E2E bring-up failures also report which links are missing.

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

`tools/run_suite.sh drift|ekf|shot_hit|estimation|mcb_parked|mcb_drive|mcb_match [args]` wraps any of these
launches: it refuses to start over a running stack, tees the output to
`/tmp/sim_suite_*.log`, then runs `tools/check_bench_log.cpp` on it. It exits 0
only when GTest passed and the checker trusts the run:

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

`colcon test` runs C++ unit tests and the ament lint checks for launch Python.
Integration executables are installed but are invoked only through launch.

| Tier | Files | Needs |
| --- | --- | --- |
| unit | `cpp/test_cv_head_aim.cpp`, `cpp/test_mcb_protocol.cpp`, `cpp/test_combat.cpp`, `cpp/test_target_odometry.cpp`, `cpp/test_depth_mm.cpp`, `cpp/test_panel_view.cpp` | GTest |
| unit | `cpp/test_cv_bench.cpp`, `cpp/test_suite_timing.cpp`, `cpp/test_match_routes.cpp`, `cpp/test_foxglove_layouts.cpp` | GTest |
| unit | `cpp/test_mcb_firmware.cpp`, `cpp/test_mcb_pty_link.cpp`, `cpp/test_mcb_bridge_pty.cpp` | hosted firmware; bridge test also needs ROS |
| ROS service | `cpp/test_e2e_referee.cpp` | scorer retries and bounded failure with dropped replies; no simulation |
| launch API | `cpp/test_suite_exit.cpp` | ROS launch, no simulation |
| lint | `test_copyright.py`, `test_flake8.py`, `test_pep257.py` | Python + pytest |
| integration | `localization/localization_suite.cpp` | gz-sim and a launch tree |
| integration | `cv/shot_hit_suite.cpp` | a launch tree, no gz |
| integration | `cv/estimation_suite.cpp` | a launch tree, no gz |
| on demand | `localization/localization_suite.cpp` | gz-sim and a launch tree |

The integration suites are separate executables, so a plain `colcon test`
does not start Gazebo or a bench stack:

```bash
colcon test --packages-select sim
colcon test-result --test-result-base build/sim --verbose
ros2 launch sim localization_tests.launch.py
ros2 launch sim shot_hit.launch.py
ros2 launch sim estimation.launch.py
ros2 launch sim e2e.launch.py
```

The on-demand EKF ground-truth tier runs after an `ekf.yaml` or rf2o change:
`ros2 launch sim localization_tests.launch.py suite:=ekf`.

An integration run ends with a `suite timing` table: wall seconds per case
in sim start, bring-up, reset, settle, scored and teardown, and the RTF.
Every bench serves Foxglove on port 8765 (`foxglove:=false` to skip). A GUI
run also opens the gz and rviz windows where a display opens, and runs
without them where none does. The CV benches fail a case whose stack lost
a node.
Before trusting a run's numbers, check its launch log, which `ros2 launch`
names at the top of its output:

```bash
ros2 run sim check_bench_log ~/.ros/log/<run>/launch.log
ros2 run sim check_bench_log /tmp/localization_drift_tests/*.log  # drift and EKF suites
```

It prints the result and the timing table, and exits 1 when a node crashed
mid-run, a GUI couldn't open or a wait gave up.

rf2o's match grades (`/scan_odom/quality`), for setting
`sentry_localization/config/rf2o.yaml`: record beside a suite, then
summarise.

```bash
ros2 run sim rf2o_quality record /tmp/q.jsonl   # Ctrl-C when the suite ends
ros2 run sim rf2o_quality summary /tmp/q.jsonl
```

The drift suite starts gz-sim once and a fresh `thornbots_pkg` stack for each
scenario, resetting the sim between them (see "Run the tests");
`restart_sim:=true` restarts gz per scenario instead. The aiming bench launches
its stack once for all its cases, through `shot_hit.launch.py
run_tests:=false` when GTest starts it. ROS topics are shared
across every process on the machine, so a stack you left running will corrupt
the measurements.

`clock_bridge.launch.py` owns Gazebo-to-ROS time publication and is included by
`sim.launch.py`. It uses the existing robot UDP-only DDS profile: a departed
reader's Fast DDS shared-memory port must not block the global clock callback.
Discovery peers, clock QoS and Gazebo timestamps are retained. The registered
`test_clock_bridge` exercises real clock delivery to five fresh ROS contexts,
checks continued advancement and teardown, and verifies the bridge maps no
Fast DDS SHM ports. It needs no Gazebo world.
The test explicitly uses normal bridge discovery: Jazzy's localhost-only RMW
mode adds SHM transport after loading XML, even for a UDP-only profile.

Rerun the drift suite whenever you tune `slam.yaml`, `amcl.yaml`, `ekf.yaml` or
the noise model. `gtest_args:=` forwards native GTest options on the
localization launch. All suite launches retain `pytest_args:=` as a
compatibility argument carrying native flags such as `--gtest_filter=*flat*`.

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
rate for each. Its pass conditions are in the `test/cv/shot_hit_suite.cpp`
assertions.

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

These add synthetic wheel-odometry error. Drift and jitter are gated by
`odom_noise_enabled` (default off); the
[pose emulator note](#pose-emulator-odom-noise-model) explains each one:

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

`e2e.launch.py` is the match test (`../docs/E2E_PLAN.md`). Every stage runs the
MCB emulator: the compiled sentry firmware (see Notes "MCB emulator") on a
pty, with `dji_serial_bridge` and `mcb_relay` on the other end, the real CV
chain from `target_selector` to `point_to_cv_target`, and no camera:
`detector_standin` feeds gz truth in place of YOLO and `roi_depth_node`.
Shots are the ones the firmware fires, each falling under gravity since the
firmware pitches up for it. All three stages use the robot's no-patrol
`point_to_cv_target` behavior: last-enemy and hit holds, then a look toward
the enemy starting zone. Idle frames clear `fire` and keep hit turns permitted;
firing requires a detection no more than 0.15 s old. The MCB fixture
(`mcb.launch.py`) and both aiming benches (`shot_hit.launch.py` and
`estimation.launch.py`) use `idle_direction=hold` for their controlled setups.
Reacquisition and moving hit rates still vary (ROADMAP T17). The
[2026-10-11 hosted-firmware run](docs/match-check-2026-10-11.md) records all
cells/segments, checker verdicts and comparison with the previous status.

| Stage | Test | What it scores |
|---|---|---|
| `mcb_parked` (default) | `test/e2e/e2e_suite.cpp` | firmware ingress from team start, then our robot parked against `target_driver`; one test per speed and path |
| `mcb_drive` | `test/e2e/e2e_suite.cpp` | our robot drives spawn to center against one red sentry |
| `mcb_match` | `test/e2e/e2e_suite.cpp` | the 2v2 center fight with ballistic shots and HP |

`mcb_parked`'s field-safe paths are separate from the gz-free aiming bench:
relative to the settled shooter, lateral at y=-1.3 with x ±1.9 m, radial
y=-0.9 to -1.7 m, diagonal centered at (0, -1.3), half-length 0.65 m.
Mesh regression checks include
a 0.40 m robot footprint. Short paths brake before reaching high requested
speeds; labels are speed limits. Each cell fails on zero firmware shots or
a hit rate under its floor. Shots `CVTarget.fire` asks for are logged
beside. `firmware_fixes` is deprecated; the hosted build runs this
checkout's code without overlays.

Stages default to board execution: follow the [Type C build](#type-c-board-execution-with-renode)
first, or pass `backend:=hosted` after the [hosted build](#mcb-emulator).

```bash
ros2 launch sim e2e.launch.py                                 # mcb_parked, all 12 cells
ros2 launch sim e2e.launch.py speeds:='0 2' paths:=lateral duration:=15
ros2 launch sim e2e.launch.py run_tests:=false target_speed:=2.0 target_spin_hz:=1.5
```

`mcb_parked`'s test-owned stack brings up a parked, non-spinning opponent
and arms firmware `SimpleAutoDriveCommand` through the modeled remote after
localization bring-up and three stationary sim seconds, with aim/fire disabled.
It waits for truth within 1 m
of field center and translation below 0.08 m/s for one sim second (120 sim
seconds maximum), selects remote MID to park, and lets the chassis settle
for three seconds. `ingress.jsonl` records position and translation speed.
The settled truth xy translates the original target paths, retaining their
-y orientation and shooter-to-target geometry despite firmware arrival tolerance.
It then enables aim/fire and waits for a valid `/cv/target_state` before the
first cell
(non-firing idle frames keep `/cv/target` fresh without one). Starting it at
the launch's moving defaults left unscored motion dependent on wall-time
bring-up speed. Both parked fixtures set the idle-look enemy zone toward
their -y paths; driving stages retain the field's -x enemy zone. The parked scoring gates
and firmware control laws are unchanged.
Each requested cell sets its speed/spin and settles for three sim seconds
before scoring.

`shots.jsonl` in `log_dir` (`/tmp/e2e_test_logs`) splits every miss into
the barrel's angle off the aim and the aim's distance from the panel, and
each case prints TargetState's centre, velocity and spin error against
truth. `pytest_args:='--e2e-spin 0'` holds the spin for every cell.
Each shot record also has `panel_right_m` and `panel_up_m`: where it crossed
the scored panel's plane, from the face centre, right as the shooter sees it
and up the canted face. Ballistic `mcb_drive`/`mcb_match` records take the
nearest front-side crossing of any panel along the flight (`crossing_panel`),
so near misses show their side too.
`states.jsonl` records stamped TargetState errors by case. `mcb_parked` shot
and state records also include head-TF and map-localization errors against gz
truth. The localization error is `map->root` truth error;
`navigation_error_m` is the `nav_odom->root` xy truth error at the same stamp.
`odom_disagreement_m` compares raw MCB POSE with TF
`odom->root` at that POSE's stamp. That agreement is what a hit depends on;
the aiming path uses raw `odom` and bypasses both navigation corrections.
Missing
stamped TF is reported as unavailable; latest TF is never substituted to
calculate these errors. `ros2 run sim compare_runs LOG_DIR_A LOG_DIR_B`
prints the first record where two runs' logs diverge.
`poses.jsonl` samples head-TF and localization at 20 Hz even when tracking
is lost and no shots fire. Every parked cell requires at least 95% finite
stamped map-localization samples, p95 error at most 0.15 m and maximum error
at most 0.20 m against gz truth. `parking.jsonl` records coverage and errors
alongside chassis displacement. Bring-up waits until `map->root` exists
(not that it has converged) before any parked scoring. `real_time_factor` reaches the test-owned stack;
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
ros2 run sim rviz_to_foxglove --package-dir src/sim  # --check: exit 1 if stale
```

`sim.launch.py` starts gz, spawns the robot, bridges its lidar, joint, odometry,
camera and head-command topics to ROS, and runs `pose_emulator`, which
publishes `/dji_serial_bridge/pose` the way the Type-C board does. It runs no
`robot_state_publisher`, so TF comes from `thornbots_pkg`'s `auto.launch.py`.
`/sim/raw_odom` and `/sim/raw_joint_states` are ground truth that real hardware
doesn't have; only sim and its tests should read them.

## Record an E2E demo

[Mac recording evidence](docs/e2e-video-2026-10-13.md).

The E2E GUI uses scene-only gz views and robot-following RViz views:
[a parked close-up](rviz/e2e-parked.rviz) and [a driving/match view](rviz/e2e.rviz). They show robot, TF, lidar, map, tracked armor,
cyan aim point/ray and scored MCB shot traces. The GUI-only `e2e_map_view`
fetches the initial map through the read-only map service and mirrors updates
to `/sim/e2e/map` once per wall second, so a missed initial DDS sample does
not leave RViz empty. The map viewer and RViz use the existing thornbots UDP
profile to avoid depending on shared-memory delivery of the large map. Green denotes an enemy hit and
red a miss. Traces appear after the scorer resolves the shot; they show the
historical ballistic path, not animated ammunition. Parked/drive endpoints use
scored flight range; match endpoints use the resolved first-impact time.
The aim ray connects the current muzzle TF to the held aim point. The
[chassis-yaw wire limitation](../ros2_dji_serial_bridge/UART_PROTOCOL.md#proposed-pose-chassis-yaw)
applies to RViz body orientation; gz shows physical chassis spin.

In a ROS container terminal with a 1920x1080 X11 desktop, install the optional
recording tools (`ffmpeg`, `wmctrl`, `python3-pil`, `python3-numpy`, `rclpy`).
Build the selected firmware backend first as in [MCB emulator](#mcb-emulator).
Use an idle ROS domain, a separate gz partition and a fresh output directory.
Then run from the workspace root:

```sh
python3 src/sim/tools/record_e2e.py capture mcb_parked /tmp/demo/parked
python3 src/sim/tools/record_e2e.py render /tmp/demo/parked /tmp/parked.mp4
```

Replace the stage with `mcb_drive` or `mcb_match`. Capture defaults to the
explicit hosted fixture; `--backend renode` selects the normal board ELF.
Parked capture defaults to lateral speeds 0/1 m/s with 35 scored seconds per
cell. `--speeds`, `--paths`, `--duration` and `--chase-settle-s` choose the
recorded cells/settings without changing the tests' scoring gates.

RViz and gz share one desktop and capture stream. The tool caps simulation at
RTF 0.25, records 20 wall-time frames/s and saves `/clock` against wall time.
Rendering crops the window chrome and enlarges the gz centre crop
(`--gz-zoom 1` keeps the full arena), then resamples the panes at 30 frames per
sim second: playback is 1x
sim time even when the actual RTF varies. The default scored telemetry window trims
bring-up and gaps between parked cells; `--start`/`--end` override it in sim seconds. Captions use absolute
sim time and resolved MCB hits/shots (`scored_at` in `shots.jsonl`), not fire
requests or ghost shots. Output is 1080p H.264/YUV420 MP4, with a JSON sidecar.
A failed test/log check retains raw capture/logs but cannot be rendered as a
passing demo.

## Notes

These notes explain why the code looks the way it does, so the in-code
comments can stay short. Each heading names a file.

### The world frame

The gz world is the field frame (REP-105, (0, 0) at the field centre, x
along the 12 m toward blue's base). Raw `odom` has the same axes but its
origin is the robot's boot position; it is not field-centred. Team initialization
seeds field-centred ROS navigation; the first absolute RELOCALIZE brings the
MCB's navigation coordinates into that frame. Saved field maps use the world frame; blank-map SLAM starts its
`map` at (0, 0) at boot, including when spawned at a team start.
`ARCC_Field_2026.sdf` uses the coarse Onshape export in
`world/real_field_assmbly/meshes/`: `Field.stl` (8,550 triangles) and
`CenterMarker.stl` (12 triangles), each for visual and collision geometry.
The export is already in metres with its centre at XY zero. Both meshes use
the URDF's z offset of -0.0133884 m and unit scale; a -90 deg model yaw
maps mesh +y to world +x, preserving the blue-base direction. The marker top
and main floor are at z = 0 (within 20 nm of STL float precision).
The inner walls enclose 12 x 8 m; the outer bounds in world coordinates are
x = ±6.168884 m, y = ±4.168884 m, z = -0.025..0.498612 m.

The earlier mesh had larger outer bounds (12.61 x 8.73 m), taller geometry
(1.41883 m before its z offset), and an approximately 8 cm lateral offset.
These are export differences, not a reason to rescale the new 12 x 8 m
interior. The coarse export does not establish precise RFID zone boundaries.
The ROS 1 launch files alongside the export are provenance; gz loads the SDF.

World-coordinate spawns, drift loops, actors and map-sweep grids keep their
field-frame coordinates. Both teams boot facing +y (field yaw π/2,
`auto_explore.SPAWN_YAW`); red starts at (-3.53, -0.426), blue at (3.53, -0.426).
These are temporary, unmeasured spooky scrim values matching the firmware (the
world stays the ARCC field; ARCC was (±4.625, 0)).
The ROS `root` remains heading-fixed at yaw zero. Both raw MCB POSE and
`pose_emulator` start at (0, 0); displacement and velocity retain the field
axes with no team rotation. RELOCALIZE changes only the MCB navigation offset,
which starts at zero. Autonomous navigation driving waits for that first
absolute fix; aiming continues to use raw odometry. AMCL waits for a valid MCB
REF_SYS team before it is constructed and seeded, including in simulation.
The pose-only fixture supplies no referee data: its caller must supply a
valid fixture REF_SYS when testing AMCL.

All MCB E2E stages, including the parked diagnostic, spawn at blue's field
start so referee-based AMCL seeding matches the world. Match routes retain
their field coordinates. The parked diagnostic drives to center before its
cells and translates its original -y target paths with the settled shooter,
preserving the former world-centre shooter geometry.
The [parked ingress verification](docs/mcb-parked-ingress-2026-10-31.md)
compares repeated hosted cells with the nightly baseline and records the board attempt.
See the [boot-heading verification](docs/boot-heading-2026-10-30.md) for
hosted firmware, MMM, physics and route test results and limits.
Route-clearance tests and match shot occlusion read every field collision
mesh and transform directly from the SDF through `combat::load_field`.
The corrected 2026-10-20 export restores the 2 m bar at world x = -1..1 m,
y = 2.98488..3.00393 m, with top z = 0.41 m; the earlier export omitted it
above the 0.208162 m platform. Placement, floor and outer bounds are unchanged.
For the canonical occupancy map derived from the corrected collision geometry
at nominal lidar height, see [saved map](../sentry_localization/README.md#saved-map).
Use mapping mode from a blank map for new-field SLAM validation (no pose graph ships).
The [2026-10-19 integration measurements](docs/real-field-2026-10-19.md)
compare the earlier export with nightly, including the known test failures;
they do not validate runtime behavior with the corrected export.

### SAPIEN: tried, not adopted

A SAPIEN engine (`sim_engine:=sapien`) was built as a faster stand-in for gz
and removed on 2026-09-24 before it ever ran a full suite. We are staying on
gz. The removed `sim/sapien_sim.py` is in the commit log if the question
comes back.

### urdf/sentry_v2: the team's low-resolution CAD

The model uses the team's **2026-10-26 low-resolution CAD** (`sentry_low_res_model`,
an Onshape URDF export of 67 links) for the chassis visual, yaw/pitch visuals and
collision hulls, sensor/muzzle frames, head joint origins and armor faces. It was
imported once into `urdf/sentry_v2/`; there is no generator, so edit the URDF and
meshes directly when the CAD changes. `thornbots_pkg` carries matching visual
assets and TF geometry, and `test_cv_bench.cpp` checks the two agree. See
[robot geometry](../thornbots_pkg/README.md#robot-geometry) for frame conventions
and the [dated investigation](../docs/investigations/sentry-cad-2026-10-26.md)
for the archive checksum, derivation and validation.

The yaw bearing is centred on the chassis axis; zero pitch points the gun
horizontally along +x, and the origin is on the ground under the chassis centre.
`camera` is the apex of the CAD's **RGB** FOV cone, 44 mm left of the IR cone,
because `/cv/panel_detections` are deprojected with the colour intrinsics. Before
2026-10-29 it sat on the D435 centre or IR apex, so real-robot aim points landed
right of the panels; the sim hid this because its detector shared the frame.
`muzzle` is on the pitch axis at the bore, 1.9 mm left of the yaw axis. `lidar`
uses the 2026-10-31 flipped-mount CAD translation with the robot-measured 2.579 rad
yaw ([lidar filter](../thornbots_pkg/README.md#lidar_self_filter)). The `head`
mesh still shows the upright Oct 26 lidar; the sim sensor does not see it.

Joint and link names (`root`, `body`, `head`, `head_pitch`, `lidar`, `camera`,
`muzzle`, `headlink`, `headpitch`) are stable; `headlink` turns about +z, CCW
like the MCB's `head_yaw`. Each `armor_i` frame sits at its face centre with +x
along the outward normal and +y horizontal, carrying a 135 x 125 mm box (the
Small Armor Module) 3 mm deep as visual and collision; normals are 15 deg up,
indices follow CCW bearing from +x and even indices are the lower pair.

The CAD's 103.8847 mm lower-edge spread exceeds the 100 mm stagger limit
([armor rules](https://github.com/Thornbots/thornbots_workspace/blob/428b294/docs/ARCC_2026_SENTRY_CONTEXT.md#armor-geometry-and-building-limits)).
Accepted on 2026-10-26 as a mechanical deviation with no material CV impact,
`UrdfConstants.ArmorRules` pins a temporary 0.104 m bound; restore 0.100 m after
the plates are corrected.

Not from the CAD: mass/inertia (the export's gram-scale values are not
credible), chassis collision, wheels and suspension (travel +-2 cm, 2200 N/m
per corner, damping), pitch limits (+-0.6 rad), controllers, plugins and topics
remain the earlier approximations. The two odometry pods are omitted; see the
[odometry pod table](../docs/investigations/sentry-cad-2026-10-26.md#odometry-pods).

`sentry_v2.urdf.xacro` wraps the URDF for gz: colours, the lidar and camera
sensors and the plugins.

### sentry_v2.urdf.xacro: motion model

Noise, gimbal dynamics, suspension and spawn coordinates remain estimates.

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
same world returned all 3000 beams at 2.22-7.17 m; the exclusion restored
that coverage on the sentry too. Both models now use the measured
angle-compensated A2M8 grid [below](#lidar-format-and-model-limits).

Hardware is the reason to prefer it: the real RPLIDAR's scanning disk sits
clear of the chassis, and only the head's own footprint blocks it.
`lidar_self_filter` models exactly that, in the one place that also runs on
the robot. Its scan-frame sector uses the Oct 26 robot measurement
([filter and evidence](../thornbots_pkg/README.md#lidar_self_filter)).
The spawned `sentry_v2` lidar joint matches the hardware TF's CAD translation
and measured scan-zero yaw; the sensor attaches to that joint, with no extra
sensor pose. The legacy `sentry` model retains its historical frame convention.

### Lidar format and model limits

Both robot models match the sentry's RPLIDAR A2M8 with
`angle_compensate:=true`: 720 fixed samples, inclusive -pi/+pi endpoints
(2pi/719 = 0.5007 degrees), 11.2 Hz, 0.05–12 m and frame `lidar`.
Keep the symmetric endpoints: rf2o derives bearings from the FOV around
zero, ignoring `angle_min`; a 0–2pi grid would rotate its interpretation by pi.
The Gazebo bridge feeds `/sim/scan`; sim's `lidar_scan` adapter supplies
`/scan_raw`, then the shared `thornbots_pkg` self-filter publishes `/scan`.
The adapter preserves the capture stamp, grid and intensities. It fills
`scan_time` from successive capture stamps (nominal 1/11.2 s at startup,
a reset or a missed frame) and `time_increment = scan_time / (N-1)`, as the
hardware driver does. Nonfinite and out-of-range returns become positive
infinity; finite boundary returns remain valid. This deliberately follows
the hardware missing-return convention, collapsing REP 117's distinct
`-inf` (too close) and `NaN` (invalid) sentinels into `+inf`.

Gazebo captures all rays at once: these timing fields describe nominal
rotary acquisition, without rolling motion distortion or the real driver's
roughly one-rotation delivery latency. Intensities are Gazebo values, not
calibrated RPLIDAR quality. Gaussian noise remains constant 0.01 m rather
than a measured range/material-dependent error model. Visual geometry and
the shared head blind sector determine no-return coverage; the real field's
38–44% infinity fraction includes material/geometry losses that Gazebo does
not model. No random dropout is added to force that fraction.

### Localization drift suite

Integration suite for `sentry_localization`'s drift and jerk correction
against the pose emulator's noise model. The current tree keeps raw MCB POSE
on `/odom` and `odom->root`; localization corrections are `map->nav_odom`,
and the fused navigation pose is measured through `nav_odom->root`. TF is
`map->nav_odom->odom->root`; see [the owning frame design](../thornbots_pkg/README.md#pose_translator).
The suite's backend axis is `--backend slam/mapping/amcl/none` (map-edge
owner) and `--use-rf2o` (whether scan matching informs the fused navigation
pose). For each scenario it resets the shared sim, launches the
robot stack, drives, samples the correction TF, asserts, and stops the robot
stack. The sim itself stops after the last scenario.

`test/localization/localization_suite.cpp` holds stack lifecycle (`run_stack`),
driving (`drive`) and the scenarios, with one parametrized `TEST_P` per
scenario; `Scenario`'s `details` go into the assertion message instead of
GTest's capture. The harness
launches the sim and each scenario's robot stack as trees in their own process
groups and won't attach to a running stack. The stack gets SIGINT if GTest dies, so a killed
run still tears it down; `sim::check_no_orphans` runs before and after the
suite.

`run_stack` picks the current navigation transform to measure:

| Backend | Edge | Gate |
| --- | --- | --- |
| `slam` | `map->nav_odom` | distance since last scan (`minimum_travel_distance`) |
| `mapping` | `map->nav_odom` | as `slam`, on a map it builds from blank |
| `amcl` | `map->nav_odom` | `update_min_d`/`update_min_a` |
| `none` | `nav_odom->root` pose | no map node; fused navigation pose, with raw `/odom` as the pod odometry baseline |

`--use-rf2o` changes the fused navigation pose and leaves `map->nav_odom`
alone. Dated measurements below retain the frame names used when those runs
were collected under the previous architecture.

`mapping` starts slam_toolbox on a blank map (`load_map:=false`) at map (0, 0).
This suite spawns mapping/none at world (0, 0), so their comparison frames
coincide. Its AMCL fixture spawns at blue's field start and publishes an
explicit valid blue REF_SYS; it does not bypass the production team gate. Like `none`, it scores
`map->root` against `/sim/raw_odom` in `noise_correction` and the
cornering-loop scenarios; the rest watch `map->nav_odom` as under `slam`. Each
scenario's stack builds a fresh map: nothing carries between scenarios.
`odom_stuck` drives two mapping laps first.

After AMCL's seeded navigation TF and valid scans are available, the fixture
drives the surveyed blue `sentry` ingress from
[`match_scenario.cpp`](src/match_scenario.cpp) and then to world (0, 0).
Truth-feedback velocity commands must reach every waypoint within 0.03 m;
a blocked ingress fails the scenario. Noise and slip injection begin only
after arrival, with the boot-relative odometry origin preserved. Scenario
reference poses are captured afterward, so the original loop, box and actor
geometry and acceptance thresholds still apply. This setup also runs with
`restart_sim:=true`.

`mapping --use-rf2o` passed 9/9
(2026-10-02, archlinux, unthrottled):
cornering loops 0.02-0.09 m truth error, `noise_correction` 0.04 m,
`scan_degraded` 0.43 m during and 0.07 m after. `--no-use-rf2o` fails 5/9
(Mac): the three 4 m/s loops at 0.41-0.42 m, `scan_degraded` 0.42 m after,
and `odom_stuck`, whose `map->odom` freezes as amcl's does without rf2o
(below), mapping laps or not. Under rf2o, `odom_stuck` passes its liveness
check while lost (1-5 m truth error), and mapping keeps writing to the map
the whole time.

Under `none`, `nav_odom->root` is the robot's own position, so its change says
nothing about error. `noise_correction` and the three cornering-loop scenarios
score `nav_odom->root`'s distance from `/sim/raw_odom` there instead, against the
same thresholds. `jerk_with_motion` is skipped for `none`: `ekf_node` fuses
`/odom` velocity only (`odom0_config`), with no travel gate, so neither
expectation is defined. With no map to miss a feature from, `drift_correction`
and `drift_correction_obstacle` should read about the same under `none`, and
so should `moving_obstacles`. They do: 0.022, 0.037 and 0.020 m, with
`noise_correction` at 0.040 m (2026-09-28, `--use-rf2o`, unthrottled).
`scan_degraded` fails under `none` (0.63 m during, 0.64 m after): with no
map layer nothing pulls the error back once the scan recovers.

amcl with and without EKF under slip, measured 2026-07-26 against a 0.30m
bound; verdicts shown against today's 0.40m `kMaxDelta`:

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
   with no ERROR in any log. Historical runs with the previous saved maps
   showed a steady ~0.1-0.15m map-to-world offset (measured before both were
   turned into the field frame, the same turn for each). That measurement
   does not establish alignment for the current [saved map](../sentry_localization/README.md#saved-map).
   A growing offset would be a real problem.
2. `noise_correction` drives the 3m square under drift and jitter (slip 0.02,
   no jerks) for a fixed 30s (60s before 2026-07-27). The second half's samples
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
   m/s. It scores like `drift_correction`, on `kMaxDelta`, and logs
   each sample's `map->root` error against `/sim/raw_odom`. It also fails if
   `actor_driver` dies mid-loop. Under `slam`, ../docs/ROADMAP.md T3 also wants the
   actors' cells checked in `/map` at the end; that check isn't built yet.
   Under `none` it scores ground-truth error, like `drift_correction`.
6. `real_accel` drives the `drift_correction` square with every leg ramped
   at 1.2 m/s^2 (close to the real chassis) instead of `--drive-accel`
   (default 20), so a 3 m side peaks near 1.9 m/s. Same metric and
   threshold as `drift_correction`.
7. `jerk_with_motion` (slam/amcl) models a collision impulse. Each trial fires
   `trigger_jerk`, drives one leg to the next corner, then requires
   `map->odom` to have moved more than 0.3x the applied jerk, or to sit within
   `kMaxDelta` of its pre-jerk value. Jerks (stddev 0.24) are biased toward
   the world origin so they don't push the robot into nearby walls, and the leg
   absorbs the actual (dx, dy) so the robot still lands on its corner. Eight
   trials share one stack (relaunching costs 15-20s each) and all must pass.
   A closing lap follows.
8. `odom_stuck` models a dead encoder: `trigger_odom_stuck` pins `/dji_serial_bridge/pose` x/y
   and velocity at zero with fresh timestamps. It checks liveness only, since
   there is no valid odometry to bound drift against: scans keep processing
   and pairwise TF spread exceeds 1cm.
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
   per phase. The blackout legs pass under 0.5 m, the rest under `kMaxDelta`
   (0.4 m). Across four runs
   (2026-09-27) the blackout peaked at 0.398-0.432 m, failing three against
   0.4 m, while before and after stayed under 0.154 m.
10. `parked_push` runs with `odom_slip_ratio` 1, so `/odom` reads still
   whatever the chassis does: a robot shoved with its wheels held. After 20 s
   parked it commands 1 cm/s for 4 s, then holds 5 s. It passes when truth
   moved 0.025-0.05 m and `parent->root` moved within 0.015 m of it.
   With rf2o enabled, `/scan_odom` must also track the displacement within
   0.015 m and report successful scan increments with no failed grades during
   the shove and settling interval. It guards rf2o's
   [reference scans](../rf2o_laser_odometry/README.md), which anchor a
   parked pose, against hiding motion under their drop distance.
   2026-10-25: truth moved 0.041 m; the estimate fell short by
   0.004-0.006 m with references (9 runs) and 0.003-0.011 m without (13).
   rf2o blends each match with the still wheel prior either way.

Two checks were removed. `jerk_stationary` (2026-07-23) re-verified a documented limit
of the travel gate instead of testing recovery. A no-leak-before-motion check
in `jerk_with_motion` (2026-07-26) failed for reasons unrelated to the
correction.

#### Geometry and helpers

Every cornering scenario traces `loop_legs()`: a 3m square centred on the
world origin, where the box spawns and scenario driving begins, with corners
at (+-1.5, +-1.5).
Each leg is `3 / --speed` seconds (0.75s at the 4.0 m/s default), and
`reposition()` drives to the start corner first. The square was widened from
2m on 2026-07-26 (`4f182e7`). Wall clearances are known on x only: +x clears
`upper_mid` (x=2.49) by 0.99m, -x clears `lower_mid` (x=-2.11) by 0.61m and
`bottom_wall`'s ramp edge (x=-3.35) by 1.85m. `lower_mid` is the tightest, so
start from it if you widen the loop. A 6-leg field tour that cleared every
wall's bounding box by ~0.77m still hit a wall after ~10 open-loop laps, as
small per-leg errors added up, which is why the suite uses the smaller loop.
`cornering` and `scan_degraded` dwell 1.0s after each leg so scan and TF settle;
nobody has validated 1.0s, so change it if the delta won't get under the
threshold.

`Helper::scans_flowing(10, 60.0)` decides when the stack is ready. slam_toolbox
and amcl publish an identity TF at startup before any scan, so waiting on TF
can start assertions on a cold stack. Once, under load, slam_toolbox registered
2 scans in 30+ seconds.

`jerk_with_motion` parses the applied (dx, dy) from the `Trigger` response
instead of assuming `odom_jerk_stddev`, since one draw can land far under its
stddev and the corrective leg needs the real vector. If parsing fails it logs
that and uses dx = dy = 0, so a change to the message format weakens the check
instead of crashing it.

`drive()` re-aims every tick at the leg's ground-truth endpoint from
`/sim/raw_odom` until within 0.03m. Its ticks, like `spin_for` and every
observe window, count sim seconds; the wall clock only guards against a
stalled `/clock`. A fixed Twist for a
wall-clock duration undershot whenever gz's real-time factor dipped under load;
gating on projected distance fixed undershoot but not lateral drift. Speed is
capped at `dist / 0.1s` so it tapers near the target. At full
4.0 m/s one tick covers 0.4m, which overshot the tolerance and oscillated at
every corner. The safety cap (3x `duration`, at least +5s) logs a warning
when hit.

`spawn_box()` creates a `<static>` box through `sim::gazebo::spawn_model`
(gz-transport) mid-scenario, after the pre-spawn baseline. `start_actor()`
runs `ros2 run sim actor_driver` and waits for its "all 3 actors spawned" log
line. `stop_actor()` runs first in every stack and sim teardown, and the next
`run_stack` removes the `spawned` models from the world.

#### Thresholds

`kMaxDelta` = 0.40 m covers `noise_correction`, both drift scenarios, the
post-blackout `scan_degraded` legs and the fallback pass for
`jerk_with_motion` (a small jerk can demand an unrealistically tiny
correction). It was 0.20 on 2026-07-26, 0.30 later that
day when no config reached 0.20 at 0.25 slip, and 0.40 on 2026-07-27 once the
chosen config (tuned `slam`, no EKF, 0.15 slip) measured 0.30-0.33m. See
`sentry_localization/README.md`'s tuning history.

`jerk_with_motion`'s 0.3 correction fraction: slam_toolbox plateaus at a
partial correction, typically 40-70% of the jerk, because scan matching
corrects the pose graph incrementally. 0.5 sat at that plateau's edge and
flaked; 0.3 keeps margin and still catches the known-broken case
(`minimum_travel_distance` at 0.5, indistinguishable from zero). It was never
checked against amcl's plateau, so suspect it first if amcl runs flake.

That calibration used 0.15 m/s and a jerk stddev of 0.3. Both have changed: 4.0
m/s, and stddev 0.5, then 0.08, now 0.24 for a ~30cm mean jerk
(magnitude of two N(0, stddev) draws is Rayleigh, mean
`stddev * sqrt(pi/2)`). Re-derive the plateau if pass rates look off.

On 2026-07-23 a correction step looped for up to 60s until the threshold was
crossed. The TF stalled for an unrelated reason, the loop never exited, and
open-loop drift took the robot off the field and crashed gz physics. Each
step now drives one bounded leg and takes one TF sample. CPU contention from
a stray rviz2 or other sessions slows scan processing to about 2
registrations in a ~35s run, so `jerk_with_motion`'s post-drive
`correction(5.0)` waits up to 5s.

### Pose emulator: odom noise model

Position follows the [boot-relative frame contract](#the-world-frame). Its
noise reset clears drift/slip state while preserving the boot origin.
It sends what the MCB sends: `head_yaw` in the world (gz's joint is relative
to the chassis), velocity in the world (gz's twist is in the chassis frame),
and the chassis's heading as `chassis_yaw`, which the firmware doesn't send
yet. `sentry_v2` picks up ~1 deg of chassis yaw in hard corners; before
2026-09-29 `/dji_serial_bridge/pose` passed the relative joint and chassis-frame velocity
through, so the lidar's TF heading was off by that yaw.

Sim ground truth has no wheel drift, so nothing would exercise `map->nav_odom`
correction. These params add it. Drift and jitter apply only while
`odom_noise_enabled` is true (default false; `sim.launch.py` passes it
through), so their nonzero defaults (0.0005 and 0.001 m) do nothing until
it is set. The other params act on their own:

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
corrects `map->nav_odom`, and that correction is what the jerk tests. Fire one by
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
~0.19 s of sim time. The default paths clear the real field's walls (the `ARCC_Field_2026`
mesh, [world frame](#the-world-frame)) by at least 0.4 m.

### sim.launch.py: spawn_robot uses -string

`spawn_model` can report failure after a slow `sentry_v2` spawn succeeds;
check `model_names()`. Spawning inside the colliding chassis stalls `/clock`.
For the other robot, `opponent_driver` uses `OpponentMover` instead of
VelocityControl.

`-topic robot_description` makes `create` subscribe over ROS, and it reliably
misses `robot_state_publisher`'s TRANSIENT_LOCAL message. `ros2 topic echo`
got it instantly on the same QoS while a matched `spawn_sentry` waited 30+
seconds. That's a `ros_gz_sim create` bug, not a race, so delays don't help.
`-string` passes the URDF text directly.

### EKF ground-truth suite

This suite (`localization_suite.cpp`, run by `localization_tests.launch.py
suite:=ekf`) asks whether fusing `/scan_odom` into `/odom` through `ekf_node` gets closer to
where the robot really is, which the drift suite can't answer. Drift scenarios
run with noise off, so only slip corrupts `/odom`; at zero slip
`pose_emulator` reports exact ground truth and no EKF can beat it. Old "EKF is
worse than raw /odom" numbers came from that setup and say nothing about the
EKF. Those scenarios also score `map->nav_odom`, while the EKF's edge is
`nav_odom->root`, dominated by the robot's own motion.

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

`cv_head_aim_core::solve_head_angles()` attempts to invert the FK chain from root to the
`muzzle` frame (root -> body -> headlink(yaw) -> headpitch(pitch) ->
muzzlelink). `test/cpp/test_cv_head_aim.cpp` checks it against a separately
written FK, and `test_cv_bench.cpp` pins the C++ core header's constants to
`thornbots_pkg`'s URDF and checks sim's model against it.

The `muzzle` aiming origin stays on the pitch axis between the flywheels;
see [robot geometry](../thornbots_pkg/README.md#robot-geometry). Only its
lateral offset is taken from the CAD bore centre. Its position is invariant
under pitch, as required by the existing closed-form solver.

It aims along the ray from the muzzle. Before 2026-09-09 it aimed from root,
assuming a ~0.35m offset wouldn't matter at range. That's true for flight time,
but a parallel offset in direction stays the same size at any range. On the
old model every stationary shot missed by ~0.33m against a 0.05m hit radius.

The existing solve is closed-form even though the muzzle's position depends on the yaw
being solved. It assumes the shot leaves along the head's +x, and whatever the yaw that
line passes `kMuzzleY` to the left of the yaw axis. So azimuth is
`bearing - asin(kMuzzleY / horizontal_range)`, with bearing and range taken
from the yaw axis, and pitch follows from the elevation seen from the unpitched
muzzle at that azimuth. The pitch-axis origin makes that elevation independent of pitch.
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
confidence; every message is an aim point, non-firing idle points included,
and the head follows each.

### MCB emulator

T31 hit-angle and report-latching status, plus a separate parked E2E
comparison and the hosted/Renode injected-hit physics checks:
[T31 verification](docs/t31-hit-contract-2026-10-14.md). The #14 angle-common
drive/match comparison is in the
[Thornbots hit-permission report](../thornbots_pkg/docs/hit-permission-2026-10-15.md).

Motor-path validation: [2026-10-11 report](docs/mcb-motor-plant-2026-10-11.md).
Raw aim/localization isolation and combat limits:
[controlled comparison and three-repeat results](docs/aim-localization-coupling-2026-10-11.md),
[follow-up frame/timing and repeatability investigation](docs/aim-followup-2026-10-12.md),
and the [2026-10-12 accuracy investigation](docs/aim-accuracy-2026-10-12.md).
The accuracy investigation records hosted-fixture measurements from a source
snapshot before track I; it does not establish board accuracy. Board boot,
UART/CAN and gz validation are in the [Type C report](docs/mcb-type-c-2026-10-11.md)
and [board auto-drive report](docs/mcb-auto-drive-2026-10-12.md).

`e2e.launch.py` defaults to `lockstep:=true`. The gz `LockstepGate` holds
in `PostUpdate` at each 5 ms MCB acknowledgment deadline. The gz MCB system publishes
`/mcb_emulator/tick` after each 1 ms firmware tick and application of its motor
outputs. `lockstep_coordinator` waits for that stamped
`std_msgs/Header` before releasing the next deadline. Its schedule is anchored
at sim zero; delayed acknowledgment
arrival never sets the phase. Startup runs free until the first acknowledgment.

`/sim/lockstep/status` reports holds, per-chain wall wait and timeout counts.
The harness requires fresh status, at least one hold, and zero timeouts;
the native log checker also rejects gate/coordinator timeout messages.
`lockstep:=false` retains the ungated control run.

This gates MCB acknowledgments, not the complete hit path: UART is not modeled
in sim time, and the other
CV/localization chains and wall-time bring-up remain asynchronous. It does not
establish identical shot sequences or finish workspace T17.

The on-demand transport test runs a small gz world in its own process group
and checks the held clock, zero timer phase after a late first acknowledgment,
and missing-consumer timeout diagnostics:

```sh
ros2 run sim lockstep_suite
```

The explicitly selected hosted backend compiles and runs the checked-out `firmware/MCBV3` C++ sources.
The Python control port has been removed. `include/sim/mcb_firmware.hpp` transports hardware
readings and DJI CAN feedback to `MCB-project/src/hosted/main.cpp`, which instantiates the actual
`SentryControl` and runs its command scheduler in 1 ms steps. UART1 uses the
PTY attached to the real `dji_serial_bridge`; MCBV3's own DJISerial parser,
mailbox, JetsonSubsystem, ballistics, aim/fire and drive commands run unchanged.

From the workspace root, in the ROS container:

```sh
git -C src submodule update --init --checkout firmware/MCBV3
git -C src/firmware/MCBV3 submodule update --init taproot-scripts
apt-get install -y scons g++-14
src/sim/tools/build_mcb_firmware.sh
colcon build --packages-select sim --symlink-install
ros2 launch sim mcb.launch.py backend:=hosted
```

Use `drive:=simple` for the firmware's waypoint route or, with
`backend:=hosted`, `drive:=auto` for NAV_GOAL. The default `stop` parks the
robot under centred manual drive. With main's firmware the keyboard peek
release schedules beyblade at boot, so the parked chassis spins. Foxglove serves
on :8765, and
gz/rviz open where a display is available. This launch uses the actual MCB
and bridge without an opponent; the `e2e.launch.py` stages score it.

`firmware_binary:=/path/to/MCB-project.elf` selects another hosted build;
`MCB_FIRMWARE_BINARY` supplies the default for both the gz system and tests.
The normal default is the firmware's `build/sim/scons-release/MCB-project.elf`.
Re-run the build after firmware edits: SCons tracks the original sources and
headers. There is no Python `firmware_fixes` overlay; the firmware under test
is exactly what the checked-out C++ implements. This is a host executable,
not the STM32 ELF flashed to the board.

#### Type C board execution with Renode

The default `backend:=renode` in the MCB and E2E launches uses Renode 1.17.0's Cortex-M4/STM32F4
platform and the normal `scons build robot=sentry profile=release` ELF.
`mcb_type_c` extracts that ELF's flash image and loads its symbols separately:
loading all ELF physical segments directly overlaps the initialized-data image.
The reset vector, startup constructors, RAM initialization, peripheral drivers,
interrupt handlers, 4,000-sample IMU calibration and command scheduler execute
from the board binary. The adapter does not write firmware RAM or force its state.

Renode was selected because QEMU's
[`STM32F405` SoC](https://github.com/qemu/qemu/blob/615ece3c406b262996e569f29ea6eed0d81c4d8d/hw/arm/stm32f405_soc.c)
still maps I²C, CAN, GPIO and DMA as unimplemented devices. That board would
need new controller models before this firmware could use its hardware paths.

Build in the Linux ROS container with ARM GCC 13, GCC 14, CMake and SCons:

```sh
src/sim/tools/install_renode.sh /tmp/mcb-renode-1.17.0
(cd src/firmware/MCBV3/MCB-project && scons build robot=sentry profile=release -j4)
colcon build --packages-select sim --symlink-install --cmake-args \
  -DSIM_RENODE_ROOT=/tmp/mcb-renode-1.17.0/renode_1.17.0-portable
ros2 launch sim mcb.launch.py backend:=renode drive:=stop
SIM_MCB_BACKEND=renode ros2 run sim mcb_physics_suite
ros2 run sim mcb_renode_connection_suite
ros2 run sim e2e_suite --backend renode --stage mcb_parked \
  --e2e-speeds 0 --e2e-paths lateral --e2e-duration 10
```

`SIM_RENODE_ROOT` enables the adapter at build time; `MCB_RENODE_ROOT` can
override the portable installation at runtime. Keep the install outside
`/tmp/renode-*`, which Renode owns for temporary-directory cleanup.
`firmware_binary` selects the board ELF with this backend. Startup waits up to 600 wall seconds for a valid
board POSE after calibration; gz physics starts after that stationary boot.
The board's boot clock therefore leads sim time by the calibration interval.
Per-run extracted flash, platform, script and Renode log remain in the printed
`/tmp/mcb-type-c-*` directory. The adapter owns and shuts down its Renode process.
Supported drive selections are `stop` and `simple`; `auto` and `external` are
hosted fixture controls and are rejected by the board adapter.
Both parked backends face the common start heading and use the same
firmware ingress and shooter-relative target paths. The calibrated gyro starts
at zero; the hosted fixture
can supply an arbitrary initial gyro angle directly, which the board cannot.
`backend:=hosted` explicitly selects the faster native fixture. All E2E stages
use the firmware's `SimpleAutoDriveCommand` with either backend; they arm it
after bring-up through the modeled remote, rather than publishing velocities.

The Jetson PTY transports actual USART1 bytes. Referee packets enter USART6,
and DBUS packets enter USART3 (remote absent through the power-on guard and
stationary calibration so firmware samples its yaw home, then centred sticks; right UP selects auto-drive, MID parks). `auto_fire:=false`
selects left MID and also disables firmware auto-aim, as with the hosted fixture.
`renode/TypeC.cs` models BMI088 SPI registers/chip selects, Pico I²C pod
readings and the MT6701 absolute encoder. The IMU is modeled in the yaw-link
frame, independently of barrel pitch. All three local angular-velocity and
specific-force components come from that link's gz pose, angular velocity and
linear acceleration, with Earth gravity included. Thus chassis tilt and
translation reach the real BMI088 SPI driver/AHRS. The exact sensor mounting
pose, noise and vibration are not robot measurements.
CAN feedback is staggered across each 1 ms virtual step to respect the board's
three-frame RX FIFO. Board-generated `0x200`/`0x1ff` commands feed the same
gz motor plant as the hosted backend. Each bus has its own `TypeCMotors` device;
feedback and DR16 repetition run on Renode's virtual clock. The adapter avoids
1.17.0's C CAN callback registration, which omits the bus descriptor and can
return CAN1 commands for both buses. It reads commands from the two devices
after advancing instruction execution. The control connection reconnects every
10,000 requests to bound 1.17.0's receive-thread stack growth; machine state
and peripheral timers continue across reconnects. It waits for the server to
listen again before calling the C client, whose failed-connect path can exit
instead of returning an error.

`TypeCCan` corrects Renode 1.17.0's CAN2 software-reset path: reset preserves
CAN1's shared filter banks instead of disabling wheel feedback. The on-demand
connection suite checks this register invariant as well as reconnects.
The motor device exposes generated-feedback counts separately from commands;
counts alone do not prove firmware reception.

The E2E harness allows 120 wall seconds per scored sim second for this backend,
versus 20 for hosted execution. Its five-wall-second clock-stall guard and
lockstep timeout rejection remain active; a slow emulator is not a passed test.
Board lockstep learns a hold one firmware batch ahead, rather than using the
hosted look-ahead. A long simulated look-ahead can exceed the coordinator's
wall deadline on a slow board emulator; reducing that startup distance keeps
the deadline and timeout rejection intact. The
[accuracy investigation](docs/aim-accuracy-2026-10-12.md#board-execution-controls)
records the controlled nightly/treatment settings.
`mcb_renode_connection_suite` exercises reconnects without booting firmware and
checks that peripheral state survives; it is built when `SIM_RENODE_ROOT` is set.

The I²C model derives from Renode infrastructure
[`066a7f13`](https://github.com/renode/renode-infrastructure/blob/066a7f13c052215632d469c995c89aea37c573b1/src/Emulator/Peripherals/Peripherals/I2C/STM32F1_I2C.cs)
under its retained MIT license. Its two local corrections clear STOP when the
condition is emitted and derive byte timing from CCR/DUTY instead of the APB
clock. SysTick uses an effective 21 MHz because this firmware selects AHB/8
and Renode ignores CLKSOURCE; DWT remains 168 MHz.
The firmware's divide-by-zero trap writes are enabled in the NVIC model.

This is instruction/peripheral execution, not a cycle-accurate STM32 timing
measurement. Renode's instruction timing cannot certify real 1 kHz deadlines.
UART input currently enters device queues without wire serialization; sensor
noise, thermal transients, bus faults and full device register behaviour are
not modeled. Projectile exits follow forward indexer shaft travel in the motor
plant, using the estimated gearbox/chamber geometry; reverse unjam travel is
not counted again. This observes physical indexing, rather than reading or
instrumenting firmware counters. A loaded-cartridge reverse stop supplies
the CAN stall current used by the firmware's indexer homing routine; its
geometry and load remain estimates, with no finite ammunition model.
The hosted backend reports issued-shot counters; the board backend reports
modeled physical exits. The [indexer recovery investigation](docs/mcb-indexer-recovery-2026-10-12.md)
explains why those observations must be distinguished.

#### Shared motor plant and UART transport

The hosted fixture's fake interfaces are calibrated IMU readings, Pico
odometry, centred DBUS input with both switches up, referee UART frames,
ADC and CAN motor feedback. The DBUS and referee packets go through the
real parsers. Referee state remains live node parameters. The firmware's
start constants are read from the executable and used to map gz truth into
the pods' frame, so its field-frame POSE agrees with the gz spawn.

`McbFirmwareSystem` owns the firmware process and PTY inside gz. It reads
pose, twist and gimbal joints from the ECM, runs one firmware cycle per 1 ms
physics step, then applies motor outputs as joint torques and a chassis wrench.
`mcb_physics:=true` removes both JointPositionControllers and VelocityControl.
The UART bridge starts after `mcb_ready` observes a CRC-valid POSE on the PTY.
The PTY link defaults to `/tmp/mcb_emulator_pty-<GZ_PARTITION>-<ROS_DOMAIN_ID>`
(`mcb_device_link:=` overrides), so concurrent stacks keep separate UARTs. The
plugin refuses a link whose pty a live stack still owns and removes only a link
that still names its own pty.
It waits 30 wall seconds for hosted execution or 600 for Renode boot.
The former ROS timer adapter
is removed; `mcb.launch.py` uses this same physics path.

The motor plant in `include/sim/mcb_motor.hpp` returns DJI 8-byte encoder,
signed shaft-RPM, current and temperature feedback for all ten motors. The
firmware's own CAN parser, encoder interface and controllers consume it. Yaw
and drivetrain commands are M3508 current units; pitch uses GM6020 voltage
and a back-EMF/resistance model. Gear ratios and motor constants follow the
sentry firmware. The chassis uses an omni-wheel force/velocity projection with
estimated per-wheel traction, capped using the total model mass read from gz
inertials; wheel speeds assume rolling without slip.
Flywheel, indexer and odometry-motor rotors have estimated inertia and load;
they have no mesh in gz. This validates control against a motor plant, not
measured tire, ESC, gearbox or launcher behaviour.

The plant adds passive wheel/roller resistance to the frictionless CAD contacts:
`rolling_resistance=0.03` times each wheel's nominal normal load, smoothed
through zero tangential speed. It dissipates motion without clamping chassis
velocity or pose. Firmware parking is main's hold, confirmed on the robot: centred
manual drive commands zero wheel velocity and the stop command writes zero
current. Neither latches a position, so navigation corrections cannot move it.
This coefficient is estimated, not a measured tire parameter. `mcb_parked`
records maximum truth displacement in `parking.jsonl` and rejects cells
moving more than 0.03 m. Only the hosted direct-sensor fixture delays gyro
yaw/rate by eight 1 ms ticks to match the firmware predictor; the board path
runs the full IMU filter, which owns sample age. CAN feedback stays current.
The hosted E2E profile uses zero residual yaw lag after setpoint-rate
filtering and the robot-timed 20 ms indexing delay (the chamber-exit point
`ProjectileExit::kExitAt` is placed to match). Spinning uses alignment gating on the
steady center-to-shooter line; chasing each rotating panel is opt-in through
nonnegative `chase_settle_s` (the previous fixture used 25 ms windows). These
are model calibrations; board gimbal timing remains uncalibrated.
Calibration and rejected treatments are recorded in the
[accuracy investigation](docs/aim-accuracy-2026-10-12.md), hosted
fixture measurements rather than board accuracy evidence.

`/mcb_emulator/shot` is the physical chamber-exit event inferred from CAN
indexer motion, stamped at that physics sample on both backends. The native
scorer launches at that stamp and adds no firing-delay estimate. Flag-only
diagnostics use the requested exit delay from CV_TARGET, without adding a
second indexer-latency estimate. Hosted
firmware request counters remain debug outputs, not physical shots.

`/mcb_emulator/gimbal_setpoint` publishes hosted firmware desired field yaw and pitch
at 100 Hz for command-versus-achieved diagnostics; it carries the physics
stamp and does not change the UART payload.

`/mcb_emulator/motors` publishes `sensor_msgs/JointState` at 100 Hz, stamped
with the physics sample time. Names identify CAN bus and hexadecimal motor
ID; positions and velocities are shaft radians and rad/s, effort is shaft
torque in N m. This is plant feedback, not a new firmware UART message.

The focused integration check drives the native UART aim command directly,
with gz gimbal inertia and no CV/localization stack:

```sh
ros2 run sim mcb_physics_suite
```

#### Hosted fixture limits

The hosted fixture still replaces boot calibration and hardware polling; ADC returns
0. CAN frames arrive once per tick without arbitration or transmission delay.
MCU interrupts, instruction timing, calibration and physical homing remain
unvalidated. The `MCB3` pipe format is private fixture IPC, not the Jetson UART
contract. A shot records a successful native
indexer request, not measured projectile exit. `/mcb_emulator/shot` is a
`std_msgs/Header` stamped with that request's sim time.

Run the native control and real-bridge checks without gz:

```sh
colcon test --packages-select sim --ctest-args -R test_mcb
```

The compiled-firmware checks also cover non-firing aim hold after target
expiry, immediate fire after fixture readiness, and delayed indexer requests.
See [firmware observations and timing-model limits](docs/panel-choice-2026-10-11.md#compiled-firmware-checks)
and the [fresh-build/live UART verification](docs/mcb-hosted-check-2026-10-11.md).

The bit-3 patrol marker is exercised through the real bridge for all sixteen
flag combinations, then through compiled firmware with the marker clear/set.
The firmware tests verify aim parsing, fire-bit independence, game-stage fire
gating, patrol permission after target expiry, and hit-turn permission after
damage. The marker is informational; current firmware ignores it. See the
[wire contract](../ros2_dji_serial_bridge/UART_PROTOCOL.md#cv_target-id1--aim-point-and-fire-decision)
and [2026-10-11 firmware verification](docs/patrol-flag-2026-10-11.md).

The native tests skip if the binary is absent; build it first to test the MCB.
The bridge tests also need ROS and cover disabled/negative diagnostic
intervals. Pure wire-format behavior is covered by GTests.

## Native port map

| Former Python | C++ implementation |
| --- | --- |
| `test/localization/*` | `test/localization/localization_suite.cpp` (`localization_suite`, drift and EKF) |
| `test/cv/shot_hit_harness.py`, `test_shot_hit.py` | `test/cv/shot_hit_suite.cpp`, `include/sim/cv_bench.hpp` |
| `test/cv/estimation_harness.py`, `test_estimation.py`, `estimation_metrics.py` | `test/cv/estimation_suite.cpp`, `include/sim/cv_bench.hpp` |
| `test/cv/estimation_limits_data.py` | `include/sim/estimation_limits_data.hpp` |
| `test/e2e/*` | `test/e2e/e2e_suite.cpp`, `test/e2e/e2e_harness.cpp`, `include/sim/e2e_test_core.hpp` |
| `test/mcb/*`, `sim/mcb_firmware.py`, `sim/mcb_emulator/pty_link.py` | `test/cpp/test_mcb_firmware.cpp`, `test_mcb_bridge_pty.cpp`, `include/sim/mcb_firmware.hpp` (also used by the gz system) |
| `sim/mcb_emulator/protocol.py` | `include/sim/mcb_protocol.hpp`, `src/mcb_protocol.cpp` |
| `sim/combat.py`, `sim/match_scenario.py` | `src/combat.cpp`, `src/match_scenario.cpp` (also used by `match_environment`) |
| `sim/suite_timing.py`, `sim/parent_death.py` | `src/suite_timing.cpp`, `src/process.cpp` |
| `sim/auto_explore.py` Gazebo helpers, `sim/odometry.py` | `include/sim/gazebo_helpers.hpp`, `include/sim/target_odometry.hpp` |
| `sim/display.py` probe, `sim/cv_target_emulator.py` constants | `src/display_probe.cpp`, `include/sim/cv_model_constants.hpp` |
| Python unit tests and `conftest.py` fixtures | `test/cpp/*`, native suite environments and value-parameterized cases |
| `tools/{check_bench_log,compare_runs,estimation_limits,shot_floors,rf2o_quality,rviz_to_foxglove}.py` | matching `tools/*.cpp`, installed as `ros2 run sim <name>` |

`suite_exit.py` creates ROS launch events synchronously, `display.py` calls the
native probe from launch, and `auto_explore.py` supplies the launch spawn
heading. These small adapters remain Python because the launch API requires
Python callables and constants. `launch/` and its ament lint tests also remain.
The native `suite_exit` tests exercise successful, failed and collection-style
child exit codes through the real launch service.

#### Firmware behaviour and historical results (then stage E2, now `mcb_parked`)

Current firmware uses corrected `getX/getY` for navigation and raw
`getAimX/getAimY` for POSE and gimbal aim. `RELOCALIZE` updates only the
navigation offset; raw POSE remains unchanged. The current wire contract is
owned by [UART_PROTOCOL message types](../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types).
The bullets and measurements below describe dated earlier behavior.

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
- Historically, `RELOCALIZE` moved odometry at once, anywhere, any HP
  (`JetsonSubsystem.cpp:76-81`). The RFID relocalize in
  `SimpleAutoDriveCommand` is gone; stuck 15 s, it spins in place.
- With an expired aim it patrols only if the last frame set
  `TYPE_C_BASED_PATROL`. A hit turn with `TURN_TO_HIT` set can override a
  live CV aim; see [firmware coordination](../ros2_dji_serial_bridge/README.md#where-the-firmware-stands).
- The sentry drives `SimpleAutoDriveCommand`'s ARCC route, spinning -8 rad/s
  moving and -12 at either end. The normal switch schedules this command,
  not the NAV_GOAL reader; `mcb.launch.py backend:=hosted drive:=auto` selects that reader
  explicitly. RMUL 3v3 waypoint advancement waits for IN_GAME, while the
  simple route can still spin before the game. The `mcb_drive` and
  `mcb_match` stages arm the firmware's own simple auto-drive after bring-up.
  Its waypoints and spin requests come from the checked-out firmware;
  the simulator controls the surrounding ghosts and referee inputs.

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

The sentry and opponent use the [team starts and boot heading](#the-world-frame).
After bring-up the scorer
arms the firmware's own `SimpleAutoDriveCommand` (the sentry's ARCC rough
route). The selected command generates its waypoints, translations and spin
requests; its own controller emits CAN motor commands applied as forces in gz.
`match_environment` supplies only ghost routes and combat inputs. It publishes
no chassis velocity or reference route for our robot. A later navigation
publisher remains separate work.

`backend:=renode` is the default and uses the normal board ELF.
`backend:=hosted` explicitly selects the faster fixture, whose remote switches
now follow the same stop/simple and aim/fire selections. Startup holds drive
in centred manual mode. At stage arm the right switch selects auto-drive;
the left stays MID until every robot's truth root enters the estimated
field-centre fight zone, then switches UP to enable automatic aim/fire.
Referee stage is IN_GAME so the firmware can advance waypoints during approach.
The real AMCL + rf2o + EKF stack remains in the loop.

`segments.json` reports hit rates and the measured failing hop per segment,
checked down the hit path: no shots, aim off the panel, head-TF stamps, then
the barrel off the aim. Fused navigation and localization errors are
frame-specific diagnostics; raw `odom` remains the aiming frame. `shots.jsonl`
records localization, head-TF and aim errors at fire time. `route.jsonl` also
samples truth displacement, speed and spin when no shots fire. Stages measure
firmware approach, driving, spinning and parked motion, not a scripted
straight/corner route; there is no scripted route-error metric for firmware
auto-drive.

`navigation_error_m` compares local `nav_odom->root` with field truth;
`localization_error_m` measures the globally corrected `map->root` pose and
therefore includes `map->nav_odom`. Navigation error describes field alignment
of dead reckoning, not globally localized navigation accuracy. These frames
coincide only at initialization. Logged xy positions, map correction and
lookup errors support checking frame composition and missing stamped
transforms directly. Pose scoring retries an unavailable transform at the
original capture stamp for at most one simulated second. `pose_score_delay_s`
records the delay; terminal gaps retain null errors and the lookup exception.
Latest TF is never substituted, and failed native checks remain failures.

A diagnostic-tier pass means nodes and clock remained live, the firmware
translated and rotated, every robot reached the configured centre zone,
stamped localization data existed, and a low hit rate named a measured failing
hop, as allowed in E2E_PLAN. `mcb_drive` also fails when its native total hit
count is zero, even with complete diagnostics; per-segment diagnostics remain
available. These checks establish diagnostic completion, not combat accuracy.
It does **not** mean combat accuracy passed: floors need repeated valid
runs, and current moving runs expose tracking and gimbal misses.
The spawn coordinates are firmware defaults, not surveyed starting zones.
[Board-stage verification](docs/mcb-auto-drive-2026-10-12.md) records clean
parked, drive and match runs, the focused CAN/gz check, and accuracy limits.

### Four-robot center fight (`mcb_match`)

```bash
ros2 launch sim e2e.launch.py stage:=mcb_match headless:=true
```

Two blue and two red sentries leave their respective spawns on separate
field-safe routes. The second lane leaves four seconds later; firing stays
disabled until every truth root reaches the centre zone. Our robot runs its
firmware waypoint/spin behavior; the three ghosts spin and use a truth-fed aimer with seed 2026,
0.015 rad Gaussian aim noise, and a 2 Hz fire rate. The selector sees both
red opponents and the blue ally. Only our robot runs the real CV stack.

The scorer advances 22 m/s ballistic shots under gravity against the field
mesh, chassis hulls and canted armor. It includes the physical muzzle velocity
at launch, including chassis motion. 22 m/s matches the checked-out sentry
firmware ballistic solver; the E2E Jetson planner uses the same value. The
first impact absorbs the shot;
damage requires more than 12 m/s relative normal speed, the exposure cone,
and the panel's 50 ms dead time. Each robot starts with 400 HP; hits cost
20 HP. Defeated robots stop moving and shooting. Referee HP, team, stage,
time and hurt panel feed the firmware's referee UART parser;
the test checks HP and team return through `REF_SYS` on the Jetson UART.

`match.json` reports per-segment measured motion and localization error, enemy hit
rates, HP, friendly intersections, damage taken and shot counts per robot.
`shots.jsonl` records first impacts and HP after each resolved impact.
The approach checks routing with firing disabled; center segments retain the
same route and localization diagnostic criterion as `mcb_drive`, while the
match pass criterion is unchanged. Accuracy floors, repeated-run
spread and standalone-versus-sequence equivalence remain uncalibrated.
Detector occlusion, non-chassis appendage blockers, heat, ammo and respawn
are not modeled. Hurt-panel feedback is supplied, but the firmware's
`delta_angle_got_hit_in` response has separate
[hosted/Renode injected-hit verification](docs/t31-hit-contract-2026-10-14.md#targeted-validation).
