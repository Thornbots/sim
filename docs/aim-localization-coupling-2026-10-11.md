# Aim/localization coupling, 2026-10-11

Aim is now structurally independent of EKF/map localization and RELOCALIZE. The raw-control experiment removed frame disagreement but did not recover moving-shooter hits. Remaining combat misses need tracker/aim/gimbal work; frame isolation is not combat acceptance.

## Setup and controlled comparison

Archlinux x86_64, existing Jazzy container, headless (no display), full default cells and durations, native 1 ms MCB physics with batch lockstep. No YOLO/occlusion: detections are truth stand-ins. Driving uses the existing scripted route through hosted firmware motor control; Renode and firmware auto-drive work on the other machine is outside this investigation.

Fresh baseline: workspace `843a9ff`, sim `0db8ec7`, thornbots `7b3032a`, bridge `bbf8e9d`, MCBV3 `49e5023`. Fresh isolated ROS build and preserved GCC 14 hosted ELF. The older [hosted-firmware report](match-check-2026-10-11.md) used different pins and recorded 1.847/2.116 m match turn/spin p95; the fresh baseline below did not reproduce that magnitude.

- **Before:** original EKF aim TF and active RELOCALIZE; one fresh run of each stage.
- **Raw control:** same baseline binaries; remap only the legacy aim TF broadcaster input from `/localization/odom` to `/odom`, and relay RELOCALIZE to an unused topic. Both approved driving/match runs completed with UART RELOCALIZE TX count zero. The temporary patches are retained with the artifacts and are absent from final source.
- **After:** raw POSE/aim on both sides, navigation RELOCALIZE active; three runs of every stage. MCBV3 `cb9f420`, bridge `f81014b`, thornbots `4982c8e`, and the sim revision containing this report. The [wire owner](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types) and [frame design](../../thornbots_pkg/README.md#pose_translator) define the split.

Raw control and final split change which TF edge AMCL observes; localization p95 is therefore a diagnostic comparison, not a controlled localization-tuning result. A single baseline/control run also cannot establish statistical causality or repeatability.

## Per-stage results

Hits are native MCB hits (enemy hits for match). Parked totals sum the native per-cell scores; shot logs can include late resolutions after a cell closes. Localization p95 is stamped `map->root` Euclidean truth error over scored pose/route samples, including intervals with no shots. Missing transforms are omitted and counted by available/total samples; no latest TF is substituted. Disagreement is raw UART POSE versus aim TF at that POSE stamp.

| Stage | Setup/run | Hits/shots | Hit % | Localization p95 m | Loc samples | Aim disagreement p95 m | Test/checker exit |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| mcb_parked | before r1 | 64/394 | 16.24 | 7.348 | 4917/4917 | 0.049 | 1/1 |
| mcb_parked | after r1 | 110/490 | 22.45 | 0.646 | 4922/4922 | 0.000 | 1/1 |
| mcb_parked | after r2 | 80/488 | 16.39 | 1.114 | 4916/4916 | 0.000 | 1/1 |
| mcb_parked | after r3 | 66/488 | 13.52 | 1.734 | 4921/4921 | 0.000 | 1/1 |
| mcb_drive | before r1 | 0/47 | 0.00 | 0.209 | 2541/2541 | 0.085 | 0/0 |
| mcb_drive | raw-control r1 | 0/48 | 0.00 | 0.171 | 2541/2541 | 0.000 | 0/0 |
| mcb_drive | after r1 | 1/47 | 2.13 | 0.242 | 2541/2541 | 0.000 | 0/0 |
| mcb_drive | after r2 | 0/48 | 0.00 | 0.353 | 2542/2542 | 0.000 | 1/1 |
| mcb_drive | after r3 | 0/46 | 0.00 | 0.295 | 2542/2542 | 0.000 | 1/1 |
| mcb_match | before r1 | 1/36 | 2.78 | 0.355 | 3387/3387 | 0.210 | 0/0 |
| mcb_match | raw-control r1 | 0/36 | 0.00 | 0.173 | 3386/3386 | 0.000 | 1/1 |
| mcb_match | after r1 | 2/36 | 5.56 | 0.274 | 3386/3386 | 0.000 | 0/0 |
| mcb_match | after r2 | 3/36 | 8.33 | 0.271 | 3386/3386 | 0.000 | 1/1 |
| mcb_match | after r3 | 1/36 | 2.78 | 0.747 | 3386/3386 | 0.000 | 1/1 |

Exit 0 means accepted by that check; exit 1 is a scored rejection. Every listed run completed its scoring window with zero lockstep timeouts. Floor, zero-hit, route, or friendly-intersection failures remain failures; none were hidden.

All three final parked runs failed their cell hit floors. Final drive repeats 2 and 3 failed on zero native hits. Raw-control match and final match repeats 2 and 3 each failed on one friendly intersection; final match repeat 1 had none. Every drive/match route segment stayed below its 0.40 m p95 route limit.

## Driving and match segments

These localization p95 values come directly from native `segments.json`/`match.json`, using scored route telemetry. Match approach firing remains disabled.

### mcb_drive

| Setup/run | Segment | Native shots | Hit % | Localization p95 m | Route p95 m |
| --- | --- | ---: | ---: | ---: | ---: |
| before r1 | approach_2 | 11 | 0.00 | 0.165 | 0.238 |
| before r1 | parked | 10 | 0.00 | 0.164 | 0.205 |
| before r1 | straight_1 | 4 | 0.00 | 0.238 | 0.060 |
| before r1 | turn | 10 | 0.00 | 0.161 | 0.054 |
| before r1 | spin | 12 | 0.00 | 0.217 | 0.018 |
| raw-control r1 | approach_2 | 11 | 0.00 | 0.202 | 0.290 |
| raw-control r1 | parked | 10 | 0.00 | 0.133 | 0.245 |
| raw-control r1 | straight_1 | 4 | 0.00 | 0.140 | 0.071 |
| raw-control r1 | turn | 10 | 0.00 | 0.168 | 0.067 |
| raw-control r1 | spin | 12 | 0.00 | 0.180 | 0.023 |
| after r1 | approach_2 | 11 | 0.00 | 0.236 | 0.300 |
| after r1 | parked | 10 | 0.00 | 0.144 | 0.247 |
| after r1 | straight_1 | 4 | 0.00 | 0.154 | 0.071 |
| after r1 | turn | 10 | 10.00 | 0.252 | 0.084 |
| after r1 | spin | 12 | 0.00 | 0.171 | 0.025 |
| after r2 | approach_2 | 11 | 0.00 | 0.325 | 0.348 |
| after r2 | parked | 10 | 0.00 | 0.303 | 0.264 |
| after r2 | straight_1 | 5 | 0.00 | 0.315 | 0.072 |
| after r2 | turn | 10 | 0.00 | 0.326 | 0.072 |
| after r2 | spin | 12 | 0.00 | 0.353 | 0.024 |
| after r3 | approach_2 | 10 | 0.00 | 0.219 | 0.264 |
| after r3 | parked | 10 | 0.00 | 0.106 | 0.239 |
| after r3 | straight_1 | 4 | 0.00 | 0.141 | 0.072 |
| after r3 | turn | 10 | 0.00 | 0.353 | 0.118 |
| after r3 | spin | 12 | 0.00 | 0.295 | 0.025 |

### mcb_match

| Setup/run | Segment | Native shots | Hit % | Localization p95 m | Route p95 m |
| --- | --- | ---: | ---: | ---: | ---: |
| before r1 | approach_2 | 0 | 0.00 | 0.269 | 0.270 |
| before r1 | parked | 10 | 10.00 | 0.156 | 0.001 |
| before r1 | straight_1 | 4 | 0.00 | 0.186 | 0.073 |
| before r1 | turn | 10 | 0.00 | 0.379 | 0.074 |
| before r1 | spin | 12 | 0.00 | 0.349 | 0.025 |
| raw-control r1 | approach_2 | 0 | 0.00 | 0.224 | 0.256 |
| raw-control r1 | parked | 10 | 0.00 | 0.153 | 0.002 |
| raw-control r1 | straight_1 | 4 | 0.00 | 0.157 | 0.071 |
| raw-control r1 | turn | 10 | 0.00 | 0.074 | 0.174 |
| raw-control r1 | spin | 12 | 0.00 | 0.040 | 0.025 |
| after r1 | approach_2 | 0 | 0.00 | 0.269 | 0.242 |
| after r1 | parked | 10 | 10.00 | 0.283 | 0.001 |
| after r1 | straight_1 | 4 | 0.00 | 0.270 | 0.072 |
| after r1 | turn | 10 | 10.00 | 0.191 | 0.077 |
| after r1 | spin | 12 | 0.00 | 0.217 | 0.024 |
| after r2 | approach_2 | 0 | 0.00 | 0.277 | 0.272 |
| after r2 | parked | 9 | 0.00 | 0.068 | 0.001 |
| after r2 | straight_1 | 5 | 0.00 | 0.122 | 0.072 |
| after r2 | turn | 10 | 20.00 | 0.278 | 0.110 |
| after r2 | spin | 12 | 8.33 | 0.269 | 0.024 |
| after r3 | approach_2 | 0 | 0.00 | 0.612 | 0.282 |
| after r3 | parked | 9 | 0.00 | 0.796 | 0.001 |
| after r3 | straight_1 | 5 | 20.00 | 0.783 | 0.072 |
| after r3 | turn | 10 | 0.00 | 0.616 | 0.098 |
| after r3 | spin | 12 | 0.00 | 0.332 | 0.025 |

## Parked cells

| Cell | Before hits/shots (%) | After r1 | After r2 | After r3 |
| --- | ---: | ---: | ---: | ---: |
| mcb_parked-stationary-lateral | 11/41 (26.83%) | 36/41 (87.80%) | 0/41 (0.00%) | 19/41 (46.34%) |
| mcb_parked-speed1-lateral | 13/41 (31.71%) | 0/41 (0.00%) | 0/40 (0.00%) | 0/40 (0.00%) |
| mcb_parked-speed2-lateral | 3/41 (7.32%) | 1/40 (2.50%) | 0/41 (0.00%) | 1/41 (2.44%) |
| mcb_parked-speed4-lateral | 3/41 (7.32%) | 1/41 (2.44%) | 1/41 (2.44%) | 0/41 (0.00%) |
| mcb_parked-stationary-radial | 1/40 (2.50%) | 23/41 (56.10%) | 2/40 (5.00%) | 6/40 (15.00%) |
| mcb_parked-speed1-radial | 2/41 (4.88%) | 10/41 (24.39%) | 10/41 (24.39%) | 3/40 (7.50%) |
| mcb_parked-speed2-radial | 3/40 (7.50%) | 5/41 (12.20%) | 4/41 (9.76%) | 2/41 (4.88%) |
| mcb_parked-speed4-radial | 11/40 (27.50%) | 0/41 (0.00%) | 7/41 (17.07%) | 3/41 (7.32%) |
| mcb_parked-stationary-diagonal | 15/38 (39.47%) | 31/41 (75.61%) | 40/41 (97.56%) | 31/40 (77.50%) |
| mcb_parked-speed1-diagonal | 2/3 (66.67%) | 1/41 (2.44%) | 13/40 (32.50%) | 0/41 (0.00%) |
| mcb_parked-speed2-diagonal | 0/14 (0.00%) | 2/41 (4.88%) | 1/41 (2.44%) | 1/41 (2.44%) |
| mcb_parked-speed4-diagonal | 0/14 (0.00%) | 0/40 (0.00%) | 2/40 (5.00%) | 0/41 (0.00%) |

## Coupling and remaining failure evidence

| Drive setup/run | Aim disagreement median/max m | Aim off panel median m | Barrel off aim median deg |
| --- | ---: | ---: | ---: |
| before r1 | 0.0278443/0.162977 | 1.005 | 5.480 |
| raw-control r1 | 0/2.01255e-09 | 1.092 | 9.250 |
| after r1 | 0/2.02224e-09 | 0.638 | 6.270 |
| after r2 | 0/2.00661e-09 | 0.710 | 5.670 |
| after r3 | 0/2.01516e-09 | 0.891 | 6.800 |

Across all nine final runs, raw aim/MCB disagreement stayed below 2.03e-9 m while the last sampled navigation RELOCALIZE TX count was 27–872 per run. The fresh baseline maximum disagreement was 0.163 m in drive and 0.254 m in match. Navigation truth error still reached 1.734 m p95 in final parked repeat 3.

The final code keeps raw `odom->root` independent of either navigation transform. Navigation alignment uses raw pose interpolated at the fused stamp. Firmware unit tests verify arbitrary RELOCALIZE fixes affect only navigation, preserve raw POSE/held aim, and allow subsequent pod displacement. The current 25-byte POSE and 15-byte CV_TARGET layouts remain unchanged, with no fallback.

`mcb_drive` now requires at least one native hit in the full scored route; a measured diagnosis cannot make zero hits pass. After repeat 2 scored zero hits and failed both the native assertion and log checker, directly exercising this rule. Per-segment diagnostic criteria remain. `mcb_match` retains its existing diagnostic criterion and zero-friendly-intersection assertion. That match criterion is still weaker than calibrated combat accuracy.

## Tests, limits and skipped work

- Fresh Jazzy builds passed. Firmware: 31 geometry/control tests and 5 hosted regression tests passed with GCC 14.
- Registered cases: bridge 42 (0 failures, 9 existing lint-file skips), thornbots 111 (0 failures), sim 169 (2 existing lint failures, no skips). All native tests passed, including 79 compiled-firmware checks and 3 real bridge/PTY checks. The new frame tests cover camera-chain invariance, interpolation, and missing/stale/future raw pose.
- Required Python 3.12 repository quality checks and `actionlint -shellcheck=""` pass in each changed repo; no baselines grew.
- Firmware hosted CI passes after pinning the updated immutable sim UART fixture in `91a3b04`; the initial CI failure was the old fixture expecting RELOCALIZE to move POSE.
- Existing sim registered lint failures are outside this brief: copyright checker rejects SPDX-only headers in `test_mcb_motor.cpp`/`mcb_physics_suite.cpp`; flake8 reports pre-existing import order in `e2e.launch.py` and dict/quote style in `shot_hit.launch.py`. New thornbots license headers were fixed and its lint rerun passed.
- Skipped: tracker tuning, hit-turn policy, extra localization integration suites, hardware/CAN/STM32 timing measurements, surveyed spawns, and the parallel Renode/firmware-auto-drive track. No frozen branches or legacy wire formats were used. `root` remains the documented heading-fixed deviation from the usual body-fixed base frame.
- The existing container was attached to, not started or stopped. Every run owned and cleaned its launch groups; final process inspection is recorded with the artifacts.

## Artifacts and reproduction

Raw data and commands: `/home/blaise/workspaces/isaac_ros-jazzy/test-aim-2026-10-11/`. Each run retains launch/stack/checker logs, shot/state/pose or route records, and native scores. `before-outcomes.json`, `raw-control-outcomes.json`, and `after-outcomes.json` record process/checker exits. The preserved before/after ELFs and temporary raw-control patches identify the treatment.

Recreate the baseline from the pins above in a separate checkout and ROS build; apply the saved remapping patches only for raw control. The prepared isolated install now contains the final implementation. For final runs, source `test-aim-2026-10-11/install/setup.bash` and set `MCB_FIRMWARE_BINARY` to `after-MCB-project.elf`, then run the normal [stage commands](../README.md#launch-sim-by-hand). The saved `run_stages.py` owns process groups, checks for live stacks and records checker results; `make_report.py` uses linear p95 interpolation, matching the native scorer. Any new integration run still requires the workspace run approval.
