# Aim isolation follow-up, 2026-10-12

Follow-up to the [original measurements and review-fix rerun](aim-localization-coupling-2026-10-11.md). The review fixes were inspected before testing: offset-delta relay gating, heading input without TF feedback, translation-only navigation alignment, clock/reanchor queue clearing, and current UART semantics agree across the stack. The backward-navigation-stamp regression test exercises late old-epoch queue blocking. A remaining README default (`odom` instead of `nav_odom`) was corrected.

## Navigation and localization measure different estimates

The local navigation metric compares stamped `nav_odom->root` directly with field truth. The global localization metric compares `map->root` with that truth, including AMCL's `map->nav_odom` correction. Their reference frames start aligned but diverge as dead reckoning drifts. A smaller global error does not imply a broken translation-only TF edge. The [sim metric contract](../README.md#driving-and-combat-diagnostics-mcb_drive) now makes that distinction explicit.

The earlier final parked run's 1.688 m navigation p95 and 0.475 m localization p95 are retained as measured: they describe local EKF drift and the corrected global estimate, respectively. They do not measure the MCB's held, gated navigation offset directly. The relay still consumes `/localization/odom`, not `/localization/map_odom`; the latter's covariance gate is an existing open limitation at the [localization owner](../../sentry_localization/AGENTS.md#open). This investigation does not retune that gate or change navigation's global-position policy.

New telemetry records truth/navigation/localization xy, the map correction and exact lookup exceptions. Independently recorded EKF output is interpolated at each scored capture stamp to check the navigation TF chain. The frame audit also composes the recorded map transform with navigation position and checks it against global position. Across all nine fresh full-stage repeats (32,535 pose samples), map composition closed within 3.80e-15 m and the navigation TF agreed with interpolated EKF output within 9.23 mm maximum (all stages). No sample lacked navigation or localization TF. Small interpolation residuals are expected because the correction and raw pose are interpolated independently; they cannot explain metre-scale drift.

## Missing transforms

All 17 missing samples in the earlier final match occur at approach startup, 7.54–7.70 s. Raw aim TF and head truth were available; both navigation and localization were absent. The successful `odom->head_pitch` lookup at each capture stamp includes `odom->root`, while `nav_odom->root` was unavailable: the additional `nav_odom->odom` edge lacked valid stamped data. The old logs did not retain exceptions, so they cannot distinguish future extrapolation from the start of a new TF history. The first fresh unchanged-functionality match run had 3386/3386 valid navigation and localization samples.

The scorer previously made one lookup about 0.1 s after capture and permanently recorded a null even if a delayed transform arrived later. Pose telemetry now retries that same capture stamp for at most one simulated second. `pose_score_delay_s` makes the extra wait visible, and terminal gaps retain null values and the lookup exception. No latest transform replaces a missing capture-time transform. This changes telemetry readiness, not the robot's TF stamps or aim path.

## Hit variation

The review-to-final stationary-radial swing was a real shot-result difference: 36/41 (87.80%) to 0/39. Median aim-to-panel error changed from 0.003 to 0.531 m, barrel-to-aim error from 1.47 to 4.19 degrees, and head-TF angular error from 0.307 to 1.512 degrees. Stationary-diagonal changed from 41/41 to 8/41; stationary-lateral moved in the other direction, 7/40 to 14/41. These associations identify inconsistent aim/tracker/gimbal behavior, not a causal effect of localization. None of the inspected review changes adds localization to the raw aim path.

## Parked chassis motion invalidates a stationary-shooter assumption

Truth positions show about 20.9 m of chassis travel across the 12 cells in both new repeats 1 and 2 (net displacement 3.30 and 2.52 m). This is actual simulated motion, not inferred from EKF error. `mcb_parked` selects hosted firmware stop mode, whose `stopMotors()` writes zero current. The [native motor model](../README.md#mcb-emulator) applies zero drivetrain wrench in that mode; the URDF wheel contacts have zero friction, inherited from the velocity-controlled model. Gimbal reaction and existing momentum can therefore move the chassis without a parking constraint. The exact contribution of each force is not established by these logs.

A parked-shooter benchmark must check that assumption. The native harness now logs each cell's maximum truth displacement in `parking.jsonl` and rejects motion over 0.10 m from its first scored position. This is an explicit bench acceptance limit; it does not freeze the robot, change motor outputs, or tune an unmeasured physical parameter. Historical parked hit rates are still useful failure observations but cannot establish stationary-shooter accuracy or a controlled before/after localization improvement. The original parked baseline was already broken at 7.348 m global p95.

Physical tire/friction/braking calibration is left open at the model owner and is outside the aim-frame implementation. Restoring that model is needed before treating parked cell hit rates as stationary-shooter acceptance.

## Test setup and artifacts

Archlinux x86_64, headless Jazzy container, default stage cells/durations, native 1 ms firmware motor physics and lockstep; detection input remains truth-based. Existing source pins before this follow-up: workspace `8b1c230`, MCBV3 `290e555`, bridge `baf0167`, thornbots `abf072a`, sentry_localization `c2713ec`, sim `9fd43be`. The preserved final GCC 14 ELF is unchanged. Read-only nightly worktrees pin sim `0db8ec7`, thornbots `7b3032a`, sentry_localization `af45710`. Nightly localization comparisons build and run all three together, preserving their original `odom` frame contract.

Artifacts: `/home/blaise/workspaces/isaac_ros-jazzy/test-aim-investigate-2026-10-12/`, including runner scripts, outcome JSON, stage logs, scores, shot/pose/route telemetry, independently captured EKF poses, `metrics.py`, `frame_audit.py`, and isolated nightly builds. The first fresh match preceded the new diagnostic binary; it is explicitly an unchanged-functionality observation. The following three-repeat stage series uses the diagnostic binary without the later bounded scoring retry. Retry validation is reported separately.

The Jazzy container was absent at takeover. Under the user's container lifecycle approval, the cached local image was started and sim dependencies restored. The dependency helper's default build rejected duplicate package checkouts; the explicitly scoped build succeeded. The first diagnostic rebuild failed before dependency installation and then passed. The container and every launch group started here are cleaned up at completion; the pre-existing ollama container is left alone.

## Repeated combat results

All three repeats use the same raw-aim implementation and unchanged final firmware ELF. Hit rates count native MCB hits (enemy hits for match); parked totals sum scored per-cell values. Local/global p95 use capture-time pose/route samples and linear percentile interpolation. RELOCALIZE counts use the maximum actual TX sequence number, rather than the last periodic diagnostic sample.

| Stage | Run | Hits/shots | Hit % | Global localization p95 m | Local navigation p95 m | RELOCALIZE TX | Valid/total poses | Test/checker exit |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| mcb_parked | final r1 | 50/488 | 10.25 | 0.475 | 1.688 | 86 | 4918/4918 | 1/1 |
| mcb_parked | repeat r1 | 73/488 | 14.96 | 0.632 | 0.819 | 86 | 4917/4917 | 1/1 |
| mcb_parked | repeat r2 | 81/491 | 16.50 | 1.874 | 3.367 | 103 | 4920/4920 | 1/1 |
| mcb_parked | repeat r3 | 76/487 | 15.61 | 3.423 | 5.163 | 155 | 4917/4917 | 1/1 |
| mcb_drive | final r1 | 0/50 | 0.00 | 0.581 | 0.313 | 4 | 2542/2542 | 1/1 |
| mcb_drive | repeat r1 | 0/48 | 0.00 | 0.330 | 0.725 | 11 | 2541/2541 | 1/1 |
| mcb_drive | repeat r2 | 0/50 | 0.00 | 0.613 | 0.668 | 7 | 2541/2541 | 1/1 |
| mcb_drive | repeat r3 | 0/48 | 0.00 | 0.348 | 0.415 | 9 | 2541/2541 | 1/1 |
| mcb_match | final r1 | 0/36 | 0.00 | 0.455 | 0.771 | 13 | 3369/3386 | 1/1 |
| mcb_match | repeat r1 | 0/36 | 0.00 | 0.265 | 0.588 | 17 | 3386/3386 | 0/0 |
| mcb_match | repeat r2 | 0/36 | 0.00 | 0.357 | 0.433 | 8 | 3386/3386 | 0/0 |
| mcb_match | repeat r3 | 0/36 | 0.00 | 0.426 | 0.413 | 8 | 3386/3386 | 0/0 |

The original pre-decoupling baseline had global p95 7.348/0.209/0.355 m and hit rates 16.24/0/2.78% for parked/drive/match. Its parked localization was broken. The first isolated after-change repeats had global p95 0.646–1.734/0.242–0.353/0.271–0.747 m and hit rates 13.52–22.45/0–2.13/2.78–8.33%; they did not record local navigation p95. New parked values overlap/exceed that accuracy spread, drives remain poor, and all three new matches scored zero. These runs do not establish an accuracy improvement or isolate a causal hit-rate regression.

The three new drives rejected zero hits. All three matches passed their existing diagnostic criterion with no friendly intersection despite zero hits; that criterion is weaker than combat acceptance. All parked repeats failed cell floors. Every run completed with zero lockstep timeouts. Raw aim/MCB disagreement remained below 2.02e-9 m, while actual correction counts were 7–155 (earlier isolated runs: 39–881).

### Stationary opponent cells

| Cell | Earlier review hits/shots | Earlier final hits/shots | Repeat 1 | Repeat 2 | Repeat 3 |
| --- | ---: | ---: | ---: | ---: | ---: |
| stationary-lateral | 7/40 | 14/41 | 41/41 | 40/40 | 27/41 |
| stationary-radial | 36/41 | 0/39 | 1/41 | 1/42 | 40/40 |
| stationary-diagonal | 41/41 | 8/41 | 1/41 | 1/41 | 1/41 |

The stage definition commands the drivetrain to stop but does not constrain chassis position; the physical-motion findings above make these nominal parked cells an uncontrolled stationary-shooter comparison. The new same-code repeats demonstrate variation across cells and runs. They do not establish that the sentry heading fix caused the earlier hit drop.

## Nightly localization comparison

Fresh independent installs ran one full default nine-scenario drift suite and two isolated `scan_degraded` runs for each build, alternating nightly/current. Package-prefix lines in every launch log confirm the intended sim, thornbots and localization installs. The full suite was 9/9 and each isolated run 1/1 on both builds; all six native log checks accepted the runs.

| Build | Run/context | Before max m | During max m | After max m | Test/checker exit |
| --- | --- | ---: | ---: | ---: | --- |
| nightly | r1 / full suite | 0.1928 | 0.4093 | 0.1548 | 0/0 |
| current | r1 / full suite | 0.1331 | 0.4171 | 0.1572 | 0/0 |
| nightly | r2 / isolated | 0.1534 | 0.4451 | 0.1800 | 0/0 |
| current | r2 / isolated | 0.1483 | 0.4613 | 0.1689 | 0/0 |
| nightly | r3 / isolated | 0.1551 | 0.4489 | 0.1829 | 0/0 |
| current | r3 / isolated | 0.1380 | 0.4095 | 0.1836 | 0/0 |

These comparisons do not reproduce a consistent regression from the sentry change. They do not prove the 0.5 m bound is reliable: the earlier current full suite failed at 0.508 m, while an intermediate current full suite passed at 0.451 m. Three fresh observations per build (only one in full-suite context) cannot quantify a failure probability or rule out a small effect. No noise parameters, covariance gates or acceptance limits were relaxed. The earlier final EKF trajectory check remains a pass (mean 0.034 m); independently recorded navigation yaw in the nine new combat repeats remained within 4.40e-10 radians of the fixed heading.

## Validation of scoring fixes

After rebuilding the bounded capture-stamp retry and parking assertion, one full drive and match plus the three stationary-opponent parked cells (`speeds:=0`) were run. The parked subset is not comparable with a full 12-cell pooled hit rate.

| Stage/context | Hits/shots | Hit % | Global p95 m | Local p95 m | RELOCALIZE TX | Valid/total poses | Test/checker exit |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| mcb_parked / stationary-opponent subset | 1/122 | 0.82 | 0.627 | 0.600 | 22 | 1230/1230 | 1/1 |
| mcb_drive / full route | 1/50 | 2.00 | 0.500 | 0.321 | 5 | 2541/2541 | 0/0 |
| mcb_match / full route | 2/36 | 5.56 | 0.582 | 0.793 | 12 | 3387/3387 | 0/0 |

The new parking guard was exercised by all three cells: maximum chassis displacement was 2.322/1.724/0.938 m for lateral/radial/diagonal, all above 0.10 m. The subset also failed hit floors. Drive scored one hit and match two enemy hits with no friendly intersection; these low rates remain accuracy failures even though their current native criteria accepted them. All three runs had zero lockstep timeouts and no terminal TF gaps. The historical terminal startup gap did not recur. These logs record final scoring delay but do not count intermediate lookup attempts, so they do not establish how often a retry was needed.

## Checks, remaining limits and skipped items

- Follow-up diagnostic and scoring builds passed. All 15 native sim test executables passed again, including 79 hosted-firmware cases and three real bridge/PTY cases. Registered CTest result: 16/18 passed, with only the two previously documented copyright/flake8 failures. No baseline was enlarged.
- The earlier review-fix firmware 31 control/geometry + five hosted tests and ROS unit results were inspected; firmware, bridge and thornbots production source is unchanged in this follow-up. Localization production source is also unchanged; its README default was corrected. The fresh localization comparison and stamped heading recordings validate the current adapter/TF behavior.
- Required Python 3.12 quality checks and actionlint pass for the changed sim, localization and workspace repositories. Changed local links/anchors were checked.
- Skipped: tire/braking/suspension calibration and the parallel Renode/firmware-auto-drive track; covariance-gate/global-MCB-navigation policy changes; tracker/gimbal tuning and calibrated combat floors; physical-robot/CAN/STM32 tests. The findings identify limits and add honest checks without retuning these unrelated systems.
- Three observations per localization build cannot establish reliability of a near-threshold stochastic test. The original 0.508 m failure is retained, and no threshold was relaxed. Parked chassis motion and low combat hit rates remain unresolved acceptance failures.
- All owned launch groups, observers and runners exited; final process inspection found no sim, ROS launch, firmware, or observer leftovers. The container started for this investigation is removed after validation. The pre-existing ollama container is untouched.
