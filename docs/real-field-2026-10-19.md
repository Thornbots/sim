# Real field integration, 2026-10-19

Geometry and frame choices are maintained in [the world frame](../README.md#the-world-frame).
The old export's outer dimensions (12.61 x 8.73 m; height 1.41883 m)
differ from the new export; the new inner walls match the rules' 12 x 8 m.
No scale fitting or waypoint changes were needed.

Compared sim nightly `b4be627f6c9d10bb264c5f386e569b7c08f82369` with
`0c17e0cf39d7005b1e965a7113e9295298c687cb` in a fresh Mac mini checkout,
`/Users/blaise/ros2_ws/real-field-20261019-baseline`, on `blaises-mini`.
Both used the existing Colima Jazzy container, domain 44, the same dependency
pins and unchanged hosted firmware `f9895ab`. Each run started a fresh stack.

Commands inside the sourced workspace:

```bash
colcon test --packages-select sim
colcon test-result --test-result-base build/sim --verbose
ros2 launch sim e2e.launch.py stage:=mcb_parked backend:=hosted headless:=true foxglove:=false
ros2 launch sim e2e.launch.py stage:=mcb_drive backend:=hosted headless:=true foxglove:=false
ros2 launch sim localization_tests.launch.py backend:=mapping scenario:=drift_correction headless:=true foxglove:=false
```

| Measurement | Nightly | Real field |
| --- | --- | --- |
| Parked cells | 12/12 pass | 12/12 pass |
| Max parked displacement (gate <= 0.03 m) | 0.028406 m | 0.001110 m |
| Parked RTF | 1.35 | 1.45 |
| Drive diagnostic | pass; RTF 1.24 | pass; RTF 1.49 |
| Blank-map drift trials | 2/3 pass; RTF 2.33 / 3.03 / 2.50 | 3/3 pass; RTF 3.23 / 3.23 / 3.43 |
| Recorded SLAM pose outputs (correct topic) | 32 / 26 | 32 / 31 / 23 |

All candidate runs and the passing baseline runs passed `check_bench_log`
on launch and stack logs. All E2E runs reported zero lockstep timeouts.
The failed baseline drift run was rejected by the checker: maximum truth error
1.1198 m exceeded 0.40 m, with a recorded vertical excursion to z = 1.4746 m.
No contact sensor was recorded, so its cause is not established. Candidate
mapping heights stayed between -0.013415 and 0.029991 m; candidate drive
remained upright (minimum up-axis z = 0.97428, about 13 degrees tilt).
The first baseline SLAM recorder used the wrong pose topic; its zero pose
count is invalid. The two later baseline trials and all candidate trials
recorded `/pose`; each published two maps. Counts span different run windows
and are acceptance evidence, not equal-duration scan-rate measurements.

Both meshes passed 18/19 registered CTest entries with the fixed firmware.
The same `RawPositionHoldRejectsMotionButIgnoresNavigationCorrections`
assertion failed on both; controls were not changed. The new geometry/route
tests passed 10/10 and combat tests 7/7. Full candidate colcon builds passed
on arm64 and x86_64; Python 3.12 repository quality and actionlint passed.
Some shutdowns produced bridge/SLAM nonzero-exit notes after SIGINT; the
native checker accepted these as teardown notes, not mid-run crashes.

This is not a regenerated-map AMCL validation: the drive diagnostic still
uses the older saved map from `sentry_localization`. Those maps must be
regenerated in their owning package. No physical robot was used.
Raw bounds, commands, firmware hashes, truth CSVs, scan counts, native logs
and the full report are archived on archlinux under
`~/sentry-deploy/2026-10-19/real-field/`. Each owned stack was stopped;
the existing container was retained.
