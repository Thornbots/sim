# Aim accuracy model investigation, 2026-10-12

This investigation records historical model/timing phases and completed hosted
repeat profiles. Artifacts are in
`/home/blaise/workspaces/isaac_ros-jazzy/test-aim-accuracy-2026-10-12/`.
The earlier phases below are single-run historical observations; the final
no-lead and nightly comparisons are recorded under the three-repeat section.

## Historical phase results

Drive/match counts use native MCB shot records (`kind == "mcb"`); drive hits
are panel hits and match hits are enemy hits. Parked brake cell totals use
the native `scores.jsonl` snapshots: raw JSONL can include later completions
after a cell score snapshot and therefore has a different total. Segment counts come from each stage's
`shots.jsonl`; localization, navigation and route values are p95 metres from
capture-stamped `route.jsonl` records, using the native linear percentile
interpolation. `loc/nav/route` is the column order.
The match approach has no sentry MCB shots, though its pose metrics are logged.

| Phase | Drive hits/shots | Match hits/shots | Run/checker status |
| --- | ---: | ---: | --- |
| physical | 1/48 | 0/36 | 0/0 |
| imu | 1/46 | 0/36 | 0/0 |
| stamp | 0/50 | 1/37 | drive 1/1, match 0/0; drive failed its zero-hit assertion |
| timed | 2/46 | 7/36 | 0/0 |
| trace | 6/50 | 7/36 | 0/0 |
| yawfix | 4/48 | 8/36 | 0/0 |
| ballistic | 5/48 | 3/36 | 0/0 |
| covariance (`.01/.0/.01`) | 2/49 | 2/36 | 0/0 |
| period | 4/49 | 3/36 | 0/0 |
| orientation fusion | 0/48 | 1/36 | drive 1/1, match 1/1; both failed |
| speed | 3/49 | 0/35 | 0/0; one run, not a repeatability floor |

The `physical-mcb_parked-r1` calibration scored 40/41 lateral (97.6%), 41/41
radial (100%), and 40/40 diagonal (100%), 121/122 overall. `physical`, `imu`,
`timed`, `trace`, and `yawfix` runner outcomes were accepted by the native log
checker. `stamp` match was accepted; `stamp` drive failed with 0/50 hits and
checker exit 1. These are single-run observations, not repeatability floors.

| Phase | Segment | Drive hits/shots | Drive loc/nav/route p95 m | Match hits/shots | Match loc/nav/route p95 m |
| --- | --- | ---: | ---: | ---: | ---: |
| physical | approach_2 | 0/11 | 0.301/0.196/0.345 | 0/0 | 0.291/0.322/0.292 |
| physical | parked | 1/10 | 0.191/0.197/0.261 | 0/10 | 0.094/0.119/0.001 |
| physical | straight_1 | 0/5 | 0.190/0.219/0.074 | 0/4 | 0.101/0.126/0.075 |
| physical | turn | 0/10 | 0.215/0.219/0.084 | 0/10 | 0.195/0.143/0.231 |
| physical | spin | 0/12 | 0.283/0.385/0.022 | 0/12 | 0.189/0.106/0.023 |
| imu | approach_2 | 0/10 | 0.399/0.614/0.323 | 0/0 | 0.206/0.442/0.278 |
| imu | parked | 1/10 | 0.275/0.606/0.251 | 0/9 | 0.131/0.464/0.001 |
| imu | straight_1 | 0/4 | 0.364/0.452/0.075 | 0/5 | 0.147/0.457/0.074 |
| imu | turn | 0/10 | 0.212/0.507/0.101 | 0/10 | 0.156/0.567/0.227 |
| imu | spin | 0/12 | 0.179/0.459/0.023 | 0/12 | 0.078/0.580/0.021 |
| stamp | approach_2 | 0/13 | 0.540/0.253/0.323 | 0/0 | 0.270/0.228/0.231 |
| stamp | parked | 0/10 | 0.427/0.286/0.250 | 0/10 | 0.322/0.213/0.001 |
| stamp | straight_1 | 0/5 | 0.443/0.299/0.075 | 0/5 | 0.457/0.230/0.072 |
| stamp | turn | 0/10 | 0.443/0.343/0.245 | 0/10 | 0.374/0.287/0.083 |
| stamp | spin | 0/12 | 0.413/0.347/0.023 | 1/12 | 0.411/0.424/0.022 |
| timed | approach_2 | 0/10 | 0.422/0.377/0.296 | 0/0 | 0.231/0.097/0.302 |
| timed | parked | 0/10 | 0.170/0.369/0.254 | 2/9 | 0.232/0.097/0.001 |
| timed | straight_1 | 0/4 | 0.171/0.382/0.075 | 2/5 | 0.235/0.107/0.074 |
| timed | turn | 2/10 | 0.198/0.392/0.085 | 0/10 | 0.248/0.152/0.124 |
| timed | spin | 0/12 | 0.177/0.388/0.022 | 3/12 | 0.234/0.144/0.023 |
| trace | approach_2 | 0/13 | 0.392/0.152/0.310 | 0/0 | 0.330/0.081/0.299 |
| trace | parked | 1/10 | 0.387/0.068/0.249 | 2/10 | 0.160/0.075/0.000 |
| trace | straight_1 | 0/4 | 0.396/0.073/0.074 | 0/4 | 0.227/0.080/0.074 |
| trace | turn | 2/10 | 0.403/0.089/0.189 | 5/10 | 0.405/0.078/0.095 |
| trace | spin | 3/12 | 0.389/0.084/0.023 | 0/12 | 0.395/0.073/0.023 |
| yawfix | approach_2 | 0/11 | 0.254/0.237/0.314 | 0/0 | 0.330/0.151/0.292 |
| yawfix | parked | 1/10 | 0.229/0.229/0.251 | 0/9 | 0.329/0.146/0.001 |
| yawfix | straight_1 | 0/5 | 0.236/0.244/0.074 | 2/5 | 0.331/0.155/0.075 |
| yawfix | turn | 2/10 | 0.249/0.284/0.241 | 4/10 | 0.304/0.163/0.225 |
| yawfix | spin | 1/12 | 0.243/0.285/0.024 | 2/12 | 0.304/0.149/0.023 |
| ballistic | approach_2 | 1/12 | 0.234/0.134/0.323 | 0/0 | 0.186/0.236/0.294 |
| ballistic | parked | 2/10 | 0.217/0.121/0.254 | 2/9 | 0.185/0.231/0.001 |
| ballistic | straight_1 | 0/4 | 0.222/0.122/0.075 | 0/5 | 0.185/0.254/0.076 |
| ballistic | turn | 2/10 | 0.255/0.147/0.248 | 1/10 | 0.209/0.285/0.121 |
| ballistic | spin | 0/12 | 0.247/0.142/0.024 | 0/12 | 0.195/0.280/0.023 |
| covariance | approach_2 | 1/12 | 0.274/0.099/0.348 | 0/0 | 0.456/0.552/0.279 |
| covariance | parked | 0/10 | 0.288/0.091/0.255 | 2/9 | 0.119/0.548/0.001 |
| covariance | straight_1 | 0/5 | 0.308/0.105/0.075 | 0/5 | 0.131/0.560/0.075 |
| covariance | turn | 0/10 | 0.320/0.122/0.158 | 0/10 | 0.133/0.557/0.064 |
| covariance | spin | 1/12 | 0.301/0.110/0.024 | 0/12 | 0.122/0.542/0.022 |
| period | approach_2 | 2/13 | 0.406/0.110/0.321 | 0/0 | 0.281/0.115/0.289 |
| period | parked | 0/10 | 0.391/0.108/0.261 | 1/9 | 0.246/0.075/0.000 |
| period | straight_1 | 0/4 | 0.397/0.106/0.075 | 0/5 | 0.260/0.084/0.076 |
| period | turn | 1/10 | 0.419/0.115/0.064 | 2/10 | 0.309/0.127/0.192 |
| period | spin | 1/12 | 0.412/0.114/0.022 | 0/12 | 0.320/0.124/0.023 |
| orientation fusion | approach_2 | 0/11 | 0.190/0.105/0.310 | 0/0 | 0.803/1.034/0.567 |
| orientation fusion | parked | 0/10 | 0.177/0.105/0.250 | 0/9 | 0.132/1.022/0.000 |
| orientation fusion | straight_1 | 0/5 | 0.174/0.113/0.076 | 0/5 | 0.136/1.030/0.075 |
| orientation fusion | turn | 0/10 | 0.195/0.144/0.126 | 1/10 | 0.152/1.029/0.076 |
| orientation fusion | spin | 0/12 | 0.178/0.145/0.023 | 0/12 | 0.142/1.021/0.022 |

`mcb_drive` drives through the route segment named `parked`; its truth pose
still moves about 0.15–0.16 m during that five-second segment. `mcb_match`
holds nearly still there. The stationary `mcb_parked` calibration pose logs
show 2.3 mm lateral displacement and effectively zero radial/diagonal
displacement over each 20-second cell.

| Stationary cell | Hits/shots | Localization/navigation p95 m | Net truth displacement |
| --- | ---: | ---: | ---: |
| lateral | 40/41 | 0.261/0.264 | 0.0023 m |
| radial | 41/41 | 0.257/0.260 | <0.001 m |
| diagonal | 40/40 | 0.253/0.256 | <0.001 m |

| Phase | Drive truth displacement in `parked` | Match truth displacement in `parked` |
| --- | ---: | ---: |
| physical | 0.150 m | 0.001 m |
| imu | 0.153 m | 0.002 m |
| stamp | 0.150 m | 0.002 m |
| timed | 0.164 m | <0.001 m |
| trace | 0.154 m | <0.001 m |
| yawfix | 0.152 m | <0.001 m |
| ballistic | 0.150 m | <0.001 m |
| covariance | 0.141 m | <0.001 m |
| period | 0.163 m | <0.001 m |
| orientation fusion | 0.153 m | <0.001 m |

These displacements are the Euclidean difference between the first and last
`truth_xy` samples for the named route segment. They describe net motion over
that interval, not its maximum excursion.

## Updated model phases and metric limits

The retained simulation plant uses passive wheel resistance `0.03`, an
estimated effective roller/bearing resistance rather than a measured tire
coefficient. The gyro model delays samples by eight 1 ms physics steps,
matching firmware's 8 ms predictor. The main simulation scheduler remains at
1 ms. The yaw gearbox/friction correction is retained in the yawfix firmware
used for later ballistic runs. None of these values is a physical-robot
calibration.

The `stamp` runner requested `pose_latency_s:=0.0`, but the nested launch did
not forward that override in the binary used for these runs. Both stack logs
show the target tracker actually ran with `pose_latency_s=0.010`; therefore
the `stamp` results are not evidence about a zero-latency ablation. The drive
run's zero-hit failure is still a valid failure for the configuration it
actually ran. A zero pose-delay setting is a simulation-only experiment; it is
not a measured physical sensor latency.

The physical through ballistic match `states.jsonl` records predate
target-identity scoring. Their center, velocity, and yaw errors compare
against `opponent_0` regardless of which opponent the tracker selected. The
covariance and later records add identity-aware diagnostics: the current diff
associates each state with the nearest enemy at its own capture stamp, uses
that body for future prediction error, and reports the nearest-to-second-
nearest association margin. The older records must not be read as
per-selected-target accuracy.

`aim_off_panel_m` is also not a direct tracker-center error. It compares the
aim point with a truth panel at the expected arrival time; the aim point can
intentionally be ahead because of lead and ballistic compensation. The old
match scorer also fell back to `opponent_0` when a miss had no known victim.
The current diff uses the resolved victim when available, but retains that
fallback on misses. Interpret this value alongside the target-state errors
and shot result, not as an isolated aim-quality score.

The ballistic phase adds inherited muzzle motion to projectile velocity and
target-arrival calculations. Its source initially used 25 m/s while hosted
firmware uses `initialShotVelocity=24`; the current source now uses 24 m/s to
match firmware. The phase results predate that speed correction. The current
sim E2E launch defaults pose latency to zero; hardware's 0.01 s tracker setting
is unchanged. A zero-latency run is a simulation experiment only.

The covariance experiment at `.01/.0/.01` was rejected after scoring fell
from ballistic's 5/48 drive and 3/36 match hits to 2/49 and 2/36. It is one
run per setting and the result does not establish a reliable covariance
effect. The orientation-fusion experiment failed both stage assertions and
both native log checks (0/48 drive, 1/36 match); its patch was fully reverted
and retained as `rejected-orientation.patch` in the artifact directory. Keep
the 1 ms scheduler; the period run is a one-run observation (4/49 drive,
3/36 match), not evidence that changing the integration step improves
accuracy. The `speed` phase completed one drive and match run (3/49 drive, 0/35 match); this is one-run evidence only. The brake phase below was measured on the source baseline before the track I merges currently underway.

## Brake phase, three repeats

Brake adds powered holding at zero command. In the three `mcb_parked` runs, maximum measured chassis displacement was 19.2, 28.9 and 22.4 mm; the prior coasting observation was 132.0 mm. This supports reduced drift under powered holding in these runs. The firmware unit regression passed all 80 tests. The source snapshot for this report predates the track I merges now underway.

The table gives native MCB hits/shots for each 20-second parked cell. These are cell outcomes, not pooled accuracy floors.

| Cell | r1 | r2 | r3 |
| --- | ---: | ---: | ---: |
| stationary lateral | 41/41 | 41/41 | 41/41 |
| speed 1 lateral | 11/40 | 24/41 | 16/40 |
| speed 2 lateral | 4/40 | 12/41 | 5/40 |
| speed 4 lateral | 12/41 | 25/41 | 20/41 |
| stationary radial | 39/40 | 40/41 | 40/40 |
| speed 1 radial | 17/40 | 0/40 | 15/40 |
| speed 2 radial | 26/40 | 20/41 | 20/40 |
| speed 4 radial | 33/40 | 31/40 | 0/40 |
| stationary diagonal | 41/41 | 40/40 | 40/40 |
| speed 1 diagonal | 6/40 | 8/40 | 32/40 |
| speed 2 diagonal | 17/41 | 16/41 | 14/40 |
| speed 4 diagonal | 25/41 | 22/40 | 27/40 |

| Stage / repeat | Drive or match segment hits/shots | loc/nav/route p95 (m) | Nulls in loc/nav/route | RELOCALIZE TX | Native result |
| --- | --- | ---: | ---: | ---: | --- |
| drive r1 | total 7/49 (14.3%); approach 0/13; parked 2/10; straight 0/4; turn 1/10; spin 4/12 | 0.114/0.157/0.254 | 0/2541 each | 1 | pass |
| drive r2 | total 3/49 (6.1%); approach 1/13; parked 0/10; straight 1/4; turn 1/10; spin 0/12 | 0.094/0.103/0.259 | 0/2541 each | 3 | pass |
| drive r3 | no stage samples | — | — | 0 | fail: localized map→root bring-up; no shots |
| match r1 | total 1/36 (2.8%); parked 0/10; straight 0/4; turn 1/10; spin 0/12 | 0.068/0.026/0.273 | 0/3369 each | 2 | pass |
| match r2 | no MCB shots | 0.193/0.230/0.228 | 0/3386 each | 3 | fail: `mcb_match_spawn_to_center` |
| match r3 | total 2/37 (5.4%); parked 0/10; straight 0/5; turn 2/10; spin 0/12 | 0.220/0.091/0.260 | 0/3386 each | 1 | pass |
| parked r1 | 272/484 across 12 cells | n/a | n/a | 19 | pass |
| parked r2 | 279/487 across 12 cells | n/a | n/a | 18 | fail: `mcb_parked_speed1_radial` |
| parked r3 | 270/481 across 12 cells | n/a | n/a | 14 | fail: `mcb_parked_speed4_radial` |

Every completed drive/match route record had numeric localization, navigation and route errors (zero null/missing values). The failed match r2 did produce pose records before its spawn-to-center assertion; its shot log is empty. Lockstep reported zero timeouts in every completed artifact; no lockstep-invalid run is counted as valid accuracy. RELOCALIZE TX counts are log events and are reported as diagnostics, not scored accuracy.

The native checker passed the remaining completed stage runs. “Pass” here means that stage’s native suite and log checker accepted that invocation; it does not mean all nine invocations passed. Failed brake runs remain in the score tables as diagnostics, not accepted accuracy results. Hit counts across repeats vary substantially, and r2 failed the speed 1 radial cell and r3 failed the speed 4 radial cell. These data show the powered-holding displacement change, but they do not establish repeatable moving-shot accuracy or a reliability floor.

## Merged nightly no-lead profile, three repeats

The no-lead profile sets residual `gimbal_lag_s=0`; projectile flight-time
prediction remains enabled. It uses the filtered firmware 25 ms rate, a
`chase_settle_s` of 25 ms, `chase_margin_s` of 25 ms, and 94 ms indexing. All
runs used the hosted backend, with the merged Track I firmware auto-drive route.
This is a controlled hosted comparison; it does not establish board-backend
accuracy. The archived nightly baseline revisions were sim `7156ab2`,
firmware `e98d548`, bridge `3a1220c`, thornbots `7b3032a`, sentry `af45710`,
and root `3b81631`; the later root docs-only merge `b32fc13` is separate.
The candidate bases were firmware `111886e`, sim `53c7f70`, thornbots `3eecd35`,
bridge `928c25b` and sentry `4617056`, plus the final-profile patches preserved
in the artifact root. `final-profile-source.json` records base revisions,
patch hashes and immutable run ELF hashes. The run firmware is
`rate-hosted-MCB-project.elf`; final-source rebuild and test logs have the
`profile-final-` prefix. All six branches contain their respective nightly
heads; the workspace also merges the later roadmap cleanup.

Drive and match report native MCB score records; parked stages report the
native per-cell `scores.jsonl` snapshots. Percentages below pool only the three
listed repeats as descriptive totals, not as an accepted reliability floor.
The `rate-hosted-final` stages all passed their native checkers. Nightly main
parked and the spinning variant failed the native accuracy floor; the other
nightly main and candidate moving stages passed.

| Hosted profile / stage | r1 | r2 | r3 | pooled |
| --- | ---: | ---: | ---: | ---: |
| no-lead `mcb_parked` | 283/482 (58.7%) | 279/481 (58.0%) | 304/484 (62.8%) | 866/1447 (59.8%) |
| no-lead `mcb_drive` | 32/35 (91.4%) | 27/35 (77.1%) | 26/35 (74.3%) | 85/105 (81.0%) |
| no-lead `mcb_match` | 20/40 (50.0%) | 20/40 (50.0%) | 20/39 (51.3%) | 60/119 (50.4%) |
| nightly main `mcb_parked` | 183/483 (37.9%) | 210/483 (43.5%) | 207/487 (42.5%) | 600/1453 (41.3%) |
| nightly main `mcb_drive` | 2/35 (5.7%) | 0/35 (0%) | 2/35 (5.7%) | 4/105 (3.8%) |
| nightly main `mcb_match` | 6/39 (15.4%) | 3/40 (7.5%) | 5/39 (12.8%) | 14/118 (11.9%) |
| no-lead moving parked (speed 2, zero spin) | 101/123 (82.1%) | 77/123 (62.6%) | 101/123 (82.1%) | 279/369 (75.6%) |
| nightly moving parked (speed 2, zero spin) | 20/122 (16.4%) | 32/123 (26.0%) | 48/122 (39.3%) | 100/367 (27.2%) |
| no-lead spinning parked (stationary, 1 Hz spin) | 0/123 (0%) | 0/123 (0%) | 0/123 (0%) | 0/369 (0%) |
| nightly spinning parked (stationary, 1 Hz spin) | 10/122 (8.2%) | 11/121 (9.1%) | 0/122 (0%) | 21/365 (5.8%) |

Each parked total is the sum of native cell score snapshots. The no-lead
parked cells by speed band (stationary, speed 1, speed 2, speed 4) were
r1 123/123, 68/119, 56/119, 36/121; r2 123/123, 57/119, 59/118, 40/121;
and r3 122/122, 53/122, 68/119, 61/122. The no-lead spinning candidate
failed because all three repeats scored zero hits in each of the stationary
lateral, radial and diagonal 1 Hz spin cells, below the native 10% cell
threshold. The launch's `target_spin_hz=0` is only bring-up configuration;
the native cases set the moving-cell spin rate. These failures show that
spinning accuracy remains unsolved, despite stable chassis motion.

| Profile / stage | loc p95 m, r1/r2/r3 | nav p95 m, r1/r2/r3 | Missing loc/nav | RELOCALIZE TX, r1/r2/r3 | max chassis displacement |
| --- | --- | --- | --- | --- | --- |
| no-lead parked | 0.886 / 0.661 / 1.395 | 0.945 / 0.679 / 1.348 | 0 / 0 | 30 / 28 / 39 | 3.080 / 3.027 / 3.039 mm |
| no-lead drive | 0.203 / 0.218 / 0.247 | 0.134 / 0.244 / 0.222 | 0 / 0 | 3 / 4 / 5 | — |
| no-lead match | 0.149 / 0.086 / 0.111 | 0.094 / 0.043 / 0.061 | 0 / 0 | 1 / 0 / 1 | — |
| nightly main parked | 0.802 / 0.896 / 1.955 | not logged | 0 / all rows | 39 / 32 / 49 | not logged |
| nightly main drive | 0.309 / 0.138 / 0.335 | not logged | 0 / all rows | 4 / 7 / 5 | — |
| nightly main match | 0.187 / 1.203 / 0.133 | not logged | 0 / all rows | 5 / 18 / 3 | — |
| no-lead moving parked | 0.377 / 0.226 / 0.210 | 0.376 / 0.227 / 0.210 | 0 / 0 | 7 / 4 / 4 | 0.677 / 0.560 / 0.476 mm |
| no-lead spinning parked | 0.193 / 0.088 / 0.244 | 0.194 / 0.087 / 0.244 | 0 / 0 | 3 / 1 / 4 | 0.709 / 0.672 / 0.732 mm |
| nightly moving parked | 0.227 / 0.179 / 0.326 | not logged | 0 / all rows | 8 / 3 / 6 | not logged |
| nightly spinning parked | 0.219 / 0.217 / 0.266 | not logged | 0 / all rows | 4 / 5 / 5 | not logged |

For the final no-lead profile, all route records had numeric localization and
navigation errors (zero nulls). Nightly logs have numeric localization but
do not contain a navigation-error field; “not logged” is not a missing
transform count. The nightly parked archives lack `truth_xy`/parking samples,
so their chassis displacement cannot be recovered. Powered holding kept the
no-lead full parked sweep within 3.081 mm maximum displacement, and the
moving/spinning parked candidates within 0.733 mm. This confirms holding
stability in these runs; it does not resolve spinning accuracy. The parked
localization/navigation p95 variation across repeats also remains open.

The spinning score failure is specific to target accuracy, not chassis
motion or lockstep: all candidates stayed below 0.733 mm displacement, with
no lockstep timeout. Native checker failure cells are visible in the archived
logs; the nightly main parked failures were r1 `speed4-lateral` and
`stationary-diagonal`, r2 `stationary-lateral` and `speed2-radial`, and r3
`speed4-lateral`. Nightly moving r1 failed its radial/diagonal floors; r2/r3
passed. Nightly spinning r1 failed lateral/radial, r2 failed lateral, and r3
failed all three cells. No failed accuracy run is counted as evidence of a
passing floor.

Final-source checks passed: firmware pure logic 35, hosted regression 5,
UART/emulator 89, ROS sim 19 suites, thornbots 8 suites, and bridge 9 suites.
ARM GCC 13 board and GCC 14 hosted sentry builds passed. Hardware, SLLidar, `main`,
Humble and Mac track-I checks were skipped. Board accuracy remains unavailable:
local Renode timed out. Moving-accuracy acceptance remains open, and native
failures mean the system is not ready to claim reliability. The historical
pre-track-I phases above are separate from this post-merge hosted comparison.

### Error chain and changes in the final profile

The parked command now latches raw aim position rather than merely requesting
zero wheel speed. Centered DBUS drive uses the same hold. A keyboard peek-release
trigger previously scheduled beyblade at startup despite centered DBUS input;
it now requires keyboard mode. Navigation fixes cannot move the hold point.
The raw-start discontinuity reanchors the latch rather than driving toward the
old team-start point. Emulator regressions exercise correction isolation,
displacement recovery, startup spin and reanchor behavior.

Absolute 40 Hz aim updates previously became one-control-tick yaw velocity
impulses in the 1 kHz firmware controller. The 25 ms rate filter preserves the
mean rate, wraps yaw and clears on a target handoff. Command/achieved-angle
replay supported zero additional hosted yaw lead after this change. The wire
delay now contains indexing latency plus alignment wait, and firmware clamps
short requests to immediate indexing rather than unsigned subtraction.
Chamber-exit replay measured 94 ms; the earlier 80 ms estimate left a 14 ms
residual. Hardware indexing and board gimbal response remain unmeasured.

These diagnostics pool all three final repeats. Each number is median / p95;
they describe stages in the error chain and must not be added as independent
error components. Shot geometry is sampled at chamber exit and modeled impact;
state forecasts use their own logged horizon.

| Diagnostic | Drive | Match | Moving parked | Spinning parked |
| --- | ---: | ---: | ---: | ---: |
| Detection present / valid tracker, shots | 105/105 / 105/105 | 119/119 / 119/119 | 369/369 / 369/369 | 369/369 / 369/369 |
| Detection age, ms | 4 / 13 | 10 / 13 | 10 / 10 | 3 / 4 |
| Tracker age, ms | 26 / 47 | 26 / 47 | 20 / 50 | 19 / 45 |
| Aim to closest impact panel, m | .0522 / .1142 | .0400 / .1803 | .0518 / .1707 | .0616 / .1196 |
| Command-to-achieved absolute yaw error, deg | 1.287 / 3.657 | .645 / 3.141 | .154 / .542 | 10.085 / 11.024 |
| Command-to-achieved absolute pitch error, deg | .291 / 1.427 | .412 / 1.441 | .642 / .882 | .701 / .829 |
| Head TF orientation error, deg | .690 / 2.852 | .450 / 2.157 | .220 / .818 | .973 / 2.049 |
| Native impact miss distance, m | .0444 / .0939 | not emitted | .0301 / .1009 | .1453 / .2051 |
| State center XY error, m | .0266 / .1299 | .0358 / .1560 | .0788 / .1281 | .0323 / .0712 |
| State velocity error, m/s | .2122 / 1.1455 | .1464 / 1.3121 | .3497 / 1.1367 | .2110 / .4946 |
| Constant-velocity forecast error, m | .0562 / .5852 | .0568 / .5396 | .1151 / .2856 | .0541 / .1200 |

Spinning misses are reproducible with detections and locks intact: the large
near-range yaw-following error is the leading measured actuator problem.
It exceeds the panel's angular width at the roughly 1 m range; forecast tails
and panel transitions also remain contributors. This profile has not solved
that case. The parked navigation p95 spread persists despite millimetre chassis
motion, so powered holding cannot explain away the localization variation.
No covariance policy or acceptance threshold was relaxed for these repeats.
Detection stand-ins do not validate YOLO, occlusion or real camera timing.

## Spinning correction and board comparison

Artifacts for this continuation are in
`/home/blaise/workspaces/isaac_ros-jazzy/test-aim-spin-2026-10-12/`.
`spin-source.json` records source bases, patches, ELF hashes and execution
controls. Firmware is unchanged from `c674b1e`; the same immutable rate-filtered
ELF is used throughout the treatment runs.

### Spinning policy ablations

All screens use the three 20-second stationary-target, 1 Hz cells, with fresh
stacks. The unfiltered ELF retains raw holding and 94 ms indexing, isolating
the rate filter. Lead/window controls are explicit launch arguments. Failed
screens remain failed; pooled percentages do not replace each cell's 10% floor.

| Screen | Lateral / radial / diagonal hits | Total | Yaw following error median / p95, deg | Native result |
| --- | --- | ---: | ---: | --- |
| Previous filtered chase, zero residual lead, 25 ms windows (three repeats) | 0/41 in every cell/repeat | 0/369 | 10.085 / 11.024 | all fail |
| Filtered chase, 25 ms residual lead | 0/41 / 15/41 / 8/40 | 23/122 (18.9%) | 4.452 / 9.341 | lateral fails |
| Unfiltered chase, zero residual lead, 25 ms windows | 0/41 / 0/41 / 0/40 | 0/122 | 8.798 / 10.864 | all fail |
| Filtered alignment gating, zero residual lead | 41/41 / 37/37 / 35/35 | 113/113 (100%) | .024 / .083 | all pass |

Removing the filter does not recover spinning. Additional chase lead reduces
following error but moves the impact prediction: aim-to-nearest-panel p95 is
.338 m in that screen. Alignment gating holds the center-to-shooter line and
times exit to a facing panel, avoiding recurring yaw jumps. Its nearest-panel
error is .014 m median / .039 m p95. Detections and valid tracks remain present;
the change addresses the measured chase/actuator interaction rather than
masking tracker loss. The gated drive/match screens passed at 30/32 (93.8%)
and 20/37 (54.1%). These screens select a policy; the fresh repeats establish
its repeatability.

`chase_settle_s=-1` now selects alignment gating by default in the robot node,
full launch and both E2E backends. Nonspinning panel lead/hysteresis is unchanged;
chasing remains opt-in with a nonnegative settling time. The 25 ms setpoint-rate
filter and indexing calibration remain unchanged. A regression checks both spin
directions at 1 Hz: yaw stays steady and every allowed delay meets the predicted
alignment at the 94 ms exit horizon.

### Fresh hosted repeats

The unchanged filtered firmware and alignment-gated planner use zero hosted
residual lead and the 94 ms indexing estimate. Moving cells use 2 m/s with no
target spin; spinning cells use a stationary 1 Hz target. Each special sweep
covers lateral, radial and diagonal trajectories. Every fresh native cell
floor and checker passed.

| Stage | Nightly hit % r1 / r2 / r3 | Alignment gating hits r1 / r2 / r3 | Gated hit % r1 / r2 / r3 | Pooled nightly → gated % |
| --- | --- | --- | --- | --- |
| Parked | 37.9 / 43.5 / 42.5 | 355/425 / 361/428 / 370/431 | 83.5 / 84.3 / 85.8 | 41.3 → 84.6 |
| Drive | 5.7 / 0 / 5.7 | 31/32 / 31/32 / 31/32 | 96.9 / 96.9 / 96.9 | 3.8 → 96.9 |
| Match | 15.4 / 7.5 / 12.8 | 20/34 / 20/37 / 20/37 | 58.8 / 54.1 / 54.1 | 11.9 → 55.6 |
| Moving | 16.4 / 26.0 / 39.3 | 89/122 / 76/123 / 91/122 | 73.0 / 61.8 / 74.6 | 27.2 → 69.8 |
| Spinning | 8.2 / 9.1 / 0 | 99/99 / 122/122 / 122/122 | 100 / 100 / 100 | 5.8 → 100 |

| Stage | Global localization p95 m r1 / r2 / r3 | Local navigation p95 m r1 / r2 / r3 | RELOCALIZE sends r1 / r2 / r3 | Maximum parked displacement mm r1 / r2 / r3 |
| --- | --- | --- | --- | --- |
| Parked | .274 / .345 / .285 | .276 / .348 / .295 | 9 / 13 / 12 | 3.550 / .962 / 3.544 |
| Drive | .230 / .323 / .289 | .331 / .390 / .274 | 5 / 5 / 5 | N/A |
| Match | .139 / .108 / .178 | .107 / .056 / .111 | 2 / 1 / 2 | N/A |
| Moving | .498 / .289 / .361 | .494 / .290 / .356 | 9 / 5 / 6 | .533 / .599 / .591 |
| Spinning | .084 / .330 / unavailable | .084 / .328 / .321 | 1 / 6 / 6 | .497 / .405 / .482 |

Spinning r3 has 1,198 missing global-transform samples: map_server's lifecycle
configure response timed out, AMCL was never activated, and `map` was absent.
Its local navigation transforms are present in every sample, and all 122
physical shots hit. This matches the existing
[lifecycle-startup failure](mcb-auto-drive-2026-10-12.md#evidence-and-acceptance-status); missing
localization values are excluded from percentiles and explicitly unavailable,
not zero error. The native combat check passes without establishing successful
global localization startup. This accuracy change does not repair that race.

Fresh drive/match physical-shot diagnostics retain detection and valid track
on all 96/108 scored exits. Drive yaw-following error median/p95 is .457/.726
degrees; match is .392/1.099 degrees. Nearest predicted panel error median/p95
is .018/.059 m drive and .015/.147 m match. Detection-age p95 is 16/13 ms;
tracker-age p95 is 47/46 ms. These are modeled impact-goal diagnostics, subject
to the target-association limitations below, not hardware calibration. Remaining
match misses persist despite fresh detections and locks; the spin yaw-following
failure is no longer the dominant source.

The hosted match denominator also includes firing at a defeated robot.
Across the three fresh matches, 97/108 shots intersect an eligible enemy panel;
37 of those occur after `opponent_0` has zero HP and the referee correctly
refuses another hit. The remaining 11 strike hull (4) or field (7). Native
`enemy_hit` stays 60/108 (55.6%); this report does not replace that acceptance
metric with the 89.8% geometric diagnostic. Every match kills `opponent_0`,
with zero friendly intersections. Continuing to select its visible panels is
an outstanding target-eligibility behavior, rather than 37 prediction misses.
Board candidate match r1 similarly has 31/36 eligible enemy-panel intersections,
including 11 after defeat; native hits remain 20/36. No death-state filtering
or referee/scorer relaxation is part of this spinning fix.

The original fresh moving sweep is below the previous filtered chase profile's
75.6% pooled score. Its state history never enters spin mode: replaying the
3/2 rad/s hysteresis gives zero spin-mode samples for both three-run sets.
Nonspinning `plan_shot` returns before reading the spin fire policy; an added
regression verifies identical aim, delay and held panel for radial, lateral
and diagonal velocities. Additional alternating paired controls keep firmware,
filter, scene and all parameters fixed except `chase_settle_s` (-1 versus .025).
Their full results below retain the lower original sweep rather than replacing it.

### Moving policy controls

Six paired sweeps alternate order: gate/chase on odd repeats, chase/gate on
even repeats. All 12 native sweeps clear every 10% cell floor.

| Repeat | Gated hits (%) | Chase hits (%) |
| --- | --- | --- |
| 1 | 91/121 (75.2) | 98/122 (80.3) |
| 2 | 90/122 (73.8) | 100/123 (81.3) |
| 3 | 90/123 (73.2) | 88/122 (72.1) |
| 4 | 91/121 (75.2) | 91/121 (75.2) |
| 5 | 102/122 (83.6) | 99/122 (81.1) |
| 6 | 87/121 (71.9) | 102/123 (82.9) |
| Pooled | 551/730 (75.5) | 578/733 (78.9) |

The observed gated/chase difference is -3.4 percentage points pooled; paired
run differences range from -11.0 to +2.5 points. Neither profile enters spin
mode in the logged state history, so the changed spin branch cannot explain
these moving differences. The original gated 69.8% and earlier chase 75.6%
remain in the record. Repeats support substantial bench variability, not a
claim of identical measured scores or a formal noninferiority result. All
moving measurements remain well above nightly's 27.2% pooled baseline.

### Board execution controls

The initial pristine-nightly screen reproduced the earlier failure: the
coordinator repeatedly missed a 50 ms look-ahead hold under its 2-second wall
deadline. The run was stopped after irreversible lockstep failures; its checker
reports `BENCH NOT TRUSTWORTHY` and its shots are not accuracy evidence.

The board comparison uses a 5 ms look-ahead, one firmware batch, on both
nightly and treatment. The 2-second coordinator timeout, 5-second clock-stall
guard, 1 ms physics step and rejection of any lockstep timeout are unchanged.
The archived nightly launch receives only this execution-control patch; its
firmware, localization, aiming, physical model and native scorer remain nightly.
The outer runner allows enough wall time for the existing native board budget;
it cannot turn a native timeout into a pass.

The first extra container lacked `arm-none-eabi-objcopy`; its zero-shot
bring-up failure is preserved and rejected. Subsequent extra starts exposed a
separate fixture-install bug: a writable `/tmp/renode-1.17.0` matches Renode's
[stale temporary-directory cleanup](https://github.com/renode/renode-infrastructure/blob/066a7f13c052215632d469c995c89aea37c573b1/src/Emulator/Main/Utilities/TemporaryFilesManager.cs#L110),
which recursively deletes non-live `renode-*` directories. The missing runtime
DLL was present in the verified archive, then deleted from the install. Earlier
root-owned installs masked the issue by denying deletion. The installer default
and maintained example now use `/tmp/mcb-renode-1.17.0`. Running the revised
installer as the ordinary container user, compiling both Type C models and
exiting retains the runtime DLL and succeeds. Extra repeats use the identical
prepared runtime under `/opt/mcb-renode`, selected with `MCB_RENODE_ROOT`;
no Renode CPU frequency, model or firmware RAM is changed. These pre-scoring
failures are not counted as accuracy repeats.

Nightly and treatment run in separate owned containers with ROS domains 13/14
and distinct Gazebo partitions. Extra third repeats use domains 15/16 in a
fourth owned container and the hosted container after its checks finish.
Hosted validation uses domain 1; the final independent-package retry uses 17. Each container
refuses a second live stack. No hardware or Mac Track I work is involved.
The board native scorer remains that build's scorer, as in the earlier hosted
comparison; these complete-stack comparisons do not isolate the aiming policy
from the earlier chamber-exit/model corrections.

## Remaining evidence gaps

In the identity-aware covariance run, state center error medians were 114 mm
drive and 105 mm match; p95 was 296 and 301 mm. The 0.135 m panel width is
smaller than the p95 tails. Velocity error p95 was 3.68 m/s drive and 4.59
m/s match; constant-velocity prediction error p95 was 1.03 m and 1.07 m, and
constant-acceleration prediction error p95 was 1.11 m and 1.15 m. This
covariance setting was rejected and these are one-run diagnostics, but they
show that center position alone does not bound future miss distance. The
prediction horizon still depends on the modeled 24 m/s shot and capture-time
truth.

In the same run, absolute gimbal target-to-achieved yaw error p95 was 11.72
deg drive and 15.10 deg match; pitch error p95 was 11.62 and 11.05 deg. The
simulation uses synthetic panel detections as truth stand-ins. These results
do not include YOLO detection error, image noise, or camera occlusion.

Match `aim_closest_panel_m` in the current diff compares the impact-goal
point, including inherited muzzle motion, with the nearest panel for the
history selected by the scorer. It still depends on the modeled arrival
horizon and ballistic assumptions; a missed shot can still fall back to
`opponent_0`. `aim_off_panel_m` remains a legacy lead/impact diagnostic and
should not be interpreted as direct tracker-center error.

The preceding no-lead/chase acceptance status is historical and is superseded
by the spinning correction and board comparison above. Those powered-holding
repeats established chassis stability within the 3 cm bound; spinning accuracy
was still open at that point.
Bridge #8 and sentry_localization #6 are ready as reviewed package layers,
subject to coordinated deployment. Firmware #89, thornbots #13, sim #16 and
workspace #40 remain blocked for the combined accuracy acceptance: the
spinning floor fails and the board backend has no fresh accuracy comparison.
Every PR remains a draft. Repository quality and actionlint passed in all
six repositories without baseline growth. The final-source ROS checks passed
all 19 sim, 8 thornbots and 9 bridge suites.
The combined colcon result is 377 cases, zero failures/errors and nine skipped
bridge cppcheck file cases: ament reports cppcheck 2.13 performance issues.
This existing tool skip is not a skipped runtime or emulator test.
The final Renode reconnect regression passed (1/1); this exercises peripheral
connection behavior, not board combat accuracy. GitHub's firmware hosted/UART
job also passed for `c674b1e`; ARM matrix and ROS jobs are draft-skipped, with
local board builds and ROS results reported above.
Those earlier runs left no owned launch trees, and their Jazzy container was
stopped. The spinning/board continuation uses separate owned containers and
records its final cleanup with the new results.

## Artifacts

Each phase directory contains `shots.jsonl`, `states.jsonl`, `route.jsonl`,
`navigation.jsonl`, `scores.jsonl`, and checker/stack/launch logs where
available. Stage summaries are `segments.json` for drive and `match.json` for
match. Runner and outcome records are at the artifact root, including
`physical-outcomes.json`, `imu-outcomes.json`, `stamp-outcomes.json`,
`timed-outcomes.json`, `trace-outcomes.json`, `yawfix-outcomes.json`,
`ballistic-outcomes.json`, `covariance-outcomes.json`, `period-outcomes.json`,
and `orientation-outcomes.json`. The rejected orientation experiment is
preserved in `rejected-orientation.patch`.
