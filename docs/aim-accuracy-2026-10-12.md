# Aim accuracy model investigation, 2026-10-12

This investigation records the completed model and timing phases in
`/home/blaise/workspaces/isaac_ros-jazzy/test-aim-accuracy-2026-10-12/`.
Each phase has one `r1` run per listed stage. The speed phase is in progress;
the final three full-stage repeats remain pending.

## Phase results

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

The three powered-holding repeats above establish chassis stability within
the 3 cm bound. They do not establish reliable moving-target accuracy. Fresh
comparisons with the merged nightly board route follow in a separate section.

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
