# Track A stamped-input investigation, 2026-10-14

Mac arm64, reused Jazzy container, fresh worktree from workspace `5e1fd040`.
The original checkout was not edited. No physical robot or Archlinux run is
part of this investigation. Native runners and `check_bench_log` decide
completion; their failures remain in the record.

## Controls and acceptance boundary

Nightly pins: sim `e81064a`, thornbots `bd3d94f`, localization `e930308`,
bridge `9d849f7`, firmware `432216d`. Sources were copied from the fresh
worktree into an isolated container build, leaving the existing installed
workspace untouched. The freshly built hosted ELF SHA-256 is
`70feb397e468906360e12f65ce9f37bc3ccf3f60e53b8c3286bf77b416dbaaad`.
All hosted profiles use that same executable and the source-default launch
controls. Bench invocations run sequentially. Native timing and rejection
deadlines are unchanged; automatic display detection selects headless on
this Mac.

The first strict treatment is sim `c57d8aa` and thornbots `6890f29`.
Its native passes do not establish no regression: the three-run drive hit
rate decreased, so that treatment is rejected. A follow-up at thornbots
`3b51e4d` retains recent same-track states with available original-stamp TF;
it changes input selection, without tracker parameter tuning. Sim remains
`c57d8aa`, and other package/ELF pins remain nightly.

This work does not close T17. UART serialization, the remaining input
consumption chains, deterministic bring-up, and the required identical-shot
comparisons remain unverified. The board parked-cell sweep is still pending.
The [lockstep plan](../../E2E_LOCKSTEP_PLAN.md#order-and-done-when) orders
combat-floor calibration after those gates. No new enforced floor is selected
here; existing parked-cell 10% floors and 3 cm displacement bounds remain.

Remaining code departures from the intended capture/consumption contract:
the PTY delivers bytes without modeled wire serialization, the bridge's
arrival-time-minus-wire calculation therefore does not establish simulated
capture timing, and an MCB-only acknowledgement cannot certify consumption by
the other ROS nodes. The wall-time robot-start timer also remains. A zero
fallback counter must not be reported as proof that these paths are lockstep.

## Pinned hosted baseline

Drive uses native MCB `hit`; match uses native MCB `enemy_hit`, including
shots at defeated opponents in the denominator. Parked totals use the native
per-cell score snapshots, rather than later impact completions in JSONL.

| Stage | Selected repeats | Hits / shots | Pooled hit rate |
| --- | --- | ---: | ---: |
| Drive | r1 / r2 / r4 | 28/32 + 31/32 + 29/32 = 88/96 | 91.7% |
| Match | r1 / r2 / r3 | 20/36 + 20/36 + 20/38 = 60/110 | 54.5% |
| Parked, one 12-cell sweep | r2 | 362/428 | 84.6% |

All selected runs pass the native runner and checker with zero lockstep
timeouts. Parked has one repeat, not the required three-run comparison.
Every moving-stage approach fired zero shots. Match's drive segment also had
no sentry shots; spin contained the scored shots. A per-stage aggregate does
not turn these no-shot segments into combat accuracy passes.

Baseline drive r3 fails the localized `map->root` bring-up wait before
scoring. It is retained with runner/checker exits 1/1 and excluded from the
accuracy denominator. Separate r2/r3 preflight refusals detected owned unit
test processes and launched no simulation; those are orchestration records,
not additional navigation failures. Parked r1 failed runner setup before
scoring (`COLCON_TRACE` unset under shell nounset). The external runner was
corrected; native startup limits were unchanged.

## Inspect inputs before tracker tuning

The accepted baseline parked sweep's worst cell was speed 4 diagonal,
19/30 (63.3%). Every cell cleared its unchanged floor. Across all native
shot rows, detection age reached 19 ms and tracker age 64 ms; every logged
tracker-valid value was true. Maximum stamped `odom_disagreement_m` was
approximately `1.77e-11 m`. The 4,918 pose records contain navigation,
globally corrected navigation and localization positions. An early
`map_pose_publisher` missing-map warning does not establish missing navigation
during scoring. No lost-track warning was found.

There are 431 raw MCB shot rows, versus 428 in the scored snapshots: three
additional successful impacts completed after the cell snapshots. The
reported parked hit rate uses 362/428, not the raw 365/431. Baseline logs do
not include the new complete input ledger, so their shot/state samples cannot
establish tracker validity during every no-shot interval.

The new ledger records empty/nonempty detection batches, consumed tracker
measurement headers, valid/invalid target states, raw MCB POSE, `/odom` and
`/localization/odom`. Each row retains its capture stamp, observed arrival
simulation time, cell and frame. See the [schema owner](../README.md#mcb-emulator).

## Rejected strict-only drive control

| Repeat | Native hits / shots | Native runner / checker | Latest-TF fallbacks |
| --- | ---: | --- | ---: |
| r1 | 22/30 | 0/0 | 0 |
| r2 | 27/29 | 0/0 | 0 |
| r3 | 27/29 | 0/0 | 0 |
| Pooled | 76/88 (86.4%) | all native accepted | 0 |

This is 5.3 percentage points below nightly's 91.7%; native completion is not
no-regression acceptance. The treatment also records drive/spin localization
tails near 0.48 m in r2/r3, versus baseline spin p95 about 0.18–0.23 m.
No floor was changed to accept these results. The history treatment addresses
newer predictions displacing a still-usable stamped state while TF catches
up; it does not substitute latest TF or override invalid/expired tracks.

### Input ledger audit of the rejected control

| Repeat | Valid / invalid target states | Empty / nonempty detection batches | Longest tracker measurement gap | Valid states ahead of prior observed `/odom` |
| --- | ---: | ---: | ---: | ---: |
| r1 | 1,396 / 2 | 240 / 1,562 | 3.333 s | 1,377 / 1,396 (98.6%), at most 27 ms |
| r2 | 1,396 / 3 | 265 / 1,827 | 3.184 s | 1,389 / 1,396 (99.5%), at most 13 ms |
| r3 | 1,368 / 2 | 240 / 1,821 | 3.467 s | 1,368 / 1,368 (100%), at most 11 ms |

These counts include startup, scoring-off and no-shot intervals. During the
longest measurement gaps, 199/200, 190/191 and 207/208 detection batches were
empty. The evidence identifies missing detector output during those gaps;
it does not show the tracker dropping a steady stream of nonempty input.
The comparison against prior observed `/odom` is a callback-order proxy,
not a direct record of TF lookup success.

Raw POSE and `/odom` stamps were monotonic, without duplicates. Median cadence
was 10 ms, with maximum gaps 22/21/23 ms. Detection cadence was about 17 ms,
with maximum gaps 20/22/63 ms. Native-shot detection/tracker age p95 was
15.6/40.8, 13.2/59.2 and 13.1/49.6 ms. Native-shot navigation/localization
errors were numeric throughout; odometry disagreement p95 was at numerical
precision. Detection arrival-clock minus capture-stamp ranged from -2 to
+41 ms, -2 to +7 ms and -71 to +54 ms. Negative differences expose that the
scorer's observed clock can lag another node's stamped output; they are not
negative physical transport delays. This is additional evidence that the
complete clock/consumption barrier is still missing.

## Stamped-state history treatment

| Hosted stage | Native hits / shots by repeat | Pooled | Pinned nightly |
| --- | --- | ---: | ---: |
| Drive | 30/30, 31/31, 27/30 | 88/91 (96.7%) | 88/96 (91.7%) |
| Match | 20/36, 20/34, 20/36 | 60/106 (56.6%) | 60/110 (54.5%) |

All six repeats pass runner/checker with zero lockstep timeouts and
zero latest-TF fallbacks. Localization/displacement diagnostics remain in the
artifacts: r1/r2 have spin localization p95 about 0.415/0.411 m, versus
0.194 m in r3; maximum displacement is about 5.09/5.08/4.86 m. The hit-rate
improvement does not erase those navigation tails.

The drive worst run is 90%; subtracting a stated ten-percentage-point margin
would propose 80%. This is a proposal from three native-accepted repeats,
not an installed calibrated floor. Match's worst run is 20/36 (55.6%);
the same margin proposes 45.6% (rounded for display), also unenforced.
Both stages improve their pooled hit rate over the three pinned nightly
repeats. `compare_runs` still rejects drive r1
versus r2: the first shot record is stamped 5.119 versus 10.119 s and other
fields also differ. Raw shot counts differ. No time normalization, tolerance
increase or record filtering was used to hide the failure. UART delivery,
consumer ordering and wall-time bring-up still need implementation before
this evidence can satisfy T17 or authorize final combat calibration.

The history treatment's first hosted 12-cell parked sweep passes all native
cell checks and the checker, at 350/430 (81.4%). The actual chassis parking
log's maximum displacement is 0.003147 m, within the unchanged 0.03 m bound.
Tracker-center errors are a separate diagnostic and must not be substituted
for chassis displacement. This single sweep is below the single nightly
sweep's 84.6%; neither is the required three-repeat parked comparison.

## Checks and artifacts

RTF is the native timing table's total over sim-timed spans. Waits below are
cumulative wall seconds from each stack log's unique `lockstep (final)` line,
including bring-up. Repeat order is the selected order above.

| Profile / stage | RTF r1 / r2 / r3 | gz wait s | MCB-batch wait s |
| --- | --- | --- | --- |
| Nightly drive (r1/r2/r4) | .99 / .93 / 1.21 | 32.70 / 34.31 / 29.05 | .411 / .0898 / .0709 |
| Nightly match | 1.03 / .76 / .82 | 42.00 / 53.19 / 50.32 | .0735 / .402 / .112 |
| History drive | .60 / 1.21 / 1.22 | 51.22 / 28.08 / 28.80 | .122 / .0861 / .856 |
| History match | 1.02 / 1.02 / 1.00 | 41.68 / 41.25 / 42.10 | .0729 / .0827 / .307 |

All selected final lines report zero timeouts. Gazebo stepping accounts for
the largest reported wait, but MCB batch is the only configured consumption
chain. This table cannot identify the bottleneck of the complete hit path.

Native acceptance is not a claim that every process exited cleanly. Apart
from SIGINT exits, accepted selected runs retain parameter-bridge exit `-6`
in nightly drive r1/r4 and history drive r1, clock-bridge exit `-11` in
nightly drive r2, and scan-bridge exit `255` in nightly match r1. Nightly
drive r1 also records Gazebo SIGTERM. The runner/checker statuses remain
unchanged; teardown repair remains the existing workspace E track.

Isolated Jazzy sim/thornbots checks pass 328 tests with zero errors,
failures or skips. The history follow-up passes all 132 thornbots tests.
Python 3.12 repository quality and actionlint pass without baseline growth.
Hosted and ARM firmware builds pass; a board build alone is not board-stage
acceptance. Empty/missing shot/state comparison and invalid tolerance cases
now fail closed in `compare_runs`.

Raw controls and rejected attempts are under container
`/tmp/track-a-20261014/results/`; the initial strict controls are under
`/tmp/track-a-candidate-20261014/results/`; the history follow-up uses
`/tmp/track-a-history-20261014/results/`. Each attempted stage retains its
launch/stack/checker logs and outcome manifest. Initial strict-control
outcome profile/workspace labels were normalized to `branch`/`5e1fd040`,
preserving original metadata separately; scores and exit statuses did not
change. The external runner and normalization record accompany the artifacts.
