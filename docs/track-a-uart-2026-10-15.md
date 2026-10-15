# Track A sim-time UART, 2026-10-15

Mac arm64, reused Jazzy container, isolated build at
`/tmp/track-a-uart-20261015`. Workspace worktree began at `5e1fd040`, then
fast-forwarded to `fb31502` for an unrelated roadmap update; package pins
were unchanged. The [preceding controls](track-a-stamped-inputs-2026-10-14.md)
own nightly comparisons and rejected strict-TF treatments. No physical robot
or Archlinux runs were performed.

## Transport check and failed whole-stack attempt

The new opt-in fixture models both UART directions at 115200 baud, ten bits
per byte. A private envelope carries the firmware-cycle capture stamp around
unchanged DJI bytes; the existing bridge parser and CRC checks remain active.
The bridge acknowledges dispatch, not downstream consumption. Delivery,
stamp, ownership and launch behavior live at the
[interface owner](../README.md#sim-time-uart-fixture).

Four timeline tests and two socket transport tests pass. The on-demand real
bridge test passes and checks the exact capture stamp and POSE fields after
an actual best-effort topic-flow handshake. Its first attempt failed because
graph discovery preceded endpoint matching: the bridge logged publication,
but the subscriber missed the first frame. The test now establishes actual
flow before checking one frame; this is a startup observation, not a claim
that scored message loss is acceptable.

The first hosted whole-stack attempt used stationary lateral, speed zero,
duration ten seconds and the same pinned hosted ELF as the preceding controls.
Runner and checker both exited 1. During settling, Gazebo rejected a late
simulator UART transmit, then the harness reported a stalled clock. The
coordinator also recorded missed holds at 19.945 and 20.000 seconds. This
attempt is rejected; it supplies no accepted hit-rate or floor sample.

Raw POSE input rows demonstrate carried stamps, for example capture 0.113 s
and observed arrival 0.116 s. The failure exposes the missing outgoing
aimer → relay → bridge barrier: the MCB-only hold cannot ensure an outgoing
frame reaches the wire queue before its modeled deadline. No timestamp was
shifted, deadline enlarged, or checker result overridden. Socket mode remains
opt-in until those barriers are implemented.

## Checks and retained artifacts

The isolated sim, thornbots and bridge colcon results report 399 tests,
zero errors/failures and 95 skipped checks. Initial include-order/format and
launch-flake8 failures were corrected and their checks rerun. The real-bridge
test passes separately. Host Python 3.12 repository quality and actionlint
pass without baseline growth.

Controls and rejected attempts were exported to
`/Users/blaise/ros2_track_a_artifacts_20261015/`, with a manifest and per-file
SHA-256 values. The rejected UART attempt is
`track-a-uart-20261015-stationary-r1.tar.gz`, SHA-256
`633894ca940630305b80693b33bb4d62f6588a2604cec7968b9bd979d7676b87`.
The host coordinator independently checked all four exported archive hashes
and tar listings. The original checkout and unrelated containers were untouched.

## Paused startup and consumption experiment

The next treatment starts paused at sim zero, verifies lifecycle/model/node
readiness and uses exact capture acknowledgments. It adds raw-POSE TF and
scorer receipt, aimer/relay/UART queue, tracker/aim-state and opponent-path
barriers. Detector capture times have a zero-phase schedule and noise is
keyed by seed, robot, capture stamp and component. The aimer applies queued
states at a fixed 14 ms capture age; no tracker gains were changed.
See the [profile contract](../README.md#mcb-emulator) and
[CV options](../../thornbots_pkg/README.md#point_to_cv_target).

All runs below use the same hosted ELF and ten-second stationary-lateral
cell. Revisions r1–r3 are rejected development evidence for the later r4–r7
treatment, even where the native checker accepted an earlier revision.

| Attempt | Runner/checker | MCB hits | Lockstep/startup result | Total wall / RTF |
| --- | --- | --- | --- | --- |
| startup r1 | 0/0 | 21/21 | Earlier partial barriers; zero timeouts; UART peer-close during teardown | 21.58 s / 1.16 |
| startup r2 | 1/1 | none | Navigation startup: map_server/AMCL not lifecycle-active before 30 s deadline | 45.09 s / — |
| startup r3 | 1/1 | rejected | Four vision capture ACK gaps: 1.517, 2.617, 2.917, 4.717 s | 28.86 s / 0.63 |
| startup r4 | 0/0 | 21/21 | Zero timeouts/fallbacks; clock bridge exit -6 during teardown | 25.23 s / 0.79 |
| startup r5 | 0/0 | 21/21 | Zero timeouts/fallbacks | 23.09 s / 0.82 |
| startup r6 | 1/1 | none | First raw-POSE/articulated-TF ACK missing at 0.011 s; subsequent coordinator timeouts | 64.07 s / — |
| startup r7 | 0/0 | 21/21 | Zero timeouts/fallbacks | 19.09 s / 1.00 |

The r3 ledger showed a detector callback stamping 1.519 s instead of its
scheduled 1.517 s capture. Fixed capture scheduling corrected this gap.
The three accepted repeats of the final treatment pool to 63/63 hits, but
**do not pass repeatability**: zero-tolerance `compare_runs` rejects every
pair. The first shot at 3.146 s has matching combat fields, while its
localization/navigation XY and errors differ. Matched input payloads first
diverge in `/localization/odom` at 0.231 s (r4/r5 and r4/r7), or 0.331 s
(r5/r7). Earlier `/odom` arrival order differs while carried values match.
These observations identify the still-asynchronous filter path; they do not
justify tracker tuning or removal of diagnostics from comparison.

| Accepted run | Final sim / coordinator wall | Holds | Largest measured wait | Raw-POSE / aimer-UART wait |
| --- | --- | --- | --- | --- |
| r4 | 14.3 / 24.0 s | 4,292 | gz step/gate 17.69 s | 3.060 / 0.077 s |
| r5 | 14.1 / 21.0 s | 4,230 | gz step/gate 16.77 s | 3.101 / 0.085 s |
| r7 | 13.9 / 18.0 s | 4,160 | See retained coordinator log | See retained MCB log |

The final isolated build reports 427 registered checks, zero failures/errors
and 95 skips; six lockstep integration tests and the real-bridge test pass
separately. These include a future Gazebo ACK failing to satisfy an earlier
logical stamp, delayed logical ACKs, fixed state eligibility, deferred track
handoffs/invalid states and outgoing CV capture-stamp preservation. Host
quality and actionlint pass in each changed repository without baseline growth.

At this checkpoint T17 remained open: localization/scorer consumption and cell boundaries,
identical-shot comparisons, throttled/two-core gates and the board parked
sweep remain. Archlinux is excluded by the user's brief. No new combat floor
is enforced; existing floors are unchanged. Calibration remains behind the
[lockstep gates](../../E2E_LOCKSTEP_PLAN.md#order-and-done-when).

## Ordered localization and fixed scoring

The next experiment retains the installed robot_localization 3.8.3 estimator
and all configured gains/noise, while ordering captured odometry before its
30 Hz update. rf2o's optional strict scan path waits for exact scan-time TF
and bracketed raw odometry; receipts distinguish emitted odometry from a
scan with no output. The MCB's post-input watermark closes the raw-input
queue boundary. These are optional simulation scheduling changes.

| Treatment/run | MCB hits | Flag hits | Runner/checker | Wall / RTF | Largest cumulative wait |
| --- | --- | --- | --- | --- | --- |
| Ordered EKF r1 | 21/21 | 21/21 | 0/0 | 19.97 s / 1.00 | gz 13.56 s |
| Ordered EKF r2 | 21/21 | 21/21 | 0/0 | 30.09 s / 0.99 | gz 13.70 s |
| Ordered EKF r3 | 21/21 | 21/21 | 0/0 | 20.97 s / 0.96 | gz 14.21 s |
| Gazebo seeded r1 | 21/21 | 21/21 | 0/0 | 21.24 s / 0.89 | gz 15.13 s |
| Gazebo seeded r2 | 21/21 | 20/20 | 0/0 | 24.6 s / 0.84 | gz 15.92 s |
| Gazebo seeded r3 | 20/20 | 19/19 | 0/0 | 26.6 s / 0.82 | gz 15.95 s |

All six have zero timeouts/fallbacks, but none establish the full identical
record gate. Ordered EKF r3 overlapped a test-build window and is excluded
from calibration conclusions. EKF r2 needed SIGKILL for the coordinator at
teardown. Unseeded runs still differed in localization payloads: Gazebo's
lidar noise had an independent random seed. With `--seed 2026`, every shared
carried input payload matched exactly by source and capture stamp, including
raw `/odom` and `/localization/odom`. This matching was diagnostic only;
the acceptance comparator remained exact and rejected scorer timestamps,
state collection and shot-tail differences. Run ends were 14.224, 13.934
and 13.638 s. No input or output field was removed from comparison.

These six runs and associated checks are archived as
`track-a-localization-20261015.tar.gz`, SHA-256
`ce5a365fe4522aef55ebf2274fbd82ee5a42cee7a33a93ef24db8c0b306c2d3b`.
The preceding startup r1–r7 archive is `track-a-startup-20261015.tar.gz`,
SHA-256 `1a1f81f0e2e3488a0224602d9d39cebb3b922389eb6721efadccd8b2c9158e99`.
Both include per-file manifests and were independently hash/list checked.

Fixed scorer attempts use a parameter reset at 1 s, settle to 4 s, score
captures in [4,14), then drain before finalization. Scorer r1 failed AMCL
lifecycle startup before unpausing and is retained. Scorer r2–r4 all pass,
with **identical 40 shot, 600 state and 200 pose records** in every exact
pairwise comparison. Each has 20/20 MCB and 20/20 flag hits. MCB launches
span 4.146–13.646 s; flag launches span 4.144–13.644 s. Scoring ticks span
4.7–14.2 s. All three have zero timeouts/fallbacks.

| Fixed scorer run | Wall / RTF | Final holds / sim | Largest cumulative wait | Scorer wait |
| --- | --- | --- | --- | --- |
| r2 | 29.68 s / 0.64 | 4,499 / 15 s | gz 22.06 s | 0.245 s |
| r3 | 26.81 s / 0.73 | 4,499 / 15 s | gz 18.89 s | 0.223 s |
| r4 | 32.03 s / 0.88 | 4,499 / 15 s | gz 15.02 s | 0.175 s |

R4's lidar filter needed SIGKILL during teardown, retained despite native
checker acceptance. Review then corrected arbitrary-duration boundary
alignment, made the EKF prerequisite an exact-stamp set, and extended the
drain past the one-second pose-join retry. Scorer r5 passed with all queues
drained at 15.1 s and compared exactly to r2 (all three record files).
The installed adapter's two on-demand tests pass, including watermark
withholding and missed-tick rejection. Registered checks report 466 tests,
zero errors/failures and 95 skips across the five affected packages.

## Stationary-lateral Mac gates

Final scheduling sources: sim `314259a`, thornbots `8e56c1c`, bridge
`ab33c10`, localization `810a69c`, rf2o `06a2562`; workspace `c7489da`.
The hosted ELF remains the pinned firmware control. Scorer r5/r6/r9 supply
the three accepted unthrottled repeats. The accepted RTF=1 repeat and CPU2
run compare exactly to r5: **40 shots, 600 states and 200 poses** at zero
tolerance. Every accepted run has 20/20 MCB hits, 20/20 flag hits, zero
timeouts and zero latest-TF fallbacks. This closes this cell's Mac gates;
it is not a full parked-stage or combat calibration.

| Run | Wall / RTF | gz wait | MCB | Vision | Path | EKF | Scorer |
| --- | --- | --- | --- | --- | --- | --- | --- |
| r5 | 22.21 s / 0.93 | 14.64 s | .030 s | .405 s | .0247 s | .813 s | .209 s |
| r6 | 28.34 s / 0.67 | 21.09 s | .0375 s | .393 s | .134 s | .488 s | .254 s |
| r9 | 30.80 s / 0.96 | 14.80 s | .019 s | .401 s | .0209 s | .479 s | .206 s |
| RTF=1 repeat | 33.90 s / 0.76 | 18.49 s | .0192 s | .407 s | .0433 s | .481 s | .346 s |
| CPU2 | 36.75 s / 0.50 | 28.21 s | .111 s | .542 s | .297 s | .603 s | .267 s |

Waits are cumulative chain-category totals, with gz largest in every run.
CPU2 uses `taskset -c 0,1` on the entire launch, so Gazebo inherits the same
restriction as its ROS peers. The container allowed CPUs 0–9; no different
CPU pair was substituted. Live Gazebo `/proc` affinity was not sampled.
The exact event window remains [4,14), and drain ends at 15.1 s.

Rejected attempts remain part of this profile: r7 missed the 0.134 s EKF
ACK and then failed the exact-update guard after the coordinator timed out;
r8 and the first RTF=1 attempt missed the first 0.005 s MCB ACK. No wait
budget changed. Their runner/checker exits are 1/1. Successful native
checks still sometimes required SIGKILL at teardown; retained logs identify
the affected process. All owned stacks were absent after native teardown.

Fixed-scorer evidence is archived as `track-a-fixed-scorer-20261015.tar.gz`,
SHA-256 `e925f54c2e7baf804c4721700fa9b84498c9032e5c44400ffc3d051f2f3c29bf`.
Its manifest verifies 173 payload files; the host independently checked
the archive hash. Earlier unsuccessful and pre-final treatments are included.

## Default localization regression checks

The optional profile remains off in these independent native runs. Both
drift scenarios use slip 0.15 in the harness. Runner and native log checker
exit 0 for all three, with no log errors; thresholds are unchanged.

| Native check | Measurement | Wall / RTF |
| --- | --- | --- |
| scan_degraded | max before .2009 m (<.4), during .4658 m (<.5), after .2324 m (<.4) | 21.30 s / 2.34 |
| drift_correction | map-to-nav_odom change .2138 m over 30 s (<.4) | 21.82 s / 2.29 |
| EKF ground truth | 22 samples; fused mean .0434 m vs raw .1457 m, 70.2% improvement | 25.07 s / 2.40 |

The EKF check uses its unchanged defaults (slip .05, drift standard deviation
.00). Full native stdout, timing tables, generated sim/robot logs and checker
exits are retained under `default-validation/` in the isolated workspace.

## Full hosted parked sweep — rejected

The first full hosted 12-cell sweep used paused startup and ordered localization,
with 20 scored sim seconds per cell. It used ROS domain 73 and the unique
`track-a-hosted-parked-lockstep-20261015` Gazebo partition. The runner and
checker both exited 1 (R/C 1/1). Only the first three cells completed with
usable scores: stationary lateral 40/40, speed 1 lateral 32/36 and speed 2
lateral 31/36. The speed 4 lateral cell and the remaining eight cells are
invalid; they ran after the simulator failure and provide no scores.

The failure reached the outgoing relocalization path. `mcb_relay` sent
`RELOCALIZE` at log time 1792101364.881118781; the bridge logged TX #1 at
1792101364.883529169. The simulator-side UART transport then reported
sequence 3157 with capture time 78.937 s, ready time 78.938475695 s and
current sim time 78.939 s. Gazebo aborted on the late transmit. The E2E clock
stalled, and the coordinator accumulated 22 lockstep timeouts during teardown.
This confirms the outgoing relocalization transport still lacks its required
consumption barrier; the attempt is not accepted as a gate or floor sample.

The first three cells had no lockstep timeouts or latest-TF fallbacks. Tracker
logs reported zero dropped detections in the observed windows before the
crash. That does not validate the nine unfinished cells or justify tracker
tuning; no tracker tuning or floor changes were made. The runner, checker,
source manifest, outcome and raw logs remain in the container at
`/tmp/track-a-uart-20261015/hosted-parked-lockstep-r1/`.

## Rejected stationary attempts and relocalization barrier

The retained r7/r8/first-RTF=1 logs predate prerequisite instrumentation.
R7's initiating missing ACK is EKF logical tick .134 s; after the coordinator's
two-second wait it fails the exact-tick guard, then raw POSE .141 s times out.
Recorded raw odometry .131 s does not prove that the adapter's input queue,
MCB consumed watermark and scan receipt were all available. In particular,
the first .100 s scan initializes rf2o without needing an odometry bracket;
the logs do not establish a scan-prior deadlock. That prerequisite remains
unidentified in the original attempt.

R8 and the first RTF=1 attempt lack the exact .005 s MCB ACK. The first UART
packet follows timeout by 7–8 ms; this timing is a consequence of releasing
the hold and does not establish the original cause. Startup checked node names
but omitted the coordinator and endpoint matching. The new startup barrier
requires the coordinator's MCB tick subscription to match before unpausing.
New logs retain the first five MCB publications and reader counts, ACK
counts/ages, and blocked EKF watermarks, queued inputs and scan receipts.
No timeout budget or estimator gain changed.

The relocalization fix pairs each logical EKF update receipt with its filtered
capture and exact raw-input cutoff. The relay makes its decision at that
logical tick, preserving the correction's future application stamp inside a
simulator-only envelope. The bridge sends unchanged DJI bytes with decision
time and acknowledges correction or no-correction before physics advances.
MCB's consumed watermark already follows same-tick CV UART consumption, so
coincident CV/relocalization ordering is preserved.

| Development attempt | Native runner/checker | Result |
| --- | --- | --- |
| relocalize-stationary-r1 | 1/1 | Dedicated E2E relay lacked profile parameter; missing .034 s relocalization ACK and cascades; no scored output |
| relocalize-stationary-r2 | 0/0 | After launch wiring fix: 20/20 MCB, 20/20 flag; zero timeouts; 22.07 s wall; max parking displacement .0000678 m |
| relocalize-stationary-r3 | 1/1 | Overbroad readiness waited for EKF publisher at sim zero; unchanged 30 s startup timeout, no score or holds |
| relocalize-stationary-r4 | 1/1 | Subscriber-side readiness passed, but MCB published 1–5 ms with zero readers; first received tick .006 s, missing .005 s ACK; score rejected |
| relocalize-stationary-r5 | 0/0 | Both readiness signals before unpause; first five ticks each have one reader, first ACK .001 s; 20/20 MCB and 20/20 flag; zero timeouts |

The startup check now waits only for the MCB tick endpoint. The installed EKF
requires a positive clock before creating its update publisher. The two
real-bridge tests check capture/application stamp separation, unchanged payload
and no-correction ACKs; two adapter tests check carried output metadata and
exact updates. Seven coordinator tests include real publisher matching.

R4 reproduces the earlier first-tick failure mechanism with direct publisher
evidence. Coordinator readiness at wall log time 1792102371.914 preceded
unpause at 1792102372.054, yet all five initial MCB tick publications reported
zero matched readers. Subscriber-side publisher discovery is insufficient.
The next revision requires publisher-side `/mcb_emulator/tick_ready` as well
as coordinator readiness, and services the MCB executor while paused.
The initial build was interrupted when Docker status/helper calls and direct Colima
SSH banner exchange became unresponsive before this revision could build;
the reused VM/container were not restarted.

After recovery, r5 validates the complete scored window with the publisher-side
barrier: 15.1 sim seconds in 26.0 coordinator wall seconds, 4,529 holds,
RTF .69 and zero timeouts. Maximum parking displacement is .0000678 m.
The temporary .100 s EKF hold reports MCB watermark .099 s and then clears;
no .134 s failure recurs in this attempt. It is one accepted cell, not the
remaining repeatability gates or calibration. The old r7 prerequisite remains
unproved rather than assigned a speculative cause.

The host archive audit also corrected the earlier investigation's nightly
match denominator: r1/r2/r3 contain 20/36, 20/36 and 20/38 actual MCB enemy
hits, totaling **60/110 (54.5%)**. Each native run was accepted; separate flag
records are excluded. The earlier history treatment remains 60/106 (56.6%).
The three hosted drive controls remain 88/96 nightly versus 88/91 history.
These are earlier profiles, not calibration or acceptance of this revision.

The rejected full sweep, default localization regressions, development
stationary r1–r4 and their scripts/test logs are archived in
`track-a-relocalize-diagnostics-20261015.tar.gz`, SHA-256
`8e7bd0601969d6fdbf503b12bf3a0c19f5371930ff0aafeb6aa14884f964c247`.
All 94 payload hashes verified; the host independently checked the archive
hash and listing. Docker and direct SSH recovered without a VM/container
restart by this task; runtime validation resumed afterward.
