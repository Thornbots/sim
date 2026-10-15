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
