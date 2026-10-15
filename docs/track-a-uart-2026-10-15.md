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

T17 remains open: localization/scorer consumption and cell boundaries,
identical-shot comparisons, throttled/two-core gates and the board parked
sweep remain. Archlinux is excluded by the user's brief. No new combat floor
is enforced; existing floors are unchanged. Calibration remains behind the
[lockstep gates](../../E2E_LOCKSTEP_PLAN.md#order-and-done-when).
