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

T17 remains open: complete consumption barriers, fixed-time bring-up, keyed
seeds, identical-shot comparisons and the board parked sweep remain. No new
combat floor is enforced; existing floors are unchanged. Calibration remains
behind the [lockstep gates](../../E2E_LOCKSTEP_PLAN.md#order-and-done-when).
