# Parked E2E epoch-zero startup, 2026-11-04

A Mac mini agent container built the checked-out hosted firmware into a fresh
isolated `/tmp` tree and ran `test/test_e2e_epoch_zero.py` against the real
`e2e.launch.py`. The observer subscribed before launch. Both the normal order
and a six-second delay before starting the coordinator produced these first
stamps (milliseconds of simulation time):

| Output | First | Second |
| --- | ---: | ---: |
| Truth-backed detector capture | 17 | 34 |
| Aim timer | 25 | 50 |
| Firmware tick | 1 | 2 |

All five latched readiness headers had stamp zero. The coordinator reported
that the epoch-zero gate was held and all five were ready before releasing its
first 5 ms batch. A separate launch added a deliberately absent readiness
subscriber requirement and a 10 s deadline: it exited nonzero, observed only
zero ROS clocks, and observed no firmware ticks. The launch rejected scored,
driving and Renode uses of this option. The launch also logged intermittent
expired CV target transmissions after startup; this is not a scored result.

One repeated test using the same Gazebo partition for consecutive launches
failed closed when the delayed coordinator received a positive native clock
before its barrier. The clock's source was not identified. The test now gives
each launch a separate partition to isolate transport discovery.

`lockstep_suite` passed 18 tests, `test_detector_standin` passed five, the
actual UART epoch-zero test passed five cases, and the UART plugin test passed
seven success/fault cases. The first successful stack's lockstep report showed
zero timeouts. These results establish timer initialization and the bounded
scene/bootstrap barrier, not deterministic downstream message consumption.
Selector, relay, translator and TF first-data gates, repeated scored outputs,
and integration for driving and match remain open in
[the E2E plan](../../docs/E2E_PLAN.md).
