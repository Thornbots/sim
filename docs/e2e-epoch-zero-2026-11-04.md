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

## Driving and match, 2026-11-04 (evening)

`test/test_e2e_epoch_zero.py` now runs every stage in both startup orders,
plus a per-stage missing-readiness launch. Driving and match spawn each ghost
at the position and yaw `match_scenario` samples at t=0 (via `e2e_suite
--print-launch-constants`, so launch and scenario share one source) and the match adds
`opponent_1` and `ally_0` readiness. The observer joins the barrier through
`epoch_readiness_extra` and, while holding it, asserts that every observed
clock is zero and no firmware tick has been published. First stamps:

| Stage | Detector | Aim | Firmware | Path |
| --- | --- | --- | --- | --- |
| `mcb_parked` | 17, 34 | 25, 50 | 1, 2 | 17, 34 |
| `mcb_drive` | 17, 34 | 25, 50 | 1, 2 | 10, 20 |
| `mcb_match` | 17, 34 | 25, 50 | 1, 2 | 10, 20 |

Each missing-readiness launch exited nonzero with only zero clocks and no
firmware tick. `lockstep_suite` passed 18 tests; `test_detector_standin` and
`test_opponent_startup` (now with yaw and wrong-yaw cases) passed 16 gtest
results; the UART epoch-zero (5) and plugin (7) scripts passed.

**Gazebo exit 134, explained.** The intermittent abort was
`McbFirmwareSystem::PreUpdate` throwing "UART epoch zero missed": lockstep
released before the new plugin had booted. The coordinator had received
`/mcb_emulator/epoch_zero_ready` about 3 s before this launch's plugin
configured; the only publisher of that topic is the plugin, so the latched
header came from the previous launch's gz server, which outlived the test's
process-group teardown on the same ROS domain. A probe reproduced a gz server
surviving that teardown. The test now waits until no process carries the
launch's unique `GZ_PARTITION`; with it, the parked missing-readiness to
driving transition that had aborted passed in both passes that reached it.
This also likely explains the earlier unidentified positive clock. The coordinator itself cannot tell a stale
latched readiness from a fresh one; launches sharing a ROS domain must not
overlap.

**Aim phase is not yet repeatable.** Of four full-matrix passes after the
teardown fix, two matched the table in every run; the other two stopped at a
normal-order parked run whose aim ticks were 25, 51 ms and 26, 50 ms. Detector,
firmware and path stamps never moved. The aim stamp is
`now()` in its timer callback, and lockstep holds physics only for MCB acks,
so a late aim executor sees a later clock. Making aim a lockstep chain is part
of the open downstream consumption gates; scored runs stay gated.
