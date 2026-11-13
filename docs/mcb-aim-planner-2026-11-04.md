# MCB aim planner in the no-MCB benches, 2026-11-04

Historical evidence; live actions and acceptance are maintained in
[ROADMAP](../../docs/ROADMAP.md) and [ROBOT_TESTS](../../docs/ROBOT_TESTS.md).
The maintained design is [MCB aim stand-in](../README.md#mcb-aim-stand-in).

Sentry aiming moved from the Jetson's `point_to_cv_target` to the MCB
([TARGET_STATE](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#target_state-id7--tracked-enemy-armor-model),
[MCBV3 aiming planner](../../firmware/MCBV3/README.md#mcbv3-aiming-planner)). The
aiming and estimation benches run no MCB, so `mcb_aim_standin` runs the
firmware's own `AimPlanner.cpp`, compiled from the checkout, on the same
`TargetState`. Nothing was copied.

## Setup

Mac mini (arm64), `thornbots-mac` agent container, 6 CPUs, `feat/mcb-aim` in
sim, `ros2_dji_serial_bridge`, `thornbots_pkg` and MCBV3 (hosted ELF rebuilt
with GCC 14 from the checked-out sources). Every bench run below was accepted
by `check_bench_log` (`tools/run_suite.sh`), all at `real_time_factor:=0`
(about 54x for the aiming bench, 23x for estimation).

The stand-in keeps the old bench configuration so the numbers compare:
`gimbal_lag_s:=0` (the bench's gimbal is perfect; the firmware constant is
0.05 s) and `chase_settle_s:=0` (chase the facing panel; the firmware shotgates).
Everything else is `AimTuning`'s defaults, equal to the firmware's sentry
constants.

## Aiming bench

Recorded before this change ([panel choice, 2026-10-11](panel-choice-2026-10-11.md#aiming-bench)):
the unmodified ten-cell bench scored hit rates (hits per fired shot) of
97.17-99.25%. The per-cell score floors in `shot_floors_data.hpp` were the
lowest of three runs minus 10 points, 0.839-0.880.

Now, ten default cells, one run (chase, perfect gimbal):

| Cell | Score | Shots fired / 1200 | Hits per fired |
| --- | ---: | ---: | ---: |
| flat stationary | 99.3% | 1201 | 99.3% |
| flat 0.5 m/s | 74.0% | 600 | 98.7% |
| flat 1 m/s | 72.6% | 600 | 96.8% |
| flat 2 m/s | 72.0% | 600 | 96.0% |
| flat 4 m/s | 72.1% | 600 | 96.2% |
| staggered stationary | 99.2% | 1200 | 99.2% |
| staggered 0.5 m/s | 73.9% | 600 | 98.5% |
| staggered 1 m/s | 73.5% | 600 | 98.0% |
| staggered 2 m/s | 72.4% | 600 | 96.5% |
| staggered 4 m/s | 72.4% | 600 | 96.5% |

Hit rates are 96.0-99.3% against 97.17-99.25% before, and three repeats of
all 40 floor cells (12 launches) span 94.7-99.7% per cell. The pre-change
runs for the same ten cells are not retained, so only the range is compared;
spinning cells are about 1 point lower at the fast end, within the 3-run spread
of 0.5-1.5 points.

**Scores fall for moving cells, and this is real.** The bench fires up to 40 Hz
and scores keep-up (fired / 40 Hz). The Jetson planner could request every
tick. The firmware serializes: a pending request holds its aim and blocks the
next replan, so chase mode requests at most every second 25 ms period (600 of
1200), where the real indexer's 50 ms minimum caps it at 20 Hz anyway.
Stationary cells still fire every tick (their request delay is under one
period). Stationary scores also rose, floors 0.86 to 0.89 and runs to 0.99. Likely
cause: the old scorer exited a shot 50 ms after the wire delay
(`kFireLatency`) while the planner modeled 40 ms; the scorer now takes the
stand-in's request plus the firmware's 20 ms request-to-exit. `kFireLatency` is
removed. This attribution was not isolated.

The 40 `FLOORS` were re-measured with `ros2 run sim shot_floors` over the 12
launches (lowest score minus 10 points): moving cells 0.610-0.645, stationary
0.891-0.893. This lowers the moving floors from about 0.85 because the cap
above is a property of the new architecture, not a hidden regression; the hit
rate per fired shot did not move materially. Repeatability: the 3 runs of a
cell differ by at most 0.02 in score.

Firmware spin mode (`chase_settle_s:=-1`, shotgating), one ten-cell run, same
perfect gimbal: stationary 99.2-99.3%; fired 241, 218, 183, 120 shots at
0.5-4 m/s with hits per fired 98.8%, 97.7%, 96.7% and 82.5% (flat) or 99.6%,
97.3%, 93.9% and 80.0% (staggered). The bench default stays chase so the
recorded comparisons hold; shotgating's 10-20% keep-up scores below any
floor, and the bench scores keep-up against 40 Hz, so a firmware-spin
scoring rule is open.

## Estimation bench

`estimation.launch.py` (12 cells, `mcb_aim_standin` aim-only, nothing fires),
three runs: 12 passed each, zero lockstep timeouts, 23.0-23.8x. `compare_runs`
accepts run pairs at its default tolerance (the runs are not byte-identical:
`estimation.jsonl` differs from line 4, as the repeatability notes warn).
No tracker limit was changed. The head's own lag is real in this bench, so
the stand-in keeps the firmware's 0.05 s `gimbal_lag_s` there.

## Unit and protocol tests

Registered tests (`colcon test`): 236 result entries, 4 failure entries from two
tests, none from this change:
`sim_copyright` (`test_detector_standin.cpp`, `test_opponent_startup.cpp`,
`test_e2e_epoch_zero.py` lack a license the lint recognizes) and
`sim_flake8` (`test/test_uart_epoch_zero.py` import order), both present on
`nightly`. New: `test_mcb_aim` (cadence, fire lock, timeout, idle fallback,
constants), a 13-byte `CV_TARGET`, an 80-byte `TARGET_STATE`, and
firmware tests for state aim, fire, confidence, detection-age and priority.
`test_mcb_firmware` (48) and `test_mcb_bridge_pty` (3) pass against the
rebuilt hosted ELF through the real bridge.

## E2E parked, hosted fixture

`mcb_parked backend:=hosted`, firmware planning for itself:

- The stage could not start: the idle look faced the enemy start (-x) while the
  stationary opponent sits south of the parked sentry, so no `TargetState`
  ever arrived. This is not from the wire change: `a39dd2d` (2026-10-31)
  removed the parked zone override. `e2e.launch.py` now moves red's start for
  `mcb_parked` only, restoring the south look.
- Ingress to center is intermittent, as the
  [2026-10-31 report](mcb-parked-ingress-2026-10-31.md) records: it stopped
  short in 2 of 7 launches.
- Cells then ran. speed1 lateral: 0 shots in 15 s (the known zero-shot
  intermittency); speed2 lateral: 7/16 hits (15 s), 0/2 (30 s); speed4 lateral:
  2/28 (30 s). Localization gates passed (p95 under 0.07 m). Against the
  2026-10-31 hosted runs (speed2 4/4, 7/8; speed4 9/15 to 15/19) the hit rates
  are lower and the 30 s run fails the cell's 10% floor.

**Likely cause, not verified:** the hosted fixture was calibrated with zero
residual yaw lag and `chase_margin_s` 0.025, but the firmware now carries
`gimbal_lag_s` 0.05 and margin 0 (the board values). A firmware knob or a
hosted-fixture constant is needed to test it; it was not changed here.
The Renode board stage was not run (see the report).
