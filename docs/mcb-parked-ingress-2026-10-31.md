# Parked ingress verification — 2026-10-31

The [parked diagnostic](../README.md#launch-sim-by-hand) now boots at the blue
team start, uses firmware SimpleAutoDrive through the modeled remote to reach
center, then parks for its original shooting cells. This report records the
verification; the README owns the fixture behavior.

## Sources and isolation

- Workspace checkout: `/Users/blaise/mcb-parked-ingress`, initialized from
  workspace `origin/nightly` at `9345ec6`.
- Stack pins: MCBV3 `e708e50`, operator `8782c52`, localization `8d470c9`,
  bridge `c823c30`, sim base `cd9a5ba`. Other packages retain workspace pins.
- Container: `isaac-agent-parked-ingress`, image `thornbots-mac`, three-CPU
  cap, ROS domain 118, private gz partition. Other mini workloads can contend
  for host resources; elapsed times are not performance measurements.
- Evidence: `/Users/blaise/mcb-parked-ingress-evidence`; supplied nightly
  comparison: `/Users/blaise/mcbz-evidence/baseline/parked-{a,b}`.
- Hosted release built with GCC 14; board release built with ARM GCC 13 using
  `scons build robot=sentry profile=release -j3`. Renode 1.17.0 was installed
  with `tools/install_renode.sh` and enabled in the sim CMake build.

## Commands

Each command ran through `tools/run_suite.sh`, including its native checker.
The hosted firmware environment selected the hosted ELF for hosted stages;
Renode used the board ELF in `MCB_FIRMWARE_BINARY`, which overrides launch defaults.

```sh
src/sim/tools/run_suite.sh mcb_parked backend:=hosted headless:=true log_dir:=/evidence/run1
src/sim/tools/run_suite.sh mcb_parked backend:=hosted headless:=true log_dir:=/evidence/run2
src/sim/tools/run_suite.sh mcb_drive backend:=hosted headless:=true log_dir:=/evidence/drive
src/sim/tools/run_suite.sh mcb_parked backend:=renode headless:=true log_dir:=/evidence/renode
```

## Hosted results

Both full 12-cell parked runs finished **7 passed, 5 failed**. The native
checker rejected both with `BENCH NOT TRUSTWORTHY` because the tests failed;
these are not overall passing suites. All speed2/speed4 cells passed. Every
failure was a stationary/speed1 zero-shot cell, consistent with the documented
[nightly chamber-exit issue](boot-heading-2026-10-30.md#nightly-parked-baseline--round-2).
Speed1 lateral fired in run 1 and speed1 diagonal fired in run 2; the issue is
intermittent. No floors, assertions, firmware, or backend models were changed.

MCB hits/shots, with the two supplied nightly runs for comparison:

| Cell | Nightly A | Nightly B | Run 1 | Run 2 |
|---|---:|---:|---:|---:|
| stationary-lateral | 0/0 | 0/0 | 0/0 | 0/0 |
| speed1-lateral | 0/0 | 0/0 | 2/2 | 0/0 |
| speed2-lateral | 4/4 | 1/1 | 1/1 | 7/8 |
| speed4-lateral | 9/15 | 16/27 | 14/14 | 15/19 |
| stationary-radial | 0/0 | 0/1 | 0/0 | 0/0 |
| speed1-radial | 0/0 | 0/0 | 0/0 | 0/0 |
| speed2-radial | 6/6 | 4/4 | 5/5 | 13/13 |
| speed4-radial | 6/6 | 10/10 | 12/12 | 11/11 |
| stationary-diagonal | 0/0 | 0/0 | 0/0 | 0/0 |
| speed1-diagonal | 0/0 | 0/0 | 0/0 | 1/1 |
| speed2-diagonal | 3/4 | 2/3 | 5/7 | 2/2 |
| speed4-diagonal | 8/12 | 8/13 | 4/7 | 5/6 |

- Ingress settled at world xy `(-0.03648, 0.51584)` and
  `(-0.03280, 0.52086)`, after 11.549 and 11.760 sim seconds from arm through
  the three-second parked settle. Both selected firmware `simple` and then
  `stop` through the emulator's modeled remote; no velocity publisher drove
  the sentry. The complete traces are `run{1,2}/ingress.jsonl`.
- Shot range medians: 1.178 / 1.171 m; spans 0.662–2.071 / 0.660–2.074 m.
  Nightly A/B: 1.217 / 1.135 m, spans 0.656–2.068 / 0.660–2.063 m.
  These use every shot record's `range_m`, including fire-decision records.
- Fire-decision hit rates: 0.9724 / 0.9739, versus nightly 0.9661 / 0.9604.
- All 24 localization/parking assertions passed: stamped localization coverage
  100%, maximum localization error below 0.059 m, maximum within-cell chassis
  displacement 0.01264 / 0.02891 m. Both runs had zero lockstep timeouts.
- Final `mcb_drive backend:=hosted`: 1 passed in 38.69 wall seconds, native
  checker `bench ok`.

Preliminary attempts are retained separately. Two reached center but could
not acquire the restored south-side target while idle look still faced -x.
The first retry with the corrected idle zone stopped at the first waypoint
after an approximately 0.08 m AMCL correction during that leg. The final
fixture gives localization three stationary sim seconds before arm; both
subsequent full runs reached center. This does not establish general route
repeatability or fix firmware waypoint recovery.

## Board attempt and remaining limits

The full Renode attempt selected the normal hardware release ELF through
`MCB_FIRMWARE_BINARY`. The first attempt hit the adapter's control-server
startup timeout while Renode downloaded `STM32F40x.svd.gz`. A standalone
Renode `sysbus ApplySVD` invocation warmed its normal cache; no backend code
or peripheral model was changed.

The retry completed its power-on guard in 11.68 wall seconds and calibrated
with 16,004 CAN frames and board start `(0,0)`. Gazebo reached about 1.1 sim
seconds with 217 lockstep holds and zero timeouts. The production EKF seed
waiter then failed its existing wall-time limit:

```text
No referee team and subsequent fresh raw /odom within 15 s; EKF was not started
```

That exit shut down the stack before localization or ingress. All 12 tests
failed in bring-up (108.64 wall seconds); the native checker reported
`BENCH NOT TRUSTWORTHY`. **Board ingress and parked shooting remain unverified.**
This is not a board accuracy result, and neither the production startup
deadline nor the Renode backend was changed. Board artifacts and emulator
logs were archived under `renode-board/` in the evidence directory.

Host Python 3.12 repository quality and `actionlint -shellcheck=""` passed with
zero new diagnostics. The task container and its volumes were removed after
archiving; no other containers remained, so the helper also stopped Colima.
No shared workspace checkout or PR body was edited.
