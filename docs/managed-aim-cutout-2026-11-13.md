# Managed aim spin cutout, 2026-11-13

The [firmware planner](../../firmware/MCBV3/README.md#target-handoff-redesign)
uses fixed crossing-line aim above an absolute model yaw rate of π/2 rad/s.
At or below the threshold it retains panel chase. The stand-in compiles that
same planner and control lifecycle; there is no second implementation to drift.
The explicit `spin_mode:=chase` comparison override disables the cutout.
No controller gains, panel outlines, scoring floors or interlocks changed.

The crossing-line solve retains translation, acceleration, shooter motion,
ballistics and analytic rates. The selected panel supplies radius and height;
rotation no longer drags the aim around the target. A fire request still
requires both the measured-motion impact to lie inside the physical panel and
the existing face-crossing window to open in the current control tick.

## Fixture and reproduction

Sources start from workspace `12507703`, MCBV3 `67504bc` (#141, containing
nightly `f53d7fd`) and sim `f3462f0` (#106 stacked on #107). Manager `15e5647`
and bridge `6232e4f` stay at the coordinated workspace pins. Changes are the
cutout and shared stand-in coverage, not a tracker or plant retune.
The measured firmware change is signed commit `c3313b3`.

The task-owned four-CPU container runs detector panels from Gazebo truth through
the selector, tracker, target manager, actual UART bridge and GCC 14 hosted
firmware. No perfect TargetState bypass, physical robot or Type C execution is
used. The fixture retains ingress, 1 ms physics, three seconds settling and
20 seconds nominal scoring per cell. Both complete campaigns include all twelve
speed/path cells and the disappearance cell.

```sh
tools/run_suite.sh mcb_parked backend:=hosted headless:=true log_dir:=/evidence/repeat-1
tools/run_suite.sh mcb_parked backend:=hosted headless:=true log_dir:=/evidence/repeat-2
```

Raw evidence is under
`/Users/blaise/ros2_ws/evidence/managed-aim-cutout-2026-11-13/`:
per-repeat scores, shots, states, parking, ingress and stack logs, launch/checker
output, build/test logs, ARM ELFs and source patches.

The nightly comparison is the single run at
`/Users/blaise/ros2_ws/evidence/mcb-parked-ingress-2026-11-12/runs/nightly-full12/`.
The redesign chase comparison is the single
[managed parked run](managed-aim-2026-11-12.md#managed-parked-run).
These are historical comparisons, not interleaved matched trials or new floors.

## Verification

- GCC 14 firmware unit tests: 78/78, including strict threshold, both directions,
  same-track transitions, fixed-aim rates and independent impact/window vetoes.
- GCC 14 hosted firmware and regression tests: build passes, 49/49 tests,
  plus three hosted handshake checks.
- Compiled firmware UART/protocol tests: 64/64.
- ARM GCC 13.2.1 release builds: infantry, hero and sentry, `sysid=none`, all pass.
- Stand-in tests: 10/10, including firmware parity through threshold refreshes
  and stationary firing with the explicit shot-gating override.
- Registered sim tests: 275 result records, zero errors, failures or skips.
  The standalone stand-in suite also passes under GCC 14.
- Workspace, firmware and sim quality checks and actionlint: no new diagnostics.

The first registered run caught a stale planner object compiled before the last
stationary-override fix during the initial ROS build. Rebuilding sim and rerunning
the full registered suite passed; campaigns use that rebuilt install.

## Results

Both campaigns pass **13/13 cells**, each with native `bench ok`, zero lockstep
timeouts and zero latest-TF fallbacks. Native elapsed times are 425.18 s and
353.11 s. Each entry below is **shots / hit % / hits**. Hits are counted from
physical-shot records and checked against score summaries; the historical
redesign counts follow its reported shots and rates. Percentages use exact
counts for these runs and nightly; the supplied redesign percentages are retained.

| Cell | Nightly | Redesign chase | Cutout 1 | Cutout 2 |
| --- | ---: | ---: | ---: | ---: |
| stationary-lateral | 258 / 97.7 / 252 | 259 / 97.3 / 252 | 257 / 98.4 / 253 | 257 / 97.7 / 251 |
| stationary-radial | 209 / 100.0 / 209 | 270 / 98.9 / 267 | 282 / 99.3 / 280 | 282 / 98.6 / 278 |
| stationary-diagonal | 205 / 100.0 / 205 | 250 / 100.0 / 250 | 279 / 100.0 / 279 | 280 / 100.0 / 280 |
| speed1-lateral | 164 / 39.0 / 64 | 18 / 55.6 / 10 | 164 / 43.3 / 71 | 155 / 47.7 / 74 |
| speed1-radial | 164 / 19.5 / 32 | 7 / 85.7 / 6 | 148 / 81.8 / 121 | 149 / 77.9 / 116 |
| speed1-diagonal | 164 / 29.9 / 49 | 16 / 56.3 / 9 | 159 / 49.1 / 78 | 155 / 48.4 / 75 |
| speed2-lateral | 202 / 21.8 / 44 | 71 / 45.1 / 32 | 128 / 50.0 / 64 | 133 / 51.1 / 68 |
| speed2-radial | 211 / 20.4 / 43 | 55 / 72.7 / 40 | 132 / 86.4 / 114 | 121 / 87.6 / 106 |
| speed2-diagonal | 221 / 19.5 / 43 | 72 / 70.8 / 51 | 136 / 38.2 / 52 | 135 / 43.0 / 58 |
| speed4-lateral | 232 / 35.3 / 82 | 202 / 25.3 / 51 | 79 / 30.4 / 24 | 82 / 30.5 / 25 |
| speed4-radial | 234 / 25.2 / 59 | 232 / 73.3 / 170 | 80 / 87.5 / 70 | 76 / 88.2 / 67 |
| speed4-diagonal | 225 / 30.2 / 68 | 197 / 42.1 / 83 | 76 / 25.0 / 19 | 79 / 25.3 / 20 |
| disappears-lateral | — | 194 / 99.0 / 192 | 201 / 100.0 / 201 | 200 / 99.5 / 199 |

π/2 rad/s is **0.25 Hz**, so every spinning cell (2, 1.667 and 1 Hz) uses
gating. At 2 Hz, pooled hits rise from chase's 25 to **270 and 265**, with
471/459 shots instead of 41. Pooled hit fractions are 57.3/57.7%, versus 61.0%
for the very few chase shots. The earlier 78–89% shot-gating result does not
reproduce in the lateral or diagonal cells of this fixture.

The 1 Hz cells pay a substantial throughput cost: pooled shots fall from 631
to **235/237**, and hits from 304 to **113/112** (−62.8/−63.2%). Pooled hit
fraction stays near chase: 48.1/47.3% versus 48.2%. Radial accuracy improves,
but its hit count still falls from 170 to 70/67. The requested cutout therefore
does not improve every spinning cell. Across the twelve ordinary cells, hits
are **1,425/1,418**, versus chase's 1,221 and nightly's 1,150.

Disappearance produces **3/5 physical exits after hide**, versus six in the
prior redesign run; nightly has no disappearance comparison. These counts are
physical chamber exits, not fire requests. Final exits are 222/237 ms after
hide, versus 257 ms in the prior redesign run.

No mode hysteresis or dwell was added. Valid model-state logs show no sustained
chatter near the cutout. Both runs have one brief near-zero rate excursion in
the 2 Hz radial startup, then return well above the threshold; these are not
repeated near-threshold crossings. Other cells have zero within-cell threshold
transitions in the valid model samples. The reconstruction uses the logged
absolute rate error plus the cell's constant truth spin, not a new firmware
instrumentation stream.

## Cleanup

Post-campaign process inspection found no Gazebo, ROS launch or installed-node
processes. Removed `isaac-agent-aim-cutout` and its task volumes. No other
container was touched. The helper found no running containers and **stopped
Colima at 20:05 UTC**, before the scheduled 20:25 UTC nightly wait. Colima was
not left running. No physical robot, deployment or PR merge was performed.
