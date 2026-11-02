# Hosted-firmware match stages, 2026-10-11

Linux x86_64, existing Jazzy container, T32 + T33 package merge commits
(sim `341477c`, thornbots `0b15f48`, bridge `bb19633`, MCBV3 `f01fda7`).
Fresh isolated ROS build and a fresh GCC 14 hosted sentry ELF; the stack log
confirms that ELF, `auto_fire=True` and zero UART CRC errors. All three stages
used their full default cells/durations, headless, native MCB batch lockstep.
The tests launch truth detections and the real tracker/aiming/relay/bridge;
MCB parsing and control execute compiled firmware. Driving remains the
`match_driver` model, rather than firmware auto-drive.

```bash
export MCB_FIRMWARE_BINARY=/path/to/fresh/MCB-project.elf
for stage in mcb_parked mcb_drive mcb_match; do
  ros2 launch sim e2e.launch.py stage:=$stage headless:=true log_dir:=/path/to/$stage
  ros2 run sim check_bench_log /path/to/$stage/{launch,stack}.log
done
```

Actual commands source the isolated `test-mcb-2026-10-11/install` overlay;
explicit source/build/test commands and the self-cleaning sequential runner
are retained alongside the raw artifacts. This is one run per stage and
does not establish T17 repeatability or calibrated combat floors.

## Results and log checker

| Stage | Test result | Process exit | Native checker |
| --- | --- | ---: | --- |
| mcb_parked | 8 passed, 4 failed in 254.96s | 1 | rejected |
| mcb_drive | 1 passed in 34.07s | 0 | accepted |
| mcb_match | 1 passed in 47.14s | 0 | accepted |

## Parked cells

| Path | Speed m/s | Native hits/shots | Hit % | 10% floor |
| --- | ---: | ---: | ---: | --- |
| lateral | 0 | 41/41 | 100.00 | pass |
| lateral | 1 | 2/29 | 6.90 | fail |
| lateral | 2 | 0/41 | 0.00 | fail |
| lateral | 4 | 0/37 | 0.00 | fail |
| radial | 0 | 41/41 | 100.00 | pass |
| radial | 1 | 6/41 | 14.63 | pass |
| radial | 2 | 10/40 | 25.00 | pass |
| radial | 4 | 10/39 | 25.64 | pass |
| diagonal | 0 | 41/41 | 100.00 | pass |
| diagonal | 1 | 9/37 | 24.32 | pass |
| diagonal | 2 | 16/41 | 39.02 | pass |
| diagonal | 4 | 1/30 | 3.33 | fail |

Eight of twelve pass, versus T17's last recorded Mac runs of eight and seven
of twelve (workspace `a65e025`); the older `e834ba2` result was nine of twelve.
All stationary cells now hit 100%, compared with the recorded 72–100% range
and the older stationary-lateral 13% result. Moving cells span 0–39%, versus
0–30% in the old note. Specifically, speed1-radial is 14.63% versus 0%/30%;
speed2-lateral is 0% versus 20%/5%. Those earlier runs differ in architecture
and pins, so these comparisons do not attribute causality to T32 or T33.
The checker rejects the four failed floor cells and the resulting launch
failure; these are scored failures rather than lockstep timeouts.

## Driving segments

| Segment | Shots | Hit % | Localization p95 m | Route p95 m |
| --- | ---: | ---: | ---: | ---: |
| approach_2 | 12 | 0.00 | 0.168 | 0.149 |
| parked | 10 | 0.00 | 0.223 | 0.146 |
| straight_1 | 5 | 0.00 | 0.218 | 0.030 |
| turn | 9 | 0.00 | 0.562 | 0.028 |
| spin | 12 | 0.00 | 0.483 | 0.010 |

All five segments still hit 0%, as in the recorded 2026-10-08 drive note.
Each diagnoses median aim error over 0.10 m. Localization p95 is now
0.168–0.562 m, versus the recorded 0.46–0.91 m range. The diagnostic test and
checker accept the run despite zero hits; combat-accuracy acceptance remains
open and no floors were weakened.

## Four-robot match

| Segment | Native shots | Enemy hit % | Localization p95 m | Route p95 m |
| --- | ---: | ---: | ---: | ---: |
| approach_2 | 0 | 0.00 | 0.183 | 0.121 |
| parked | 10 | 0.00 | 0.384 | 0.000 |
| straight_1 | 5 | 0.00 | 0.822 | 0.031 |
| turn | 9 | 0.00 | 1.847 | 0.028 |
| spin | 12 | 0.00 | 2.116 | 0.009 |

All combat segments diagnose median aim error over 0.10 m. Approach firing
remains disabled until all robots reach center. Native firmware fired
36 shots with zero enemy hits and
0 friendly intersections; the prior recorded x86 run
caught allied intersections. Ghost shot counts: ally 37,
opponent 0 37, opponent 1 37.
Damage taken: 0; final HP: {'ally_0': 400, 'opponent_0': 400, 'opponent_1': 380, 'sentry': 400}.
The diagnostic test/checker accept the run; this does not establish combat
accuracy.

Zero lockstep timeouts were reported in every stage. All owned launch groups
exited, and the final process check found no stacks, installed nodes or
hosted firmware children left running. No pitch-bridge crash or temporary-world
cleanup exception recurred during these three shutdowns.

## Firmware coverage and limits

The [patrol-marker verification](patrol-flag-2026-10-11.md) exercises all
flag combinations through compiled firmware and the real bridge: aim parsing,
fire/stage gating, expired-target patrol and damage-turn permission. The
[aim hold and indexer timing tests](panel-choice-2026-10-11.md#compiled-firmware-checks)
also passed against this ELF. Nonzero-yaw damage reproduces the known T31
angle scaling discrepancy with bit 3 clear/set. The E2E scorer's inline
`t + .05` models flight start 50 ms after an indexer request; physical exit
and MCU/CAN dynamics remain unverified.

Raw launch/stack logs, shot/state/pose records, scores and checker verdicts:
`/home/blaise/workspaces/isaac_ros-jazzy/test-mcb-2026-10-11/` on this box.
