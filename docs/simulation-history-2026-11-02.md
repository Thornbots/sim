# Simulation experiment history, collected 2026-11-02

Historical evidence migrated from [sim README at `5ffcf189`](https://github.com/Thornbots/sim/blob/5ffcf189a86e42fe52a069de3b1f0ae13119e605/README.md).
Dates within each observation are the experiment dates; collection is not a new run.
Original verdicts, frame names and uncertain claims are retained, not current validation.
Current usage/design belongs in the [README](../README.md#notes). Live software
commitments and physical acceptance belong in [ROADMAP](../../docs/ROADMAP.md)
and [ROBOT_TESTS](../../docs/ROBOT_TESTS.md), respectively. Suggestions below
are historical context, not an additional action list.

## SAPIEN trial

A SAPIEN engine (`sim_engine:=sapien`) was built as a faster stand-in for gz
and removed on 2026-09-24 before it ever ran a full suite. We are staying on
gz. The removed `sim/sapien_sim.py` is in the commit log if the question
comes back.

## Mapping backend measurements

`mapping --use-rf2o` passed 9/9
(2026-10-02, archlinux, unthrottled):
cornering loops 0.02-0.09 m truth error, `noise_correction` 0.04 m,
`scan_degraded` 0.43 m during and 0.07 m after. `--no-use-rf2o` fails 5/9
(Mac): the three 4 m/s loops at 0.41-0.42 m, `scan_degraded` 0.42 m after,
and `odom_stuck`, whose `map->odom` freezes as amcl's does without rf2o
(below), mapping laps or not. Under rf2o, `odom_stuck` passes its liveness
check while lost (1-5 m truth error), and mapping keeps writing to the map
the whole time.

## No-map and AMCL slip measurements

Under `none`, `drift_correction`, `drift_correction_obstacle` and
`moving_obstacles` measured 0.022, 0.037 and 0.020 m, respectively, with
`noise_correction` at 0.040 m (2026-09-28, `--use-rf2o`, unthrottled).
`scan_degraded` fails under `none` (0.63 m during, 0.64 m after): with no
map layer nothing pulls the error back once the scan recovers.

amcl with and without EKF under slip, measured 2026-07-26 against a 0.30m
bound; verdicts shown against the source README's 0.40m `kMaxDelta`:

| `odom_slip_ratio` | `amcl` | `amcl` + `use_rf2o:=true` |
| --- | --- | --- |
| 0.0 | 0.1478 m (PASS) | 0.2043 m (PASS) |
| 0.25 | 0.4033 m (**FAIL**) | 0.1642 m (PASS) |

At zero slip `/odom` is near perfect and fusing rf2o's noise only hurts.
Under slip the EKF turns a fail into a pass. That was the first sign it helps
a backend that owns a map. 0.25 is harsher than the defaults (0.02, or 0.15 in drift
scenarios). `slam --use-rf2o` measured worse than plain `slam`; see
[localization tuning history](../../sentry_localization/README.md#outcome-tuned-slam-no-ekf).

## Frozen odometry measurements

Measured 2026-07-27: `amcl` fails, stuck at 0.0000m for 30s, because the
scan-match gate runs on odom-reported travel and frozen odom never reopens
it. The stack really does depend on odometry to stay live. `amcl --use-rf2o`
passes at 1.3071m, because the EKF keeps reporting travel. Passing isn't
tracking: 2026-09-25 it passed at 1.35m while `odom->root` stayed inside
about 1m and ground-truth error cycled 0.2-3.9m per lap. rf2o seeds each
match from `/odom`, so a frozen `/odom` pins it too, and at 4 m/s the 10 Hz
lidar moves 0.4m between scans, too far for rf2o to match without a good
seed. Matching from rf2o's own last motion as well didn't help, since that
motion was already near zero. amcl has no motion to spread its particles
and turns its estimate (up to 0.66 rad) to fit the scan. Losing the robot
here is a known limit of this sensor set, not a defect; the scenario stays
a liveness check. The gz robot itself doesn't turn: over a 120 s run
(2026-09-28) every `/sim/raw_odom` message held true yaw within
-0.15..0.00 deg and the head within -0.28..0.07 deg, while `map->odom`
yaw sat at -0.3 to -0.58 rad. A turning robot in rviz is amcl's
estimate. The scenario logs both ranges at the end.

## Scan blackout measurements

Across four `scan_degraded` runs against a 0.4 m bound
(2026-09-27) the blackout peaked at 0.398-0.432 m, failing three against
0.4 m, while before and after stayed under 0.154 m.

## Parked push and removed checks

2026-10-25: truth moved 0.041 m; the estimate fell short by
0.004-0.006 m with references (9 runs) and 0.003-0.011 m without (13).
rf2o blends each match with the still wheel prior either way.

`noise_correction` used a 60 s window before 2026-07-27; the source README
records a fixed 30 s window afterward.

Two checks were removed. `jerk_stationary` (2026-07-23) re-verified a documented limit
of the travel gate instead of testing recovery. A no-leak-before-motion check
in `jerk_with_motion` (2026-07-26) failed for reasons unrelated to the
correction.

## Lidar self-visibility trial

Robot-visual exclusion from lidar rendering was tried in July 2026, reverted as "all-or-nothing per visual", and is
back because all-or-nothing turned out to be the right answer. Measured
2026-09-23 on the old model, with the exclusion off: the head blanked 118-180 deg of
`/scan_raw`, and a further rear sector out to -45 deg came and went with the
head's pose -- between 863 and 1613 of 3000 beams, reported as `-inf` because
the self-hits land inside `range_min`. `lidar_self_filter` then blanked 1.0
rad (126-183 deg), so up to 140 deg of the scan was being lost to something
nothing in the stack knew about. A bare `gpu_lidar` at the same height in the
same world returned all 3000 beams at 2.22-7.17 m; the exclusion restored
that coverage on the sentry too. Both models now use the measured
angle-compensated A2M8 grid [below](../README.md#lidar-format-and-model-limits).

## Camera transform sign trial

The old model's `headpitch` carried a fixed -0.38885 rad yaw, and getting its
sign wrong cost a debugging cycle: mean `pos_err` was ~2.45m with +0.38885 and
~0.13m with -0.38885. Compare vectors rather than magnitudes, as with rf2o's
`angle_min` bug, because `tan(+x)` and `tan(-x)` have the same magnitude.

## Muzzle-origin trial

It aims along the ray from the muzzle. Before 2026-09-09 it aimed from root,
assuming a ~0.35m offset wouldn't matter at range. That's true for flight time,
but a parallel offset in direction stays the same size at any range. On the
old model every stationary shot missed by ~0.33m against a 0.05m hit radius.

## Firmware solve suspicion

Type-C probably has the same bug. It receives a world-frame position and runs its
own gimbal solve, and only the firmware knows where the real barrel sits, so
raise it with the firmware team (see [firmware coordination](../../ros2_dji_serial_bridge/README.md#where-the-firmware-stands)).

## Head controller tuning

At the old gain 0.3 and 15Hz the head trailed a 0.5 m/s target by 9.4cm against a 5cm hit
radius (2026-09-17), a lag no 1kHz gimbal loop would have. The old `sign_yaw`/`sign_pitch` params are gone, because the IK geometry
sets the sign and the test checks it analytically.

## Depth transport measurements

Over DDS a
640x480 frame is past Fast DDS's 512 KB shared-memory segment, and most frames dropped: 14-18 of 60
Hz arrived (2026-09-29). Depth costs sim speed: the Mac's bare
`sim.launch.py` runs at RTF 2.66 without it and 1.1 with it (llvmpipe).

## Previous saved-map alignment

Historical runs with the previous saved maps
showed a steady ~0.1-0.15m map-to-world offset (measured before both were
turned into the field frame, the same turn for each). That measurement
does not establish alignment for the current [saved map](../../sentry_localization/README.md#saved-map).

## Loop geometry trials

The square was widened from
2m on 2026-07-26 (`4f182e7`). Wall clearances are known on x only: +x clears
`upper_mid` (x=2.49) by 0.99m, -x clears `lower_mid` (x=-2.11) by 0.61m and
`bottom_wall`'s ramp edge (x=-3.35) by 1.85m. `lower_mid` is the tightest, so
start from it if you widen the loop. A 6-leg field tour that cleared every
wall's bounding box by ~0.77m still hit a wall after ~10 open-loop laps, as
small per-leg errors added up, which is why the suite uses the smaller loop.

## Threshold calibration

`kMaxDelta` was 0.20 m on 2026-07-26, 0.30 later that
day when no config reached 0.20 at 0.25 slip, and 0.40 on 2026-07-27 once the
chosen config (tuned `slam`, no EKF, 0.15 slip) measured 0.30-0.33m. See
[localization tuning history](../../sentry_localization/README.md#outcome-tuned-slam-no-ekf)'s tuning history.

## Jerk calibration and bounded sampling

The jerk correction-fraction calibration used 0.15 m/s and a jerk stddev of 0.3. Both have changed: 4.0
m/s, and stddev 0.5, then 0.08, now 0.24 for a ~30cm mean jerk
(magnitude of two N(0, stddev) draws is Rayleigh, mean
`stddev * sqrt(pi/2)`). Re-derive the plateau if pass rates look off.

On 2026-07-23 a correction step looped for up to 60s until the threshold was
crossed. The TF stalled for an unrelated reason, the loop never exited, and
open-loop drift took the robot off the field and crashed gz physics. Each
step now drives one bounded leg and takes one TF sample. CPU contention from
a stray rviz2 or other sessions slows scan processing to about 2
registrations in a ~35s run, so `jerk_with_motion`'s post-drive
`correction(5.0)` waits up to 5s.

## Actor avoidance trial

The first version only swept the velocity 0.5 s ahead and hopped blocked
boxes forward, so on 2026-09-24 they landed on the robot's next leg and it
drove into them. The default paths now run from the loop's middle, 1.1 m
from every edge and always clear, across the -x, +y and +x edges.
`closest box` in the log is the nearest a box centre got to the robot's.

## Actor collision-cost trial

With collision, each box sat on the field's STL
mesh and ODE ran box-on-mesh contact every 1 ms step: gz's real-time factor
fell from ~1.0 to 0.24-0.42 and the scenario took twice as long as
`drift_correction`.

## Actor transport trial

The first
version started a `gz service` process (Ruby) per call, and ticks lagged to
~0.19 s of sim time.

## Target-bearing check

On 2026-07-27, `/cv/target`'s left/right sign
matched an independent bearing from `/sim/raw_odom` and
`/target/ground_truth_odom` in 20/20 samples.

## Truth publication trial

It used to publish on its own 60 Hz timer, which ran just ahead
of `target_driver`'s samples and sent each one ~17 ms late.

## Correction-fraction trial

0.5 sat at that plateau's edge and
flaked; 0.3 keeps margin and still catches the known-broken case
(`minimum_travel_distance` at 0.5, indistinguishable from zero). It was never
checked against amcl's plateau, so suspect it first if amcl runs flake.
