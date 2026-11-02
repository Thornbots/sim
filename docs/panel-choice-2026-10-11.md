# Panel choice measurement, 2026-10-11

Linux x86_64 Jazzy, fresh local sim/bridge/aiming overlay. T32's node chooses
the most face-on panel near impact, holding its world normal until another
is better by the configured margin. The draft also needed hold resets on
late/invalid updates, margin validation, and isolation from no-lead spin mode.

## Aiming bench

`shot_hit.launch.py`, 30 sim seconds scored after 3 seconds settling, 40 Hz
fire, two panel layouts. A stationary target starts at 45 degrees; the
slow-turning target starts there and turns at 0.2 rad/s, below spin exit.
Each margin/scenario/layout was measured three times. Margin zero is the
immediate best-panel control. The bench's gimbal is perfect; it scores the
real ROS aiming node against target truth, rather than firmware or tracker.

Launch arguments used:

```bash
ros2 launch sim shot_hit.launch.py only_stationary:=true target_yaw_deg:=45 \
  target_spin_hz:=0 panel_switch_margin_rad:=0 model_yaw_jitter_rad:=0.12 \
  duration:=30 log_dir:=/path/to/run
```

Repeat with margin 0.2/0.3, spin 0.0318309886 Hz, and jitter zero for controls.
The 0.12 rad amplitude, 4 Hz sinusoid perturbs only the model's yaw; actual
target motion, yaw rate and panel hit geometry stay unchanged. It is a
controlled orientation-error stress case, not measured tracker noise.

Three-run means, with observed flip-count ranges; yaw swing is total variation
of scored aim bearings in radians. Hit rates count hits per fired shot, including
unresolved shots at the end of the scoring window.

| Jitter | Target | Layout | Margin | Hit % | Flips / 30 s | Yaw swing |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| 0.12 | still | flat | 0 | 99.306 | 240 | 31.751 |
| 0.12 | still | flat | 0.2 | 99.278 | 240 | 32.056 |
| 0.12 | still | flat | 0.3 | 99.278 | 0 | 3.365 |
| 0.12 | still | staggered | 0 | 99.306 | 239-240 | 31.712 |
| 0.12 | still | staggered | 0.2 | 99.278 | 239-240 | 31.977 |
| 0.12 | still | staggered | 0.3 | 99.306 | 0 | 3.365 |
| 0.12 | slow | flat | 0 | 99.306 | 38 | 9.081 |
| 0.12 | slow | flat | 0.2 | 99.278 | 6 | 5.294 |
| 0.12 | slow | flat | 0.3 | 99.250 | 4 | 5.040 |
| 0.12 | slow | staggered | 0 | 99.278 | 36-40 | 9.147 |
| 0.12 | slow | staggered | 0.2 | 99.306 | 4-6 | 5.200 |
| 0.12 | slow | staggered | 0.3 | 99.278 | 4 | 5.025 |

Exact-yaw controls at margins 0/0.3 retained zero stationary flips, four slow
turning flips, and hit rates 99.25-99.33%. Stationary yaw swing was zero;
slow yaw swing was 0.968-0.982 rad. Under jitter, stationary staggered pitch
swing fell from 8.123 to 0.198 rad with margin 0.3; slow staggered pitch swing
fell from 1.367 to 0.211 rad.

Select 0.3 rad: it removes noise-driven switches in this stress case while
preserving genuine slow-turn panel transitions. Margin 0.2 fails the stationary
stress case. All 30 launches (60 scored cells) exited zero, passed the native
log checker and left no processes. Custom cells have only the existing
placeholder 50% score floor; these runs do not calibrate new CI floors.

A final unmodified ten-cell default aiming bench also passed the native log
checker: hit rates were 97.17-99.25%, with no panel-margin override.

## Compiled firmware checks

Three additional native fixture tests run MCBV3 `86c2241` over UART:
non-firing aim holds after the 200 ms target-validity window; zero-delay fire
requests a shot within 10 ms once hardware/referee/scheduler are ready;
delay 150 ms requests the native indexer at 70-79 ms, accounting for its
80 ms latency allowance. All 14 firmware tests pass without skips.

An initial zero-delay probe before readiness produced no shot. Warming the
fixture reproduced the documented behavior in the unchanged pinned firmware;
no firmware change was needed. These are indexer-request observations, not
projectile exits. In `test/e2e/e2e_harness.cpp`, `Scorer` subscribes to
`/mcb_emulator/shot` and queues the MCB shot at `t + .05`, where `t` is the
indexer-request stamp published by `src/mcb_emulator_node.cpp`. That inline
50 ms delay is separate from the aiming bench's `cv_bench.hpp::kFireLatency`
and the Jetson's `firmware_latency_s` planning parameter. A request measured
at 70-79 ms therefore starts its E2E modeled flight at 120-129 ms after
receipt, rather than 150 ms. This is a source-derived E2E timing implication,
not a measured live projectile exit. Hardware calibration remains open;
this measurement did not change either placeholder.

Raw logs, scores, probe script and firmware behavior logs are retained under
`/home/blaise/workspaces/isaac_ros-jazzy/test-mcb-2026-10-11/` on this box.
