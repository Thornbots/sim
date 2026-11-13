# Managed firmware aiming validation, 2026-11-12

The [stand-in](../README.md#mcb-aim-stand-in) now shares firmware `AimControl`,
including receipt expiry, clear precedence, continuous rates and the measured
impact gate. That original stand-in change made no launch edits. Launch owners should remove obsolete
`gimbal_lag_s` overrides; tick/ack topic interfaces are unchanged.

## Builds and tests

Validation used the task-owned four-CPU Linux arm64 Colima container and
`--symlink-install`. Bridge `design/target-handoff` commit
`2e9a435b709697f3fecb3f21034505daa4cf40eb` supplies the actual `bool clear`
field in `msg/TargetState.msg`. Final builds do not use the earlier temporary
uncommitted stub. No bridge source edits are part of this work.

```sh
colcon build --symlink-install --packages-up-to sim --parallel-workers 4
colcon test --packages-select sim --return-code-on-test-failure
colcon test-result --verbose
```

Colcon reports **269 result records, zero errors, four failure records and one
skip**. This includes CTest aggregates: the unique GTest/pytest records are
**237: 233 passed, three failed, one skipped**. The failures are the unchanged
nightly `McbPitchBoot` parameter cases at 0/−0.2/+0.2 rad; CTest contributes the
fourth failure record. The existing UART-relay test skips when its expected
`T17_FIRMWARE_BINARY` environment is absent. No skip or exclusion was added.

The eight stand-in unit tests pass. The real ROS bridge/PTY suite passes all
three tests. Protocol tests pass all nine tests; hosted firmware UART tests
pass 52/55 with the same three pitch-boot failures. Firing fixtures explicitly
supply aligned measured feedback through a shared test-only Reticle reference;
they isolate the known motor-fixture offset mismatch without changing the
production adapter or pitch-boot tests. Miss-then-align and clear-over-valid
tests exercise actual firmware over UART.

Package quality and actionlint pass with zero new diagnostics. Firmware
numerical, lifecycle and timing evidence is maintained in
[MCBV3's results](../../firmware/MCBV3/docs/managed-aim-2026-11-12.md).

## Identical-fixture comparison

Baseline: sim nightly `b40ada7fd5efce3091590baeb3318a12c05cc6d0` stand-in and
MCBV3 nightly `e7605e5` planner, where chase is already the default. Both
executables were built against the same actual bridge message commit. Only the
stand-in executable was swapped; launch/scorer/fixture sources stayed identical.

Input was raw perfect `target_state_truth`, not tracker detections and not
Agent B's target manager. Both used the same deterministic lateral paths, fixed
shooter at `(0, 0, 0.4)`, 22 m/s, 20 ms request latency, perfect gimbal,
zero model-yaw jitter, flat and staggered layouts, three seconds settling and
30 seconds scoring per cell. Each comparison was repeated three times. There
is no random detection seed in this truth fixture.

Each entry is mean shots per run, pooled fired-shot hit percentage, and mean
native score with its three-run min–max. **120/120 final cells passed** across
both implementations and the three campaigns, and every suite passed its native
log checker. Floors were unchanged.

```sh
tools/run_suite.sh aim headless:=true log_dir:=/evidence/default-N
tools/run_suite.sh aim headless:=true skip_stationary:=true target_spin_hz:=0.0 log_dir:=/evidence/moving-N
tools/run_suite.sh aim headless:=true only_stationary:=true target_spin_hz:=2.0 log_dir:=/evidence/spinning-N
```

### Default stationary and combined translation/spin sweep

| Layout | Speed m/s | Spin Hz | Nightly shots / hit % / score % (range) | Managed shots / hit % / score % (range) |
| --- | ---: | ---: | --- | --- |
| flat | 0 | 0 | 1200 / 99.3 / 99.30 (99.2–99.3) | 1201 / 99.3 / 99.30 (99.3–99.3) |
| flat | 0.5 | 2 | 481 / 98.5 / 69.01 (68.9–69.2) | 791 / 98.4 / 81.68 (81.1–82.3) |
| flat | 1 | 1.857 | 445 / 98.1 / 67.27 (67.0–67.5) | 891 / 98.2 / 85.61 (85.3–86.0) |
| flat | 2 | 1.571 | 459 / 96.9 / 67.02 (66.5–67.7) | 907 / 97.6 / 85.71 (85.2–86.4) |
| flat | 4 | 1 | 511 / 96.0 / 68.41 (67.1–70.2) | 1018 / 97.6 / 90.25 (89.8–90.6) |
| staggered | 0 | 0 | 1200 / 99.2 / 99.22 (99.2–99.2) | 1200 / 99.2 / 99.20 (99.2–99.2) |
| staggered | 0.5 | 2 | 479 / 98.9 / 69.16 (69.0–69.4) | 802 / 98.0 / 81.77 (80.0–83.0) |
| staggered | 1 | 1.857 | 446 / 97.5 / 66.89 (66.9–66.9) | 891 / 98.1 / 85.43 (85.1–85.7) |
| staggered | 2 | 1.571 | 462 / 97.1 / 67.27 (66.5–68.0) | 911 / 98.1 / 86.29 (85.7–86.7) |
| staggered | 4 | 1 | 514 / 96.0 / 68.59 (67.6–69.3) | 1009 / 97.9 / 90.08 (89.6–90.3) |

### Translation only (zero spin)

| Layout | Speed m/s | Spin Hz | Nightly shots / hit % / score % (range) | Managed shots / hit % / score % (range) |
| --- | ---: | ---: | --- | --- |
| flat | 0.5 | 0 | 1200 / 98.3 / 98.30 (98.2–98.3) | 1200 / 98.7 / 98.70 (98.7–98.8) |
| flat | 1 | 0 | 1200 / 97.4 / 97.36 (97.2–97.6) | 1200 / 98.3 / 98.33 (98.2–98.4) |
| flat | 2 | 0 | 1200 / 96.5 / 96.50 (96.4–96.6) | 1200 / 98.0 / 97.97 (97.8–98.1) |
| flat | 4 | 0 | 1200 / 97.3 / 97.31 (97.2–97.4) | 1201 / 98.7 / 98.69 (98.3–99.2) |
| staggered | 0.5 | 0 | 1200 / 98.2 / 98.20 (98.2–98.2) | 1200 / 98.8 / 98.82 (98.8–98.9) |
| staggered | 1 | 0 | 1200 / 97.3 / 97.28 (97.2–97.3) | 1200 / 98.2 / 98.21 (98.1–98.4) |
| staggered | 2 | 0 | 1200 / 96.6 / 96.55 (96.3–96.8) | 1200 / 97.9 / 97.90 (97.6–98.1) |
| staggered | 4 | 0 | 1200 / 97.1 / 97.16 (97.1–97.2) | 1200 / 98.2 / 98.21 (97.7–99.2) |

### Spin only (stationary center, 2 Hz)

| Layout | Speed m/s | Spin Hz | Nightly shots / hit % / score % (range) | Managed shots / hit % / score % (range) |
| --- | ---: | ---: | --- | --- |
| flat | 0 | 2 | 480 / 99.3 / 69.51 (69.4–69.6) | 778 / 99.4 / 81.87 (81.1–82.6) |
| staggered | 0 | 2 | 479 / 99.3 / 69.48 (69.3–69.6) | 780 / 99.3 / 81.94 (81.3–82.4) |

The native score combines fired-shot hit fraction and hits per 1,200 expected
40 Hz shots with equal weights. It measures throughput as well as placement.
Three-run ranges are retained; these observations do not establish a new
repeatability floor. This scorer follows straight
rays through published aim points, not calibrated gravity/drag. These runs do
not establish ballistic accuracy, controller-plant behavior or Type C deadlines.

An initial redesigned run was rejected by the native checker on both stationary
floors because ROS `/clock` callback coalescing skipped control ticks and rounded
up the indexer cooldown. The adapter now advances intervening 1 ms decisions
before applying new input. Final comparisons use that corrected adapter;
no floor was lowered and the rejected run is excluded. All trusted final runs
pass the native checker. The checker accepts its pacing-gate-drop lifecycle notes;
there are no accepted lockstep timeouts.

## Integration

Integrate this PR with MCBV3 #141 and bridge #46; use a coordinated workspace
gitlink bump and a High changelog/reflash requirement. The parent owns that
workspace PR and the end-to-end manager/firmware bench. No workspace PR or merge
is included here. The accepted radius and malformed-assignment dispositions
are maintained in the firmware evidence.

## Nightly integration follow-up

Merged sim nightly `83131d8` and firmware nightly `8405b0d` into the package
branches without rewriting their history. The firmware uses the nearest-zero
pitch reading on every robot; sim's encoder zero is now −0.26 with a source
parity test. The managed UART firing fixture consequently supplies the requested
pitch directly, removing its previous 0.17 rad offset compensation. The earlier
pitch-boot failure counts above describe the historical baseline, not this merge.

The stacked managed-fixture change explicitly enables the manager in E2E and
includes it in graph checks. Node-specific launch parameters configure its
input-frame acknowledgement without changing tracker none marks. The new
detector test checks a visible panel's confidence and exact capture-time
disappearance, including continued empty-frame acknowledgements; lifecycle
tests check physical exit stamps, inclusive disappearance boundaries and
explicit no-shot results. No arrival, localization, hit-rate or timeout limits
were changed. The previous ingress failure did not reproduce in this task's
full-run attempt: the chassis settled at `(−0.0058, −2.3904)` after 11.8 simulated
seconds. Full acceptance results follow below.

The first managed full-run attempt used thornbots_pkg `eeb09f9`, bridge
`a71ab63` and the skip-only acknowledgement. It passed ingress, then stopped in
the stationary lateral cell at detector frame `37.317 s`: neither bridge
`tx_ack/target_state@hold` nor manager `skip_ack@tick` answered. The existing
60 s stuck-producer guard failed the run; all 13 cells failed and the native
checker rejected it. No scores from this attempt are accepted. A held-model
refresh retains its original model stamp, so it cannot satisfy the hold-stamped
bridge acknowledgement and is not a skipped send. This interface gap was
reported to the manager/bridge owner; no peer sources were edited here.

The replacement uses thornbots_pkg `15e5647` and bridge `6232e4f`.
E2E configures manager `state_ack_topic=/target_manager/frame_ack` and
`consumer_ack_topic=/dji_serial_bridge/tx_ack/target_state`. Its input-frame
Header acknowledges every consumed input after all emitted states have reached
the bridge queue. The coordinator subsequently flushes and awaits completion;
model header stamps retain their original meaning. The detector group uses
this frame acknowledgement rather than accepting a hold-stamped bridge packet
acknowledgement. Only tracker TF-drop none marks remain as an alternative;
the manager's unused skip-mark output is disabled in this fixture.

After the fixture correction, colcon reports **273 result records, zero errors,
zero failures and zero skips**. `T17_FIRMWARE_BINARY` points at the rebuilt
hosted sentry binary so the UART-relay backend test runs instead of skipping.
All six detector tests and both new lifecycle tests pass. The standalone
lockstep suite passes **23/23** and its native log checker accepts the run.

### Managed parked run

```sh
tools/run_suite.sh mcb_parked backend:=hosted headless:=true log_dir:=/evidence/parked-frame-ack
```

All **13/13 cells pass**, and the native checker reports `bench ok`. This is one
full run, not a three-repeat accuracy floor. Input is `detector_standin` panels
from Gazebo truth, with confidence 0.9, through the real selector, tracker,
manager, bridge and hosted firmware. It uses the launch defaults: 20 s nominal
scoring duration, three seconds settling, all three paths, speeds 0/1/2/4 m/s
and their normal spin sweep, plus disappearance. No truth TargetState bypass is
used in this run. The previous chase-default stand-in comparison above uses a
different, explicitly described fixture; it is not a baseline for this table.
The integrator's parallel nightly parked campaign supplies that comparison.

| Cell | Shots | Hit % | First shot from window start, s | Shots after disappearance |
| --- | ---: | ---: | ---: | ---: |
| stationary-lateral | 259 | 97.30 | 4.485 | — |
| speed1-lateral | 18 | 55.56 | 3.525 | — |
| speed2-lateral | 71 | 45.07 | 0.186 | — |
| speed4-lateral | 202 | 25.25 | 0.336 | — |
| stationary-radial | 270 | 98.89 | 0.014 | — |
| speed1-radial | 7 | 85.71 | 1.212 | — |
| speed2-radial | 55 | 72.73 | 0.257 | — |
| speed4-radial | 232 | 73.28 | 0.157 | — |
| stationary-diagonal | 250 | 100.00 | 0.010 | — |
| speed1-diagonal | 16 | 56.25 | 1.648 | — |
| speed2-diagonal | 72 | 70.83 | 0.372 | — |
| speed4-diagonal | 197 | 42.13 | 0.364 | — |
| disappears-lateral | 194 | 98.97 | 0.074 | 6 |

There are 1,843 physical shots, all adjudicated. The disappearance cell reveals
at `329.001 s` and hides at `339.001 s`; its final post-disappearance exit is
257 ms after hiding. These are physical chamber exits, not fire requests. The
run reports zero lockstep timeouts and zero latest-TF fallbacks. Worst cell
localization p95/max are 0.05021/0.05039 m; maximum chassis displacement is
0.02637 m, below the unchanged 0.03 m gate. The native elapsed result is
`13 passed in 478.45s`.

The epoch-zero launch regression also passes all nine scenarios: ordinary and
six-second-delayed coordinator startup plus missing-readiness failure for each
of parked, drive and match. It now observes manager readiness and asserts the
first two `frame_ack` stamps are the detector input stamps, 17/34 ms. Missing
readiness still fails closed before a positive firmware tick. These short
checks do not score complete drive/match campaigns.

Final colcon validation against those updated peer pins again reports 273
records: **241 individual package cases pass**, plus their CTest aggregates,
with no errors, failures or skips. Quality and actionlint pass in both changed
packages. The real ROS/PTY unit fixtures use their existing 15 s bridge-stop
fallback for two bridge processes, before and after the peer update; those
process groups are killed and reaped by the fixture. The full parked run and
epoch-zero stacks leave no running installed nodes or Gazebo/launch processes.
No Type C board timing or complete drive/match scoring campaign was run here.
