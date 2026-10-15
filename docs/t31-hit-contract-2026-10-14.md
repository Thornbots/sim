# T31 hit permission and REF_SYS contract, 2026-10-14

The Jetson clears CV_TARGET bit 2 whenever it holds any target estimate,
including coasting or predicted tracks during brief occlusion and
unconverged tracks. It sets the bit only without an estimate, including Jetson
patrol. Expiry is checked on every publish tick; reacquisition clears
permission on the next frame with no cooldown. Permission does not depend on
firing or a successful aim solve.

Firmware owns hit turns. The former Jetson path added a chassis-relative
REF_SYS delta to gun yaw; current POSE does not carry chassis yaw, so that
conversion and its sign/duration parameters are removed. Wire semantics stay
at [UART_PROTOCOL](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#ref_sys-id3--referee-system-status).
The current UART payloads are unchanged.

## Targeted validation

- The ARM GCC 13 firmware build and hosted firmware build pass. Firmware
  coverage is 39 pure-unit tests and five hosted regressions; all 86 simulator
  UART/control tests pass.
- The original Renode physics smoke passed in 186.455 seconds. It recorded 541
  POSE and 60 REF_SYS frames; CRC8/CRC16 counts were both zero. This smoke did not inject damage or validate the hit angle.
- The expanded physics test passes hosted (8.813 seconds) and Renode
  (197.735 seconds). With fixed chassis yaw π and measured head yaw π−0.5,
  actual referee-UART damage updates report +π/2 for armor 1 and −π/2 for
  armor 3, then 123 on the following REF_SYS at unchanged HP. Both cases
  require a fresh POSE sample at the nonzero head yaw. Renode received 584
  POSE and 65 REF_SYS frames, with zero CRC8/CRC16 errors. Native run/stack
  log checks pass for both backends; their launch groups shut down cleanly.
- Thornbots passes its eight registered suites, including target, coasting,
  unconverged, no-target/patrol, failed aim with a held estimate, timeout and
  immediate reacquisition checks. With patrol disabled and no target, it emits
  no frame, preserving the existing no-point behavior.
- Registered ROS/lint checks: eight thornbots, 19 sim and nine bridge suites;
  382 cases, zero errors/failures and nine existing lint skips. Host Python
  3.12 repository quality and actionlint pass without baseline growth.

## Parked E2E comparison

The selected hosted cell is `mcb_parked`: stationary target on the lateral
path, with the default 20-second scoring window. Each variant was run three
times. The table pools hits and shots across those three runs; no other parked
cell was scored.

| Variant | Pooled hits/shots | Hit rate |
| --- | ---: | ---: |
| Original nightly | 122/122 | 100% |
| Full T31 candidate | 122/122 | 100% |

One early `nightly-mcb_parked-1` attempt aborted after a collision. It is
excluded; none of the six scored runs were excluded. This parked result covers
the selected stationary/lateral cell only and does not establish moving-target
accuracy.

## Separate drive/match comparison

The Thornbots [hit-permission report](../../thornbots_pkg/docs/hit-permission-2026-10-15.md)
contains the separate #14 angle-common `mcb_drive`/`mcb_match` comparison.
Both sides of that comparison use the same angle-only firmware and emulator;
only the Jetson permission implementation differs. It validates #14 independently
of latching and is not pooled with the parked comparison above.

## Limits

No physical robot or hardware measurement was performed. Renode establishes the injected referee-UART, reporting and CAN/physics
paths, not physical armor wiring, calibration or STM32 timing. The parked comparison does not measure combat accuracy; moving runs
and calibrated hit-rate floors remain separate work.

Raw run/stack logs, comparison metadata, registered test XML and fixtures are
preserved at `/Users/blaise/ros2_ws/t31-artifacts-2026-10-15`.
