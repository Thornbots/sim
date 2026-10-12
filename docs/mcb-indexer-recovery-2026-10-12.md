# Indexer recovery after board startup, 2026-10-12

The Type C parked run finished its scoring window with valid UART traffic and
zero lockstep timeouts, but no physical indexer exits. Its transport/boot
history is in [the board report](mcb-type-c-2026-10-11.md). Firmware behavior
belongs to [MCBV3's design](../../firmware/MCBV3/README.md#automatic-indexer-recovery);
the estimated motor/cartridge model belongs to [sim](../README.md#type-c-board-execution-with-renode).

## Reproduction and correction

The compiled-firmware regression `IndexerResumesWhenSetupBecomesInGame`
runs nine virtual seconds in SETUP, allowing indexer homing to complete or
time out, then sends firing targets in IN_GAME. Before the fix the firmware
requested shots but emitted no indexer CAN output; the test failed in 384 ms.
The automatic command had stopped the indexer while shooting was forbidden,
and never returned it to idle when permission resumed.

The automatic command now returns its owned indexer to idle when shooting
is allowed. Referee, heat, projectile-presence, power and minimum shot-spacing
checks are retained. Manual aiming behavior, gimbal gains, odometry frames
and UART payloads are unchanged. The regression passed in 380 ms against the
original hosted fixture, independently of the pending auto-drive adapter edit.
ARM GCC 13 release, hosted GCC 14 release, 31 package control/geometry tests,
four hosted compatibility tests, and registered MCB/clock tests pass.

The motor plant also models a loaded-cartridge stop against reverse homing.
It keeps shaft velocity at zero against the stop while reporting the applied
current through CAN. The normal firmware detects that stall and completes
its own homing; no firmware RAM or homing state is written by the adapter.
A motor-plant test checks stall feedback and subsequent forward travel.
The stop/load geometry is an estimate, and finite ammunition remains unmodeled.

## Critical firmware review

The gimbal and odometry changes remain limited to in-place sysid generator
construction, justified by the board's startup stack exhaustion. No control
gain, offset or navigation/raw-aim change was added there. The indexer recovery
is a separate normal-robot correction justified by a reproducible stopped
motor despite accepted shot requests after SETUP/calibration.

## Documentation checked

- MCBV3 README owns automatic indexer recovery; its AGENTS links still point
  to the remaining auto-drive work and sim's model limits.
- Bridge README links the firmware design. UART_PROTOCOL was checked: this
  correction changes neither packets nor timestamps.
- Sim README distinguishes board physical exits from hosted issued-shot
  counters and documents the loaded-cartridge model; this report and the
  board report contain accepted and rejected measurements.
- Workspace CHANGELOG Unreleased and current ROADMAP status accompany the
  coordinated pin bump; the remaining track I item is firmware auto-drive.
