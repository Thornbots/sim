# Firmware auto-drive in the MCB stages — 2026-10-12

Driving stages select firmware `SimpleAutoDriveCommand` through DBUS. The ghost
environment publishes no velocity or reference route for our robot. Renode is
the default; hosted execution is an explicit fixture. Design and model limits
remain at [the owner](../README.md#driving-and-combat-diagnostics-mcb_drive).

## Evidence and acceptance status

- Five registered MCB/CV targets and all 35 firmware unit tests pass.
  Quality/actionlint pass in every changed repo; baselines were not enlarged.
- Native auto-drive completed 32.797 simulated seconds with 6,548 holds and zero
  timeouts (`/tmp/dexec_6583.log`): displacement 4.668 m, p95 speed 1.774 m/s,
  spin 12.508 rad/s. This predates final board sensor/feedback corrections.
- Initial board runs flipped. The [Type C report](mcb-type-c-2026-10-11.md#can-receive-path-correction-2026-10-12)
  records the CAN2 reset bug that dropped wheel feedback, and decoded RPM after
  correction. DBUS stays absent through calibration for firmware-owned homing.
- Corrected board drive (`/tmp/dexec_77537.log`,
  `/tmp/track_i_auto_drive_homed_20261012`) passed the scored test and log checker:
  906.709 wall seconds, 37.758 simulated seconds, 6,520 holds, zero timeouts,
  CRC-clean UART. Approach displacement 4.657 m; p95 speed 1.772 m/s and spin
  12.506 rad/s. It reached center upright, with no approach shots. During spin,
  35 physical indexer exits yielded ten hits (28.6%). The measured low-accuracy
  diagnosis was aim error; this is diagnostic completion, not combat accuracy.
- That run required manual map_server/AMCL activation after a lifecycle response
  timeout. The lifecycle manager needed SIGKILL on teardown. It establishes
  firmware/motor behavior but is not clean acceptance. Fresh drive, match and
  parked board runs remain pending.

## Firmware review

Parent gimbal/odometry edits construct sysid random engines in place to avoid
startup-stack temporaries. They change no normal gains, offsets or frames.
The separate [indexer recovery](../../firmware/MCBV3/README.md#automatic-indexer-recovery)
has compiled regression evidence. This change adds hosted DBUS mode selection
and private `MCB3` sensor transport; the normal board ELF and auto-drive command
are unchanged. Concurrent raw-odometry/aim-frame changes are not included.

The emulator supplies three-axis gyro and specific force from the yaw link;
traction uses summed gz model mass. Firmware controls are not retuned. Mechanical
constants, tire behavior and board mounting remain estimates at the README.

## Documentation updated

- Sim README/AGENTS: stage defaults/design, ghosts, IMU, traction, acceptance.
- Firmware README/AGENTS: hosted mode selection and board-stage owner links.
- Bridge README firmware section/UART_PROTOCOL: private pipe; wire unchanged.
- Workspace PR: E2E plans, roadmap status, Unreleased changelog.
