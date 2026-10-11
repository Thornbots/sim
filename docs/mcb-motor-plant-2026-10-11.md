# MCB motor plant validation, 2026-10-11

Host: `blaises-mini`, Mac arm64, Jazzy `thornbots-mac` container, Mesa software
rendering. Tests used the checked-out sentry hosted release built with GCC 14.
The [MCB emulator](../README.md#mcb-emulator) owns usage, design and model limits.

## Changes tested

`McbFirmwareSystem` steps native firmware once per 1 ms gz step, passes DJI
feedback, and applies motor torque/force outputs. No gz position or velocity
controller drives our robot. The bridge waits for the firmware PTY.
Firmware is isolated from the launch process group's SIGINT and exits on EOF.

The branch incorporates sim `b3204f3` and thornbots_pkg `dcdd318` (T32), retaining
the other worker's firmware tests and panel-choice bench. The first parked,
drive and match runs preceded that merge; the final match, native checks and
focused motor check used the integrated trees.

## Results

- Firmware: 31 control/geometry tests and four hosted regressions pass; the
  sentry ARM release also builds. No board flash was performed.
- Native sim checks: 15 firmware cases, five wire cases, four motor-model cases
  and one real ROS UART-bridge case pass, with no skips. The physical-CAN case
  verifies that shaft-speed feedback changes firmware drivetrain current.
- Direct UART/gz check (`mcb_physics_suite`): targets +0.6 and -0.5 rad reached
  +0.600011 and -0.500004 rad; peak shaft speeds 226.806 and 411.137 rad/s.
  The native controller drives inertial gimbal joints with torque, and the
  test checks motion, settling, PTY removal and clean process shutdown.
- Stationary-lateral `mcb_parked`, 10 s: one run hit 21/21; another hit 0/21.
  Both had zero lockstep timeouts. The latter failed its hit-rate assertion;
  its tracker velocity error was 1.33 m/s for a stationary target. These runs
  do not establish repeatability or a new combat floor.
- `mcb_drive` passes diagnostic completion. Route p95 by segment: 0.315,
  0.252, 0.072, 0.088 and 0.024 m. Hit rates: 0%, 0%, 20%, 0%, 8.3%;
  low-hit segments report median aim error over 0.10 m. RTF 0.75, 51.53 s wall.
- Final `mcb_match` passes: zero friendly intersections, 397 referee UART
  frames, 36 firmware shots, opponent_1 HP 380. Route p95: 0.259, 0.0013,
  0.073, 0.103 and 0.024 m. RTF 0.59, 69.81 s wall. Zero lockstep timeouts.
- The accepted drive/match runs pass the native log checker. Their diagnostic
  pass does not establish combat accuracy. The initial process-group shutdown
  race caused a gz abort; the focused check and final match verify its fix.

Container logs: `/tmp/track_i_can_parked_5`, `/tmp/track_i_can_parked_final`,
`/tmp/track_i_can_drive`, `/tmp/track_i_can_match_final`; the focused check
log is `/tmp/mcb_physics_19150.log`. No live test nodes remain after teardown.
