# Patrol marker firmware verification, 2026-10-11

Linux x86_64, running `isaac_ros_jazzy_container`, isolated ROS overlay and
fresh hosted sentry source copy. T33 defines bit 3 without adopting firmware
behavior for it. The maintained wire semantics are the bridge's
[CV_TARGET flags](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#cv_target-id1--aim-point-and-fire-decision).

## Build and tests

```bash
CXX=g++-14 tools/test_cpp.sh
(cd MCB-project && scons build-sim robot=sentry profile=release compiler-suffix=-14 -j4)
CXX=g++-14 tools/test_hosted_cpp.sh
colcon build --packages-select dji_serial_bridge thornbots_pkg sim
MCB_FIRMWARE_BINARY=/path/to/fresh/MCB-project.elf \
  colcon test --packages-select sim thornbots_pkg dji_serial_bridge \
  --ctest-args -R '^test_' --output-on-failure
```

The actual build/test used explicit `--base-paths`, `--build-base` and
`--install-base` under `test-mcb-2026-10-11/`; the test install base matters,
since colcon otherwise prepends the normal, older install before the test
process. Package resolution and a direct bridge test confirmed the fresh
bridge executable. Full commands and logs are retained in that directory.

- Fresh hosted GCC 14 build: successful, warnings treated as errors.
- MCBV3: 31 core tests and four hosted regression tests pass.
- Sim: 174 registered test results pass without skips, including 78 compiled
  firmware cases and six pure protocol cases. The firmware matrix uses every
  combination of bits 0-3, verifying aim parsing, fire-bit independence,
  game-stage fire gating, patrol permission after expiry and damage-turn
  permission during a live aim. Bit 3 alone never requests a shot.
- The real bridge PTY test packs every bit combination, checking all 15 payload
  bytes' fields and the unchanged payload size. Its existing firmware/bridge
  test also passes.
- Thornbots: 110 test results pass, including sweep/hit-turn patrol markers
  and firing/non-firing target points with the marker clear.
- Bridge: six registered test results pass. Quality/actionlint pass in all
  four changed packages; existing firmware lint debt is unchanged.

The game-stage test delivers the next periodic referee update before asking
to fire; changing the fixture's referee data alone takes up to 100 ms to reach
firmware. The tests own and stop each hosted process and bridge process group.

## Observed limits

Compiled firmware patrol changes yaw by approximately -0.08 rad during the
400 ms after expiry in this fixture: -0.0002 rad per 1 ms control cycle.
The Jetson patrol defaults to -2 rad/s, ten times faster. Its README's claim
that it copied the firmware speed was corrected; neither rate was retuned.

A separate damage probe supplied IMU yaw 0.6 rad, joint yaw 0.2 rad and left
armor damage. With bit 3 clear or set, firmware chose yaw 1.38127 rad; using
radian IMU yaw directly would give 1.97080 rad. This reproduces the known T31
`HitTracker` extra PI/180 scaling through compiled firmware. It also confirms
bit 3 leaves that behavior unchanged. T31's angle/reference and REF_SYS hit
latching remain open; T33 does not adopt new hit-turn behavior.

[Aim hold and native indexer timing](panel-choice-2026-10-11.md#compiled-firmware-checks)
were revalidated by the same fresh firmware tests. Fake hardware does not
validate physical projectile exit, CAN dynamics or MCU timing.

Raw logs and the nonzero-yaw probe are retained under
`/home/blaise/workspaces/isaac_ros-jazzy/test-mcb-2026-10-11/` on this box.
