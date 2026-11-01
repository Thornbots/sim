# Opaque MCB debug bag verification — 2026-11-01

T25 uses the [DEBUG wire contract](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#debug-id6--opaque-firmware-debug)
and the firmware's [bounded send API](../../firmware/MCBV3/README.md#opaque-debug-blobs).
No production sender or debug content is enabled. Renode's UART adapter
already forwards arbitrary frame bytes and needs no new message dispatch.

## Hosted firmware → ROS → MCAP

`test_mcb_debug_bag` ran the actual hosted sentry firmware with its opt-in
`MCB_DEBUG_TEST_FLOOD=1` fixture, the real UART bridge over a PTY, and
`ros2 bag record -s mcap --max-cache-size 0`. It decoded the resulting bag
with `rosbag2_cpp::Reader`, checking both the payload and nonzero header stamp.

- Payload `{00,a5,ff,25}` matched on `/dji_serial_bridge/debug` and in MCAP.
- 82 debug messages were read from the bag, including discovery/warmup time.
- The measured six seconds delivered exactly 540 POSE and 60 REF_SYS frames
  while debug production was flooded every firmware control cycle.
- Both bridge and recorder exited cleanly; the test passed in 9.7 seconds.
- The startup topic list includes debug unconditionally with respect to
  normal/match and video modes; recording still follows `ENABLE_BAG`.

After building the hosted firmware and the ROS packages, run:

```bash
colcon test --packages-select sim --ctest-args -R test_mcb_debug_bag
colcon test-result --test-result-base build/sim --verbose
```

The firmware's separate hosted tests exercise maximum 32-byte floods, exact
10 ms mandatory slots across construction phases and clock rollover, full
wire-drain accounting, RX CV_TARGET/RELOCALIZE, rejected lengths and failed
writes. They establish the scheduling invariant against modeled 115200 8N1
airtime; this hosted run is not a measurement of board UART/interrupt latency.
No physical robot run or flash was performed.
