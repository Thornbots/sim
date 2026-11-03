# Opaque MCB debug bag verification — 2026-11-01

T25 uses the [DEBUG wire contract](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#debug-id6--opaque-firmware-debug)
and the firmware's [bounded send API](../../firmware/MCBV3/README.md#opaque-debug-blobs).
No production sender or debug content is enabled. Renode's UART adapter
already forwards arbitrary bytes; no board debug hook was added.

## Round 2: 128-byte flood

The hosted sentry firmware was built with GCC 14.2, and the ARM sentry release
with GCC 13.2. The real bridge includes T20 #34 (`3218545`) as an ancestor.
`test_mcb_debug_bag` enables the hosted-only flood fixture, drives the actual
firmware through a PTY, runs the bridge and MCAP recorder, and waits for both
recorder subscriptions and test subscriber data before measuring six seconds
of firmware time. The recorder captures DEBUG, POSE and REF_SYS.

- Payload: 128 bytes, `00 a5 ff 25` followed by `04..7f`; every received and
  recorded debug payload matched exactly, with nonzero stamps and empty frame ID.
- Registered colcon run: 6.29182 s wall time, 300 DEBUG blobs, 541 POSE callbacks
  and 60 REF_SYS callbacks. Callback boundary buffering can include one frame;
  the firmware flood tests independently verify exactly 540 POSE and 60 REF_SYS
  slots per 600 deadlines across five startup phases, including rollover.
- Max observed POSE header inter-arrival gap: **29.2617 ms**.
- DEBUG throughput: **6,103.16 payload bytes/s** over wall time; 50 blobs/s
  and 6,400 payload bytes/s over firmware time (6,850 framed wire bytes/s).
- MCAP totals including warmup: 468 DEBUG, 845 POSE, 93 REF_SYS. All 468 debug
  payloads were deserialized and compared against the firmware fixture.
- A separate repeat also passed: 540/60 POSE/REF_SYS, 300 DEBUG over six firmware
  seconds, 20.8959 ms max POSE gap, 6.15398 s wall time, 6,239.86 payload bytes/s.

Firmware tests additionally cover 128-byte acceptance, 129-byte rejection,
shared-slot copy/rejection, no producer, both-frame admission budget, deadline
order without skips, at most one delayed telemetry frame per debug frame,
CV_TARGET/RELOCALIZE receipt, failed writes and PING priority. Full registered
bridge and sim colcon suites pass. The bridge PTY health test checks the new
`debug_msgs_pub` diagnostic counter. Quality/actionlint pass without baseline growth.

Reproduce after building the matching hosted ELF and ROS packages:

```sh
colcon test --packages-select dji_serial_bridge sim --return-code-on-test-failure
colcon test-result --verbose
```

Evidence is archived at `/tmp/t25-r2-evidence`: `ros-tests.log`,
`ros-test-results.log`, firmware build/test logs, per-repository quality logs,
and `mcb_debug_bag_19343/bag` with its bridge/recorder logs.

This is a hosted cooperative-loop and ROS transport measurement. It does not
measure physical UART interrupts, baud tolerance, STM32 scheduling, motors or
robot behavior. No physical robot run or flashing was performed. The protocol
owns the scheduler assumptions and latency guarantee. The unchanged startup
recording list includes DEBUG in both normal and match bags.

## Round 1

The initial 32-byte-limit implementation used a four-byte fixture and fixed
100 ms spacing. It recorded 82 DEBUG messages including warmup and delivered
540 POSE / 60 REF_SYS during the six-second flood. That scheduler and payload
limit are superseded by round 2; the original report remains in
`/tmp/t25-evidence/REPORT.md`.
