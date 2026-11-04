# UART coordination diagnostic, 2026-11-03

The experimental `uart_deterministic` path was built in an isolated Mac arm64
container against bridge c0f04e2 and hosted firmware. The assembler and relay
unit tests passed, including split frames, sequence wrap, delayed serial delivery,
reverse input deferral and legacy relay backpressure. The first two launch
attempts exposed startup races; the readiness path was changed to start the
bridge on PTY existence and wait for coordination discovery before stepping.

A subsequent unscored `mcb_parked` hosted launch with the real gz plugin, PTY,
relay and bridge enabled metadata. A ROS probe matched 35 successful acks to
metadata identities and 30 pose capture stamps to metadata stamps. The
coordinator reported 5,139 holds and zero timeouts at 30.097 simulated seconds.
Injecting a negative ack with identity zero terminated the gz session with an
invalid acknowledgment error. The run logged dropped CV target sends, so it
is not a clean end-to-end pass.

That diagnostic did not prove held-clock equality at every frame, missing-ack
timeout, mismatched and duplicate positive acks, varied scheduling, or
coordinated-mode saturation and reverse causality. No Renode run was made:
the hardware ELF was absent. The scored harness does not forward the new mode;
full match campaigns remain blocked on foundation acceptance.

Follow-up in a fresh isolated Mac arm64 container: the hosted firmware build
completed, and `test_uart_relay` passed all seven tests with
`T17_FIRMWARE_BINARY` explicitly set to that build. This includes 3,000
coordinated frames through a saturated socket, ordered metadata and capture
stamps, coordinated reverse-input deferral, and incomplete-frame disconnect.
Both assembler tests passed. These are relay-level checks, not proof of the
plugin/bridge acceptance cases above. The plugin now checks a latched failure
immediately after spinning callbacks, before the next firmware step.

Follow-up actual-plugin fault injection used the explicit hosted firmware binary
and a ROS ack proxy. Two runs passed success (20 acks and 18 capture-stamped
poses), a 250 ms held-clock delay and resume, bounded missing-ack timeout, and
wrong-ID, early, stale, future and duplicate positive ack rejection. The plugin
publishes the held Gazebo clock while waiting, and waits for the bridge's clock
readiness before firmware starts. The bridge pairs the exact frame and capture
stamp and waits for its delivery time; the plugin checks the ack identity and
publication stamp. Earlier runs exposed clock discovery races; these checks do
not establish scored-stage acceptance or whole-match repeatability.
