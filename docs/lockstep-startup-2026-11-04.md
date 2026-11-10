# Epoch-zero lockstep startup, 2026-11-04

The isolated Mac arm64 Jazzy container built `sim` successfully and ran
`ros2 run sim lockstep_suite`: all 18 tests passed in 60.088 seconds.
The test uses the compiled coordinator, real Gazebo server and gate plugin,
ROS readiness publishers and stamped acknowledgment publishers.

The first readiness run exposed that a running world first holds at 1 ms.
That is sufficient for the older strict deadline tests, but cannot prove
epoch-zero initialization. The opt-in gate now supports a world launched
paused: it reports zero without blocking paused updates, allowing scene and
create services to operate, then unpauses on the first positive release.
Usage and configuration live in [lockstep startup](../README.md#lockstep-startup).

The passing suite covers:

- All seven existing gate, timer-phase, strict discovery-order, delayed or
  missing initial acknowledgment, and strict timeout tests.
- Readiness published before or after discovery, latched delivery to a late
  coordinator, duplicate zero readiness, and staggered readiness delayed
  3 seconds beyond the ordinary 2-second per-tick timeout. Physics and native
  clock remain at zero; a model is created and observed through Gazebo services
  during the hold. The first release and subsequent holds are exactly 5, 10
  and 15 ms where exercised.
- Missing readiness with topic-specific diagnostics, nonzero readiness before
  release, and invalid readiness during an acknowledgment wait. The latter
  stops without needing the outstanding tick acknowledgment.
- All readiness present but no gate, an already advanced world, a gate
  timeout at zero, and an epoch-zero gate mistakenly started running.
- Duplicate topics including equivalent relative and absolute names, empty
  topics, readiness without strict mode, and an invalid startup timeout.

The final sim and workspace quality checks, actionlint and whitespace checks
passed. This is startup-primitive validation. The E2E launch still needs to
connect robot, detector, opponent and plugin readiness to one startup epoch;
scored consumption gates, Renode validation and whole-match acceptance remain
open. No scored campaign was run.

## Coordinated UART clock bootstrap

In a fresh isolated Mac arm64 container, the explicit hosted firmware target,
bridge and sim built. The real paused Gazebo MCB plugin, UART bridge,
`ros_gz_bridge` and deterministic detector reached readiness with the detector
started before the plugin and after repeated zero publications. A separate
bridge observed Gazebo's native clock at zero, and the MCB tick topic remained
empty through readiness. Missing detector readiness timed out; an injected
nonzero ROS clock failed both before and after readiness while paused. The
detector component's five tests and the bridge PTY test passed. The seven
existing actual-plugin acknowledgment
cases passed with the explicit hosted ELF, including timeout and invalid acks.
The relay's seven and assembler's two tests passed with the ELF supplied, and
all 18 lockstep gate/coordinator tests passed again in 60.317 seconds.

These checks establish the bootstrap primitive, not the remaining e2e launch
or robot readiness wiring. The checked-out board ELF was unavailable; no
Renode or scored T17 run was made.
