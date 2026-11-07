# Changelog

## Unreleased

- `estimation_suite` and `shot_hit_suite` drop their simulation-clock callback (which holds the sampler
  node) on every exit from a test body. After an interrupt a failed ASSERT left it in a static, so the
  node was destroyed at exit against Fast DDS's singleton and the suite hung past launch's 5 s SIGINT
  window or aborted: `tracking:kill` failed 7 of 155 runs before, 0 of 100 after.

- With remote DDS off (the default) the sim launches now use thornbots_pkg's loopback
  `fastdds_local.xml` and no `ROS_AUTOMATIC_DISCOVERY_RANGE` instead of the `LOCALHOST` range. Under
  `LOCALHOST` a hosted `mcb_parked` cell timed out its first lockstep hold (`cv_target at 0.025 s,
  no ack`); with the profile it passes and sends no packet to the Tailscale range (the only
  off-box traffic is gz-transport multicast). Needs the matching thornbots_pkg change.

- The sim launches' `foxglove` argument now defaults to `$THORNBOTS_FOXGLOVE` or false (was true), and
  a new `remote_dds` argument (`$THORNBOTS_REMOTE_DDS` or false) keeps DDS on the machine: `LOCALHOST`
  range, no profile with Tailscale peers, and the pinned UDP-only nodes on loopback-only twins
  (`sim/dds_switch.py`). Pass `foxglove:=true` to watch a run; `remote_dds:=true` for cross-machine tests.

- `gz_bridge` frees its ROS context before `main` returns and initialises rcutils logging before
  installing the signal handler. `ros_gz_bridge`'s node is referenced from its own handles,
  services and timer, so the Fast DDS participant and rmw's graph-listener thread outlived `main` and were
  destroyed against each other in static destruction (about 2% of early interrupts killed a bridge
  with SIGSEGV, SIGABRT or "pure virtual method called"). Teardown is bounded by a 3 s watchdog that
  logs `teardown exceeded 3 s` (a bridge once ignored SIGINT and SIGTERM for 15 s in the participant's
  destruction). `tools/bridge_teardown_stress.py` repeats the interrupt. See the [investigation](docs/lifecycle-2026-11-05.md#2026-11-06-the-fast-dds-bridge-race-and-the-rtf-gate).

- `scan_odom_gate` and the `ekf_pre` lockstep chain release rf2o's odometry to the EKF at a
  fixed hold before an EKF tick instead of at a wall-timed moment, and the E2E stack sets
  rf2o's `extrinsic_lookahead_s` to 1 ms (`rf2o_extrinsic_lookahead_s`). rf2o's scan odometry
  is now bit-identical between runs, the EKF's tick membership no longer varies, and the
  Jetson-to-MCB read stream matches for 800+ lines (was 19). See the
  [investigation](docs/t17-consistent-runs-2026-11-05.md).

- Repeated hosted runs now write identical `shots.jsonl` and `states.jsonl`: three unthrottled runs of
  every `mcb_parked` cell (1883 / 14468 records), `mcb_drive` (109 / 1148) and `mcb_match` (329 / 1109),
  with 0 lockstep timeouts and 0 latest-TF fallbacks. The EKF is the Thornbots `robot_localization`
  fork (`ekf_deterministic_order`; the E2E stack refuses to start on the stock package) and its
  queued-stamp ack gates POSE delivery and rf2o's release; coordinator ack groups take `@hold` and `@tick`
  exact stamps; `target_driver`, `match_environment` and `OpponentMover` step, stamp and select path
  samples by tick, parameter changes carry the harness tick they were made at, and in `mcb_match` the
  opponents' heads are set by the mover from the path message instead of gz PID controllers; the scorer
  counts windows in chain ticks (`run_to`, `tick_s()`) and waits for the lockstep status without stepping.
  AMCL's map-frame errors moved from `shots.jsonl` and `states.jsonl` to `poses.jsonl`.
  `run_suite.sh` waits for the previous run's gz-transport advertisements to expire before launching.
  No robot behavior changes (all options are off by default). RTF=1 and gz-two-core runs are still to
  be taken ([investigation](docs/t17-consistent-runs-2026-11-05.md#2026-11-06-pass-3-the-remaining-timing-races)).

- Every Jetson-side UART producer is a lockstep chain (`producer_chains`, on with epoch-zero
  startup): the coordinator takes any-of (`a|b`) and counted (`~topic`) ack groups, lagged
  and aligned holds (`lag_s`, `align_s`, `start_s`), a boot hold and a per-hold bridge flush;
  the MCB plugin holds at a delivered POSE until `pose_consumed` reports it; `lidar_scan`
  draws lidar noise keyed by (seed, stamp, beam) instead of gz's unseeded generator;
  `mmm` is not launched. `MCB_UART_TRACE` now also logs the bytes. See the
  [investigation](docs/t17-consistent-runs-2026-11-05.md).

- `MCB_UART_TRACE=<file>` logs each Jetson-to-MCB read (sim time, bytes) from the MCB
  plugin, for finding the first input that differs between runs. T17 measurements and
  the remaining producer-ack work: [investigation](docs/t17-consistent-runs-2026-11-05.md).

- The scored E2E harness is a lockstep chain: it ticks every 50 ms, holds physics at
  each tick until it has decided, ends bring-up at a fixed 10 s of sim time, starts
  every cell on a whole simulated second and scores only inside an explicit window.
  The MCB plugin serves its parameters from its own thread and applies a change on
  the next step, so harness changes land on known steps. The harness prints each
  chain's holds and wall wait, the longest-holding chain and RTF.

- Seeded noise is keyed by (seed, robot, stamp[, panel]) instead of one shared
  stream: `detector_standin` ray noise and `match_environment` ghost aim noise
  no longer depend on message arrival order (`keyed_noise.hpp`, unit-tested).
  Ghost aim noise values differ from the previous stream's for the same seed.

- E2E lockstep stages run `point_to_cv_target` with `allow_latest_tf:=false`:
  its TF reads are stamped, not the newest to arrive, and each miss is counted on
  `/point_to_cv_target/latest_tf_fallbacks`; the harness fails a run with any.

- Epoch-zero startup is the default for lockstep E2E stages: gz starts paused,
  the coordinator releases only after detector, aim timer, opponents, MCB, robot
  spawn and a new `startup_barrier` (every stack node in the graph for 2 s) are
  ready, and the 8 s robot-start and 2 s spawn wall timers are gone from that
  path. Hosted, Renode, scored, driving and match stages are all accepted
  (the earlier hosted-only, explicit-firmware and no-scored restrictions are
  removed); startup limits are 90 s hosted and 1200 s board. The EKF starts at
  epoch zero from the spawn start (`seed_known_team`), and `initial_pose_setter`
  gives AMCL its pose once TF exists.

- Coordinated simulation-time UART is the default for every lockstep E2E stage on
  both backends (`uart_deterministic:=auto` follows `lockstep`; `false` or
  `lockstep:=false` keeps the legacy PTY). The scored harness exercises it with
  the hosted fixture and the Renode board.

- Check for Gazebo before starting any MCB E2E suite through `run_suite.sh`.

- Sim runs now end with one SIGINT per process and leave nothing behind, on completion,
  Ctrl-C, SIGTERM, SIGHUP and a SIGKILLed shell ([contract](README.md#startup-and-shutdown-contract),
  [evidence](docs/lifecycle-2026-11-05.md)). gz sim starts as the launch's direct child (`ros_gz_sim`
  ran it under `sh -c` and orphaned the server); `LaunchTree::stop()` signals the leader instead of
  the group, so nodes exit 0 instead of -2, and reports a SIGKILLed survivor (`clean()`), which fails
  the E2E suite; `gz_bridge` replaces `parameter_bridge`, which exited without `rclcpp::shutdown()` and
  crashed in Fast DDS teardown about one run in five; `head_slider_relay` no longer deadlocks on
  shutdown; `McbFirmwareSystem` stops quietly instead of throwing when gz is stopping; `run_suite.sh`
  forwards INT/TERM/HUP behind `guarded_exec`, and the E2E, estimation and aim launches run their
  suite directly. The checker rejects SIGTERM/SIGKILL escalation and notes every non-zero exit.
  `tools/lifecycle_check.sh` is the acceptance test. The board adapter exits at once on SIGINT/SIGTERM and Renode, which
  wedges if SIGINTed mid-command, has SIGINT blocked and is stopped with SIGTERM; the gz lockstep gate, the firmware
  waits and `robot_spawn_ready` yield to a stop request, so `kill_launch.sh` and a Ctrl-C during the roughly 100 s
  board boot stop the stack. Open: a Fast DDS 2.14.6 receive-thread race that still kills a `gz_bridge` in roughly
  one early interrupt in twenty.

- Sentry aiming moved to the MCB: `CV_TARGET` is a 13-byte idle look point and
  `TARGET_STATE` (id 7, 80 bytes) carries the tracked armor model; the emulator's
  protocol, firmware/bridge tests, E2E harness and bench launches follow it with no
  old-format fallback. The aiming and estimation benches run `mcb_aim_standin`,
  which compiles the firmware's `AimPlanner.cpp` from the checkout (skipped
  without it) instead of `point_to_cv_target`; `cv_head_aim` follows its aim.
  E2E no longer passes `gimbal_lag_s`, `chase_*`, `firmware_latency_s` or `v_muzzle`,
  and its aim-point diagnostics and Jetson fire-request shots are gone. Aiming
  bench floors re-measured ([measurement](docs/mcb-aim-planner-2026-11-04.md)).
  `e2e.launch.py` restores the south idle look for `mcb_parked`.

- Add optional epoch-zero `readiness_topics` to strict lockstep startup.
  The coordinator holds until every latched readiness message has stamp zero
  and Gazebo confirms a gate hold at zero; a separate bounded startup timeout
  reports missing topics. Invalid readiness and gate failures stop startup.
  E2E launch integration remains open.

- Add an opt-in `initial_position` startup to `opponent_driver`: it spawns
  the opponent once, publishes a transient-local `~/ready` only after the
  paused world is confirmed at sim time zero, and exits 1 on collision, wrong
  pose, nonzero epoch or timeout. The legacy odometry spawn is unchanged.
  `test_opponent_startup` runs it against the real paused ARCC field.
  E2E launch does not use it yet.

- Observe Jetson UART input only before a firmware cycle, so bytes arriving
  during that cycle cannot be delivered as if they had arrived earlier.

- Add an opt-in simulation-time UART relay fixture. Verified frame capture
  stamps and publication acknowledgments are still required for lockstep.

- Move historical localization, sensor and gimbal experiments into dated package history; preserve README anchors and current instructions.

- Clear estimation reset acknowledgements before sending the reset request,
  and acknowledge scorer pacing handoffs before writing between-cell results.
  Scored lockstep deadlines and native log rejection remain unchanged.

- Consolidate live software/robot commitments in workspace ROADMAP/ROBOT_TESTS; retain dated reports as evidence.

- Adopt the team's 2026-10-26 low-resolution CAD (#48) and remove
  `simplify_urdf`, its `sentry_v2.yaml` config and vendored mesh decimator;
  edit `urdf/sentry_v2/` directly. `camera` is the CAD's RGB FOV apex
  (detections are RGB-deprojected), not the IR apex.
- Shots fly at the sentry's 22 m/s (was 24 in E2E ballistics, 25 in the
  gz-free aiming bench and ghost lead), matching MCBV3 and `v_muzzle`.
- Match the robot's timed 20 ms indexer request-to-exit: a ball now leaves
  0.045 ball into a feed from the pre-indexed rest (was the next chamber
  boundary, 94 ms), once per feed. E2E stages pass `firmware_latency_s` 0.020.
  Hosted firmware tests pin the 20 ms exit and the held pending shot.
- Record where each scored shot crosses its panel (`panel_right_m`,
  `panel_up_m`, and `crossing_panel` for ballistic stages) in `shots.jsonl`.
  Hosted MCB firmware tests expect the barrel-offset yaw from MCBV3.
- Bound E2E referee parameter requests by a two-second steady-clock deadline.
  Lost responses now release the pending request and resend current match
  state; three consecutive timeouts fail explicitly. This prevents a lost
  bring-up response from leaving driving stages in `drive=stop`. Physical
  robot control is unaffected.
