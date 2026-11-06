# Changelog

## Unreleased

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
