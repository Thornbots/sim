# T17 consistent runs, 2026-11-05

Mac mini (Colima, 6 CPUs, one agent container of mine), workspace `nightly` at the
bumps named below, hosted MCB fixture unless stated. Another agent's container
(`isaac-agent-lifecycle`) ran at 380-395% CPU throughout every measurement here, so
RTF and wall waits are loaded numbers, not the machine's best. Raw logs are in my
container's `/evidence` (not kept in git); the commands are in each PR.

## What landed

| Step | Change | PRs |
| --- | --- | --- |
| UART | Sim-time UART (bytes x 10 / baud, per-frame acks) is the default for every lockstep stage; the bridge gates CV_TARGET/TARGET_STATE on the simulated wire, not `TIOCOUTQ` | sim #79, bridge #41 |
| Bring-up | Epoch-zero barrier is the default (paused gz; detector, aim timer, opponents, MCB, spawn and `startup_barrier` ready before the first release); no `ROBOT_DELAY_S` or spawn wall timers; Renode accepted; EKF starts at epoch zero (`seed_known_team`) | sim #80, sentry_localization #40, thornbots_pkg #63 |
| TF | `point_to_cv_target` reads TF at a stamp and counts misses; the harness fails any | thornbots_pkg #64/#65, sim #81 |
| Seeds | Noise drawn per (seed, robot, stamp[, panel]) | sim #82 |
| Harness | The scored harness is a 50 ms lockstep chain (+1 ms); fixed bring-up, aligned cells, scoring window, threaded and staged MCB parameters; per-chain/RTF report | sim #83 |

## Results

Stationary-lateral cell (`speeds:=0 paths:=lateral duration:=10`), first-shot sim time
before this task and after, across three unthrottled runs:

| | run 1 | run 2 | run 3 | spread |
| --- | --- | --- | --- | --- |
| before (workspace `3d29927`) | 34.986 s | 44.924 s | 38.946 s | 9.9 s |
| after (`tr_h1..3`) | 34.232 s | 34.246 s | 34.252 s | 0.020 s |

Ingress now starts at t = 14.201 s in all five runs below (before: 15.0, 21.5, 25.3 s).

Final-code runs of the same cell, `compare_runs` against run 1:

| Run | Configuration | RTF | Lockstep timeouts | Latest-TF fallbacks | First shot | `shots.jsonl` |
| --- | --- | --- | --- | --- | --- | --- |
| 1-3 | unthrottled | 0.85 / 0.81 / 0.70 | 0 | 0 | 34.232 / 34.246 / 34.252 | differs from record 0 |
| 4 | `real_time_factor:=1` | 0.65 | 0 | 0 | 34.047 | differs from record 0 |
| 5 | container pinned to 2 CPUs (`--cpuset-cpus 0-1`) | 0.48 | 0 | 0 | 34.247 | differs from record 0 |

So the acceptance condition (identical `shots.jsonl`) is **not met**. RTF=1 reached 0.65
because of the concurrent load, and "gz two-core" is the whole container on two CPUs,
not gz alone. The native log checker accepted every run (`bench ok`).

Complete hosted coverage on the final harness (s5c): all 12 `mcb_parked` cells (parked
localization p95 at most 0.090 m), `mcb_drive` and `mcb_match` pass; 0 timeouts, 0
fallbacks in every cell. The harness is the longest-holding chain throughout: over the
12-cell suite 6.65 s of wall waiting on it against 1.29 s for `mcb_batch` (64145 MCB
holds, 6415 harness holds), suite RTF about 1.0 (0.88 after the first cell). Renode
stationary-lateral on the final code: 1 passed in 1013 s (RTF 0.04), 0 timeouts.

`mcb_match` still fails its 10% enemy-hit floor about half the time, and did so
identically before this work (8.3%/7.9% on the original launch; 8.4%/7.7%/8.3% after,
against 20-22% when `opponent_1` also dies). That is the paused combat-floor item,
not a T17 effect.

## Why runs still differ

A temporary trace (`MCB_UART_TRACE=<file>` on the MCB plugin, now kept as a diagnostic)
logs the sim time and size of every Jetson read. Over the first 20 sim seconds of
the three unthrottled runs, 1472 distinct 1 ms steps saw a Jetson-to-MCB read in at
least one run, 177 in all three, and in 113 of those the bytes matched. Totals were
17000, 18000 and 16834 bytes (frame sizes 22 CV_TARGET, 89 TARGET_STATE, 17 NAV_GOAL,
39 RELOCALIZE, 10 PING). The first read already differs: 20 against 30 bytes at 1 ms.
The runs with RTF=1 and two CPUs differ from run 1 at read 1 and 0.

The cause is the open part of T17: every Jetson-side producer (`point_to_cv_target`
CV_TARGET at 40 Hz, `target_tracker` TARGET_STATE, `waypoint_nav` NAV_GOAL, `mcb_relay`
RELOCALIZE, the bridge's PING) writes the PTY from a ROS callback at wall-jittered
moments; the relay reads the PTY at the next 1 ms step, so a frame lands one or more
steps apart from run to run, and the bridge's wire-busy gate then also drops different
frames. The firmware sees a different input history and the contact dynamics amplify it.

## Remaining design (not built)

1. Bridge: in coordinated mode publish a TX ack per frame written or dropped, stamped
   with the decision stamp (stamp-less frames use `now()`), one topic per type.
2. Coordinator: allow any-of ack groups (`a|b`), so a chain can wait on either a frame's
   TX ack or the producer's "nothing to send this tick" mark.
3. Producers publish that mark on every tick: `point_to_cv_target` (its tick header
   exists), `target_tracker` (`clock_ack_topic` exists), `waypoint_nav`, `mcb_relay`.
4. Chains for POSE consumption (`/odom` and TF at the frame's capture stamp), the EKF's
   30 Hz tick, rf2o's output per scan, the detector (60 Hz) and `match_environment`
   (100 Hz) with their downstream hops, then re-measure with the trace until the read
   streams are identical, and only then run the full acceptance campaign.
5. AMCL stays off the hit path; its map-frame fields are still in `shots.jsonl`, so
   either keep them out of the identity check or move them to `poses.jsonl`.

## Other findings

- Parked localization p95 depends on head-yaw staleness: reading the idle look's TF
  0.1 s back pushed the last cell to 0.166 m (limit 0.15); 0.05 s keeps it at 0.09 m.
- `ros2 launch` of a Renode stack ignores SIGINT; `kill_launch.sh` cannot stop it
  (E-teardown). Restarting my own container cleared it.
- Epoch-zero AMCL cannot take `set_initial_pose` before any TF exists; hence
  `initial_pose_setter`.
- Concurrent load matters: RTF numbers above are with another agent's container at
  about 390% CPU.

## 2026-11-06 pass: producer chains

Mac mini (Colima, 6 CPUs), my own container (`isaac-agent-t17b`, uncapped). Another
agent's container (`isaac-agent-lifecycle`) ran at about 395% CPU throughout, so no RTF or
wall number below is a clean-machine figure, and the RTF=1 and two-core runs of the
acceptance list were not made. The Colima VM also hung once (about 01:59); runs in flight
then were discarded.

### What was built

| Step | Change |
| --- | --- |
| 1 | Bridge: `~/tx_ack/{nav_goal,cv_target,target_state,relocalize,ping}` per frame; `sim_tx_queue` holds frames until the coordinator's per-hold flush, sorts them by (type, stamp, arrival) and gates them on the hold time |
| 2 | Coordinator: any-of groups `a\|b`, counted groups `~a\|~b`, `lag_s`/`align_s`/`start_s`, `boot_hold_s`, flush topics, `topic#pkg/msg/Type` acks for messages that start with a Header |
| 3 | None-marks: `point_to_cv_target`, `target_selector`, `target_tracker`, `mcb_relay`, `waypoint_nav` |
| 4 | Chains `cv_target`, `detector`, `nav`, `ekf`, `scan`; the MCB plugin holds at each delivered POSE until `pose_consumed` sees `/odom`, the EKF input, the relay and TF at its stamp |

Things the trace showed that the plan did not expect, each fixed:

- `mmm` (launched by `auto.launch.py`) pings the MCB from a wall timer: 2-4 PING frames at 1 ms,
  different every run. Not launched in the E2E stack.
- The relay's, bridge's and tracker's own `now()` can trail the clock the producer stamped,
  so wire-busy and age gates dropped different CV_TARGETs and RELOCALIZE/TARGET_STATE payloads
  carried different ages and extrapolations. The bridge now gates on the hold time; the relay
  decides on the localization sample's stamp; the tracker runs `process_delay_s` after capture.
- Producers that fire at the same instant (CV_TARGET and NAV_GOAL at 200 ms) wrote to the PTY
  in whatever order they won; hence the queue and flush.
- The stock EKF's timer is created at its first non-zero clock and runs at 1/30 s, which is
  not a whole number of milliseconds; its tick could not be held. `boot_hold_s` fixes the first
  clock at 1 ms and the simulator runs the EKF at 34 ms (`ekf_frequency_hz`; a node-specific
  override, since `ekf.yaml`'s entry outranks a wildcard one).
- gz's lidar noise comes from an unseeded generator: the very first scan differed by 1-2 cm on
  about 130 of 720 beams. `lidar_scan` now draws it keyed by (seed, stamp, beam).
- POSE skips every 100th slot (x.x01 s) and gz's 11.2 Hz lidar runs 89 ms apart on whole
  milliseconds, so the scan chain waits 30 ms and the detector chain 25 ms.

### Results

Stationary-lateral cell, `backend:=hosted`, `duration:=10`, unthrottled, UART trace compared
run to run (matching leading lines of the Jetson-to-MCB read stream, with bytes):

| Stage of the work | Matching leading lines (3 runs) | Lines read | Runs compared |
| --- | --- | --- | --- |
| before (previous pass) | 0 | about 1470 distinct steps | `tr_h*` |
| + mmm off, TX queue, nav lag 0, EKF phase, message clock, tracker delay | 11 | 1800-1960 | c34 c35 c36 |
| + keyed lidar noise | 68 and 19 | 1900-1960 | c40 c41 c42 |

Every one of these runs passed with 0 lockstep timeouts and 0 latest-TF fallbacks. Full
coverage on the final code: all 12 `mcb_parked` cells pass (12 passed, 422 s wall for 319.6 s
sim, 95959 holds, 0 timeouts), `mcb_drive` passes (0 timeouts), `mcb_match` fails only its 10%
enemy-hit floor ("spin: low score without measured diagnosis", the known combat flake), 0
timeouts and 0 fallbacks. The coordinator's new groups are covered by two new `lockstep_suite`
tests (20 pass).

### What still differs, with the evidence

- With the lidar noise keyed, `/scan_raw` is bit-identical in two runs for the first 14.2 sim
  seconds, then differs by about 2e-7 m: once the chassis moves, the physics state diverges.
- `/odom` (the MCB's POSE) is bit-identical for the first 14 s too, then differs by 1e-5 of its
  value 11 ms after the drive starts. The harness tick that arms the drive is not the cause of
  a visible difference in the UART trace, which matches there.
- rf2o's `/scan_odom` differs at 1e-13 relative (twist x/y, covariance noise) from its first
  graded scan although its input scans and `/odom` are bit-identical, so something inside
  rf2o or its TF/odom-prior path (not the Jetson-side producers) is not bit-reproducible. That
  noise reaches the EKF, then the CV_TARGET and RELOCALIZE payloads as 1 ulp in a float, which
  is the first differing line of the trace now (451 ms to 1.63 s).
- So `shots.jsonl` is still not identical; the cause is no longer frame timing. Next: find the
  rf2o difference (compare its inputs per stamp: TF extrinsic, odom prior, scan), then the
  physics state at 14.0 s, with `MCB_UART_TRACE` and `/odom` CSVs from two runs as above.

## 2026-11-06 pass 2: rf2o, the EKF hand-off and what is left

Mac mini (Colima, 6 CPUs), my container `isaac-agent-t17c`, hosted stationary-lateral cell
(`speeds:=0 paths:=lateral duration:=10`), unthrottled. The lifecycle agent's container was
running throughout, so no wall or RTF figure here is a clean-machine number. Method: each cell
records `/odom`, `/scan_raw`, `/scan`, `/scan_odom[/quality]`, the EKF's input and output
and `/tf` in a bag next to `MCB_UART_TRACE`; two runs are compared message by message. Two
temporary traces were used and removed: a hex-float log of rf2o's per-scan inputs and state,
and an instrumented build of `robot_localization` 3.8.3 (tick time, queue size, every
measurement processed) in an overlay workspace.

### Item 1, rf2o's 1e-13: a tf2 race, not rf2o

- rf2o itself is deterministic: replaying one recorded bag (`/scan`, `/odom`, `/tf`) through a
  standalone rf2o twice gave 447/447 bit-identical `/scan_odom`. Replaying the bags of two live
  runs reproduced the live difference exactly, so the difference was in what it was fed.
- Its hex-float trace showed the first difference in the laser extrinsic (`root` to `laser`,
  from TF), 1 ulp in the quaternion product, with the scan, prior and state identical. The scan at
  1.781 s is stamped exactly on a head-TF sample. tf2 returns a sample exactly when it is the
  newest in the buffer, but when a newer one has arrived it interpolates at ratio 0, which is
  1 ulp off. rf2o retries on a 10 ms wall timer, so whether the newer sample had arrived varied.
  That also explains why only every tenth scan (the stamps that land on the 10 ms head grid) was
  ever affected.
- Fix: `extrinsic_lookahead_s` (rf2o, default 0 = unchanged robot behavior): the scan also
  waits until TF is newer than its stamp by this much. The sim sets 1 ms (a first try of
  15 ms deadlocked at 0.179 s: POSE-grid TF skips every tenth sample, so the next sample can be
  20 ms away, past the scan chain's 30 ms hold). Two new unit tests. With it rf2o's trace is
  identical for 158 scans (to 14.2 s) in clean runs and for 256+ once the EKF fix below is in.

### Items 2 and 3 were downstream of the EKF

The remaining divergence was in the EKF, not rf2o or physics:

- Its `/localization/odom` differed from the first tick that held a scan result (1.9e-6 m, then
  a steady 6e-7 m), and `nav_odom->odom` TF (EKF output against raw `/odom`) followed. The
  instrumented EKF showed the cause: the rf2o result for the scan at 357 ms was in the 375 ms
  tick in one run and the 409 ms tick in the other, because the scan chain only holds the clock
  at scan + 30 ms for rf2o's quality diagnostic, while the result still has to cross
  `scan_navigation_publisher` to the EKF and EKF ticks fall inside that window.
- Fix: `scan_odom_gate` (sim) buffers `/scan_odom`, releases it at the `ekf_pre` chain's hold
  (one step before the first EKF tick at least 30 ms after the scan), and acks only after the
  navigation adapter's output carries the stamps. Tick membership then matches between runs.
- Effect: `/odom` and `/scan` were bit-identical for 2336 and 292 messages (about 26 s of sim)
  instead of about 14 s. The 14 s "physics divergence" was therefore the MCB receiving
  different RELOCALIZE/CV_TARGET inputs (item 2), not a gz difference; no physics setting
  was changed. gz thread count, solver and contact settings were not touched.

| Stage | Leading UART lines equal | rf2o trace equal | `/odom` equal to |
| --- | --- | --- | --- |
| before (a1, a2) | 19 | 18 messages | 14 s |
| + lookahead (d1, d2) | 19 | 158 scans | 14 s |
| + gate (g1, g2) | 807 (to 20.095 s) | 290 scans (to 25.9 s) | 26 s |

### What still differs (closed by pass 3, below)

- The first UART difference is now a TARGET_STATE frame at 20.095 s, and `shots.jsonl` is
  still not identical (99 and 97 records in g1 and g2).
- The cause is equal-stamp ordering inside the stock EKF. `ekf_input` becomes two measurements
  per message (pose yaw and twist) with the same stamp; `robot_localization`'s queue breaks ties
  by heap shape, which depends on the order in which `odom0` and `odom1` messages were pushed.
  40 of the 41 ticks that differ between g1 and g2 carry a scan result, about 9% of those ticks. The
  two topics arrive over different transports to a single-threaded executor on a loaded host, so
  the push order is a wall-time race. Effect: 2e-16 in yaw, which reaches RELOCALIZE and
  TARGET_STATE as ulps.
- Closing it needs the EKF to say when it has consumed a message (a sim-only patch of
  `robot_localization` with a received-stamp topic), or distinct stamps for pose and twist
  (a split `ekf_input` with a node-specific EKF config, a robot-side interface change). Both
  are larger than a determinism fix and are not done. Everything that is gated before the
  EKF (UART, rf2o, TF, tick membership) is deterministic now.
- Not done: the acceptance campaign (three unthrottled, one RTF=1, one gz two-core run). The
  lifecycle agent's container was up for the whole session, so the RTF=1 and two-core runs were
  not taken either.

### Regression on the final code

All 12 `mcb_parked` cells passed (375.9 s, 102482 holds, 0 lockstep timeouts, 0 latest-TF
fallbacks, native checker `bench ok`); `mcb_drive` passed (0 timeouts, 0 fallbacks, `bench ok`);
`mcb_match` ran with 0 timeouts and 0 fallbacks and failed only its known 10% enemy-hit floor
(8.3%, "spin: low score without measured diagnosis"), not tuned. One of the seven live runs with the
instrumented EKF overlay (e2) died at 14 ms of sim time with "UART acknowledgment or ROS clock
timeout: unconsumed_poses=1"; it did not recur in the five runs after it or in the 14 cells of the regression above, and the
cause is not found. Startup races of that kind should be watched in the acceptance campaign.

## 2026-11-06 pass 3: the remaining timing races

Mac mini (Colima, 6 CPUs), my container `isaac-agent-t17rl`; the E-residual agent's container
(`isaac-agent-lifecycle-resid`) was up throughout, doing light work, so no figure here is a clean-machine
one. Method: a script runs a cell or stage N times, each with `MCB_UART_TRACE`, optionally a bag
(`/odom`, `/scan*`, `/localization/*`, `/tf`, `/joint_states`, the detector, tracker and bridge topics,
the TX acks and flushes) and temporary traces (removed). Runs are compared by the first differing
line of the UART trace, then by the first differing input topic *in sim time* (message index is
useless once repeats differ), then by the content of that input. Each fix below was found that way and
changed the first-divergence point of the next batch.

| # | Cause (evidence) | Fix |
| --- | --- | --- |
| 1 | EKF ties and tick membership: with the fork's tie-break alone, `/localization/odom` still differed from identical inputs (a measurement queued after the tick fired) | `deterministic_measurement_order` and `publish_queued_stamps` in the [fork](../../robot_localization/README.md#thornbots-fork); `pose_consumed` and `scan_odom_gate` wait for the queued stamp |
| 2 | The `detector` chain released a tick on the previous frame's TX ack, or on a later empty frame's none-mark, before this frame's TARGET_STATE was queued (the frame then went out at a later hold, or was dropped by the wire-busy gate) | `@hold` / `@tick` exact-stamp ack groups in the coordinator; `target_tracker` queues empty frames behind earlier ones |
| 3 | `target_driver` stamped and integrated with whatever clock the callback saw (dt and stamp 1 ms off) | scheduled tick times; `match_environment` likewise |
| 4 | `OpponentMover` used the newest path sample that had arrived, sample k or k+1 at a tick step (1 ulp in parked, mm in `mcb_drive`); the first step also started at a timing-dependent step; two bimodal values of the opponent's y (1.3 m, differing by 4e-9) came from that start | waits for the tick's sample and uses it; waits for the first sample; senders resend while the clock stands still (a sample the gz side missed would stall physics) |
| 5 | Cell changes: `set_target` took effect at whichever `target_driver` tick followed the parameter callback; `match_environment` `active` likewise | `effective_s` / `start_time_s` carry the harness tick the change was made at |
| 6 | The harness decided windows with `now_s()` (the clock this node had seen), so a cell started one whole second apart in different runs (`align(1.0)` skipped a tick), ended a tick apart, and `check_lockstep` stepped the clock while waiting for a wall-timed status | `tick_s()` and `run_to` count chain ticks; the status wait does not step; the parked ingress loop is tick-based; 1 s of stepping past the window lets in-window shots and states land, and records beyond it are dropped |
| 7 | AMCL (not gated, off the hit path) put map-frame errors in `shots.jsonl` and `states.jsonl` | those fields are only in `poses.jsonl` |
| 8 | Tracker `lookupTransform` at a query that falls on a TF sample slerps at ratio 0 once a newer sample has arrived (rf2o's race again) | `tf_lookahead_s` 1 ms in the E2E stack |
| 9 | `waypoint_nav` missed the first referee sample when its best-effort subscription had not matched yet, so its first goal came one tick later in 1 run of 8 | `epoch_zero_ready`: ready only once `/dji_serial_bridge/ref_sys` has a publisher |
| 10 | `mcb_match`: `match_environment` took the gz truth and the sentry's odometry as they had arrived, and the opponents' heads were turned by gz PID controllers fed over gz transport, whose reaction moved the chassis | stamped snapshots at fixed offsets; head angles ride in the path message and `OpponentMover` sets the joints (position and zero velocity); the PID plugins are removed from those models when `path_hz` is set |
| 11 | A run started within seconds of another timed out at its first hold ("gz not held at 0.010 s after 2.0 s wall", 4 of about 30 stage switches): the last run's gz-transport advertisements were still listed | `run_suite.sh` waits for `gz topic -l` to empty (limit unchanged) |

Things that were wrong in the hypotheses: the AMCL map-frame error was never the cause of a
`TARGET_STATE` difference; gz truth was identical in every run pair where it was dumped; the
opponents' y-values being bimodal looked like a physics start-up effect and was the mover.

### Results

All runs hosted, unthrottled, `real_time_factor:=0`, lockstep timeouts 0, latest-TF fallbacks 0, native
checker `bench ok` (`mcb_match` exits non-zero only on its known combat floor, 7.9% against 10%, and
is otherwise accepted by the checker).

| Batch | Stage | Runs | `shots.jsonl` / `states.jsonl` | UART read stream |
| --- | --- | --- | --- | --- |
| acceptance (cy1-3) | all 12 `mcb_parked` cells | 3 | identical, 1883 / 14468 records | equal to the shorter run's end |
| | `mcb_drive` | 3 | identical, 109 / 1148 | equal |
| | `mcb_match` | 3 | identical, 329 / 1109 | equal |
| stage-switch loop | `mcb_drive`, `mcb_match` alternating | 16 | identical to the first of each | |

### RTF=1 and gz two-core acceptance (2026-11-07)

Fresh agent container on the aarch64 Mac mini (Colima, 6 CPUs), the only container on the VM; workspace `nightly`
`0ec6189d` with every submodule at its recorded commit. Hosted firmware `ca02cfc5`. Per stage, one unthrottled baseline
(`real_time_factor:=0`), one `real_time_factor:=1` run and one run with `taskset -a -cp 0,1` re-applied every 0.3 s to
every `gz sim` process (all gz threads read back as affinity `0,1`), each compared with `compare_runs` against the baseline.

| Stage | Run | Suite RTF | Lockstep timeouts | Latest-TF fallbacks | `shots.jsonl` | `states.jsonl` | Native checker |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `mcb_parked` (12 cells) | unthrottled | 1.10 | 0 | 0 | reference, 1883 | reference, 14468 | ok |
| | RTF=1 | 0.80 | 0 | 0 | identical | identical | ok |
| | gz two cores | 1.10 | 0 | 0 | identical | identical | ok |
| `mcb_drive` | unthrottled | 1.13 | 0 | 0 | reference, 109 | reference, 1148 | ok |
| | RTF=1 | 0.82 | 0 | 0 | identical | identical | ok |
| | gz two cores | 1.11 | 0 | 0 | identical | identical | ok |
| `mcb_match` | unthrottled | 1.06 | 0 | 0 | reference, 329 | reference, 1109 | combat floor only |
| | RTF=1 | 0.83 | 0 | 0 | identical | identical | combat floor only |
| | gz two cores | 1.05 | 0 | 0 | identical | identical | combat floor only |

Timeouts are the coordinator's final line; fallbacks are the count of latest-TF warnings in each `stack.log`.
Three unthrottled runs per stage were taken earlier (table above). Notes:

- `real_time_factor:=1` realises 0.80 to 0.83 on this machine (the throttle adds its own wall wait), below the
  unthrottled 1.05 to 1.13. The run is still a throttled one; the order of events was identical.
- Two cores left RTF unchanged: lockstep holds gz while the other chains run, so gz is not CPU-bound on six cores.
  The pinning was verified, not assumed: 49 of 49 gz tasks sampled mid-run had affinity `0,1`. A repeat of
  `mcb_drive` (baseline 2 and pinned 2) gave the same `shots`/`states` again.
- `compare_runs` exits 1 for the throttled pairs only because `poses.jsonl` (`mcb_parked`) and `route.jsonl`
  (`mcb_drive`, `mcb_match`) differ in AMCL map-frame fields (pass 3, finding 7: off the hit path, not gated). Two
  plain unthrottled `mcb_drive` runs differ in `route.jsonl` by the same amount (`localization_error_m` 0.0141 against
  0.0153), so that difference is AMCL run-to-run noise, not an effect of throttling.
- Faults of the run itself, not of the harness: a fresh container needs `firmware/MCBV3` (and its `taproot-scripts`),
  `scons g++-14 gcc-arm-none-eabi` plus `python3-jinja2 python3-pyelftools`, and the `robot_localization` fork built in the
  workspace; with the submodules left at an older checkout the stack stalled at sim 0.0 s ("waiting for /ekf_filter_node",
  `NO DATA from MCB`), which looks like a harness deadlock but is a stale `sentry_localization`/`ros2_dji_serial_bridge`.
  Update every submodule (`git submodule update --init`) before building.

Raw per-run directories (logs, `shots`/`states`/`poses`/`route` jsonl, UART traces) were copied out of the container to
the host's `/tmp/c-accept/t17`. Commands as above: `run_suite.sh <stage> backend:=hosted log_dir:=<dir>` with
`MCB_UART_TRACE=<dir>/uart.trace`, then `ros2 run sim compare_runs <dir A> <dir B>`.

### Renode bring-up regression found by the lifecycle repeats (2026-11-07)

`BACKEND=renode tools/lifecycle_check.sh --repeat 4` (board ELF `build/hardware/scons-release`, Renode 1.17.0, same
container as above, nothing else on the VM) failed from its first run: every cell dies at the first release with
`lockstep timeout: gz not held at 0.020 s after 2.0 s wall` (4 holds, `NO DATA from MCB` after the first 11 bytes),
`FAIL: 1 unclean-shutdown log line` and, for interrupts, `suite ended before the signal`. Standalone,
`run_suite.sh mcb_parked backend:=renode firmware_binary:=<board ELF> speeds:=0 paths:=lateral duration:=10 headless:=true`
fails the same way in 3 of 3 runs (about 90 s each: ~85 s board boot, then the timeout), at low load (VM CPU 79% idle,
Renode at 110% of one core), so it is not a timing margin.

Bisect of `sim` only (everything else at the nightly gitlinks, same build flags), the same cell, passing means
gz stepping past 0.020 s with 0 timeouts (stopped by hand after 24 to 27 sim seconds, not run to completion):

| `sim` commit | Result |
| --- | --- |
| `57bccb9` (#91, before #92) | passes, 26.9 s sim in 523 s wall |
| `b934372` (#92, fork deterministic order) | passes |
| `9360b27` (#92, "identical hosted runs of every E2E stage") | **fails** at 0.020 s |
| `ca1b3ef` (nightly) | **fails** at 0.020 s |

So the break is `9360b27`. Hosted stays correct. The change that is the likeliest cause, not yet shown: with `producer_chains`
the opponent heads move to `OpponentMover`, which waits up to 5 s wall in `PreUpdate` for the path sample of each tick
(`path_hz` 60 or 100); under Renode gz spends the hold in the board's step, so the clock that releases the
`target_driver`/`match_environment` sample is behind the step that needs it. Nothing here confirms that; the next
check is gz's `PreUpdate` wait (a backtrace of the stuck `gz sim -s` at the timeout). The earlier Renode passes
(1 stationary-lateral cell in 1013 s; the 20 of 20 matrix) predate `9360b27`.

Effect: the Renode repeats 2 to 5 of E-teardown cannot be taken on nightly, and the T17 harness is unvalidated on the board
backend. Per-run logs: the host's `/tmp/c-accept/renode/lc` (`001-` to `003-` of the lifecycle run, `diag1`, `bis1` to `bis3`).


#### Cause and fix (2026-11-07)

The suspect was right (`OpponentMover`), the mechanism was not the 5 s wait for a late sample. The gz process cannot be
attached in an agent container (no `CAP_SYS_PTRACE`), so a `fprintf` in the mover showed the wait instead:
`mover wait tick=0 before=16000000 have=0 newest=17000000`, then `wait done ok=0` 5 s later.

- `target_driver` (60 Hz, `deterministic_epoch_zero`) stamps its first sample with its first tick, `ceil(1000/60)` = 17 ms,
  while the clock is still at zero, and its 50 ms wall resend repeats it for as long as the clock stands still.
- The mover (`path_hz` set) picks the newest tick at or before the step's start (`before`) and waits for a sample with
  exactly that stamp. For `before` below 17 ms that tick is 0, and no producer ever makes a tick-0 sample.
- Hosted boots in seconds: the path bridge connects after the first release, `path_` is still empty when the step at
  17 ms starts, the first branch waits for it and the tick then is 17 ms. Under Renode the 85 s board boot gives the bridge
  all the time it needs, so the 17 ms sample is already there during the steps before 17 ms and the second wait fires with
  tick 0. gz sits in `PreUpdate` for 5 s wall, longer than the coordinator's 2 s `max_wait`. Hence the exact stall at the
  hold for 0.020 s on every Renode stage, and why only `9360b27` (which added the tick wait) broke it.

Fix (`src/opponent_mover.cpp`): wait for a tick only from the producer's first stamp on (`first_stamp_`, the earliest ever
received), and with `path_hz` never fall back to the newest sample (it is ahead of the clock). Before the first tick the
model holds still, which is what the hosted run already did. The 5 s waits and every lockstep timeout are unchanged.

Checks on the fixed code (aarch64 Mac mini, one container, firmware `ca02cfc5`, Renode 1.17.0):

- Renode, `run_suite.sh mcb_parked backend:=renode ... speeds:=0 paths:=lateral duration:=10`: 1 passed in 736 s, 40.5 s
  sim, 12,981 holds, 0 lockstep timeouts, `bench ok` (before: dies at 0.020 s after about 90 s, 3 of 3).
- Hosted, two unthrottled runs per stage, `compare_runs` between them:

| Stage | Runs | `shots.jsonl` | `states.jsonl` | Lockstep timeouts | Latest-TF fallbacks | Native checker |
| --- | --- | --- | --- | --- | --- | --- |
| `mcb_parked` (12 cells) | 2 | identical, 1883 | identical, 14468 | 0 | 0 | ok |
| `mcb_drive` | 2 | identical, 109 | identical, 1148 | 0 | 0 | ok |
| `mcb_match` | 2 | identical, 329 | identical, 1109 | 0 | 0 | combat floor only |

`compare_runs` exits 1 for all three only through `poses.jsonl` (`mcb_parked`) and `route.jsonl` (`mcb_drive`,
`mcb_match`), the AMCL map-frame fields that are off the hit path and not gated (see above). The counts equal the
earlier accepted references.
