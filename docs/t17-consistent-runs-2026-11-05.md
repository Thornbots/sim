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
