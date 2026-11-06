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
