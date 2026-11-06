# Lifecycle checkpoint — 2026-11-05

Mac mini, isolated agent container, workspace `3d299273`, sim `b6d617c7`,
firmware `ca02cfc5`, hosted backend. The E2E wrapper's Gazebo preflight was
extended to all three MCB stages; each returned 1 with `gz` absent from PATH.
Host quality checked 241 files with no diagnostics; actionlint passed.
The seven-package build passed, and registered sim tests reported 245 tests,
zero errors, failures or skips with both firmware environment variables set.

One default tracking run passed 12/12 cells and the native log checker; its
scored total RTF was 18.79. One shortened parked diagnostic (`speeds:=0`,
`paths:=lateral`, `duration:=10`) passed 1/1 and both outer and stack native
log checks. This diagnostic is not the lifecycle acceptance matrix. Immediate
post-run process snapshots showed no installed nodes, Gazebo or launch process.

During the parked shutdown the stack log reported that the odometry bridge's
child status had already been read and assigned exit 255; the native checker
accepted it with an unclean-shutdown note. During the run the firmware process
was in a different process group from the Gazebo launch, though in its session.
Neither observation establishes a fix for orphaned descendants or DDS races.
The previously recorded scan-bridge core at `/Users/blaise/ros2_ws/src/sim/core`
was absent, and no core of its recorded size was found under `ros2_ws`; a
matching-library scan-bridge backtrace remains open. Raw logs are retained in
the task worktree at `artifacts/lifecycle-2026-11-05/`.

A second shortened parked diagnostic tested an uncommitted change sending
SIGINT only to the launch leader, avoiding the group signal before launch
forwards it. The cell passed, but launch exited with group members still
present and `LaunchTree` sent SIGKILL. The experiment was reverted; it does
not satisfy graceful cleanup. Immediate process checks after the fallback
found no installed nodes, Gazebo or launch process.

## Second pass — causes found and fixed

Mac mini, isolated container `isaac-agent-lifecycle`, aarch64 Ubuntu 24.04,
Jazzy, Fast DDS 2.14.6, hosted firmware `ca02cfc5`. Method: shut the stack down
repeatedly with cores and `gdb` enabled, then change one thing at a time.

**Process layout.** The firmware process is spawned by the gz plugin with its own
process group on purpose (a group SIGINT must not kill a tick mid-read) and exits
when gz closes its stdin; it is not an orphan source. The orphan source was gz: a
single SIGINT to the launch leader left `gz sim -s` (ppid 1) and the firmware
running, because `ros_gz_sim` starts gz as `sh -c "ruby gz sim ..."` and launch
signals only the shell. The old group-wide `LaunchTree` signal hid this and
delivered SIGINT twice to every other node.

**Double signal.** 12 group-signal shutdowns: 216 node exits of -2 (the checker
whitelisted -2), plus one SIGSEGV, two SIGABRT and one exit 255 (the 255 earlier
read as an unclean odometry bridge was `dji_serial_bridge_node`). The first-pass
experiment of signalling only the launch leader failed for the gz reason above,
not because the idea was wrong. Leader-only SIGINT with gz as the launch's child
exits every node with 0 in about 1 s.

**Scan/odometry-bridge backtrace** (matching aarch64 libraries, `libfastrtps.so.2.14`;
[full text](lifecycle-evidence-2026-11-05/odometry-bridge-sigsegv-aarch64.txt)).
`parameter_bridge` died with SIGSEGV in `eprosima::fastrtps::rtps::PDP::~PDP()`
on the main thread: `exit` → `DomainParticipantFactory::~DomainParticipantFactory`
→ `RTPSDomainImpl::removeRTPSParticipant`. Concurrently thread 6 was in
`rclcpp::SignalHandler::deferred_signal_handler` → `Context::shutdown` →
`rcl_logging_rosout_fini` → `destroy_publisher`, blocked on a publisher mutex,
and thread 7 was still in `__rmw_wait`. Other aborts in the same runs were glibc
`pthread_mutex_lock` and `__pthread_tpp_change_priority` assertions on destroyed
mutexes, in the cmd_vel, scan, path and joint bridges. `nm` shows `parameter_bridge`
imports `rclcpp::spin` but not `rclcpp::shutdown`: it returns from `main` while the
signal-handler thread is still shutting the context down. This is the same race as
the x86 core in [shutdown findings](mcb-hosted-check-2026-10-11.md#shutdown-findings).
With leader-only SIGINT it still hit 4 of 24 idle-stack shutdowns (SIGABRT) with
`parameter_bridge`; `gz_bridge` (same arguments, plus `rclcpp::shutdown()`) had 0
bridge deaths in 64 shutdowns (24, then 40 with half under 8 CPU hogs), against
nodes of ours and third parties that already called shutdown, none of which died.

**Other faults found by the lifecycle check:** a `head_slider_relay` deadlock (pipes
without CLOEXEC let the other relay's echo child hold this reader's write end, and
`stop()` joined one relay before signalling the other; the stack outlived a SIGKILLed
shell until launch's own SIGKILL), and `McbFirmwareSystem::PostUpdate` throwing on a
UART-acknowledgment timeout while the bridge was shutting down (gz exit -6, pty link
left; appeared when T17 made coordinated UART the default).

**Open:** two gz server SIGKILLs from launch's escalation among 88 idle-stack leader
shutdowns (one with `parameter_bridge`, one with `gz_bridge`); neither reproduced in
the 40 instrumented shutdowns, there is no stack from either, and the cause is unknown.
Candidates: the lockstep gate's 5 s `max_wait_s` after the coordinator has exited,
and host load (load average 16 on 12 cores at the time).

### Acceptance runs

`tools/lifecycle_check.sh` ends `mcb_parked`, `mcb_drive`, `mcb_match` and the
default estimation bench (`tracking`) by completion, SIGINT to the shell's group,
SIGTERM, SIGHUP to the group and SIGKILL of the shell, and asserts the shell or
stack is gone within 60 s, no process, gz topic, pty link or temporary world is left
within 10 s, and no log shows SIGKILL/SIGTERM escalation, a died process other than
the interrupted suite, or an unclean-exit note.

- Hosted backend, sim `b6bca84` on nightly `08c79ae`: **60 of 60 runs passed**
  (3 repeats of 20). Stack gone 1 s after every signal. Two `mcb_match`
  completions failed the suite's own combat score gate (hit rate below 0.1 with no
  diagnosis), which is the match track's gate and was not loosened.
- Before the rebase onto T17 and the gz_bridge change, intermediate matrices passed
  parked, drive, match and tracking endings once each and found the relay deadlock.

**Final hosted matrix** (sim `48b6f5f` on nightly with T17's UART and epoch-zero defaults,
workspace dependency pins current): **100 of 100 runs passed** (5 repeats of 20:
`mcb_parked`, `mcb_drive`, `mcb_match` and the default `tracking` sweep, each by
completion, SIGINT, SIGTERM, SIGHUP and SIGKILL of the shell). Every interrupted run
ended 1 s after the signal (`tracking` 6 s total) with no leftover process, gz topic,
pty link or temporary world, and no SIGKILL, SIGTERM escalation, died process or
unclean-exit note. Fault-finding runs before this one are listed above.

Two of the 100 completions (runs 11 and 91, both `mcb_match:complete`) ended with the
suite's own verdict failed. This is the known match combat-floor variance, not a
lifecycle fault: both fail `e2e_suite.cpp:410` ("spin: low score without measured
diagnosis", spin-segment hit rate 0.084 and 0.079 against the 0.1 floor, no
diagnosis), while the other three `mcb_match` completions passed. In both runs the run
reached its summary line, and every shutdown assertion passed. The score gate was not
loosened; it belongs to the [match track](../../docs/ROADMAP.md#a-the-match-test).

## Renode backend and `kill_launch.sh`

A first Renode matrix (sim `48b6f5f`) failed its first run, and every later run then
failed the precheck because that run left `/tmp/mcb_emulator_pty-*`. The pty was left by
gz being SIGKILLed, not by Renode: after a completed `mcb_parked` run gz ignored
launch's SIGINT and SIGTERM for 15 s.

- **Gate wait.** `LockstepGate::PostUpdate` waits for the coordinator's release for up
  to `max_wait_s` (5 s, or 30 s with epoch-zero startup) and did not look for a stop
  request. The coordinator exits with the rest of the stack, so a gate that was
  holding at that moment sat out the wait. Under Renode gz is held 86% of the wall
  time (552 s of 639 s in the run), so this hit every run; hosted runs are held far
  less, which is the unexplained gz SIGKILL seen about once in 50 idle shutdowns
  earlier. The gate now waits in 10 ms slices and emits Stop on SIGINT/SIGTERM,
  seen through `gz::common::SignalHandler`.
- **Firmware waits.** The plugin's wait for the board's "MCB3" greeting is a 600 s
  `poll`; a Ctrl-C during the roughly 100 s board boot could not interrupt it. Reads now
  poll in 100 ms slices and end on the same stop request.
- **Direct signals to descendants (the T17 agent's report).** `kill_launch.sh`
  pidfd-signals every descendant of the launch, not the group. That SIGINT killed the
  firmware adapter `mcb_type_c`, which sits in its own process group on purpose, and
  gz aborted on "MCB firmware exited (-2)". `mcb_type_c` now ends its loop and tears
  down on SIGINT/SIGTERM, and gz treats a firmware that died of SIGINT/SIGTERM as a
  stop request. A throw there still aborts for any other exit.
- **Renode ignores SIGINT.** Renode exits within 1 s on SIGINT or SIGTERM while idle,
  or with a connected idle client (measured), but a SIGINT that arrives while it is
  executing a command, as `kill_launch.sh` or a dying owner's parent-death signal
  deliver it, leaves it wedged in a futex wait that ignores SIGINT and SIGTERM. Its
  .NET runtime also replaces an inherited SIG_IGN with its own handler (`SigCgt` bit 2).
  `LaunchOptions::emulator()` therefore blocks SIGINT in Renode's signal mask, which
  the runtime does not override, stops it with SIGTERM after the control client has
  disconnected, and uses SIGKILL as its parent-death signal: an emulator holds no
  state to save, and a SIGINT there is what wedges it.

**Standalone launch stop test** (`mcb.launch.py` started with `dexec.sh -d`, stopped with
`kill_launch.sh <pid>`, 40 s or 75 s after start): before the fixes gz aborted (hosted
firmware path) or needed SIGKILL (Renode) and an orphaned Renode survived; after,
3 of 3 Renode runs at 75 s, 3 of 3 at 15 s (during boot calibration) and 3 of 3
hosted runs left no process, pty link or world, with no abort and no SIGKILL. Nodes
that receive both `kill_launch.sh`'s direct SIGINT and launch's forwarded one still show
exit -2: that script's per-descendant design delivers two signals, so use the
run_suite/`LaunchTree` path when exit codes matter.

**Two log observations in the interrupted Renode drive/match runs.**
`spawn_to_center` FAILED about 4 s after `[ RUN ]` with "E2E ROS context stopped during
scoring": that is the signal's effect (the log shows `[ RUN ]`, then launch's `user
interrupted with ctrl-c`, then the exception within the same second), not an earlier
failure. `lifecycle_manager_localization: CRITICAL FAILURE: SERVER map_server IS DOWN
after not receiving a heartbeat` appears in one log, about 50 ms after
`map_server: Deactivating / Destroying bond` during the same shutdown: Nav2's manager
noticing the map server that its own process just deactivated. No heartbeat message
precedes the interrupt, so it is teardown order inside `sentry_localization`'s process
(a log line; the process exits 0), not an RTF-induced heartbeat loss.

## Startup-time interrupts, the board adapter and final results

Interrupting while the stack is still starting (the common Ctrl-C) found four more faults,
each reproduced with cores enabled before it was changed:

- **`seeded_ekf`** (sentry_localization): `KeyboardInterrupt` escaped `rclpy.spin_once` while
  waiting for the seed (exit -2), and `robot_localization`'s `ekf_node`, stopped while it still
  logs "Waiting for clock to start...", leaves the loop, creates a publisher on the shut-down
  context and aborts with `RCLError ... context is invalid` (core;
  [backtrace](lifecycle-evidence-2026-11-05/ekf-node-startup-abort-aarch64.txt)). That is a
  `robot_localization` bug; the wrapper now supervises the child, forwards stop signals, and
  absorbs that one abort during a requested stop with its core suppressed
  ([sentry_localization #42](https://github.com/Thornbots/sentry_localization/pull/42)):
  12 of 12 stops exit 0 on each path, against exit -2 and -6 before.
- **`gz_bridge`**: `rclcpp::shutdown()` ran after the node had left scope, so the participant
  was destroyed under the signal-handler thread (heap corruption in `~DomainParticipantImpl`,
  2 aborts in 20 early interrupts); it now runs before the node is destroyed.
- **Board adapter.** The power-on guard is one ~11 s uninterruptible Renode call. gz waited 20 s
  for the adapter after a boot-time Ctrl-C, longer than launch's SIGINT plus SIGTERM budget,
  so launch SIGKILLed gz and the adapter and Renode outlived it. The adapter now exits on
  SIGINT/SIGTERM, gz signals it when closing the firmware, and a clean exit counts as a stop.
- **`robot_spawn_ready`** exited 1 when stopped before the spawn was verified; it now exits 0.

### Open

- **Fast DDS receive-thread SIGSEGV in a `gz_bridge`, at a boot-time interrupt.** 1 of 20
  `mcb_drive` boot-time SIGINTs (and one in each of two earlier matrices) killed a bridge
  (`scan`, `opponent_path`) with SIGSEGV. The core's crashing thread is Fast DDS's own UDP
  receive thread: `EDPSimpleSUBListener::onNewCacheChangeAdded` → `PDP::removeReaderProxyData` →
  `StatefulWriter::matched_reader_remove`
  ([backtrace](lifecycle-evidence-2026-11-05/gz-bridge-fastdds-receive-thread-sigsegv-aarch64.txt)),
  handling a remote reader's removal while the bridge's own endpoints are being torn down. It is
  inside libfastrtps 2.14.6, not in sim code, and did not reproduce in 40 drive-stage shutdowns
  from the leader-only hunt. It is not absorbed or whitelisted: the lifecycle check fails on it.
  **E-teardown therefore stays open.**
- The same Fast DDS race also killed a `gz_bridge` in one of 20 merged-code hosted runs (`mcb_parked:int`, 3 s into the test), so it is not confined to boot-time interrupts.
- ~2 gz server SIGKILLs among the first 88 idle shutdowns were attributed to the lockstep gate
  wait fixed above; none occurred in the 40 + 40 + 30 shutdowns after it.
- A bring-up flake (`localized map->root chain` not ready within 30 s under a host load average
  of about 18) failed 1 of 60 hosted runs before the signal; its shutdown was clean.

### Results on the final code (aarch64 Mac mini, hosted and Renode board backends)

| Check | Result |
| --- | --- |
| Unit tests (`test_suite_exit`, `lockstep_suite`) | 28 and 18 passed |
| Hosted matrix, 100 runs (4 suites x 5 endings x 5), sim `48b6f5f` on nightly with T17 UART and epoch zero | 100 / 100 |
| Hosted matrix, 60 runs after T17's harness changes | 57 / 60: 2 harness false positives from a concurrent `docker exec` (fixed), 1 bring-up flake above |
| Hosted regression, 20 runs, final binaries | 20 / 20 (1 suite verdict failed: match combat floor) |
| Renode matrix (parked complete/int/kill, drive int/kill, match term/kill), twice | 7 / 7 and 7 / 7 |
| Boot-time interrupts, Renode, 10 runs, then hosted 14 runs (after the adapter fix) | 9 / 10 and 13 / 14 (the SIGSEGV above, both in `mcb_drive`), plus 19 / 20 in the cored repeat |
| `kill_launch.sh` stop of a standalone `mcb.launch.py` (Renode at 75 s and 15 s, hosted) | 9 / 9 |
| On the merged code (T17 producer chains and rf2o gate, sentry_localization #42, final sim): hosted, 40 runs after the last fix | 40 / 40 |
| The same merged code, before the per-test `rclcpp::ok()` assertion and harness fix | hosted 17 / 20 (a `gz_bridge` SIGSEGV from the open race, a `docker exec` false positive, the estimation suite needing SIGTERM); Renode matrix 7 / 7; `seeded_ekf` stops 12 / 12 on both paths |
| Leader-SIGINT shutdown hunts: with `gz_bridge`, 24 then 40 (half under CPU load); after the startup fixes, 30 early (8 s) and 40 on the drive stage | 24: 1 gz SIGKILL (the gate wait, fixed afterwards); 40, 30 and 40: no deaths, no cores |

The `lifecycle_check.sh` rows above are 308 runs (100 + 60 + 20 + 14 + 10 + 7 + 7 + 20 + 3 + 20 + 7 + 40), plus the 9
`kill_launch.sh` stops. The harness found the relay deadlock, the plugin abort, the gate wait, the adapter orphan
and the estimation-suite exit; each was fixed at its cause. Counts are runs, not rates; the 20-run repeat and
the boot-time rows are the evidence for the open race, not for its absence.

## Independent review of sim #68 (estimation reset and pacing)

Reviewed from the merged diff by a different agent than the author, then exercised by the lifecycle matrix.

- **Reset ordering.** `start_case()` enables pacing (`/bench/pacing` true, which clears the world's
  `progress_`) and only then sends the `bench_world` reset. Sim time in `bench_world` only moves
  forward (`case_t0_ = t_`), so a progress sample from the previous cell can only be older than the
  world, never ahead of it: the world waits for the scorer rather than free-running, and no stale high
  stamp can hold the pacing gate open for a cell. Clearing the sampler before the request keeps
  `/bench/case` whichever order the response and the callback arrive in. No defect found.
- **Handoff.** An inactive `/bench/pacing` makes the world ignore delayed `/bench/progress` samples, and
  `PacingGuard` pauses on every exit path, including failures. A missing or rejected handoff throws
  and fails the run. The `no_pacer` read of `progress_` under the mutex fixed a data race.
- **Minor, not changed.** `set_pacing()` creates a service client per call (cheap, destroyed on
  return); between cells with no pacer the world sleeps one step per iteration, which is outside the
  scored span.
- **Interrupt behaviour** was the one real gap: after SIGINT every remaining cell ran to its own
  failure (`pacing cleanup: could not create client`), outlasting launch's 5 s SIGINT window; fixed
  here with an `rclcpp::ok()` assertion at the top of each test body.
- **Evidence.** The default tracking sweep (12 cells, native log checker accepted) completed 5 times in
  the 100-run hosted matrix and was interrupted by SIGINT, SIGTERM, SIGHUP and a killed shell 5 times each
  in it, with a clean exit every time; none of those completed sweeps showed a pacing timeout or a
  lockstep timeout (the later matrices ran the same tracking cases, with one SIGTERM-escalation failure
  fixed above).

This review covers #68 only. RTF and D-speed acceptance are not part of it.

## 2026-11-06: the Fast DDS bridge race and the RTF gate

Mac mini (aarch64), isolated container, Jazzy, `libfastrtps` 2.14.6, `ros_gz_bridge` 1.0.24.

### Upstream check

- Apt offers `ros-jazzy-fastrtps` 2.14.7 (2026-09-11) and no later 2.14.x; 2.14 is the last v2 minor.
  The 2.14.6 to 2.14.7 diff (25 commits) touches no code in `StatefulWriter`, `RTPSParticipantImpl`
  endpoint removal or `EDP::unpairReaderProxy`. Its relevant fixes are `RTPSParticipantAttributes` data
  races (#6370) and thread-safe `on_xxx_matched` (#6371/#6382, listener pointer re-reads in
  `DataWriterImpl`/`DataReaderImpl`). It is not a fix for this crash, and no issue or PR names
  `StatefulWriter::matched_reader_remove`. The nearest open report is
  [Fast-DDS #3303](https://github.com/eProsima/Fast-DDS/issues/3303) (a discovery thread reaching an
  entity another thread is deleting, listener-pointer variant, open since 2023). So the image was not
  changed and nothing was filed upstream: the cores below show the process destroys the participant
  against a thread that is still running, which is ours to avoid.
- `unpairReaderProxy` takes `endpoints_list_mutex` shared and `deleteUserEndpoint` takes it exclusive, so
  the receive thread cannot reach a writer that is being deleted. The writer was already gone because the
  participant was being destroyed.

### Reproducer and what the cores show

`tools/bridge_teardown_stress.py` (first run as a scratch variant with four sim-style bridges and three rclpy
peers) interrupts everything back to back after a random delay, as launch does. Stock `gz_bridge`,
delay 0.2 to 4 s: **3 of 150 iterations** killed a bridge (SIGABRT "pure virtual method called" twice,
SIGSEGV once). Cores (GDB) show `main` inside `exit` to `~DomainParticipantFactory` to
`~DomainParticipantImpl` (deleting a `DataReader`, joining the shared-memory or resource-event threads)
while a second thread, a plain `std::thread` in `librmw_fastrtps_shared_cpp`, was inside
`rmw_fastrtps_shared_cpp::_take` to `DataReaderImpl::take` on the reader being deleted (rmw's
graph-listener thread, which takes from `ros_discovery_info`). The listener thread is joined only by
`rcl_context_fini`, which never ran.

Why it never ran: `context.use_count()` was 3 at exit, and the `RosGzBridge` node was still referenced
(use_count 2 to 3) after `spin()`, `remove_node` and releasing our pointer. The node is referenced from its
own handles, services and heartbeat timer, so it is never destroyed; neither does stock
`parameter_bridge` destroy it. The global context therefore lives until static destruction, where Fast DDS's
singleton destroys the participant under rmw's live thread. The original 2.14.6 backtrace
(`matched_reader_remove` on the UDP receive thread) is consistent with the same teardown seen from another
thread; its core is gone, so that is inferred from the matching failure rate and fix, not shown.

### Fix

`gz_bridge` owns its `rclcpp::Context` and installs the signal handlers itself; `release()` (a subclass
clearing the protected `handles_`, `services_` and `heartbeat_timer_`) drops the self-references, the node is
destroyed, `context.reset()` runs `rcl_context_fini` on the main thread (use_count 1) and the participant is
deleted before `main` returns. Initialising rcutils logging before `install_signal_handlers()` closes a
second window: an interrupt within about 50 ms of start made the handler thread and `main` both run
`rcutils_logging_initialize` (SIGSEGV in `rcutils_hash_map_init`, 1 of 300).

| Build (same harness, delays in s) | Iterations | Bridge deaths |
| --- | --- | --- |
| Stock `gz_bridge` (`rclcpp::shutdown()` only), 0.2 to 4 | 150 | 3 iterations |
| Owned context only (node still leaked), 0.2 to 4 | 150, then 200 | 3, then 3 iterations (4 bridges) |
| + `release()`, 0.2 to 4 | 200 | 0 |
| + `release()`, 0.05 to 2 | 300 | 1 (the logging window above) |
| + `release()`, 0.2 to 6 | 300 | 0 |
| + early logging init (final), 0.02 to 1 | 300 | 0 |

A further 300 iterations of the final build were lost to the 09:10 Colima hang and are not counted. The
harness's rclpy peers exited 1 in 4 iterations when interrupted before their node existed; peers are not
counted. Owned-context-only is a negative result worth keeping: destroying the context alone does not help
while the node is leaked.
