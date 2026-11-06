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
