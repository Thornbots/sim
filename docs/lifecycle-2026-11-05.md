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
