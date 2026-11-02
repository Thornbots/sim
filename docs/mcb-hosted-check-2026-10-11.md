# Hosted MCB and shutdown check, 2026-10-11

Linux x86_64, workspace `db34069`, MCBV3 `86c2241`, sim `116736b`, bridge
`8dd2fdd`, already-running Jazzy container. Fresh isolated builds of the
checked-out firmware, sim and UART bridge passed without compiler warnings.
The default firmware helper used GCC 13.3; MCBV3's two test runners used GCC 14.
The helper checks for g++-11 although SCons defaults to the system compiler.

Native results, no skips: protocol 5/5, firmware 11/11, real-bridge PTY 1/1;
MCBV3 core 31/31 and hosted regression 4/4. The compiled-firmware tests exercise
aim/fire, current UART payload sizes and CRC rejection, referee stage gates,
POSE/REF_SYS, relocalization, simple drive and NAV_GOAL.

`mcb.launch.py auto_fire:=true drive:=stop` used the fresh firmware and bridge.
CV_TARGET with fire clear moved the physical Gazebo head joint and returned
POSE yaw 2.67804 and 3.60504 rad for requested 2.67795 and 3.60524 rad, errors
below 0.00020 rad. POSE and REF_SYS arrived at 90.007 and 10.000 Hz in sim time;
PING 42, 167 and 255 echoed with zero CRC errors. An initial check with
`auto_fire:=false` could not aim: that switch removes the combined firmware
aim/fire command, rather than only suppressing its shots.

## Shutdown findings

The first x86 bring-up's head-pitch `ros_gz_bridge parameter_bridge` died on
SIGINT. Its core shows a null mutex in Fast DDS 2.14.6
`DataReaderHistory::get_first_untaken_info`, reached through `__rmw_wait`,
`rcl_wait` and `rclcpp::Executor::spin_until_future_complete_impl`. Concurrently,
the main thread is removing a participant in the Fast DDS factory destructor
from libc's exit handlers. This supports an executor/participant teardown race
in the dependency stack; it does not establish a sim-owned defect.

The Mac mini's `/Users/blaise/ros2_ws/src/sim/core` is a 194,961,408-byte
aarch64 ELF core dated Oct 8 19:18, from `parameter_bridge /scan`. Bridge cores
therefore exist on both architectures, for different bridges. The Mac's Docker
socket/container was unavailable during this inspection; it was not started,
so a matching-library ARM backtrace remains unverified.

The corrected x86 bring-up did not reproduce the pitch-bridge crash. It did
repeat a sim-owned `FileNotFoundError`: nested launch shutdown events invoked
temporary-world deletion twice. Cleanup now tolerates an already-removed world,
and the native checker rejects launch exceptions, including during shutdown.
The corrected bring-up also left a Gazebo server after group SIGINT/TERM;
SIGKILL of that owned process group removed it. Bridge teardown and orphan-free
interruption remain open under workspace track E.

A further bring-up with idempotent world deletion shut down without exceptions,
bridge crashes or leftover processes; the native checker accepted its log.
All 13 suite-exit/log-checker unit cases passed, including the new shutdown
exception rejection case. Host Python 3.12 quality and actionlint checks passed.

Raw builds, XML reports, live probes, launch logs, x86 core and backtrace are
retained on the verification box under
`/home/blaise/workspaces/isaac_ros-jazzy/test-mcb-2026-10-11/`.
No match stages were part of this check.
