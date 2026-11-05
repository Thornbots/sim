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
