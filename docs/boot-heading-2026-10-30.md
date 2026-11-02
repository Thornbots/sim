# Shared boot heading verification — 2026-10-30

The [world-frame contract](../README.md#the-world-frame) is tested with both
teams at field yaw π/2. Field geometry and match route coordinates are unchanged.
The parked diagnostic retains the hosted fixture's field-safe -y target paths
for both backends; its acquisition check is part of the parked run below.
Previously Renode alone rotated those targets to π. No board accuracy result
is inferred from the hosted run.

The current parked ingress and geometry verification supersedes this report's
parked spawn behavior: [2026-10-31 results](mcb-parked-ingress-2026-10-31.md).

## Sources and isolation

- Sim branch `fix/team-boot-heading` was created at `444e8257`, then rebased onto
  nightly `74fd6ec` to retain the production shot-hit cadence change.
- Firmware executable source: `3e64c33`; final dependency `d2e8294` only amends
  README wording. Its shared working-tree odometry and patrol changes were
  copied before the hosted build.
- Operator: `afe58b1`; localization: `bfa9713` including nightly cleanup fix
  `1bb9a85`. Current sources were copied into the integration snapshot.
- Own container `isaac-agent-boot-heading-sim`, ROS domain 185, private gz
  partition `agent-boot-heading-sim-185`, eight-CPU cap. Shared package checkouts
  were only read. Evidence lives at `/tmp/boot-heading-sim-evidence` on the host.
- No display was available. No physical robot, robot configuration, field mesh,
  UART format, acceptance floor, or baseline was changed.

## Commands

Host setup used `agent_container.sh up boot-heading-sim -w
/tmp/boot-heading-sim-ws/src -e /tmp/boot-heading-sim-evidence -c 8` after copying
the workspace. Container commands below used `dexec.sh` with
`ISAAC_ROS_CONTAINER=isaac-agent-boot-heading-sim`:

```sh
# Root setup, then builds as admin:
src/isaac_ros_common/docker/scripts/install-sim.sh
apt-get update
apt-get install -y scons g++-14 python3-jinja2 python3-pyelftools python3-lxml python3-yaml
MCB_BUILD_JOBS=4 src/sim/tools/build_mcb_firmware.sh
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --symlink-install --packages-select thornbots_pkg
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --symlink-install --packages-select sentry_localization sim
colcon test --packages-select sim --event-handlers console_direct+
colcon test-result --test-result-base build/sim --verbose
SIM_MCB_BACKEND=hosted ros2 run sim mcb_physics_suite
ros2 run sim check_bench_log /evidence/physics-final.log
MMM_EVIDENCE_DIR=/evidence/mmm-current ros2 run sim mmm_suite
src/sim/tools/run_suite.sh mcb_parked backend:=hosted speeds:=0 paths:=lateral log_dir:=/evidence/parked-clean
src/sim/tools/run_suite.sh mcb_drive backend:=hosted log_dir:=/evidence/drive-clean
src/sim/tools/run_suite.sh mcb_match backend:=hosted log_dir:=/evidence/match-clean
```

The host Python 3.12.15 venv `/tmp/quality-venv-312` contains the exact workspace
quality requirements. From `sim/`, with its `bin` directory on PATH:

```sh
python ../.github/quality/check.py
/tmp/boot-heading-sim-tools/actionlint -shellcheck=""
git diff --check
```

`actionlint` 1.7.7 was installed outside the checkout using
`GOBIN=/tmp/boot-heading-sim-tools go install github.com/rhysd/actionlint/cmd/actionlint@v1.7.7`.
`smoke.sh` verified that sim and thornbots resolve to the local overlay.

## Results and limits

- Quality: 198 files, zero existing or new diagnostics; actionlint and diff
  whitespace checks passed.
- Registered sim tests: 230 results, zero failures/errors/skips. Coverage
  includes both teams' displacement and velocity vectors through real firmware,
  field yaw telemetry, common ghost boot heading, unchanged ingress waypoints,
  field clearance, and real bridge/PTY transport.
- Hosted CAN/gz physics: passed; native log checker accepted. Requested joint
  yaw +0.6/-0.5 rad measured +0.599381/-0.500638 rad, with the expected wire
  field-yaw offset and hit-angle checks.
- Full MMM: all three cases passed (108.3 s), including boot-sample heading,
  heading-fixed root, successful validated saving, bridge/firmware restarts,
  dropped/corrupted packets, setup failure, and transmit-queue saturation.
  Each case's native log checker accepted the run.
- Drive: passed (34.45 s), native checker accepted. Match: passed (46.41 s),
  native checker accepted. These establish route/localization diagnostic
  completion, not combat accuracy; drive still reports aim/attitude diagnostics.
- Parked stationary/lateral: failed (32.22 s), zero physical MCB shots despite
  806 fire decisions. Acquisition succeeded: 1,209 states, median centre error
  0.032 m. There were zero lockstep timeouts in this clean run. Its checker
  rejected the failed suite; this is not an accepted combat result, and the
  zero-shot result is pre-existing; see the nightly baseline below.
- Final process check found no remaining installed nodes, gz, or ROS launch/run
  processes. The task container and its volumes were removed afterward.

Initial setup attempts exposed root-owned agent build/log volumes from the
installer's workspace-owner lookup; ownership was repaired to `admin` within
this container. Firmware tools/Python modules were installed after initial
missing-dependency failures. A command started before overlay creation could
not find sim; it was rerun after sourcing the completed overlay. The sequential
runner also needed nounset disabled while sourcing colcon's setup script.

The first registered run had one raw-angle assertion fail on equivalent yaw
commands separated by 2π. It now compares wrapped error at the same encoder
resolution, and the full rerun passed. The first parked run acquired detections
but hit a lockstep timeout while rebuilding operator sources; its native log
checker rejected it, so no scoring result from that run is accepted. The first
MMM run was interrupted to update the localization baseline; an assertion on
post-dwell chassis yaw was corrected to check the first captured boot sample.
Two inert CLI wrappers from that interrupted run initially made the route
preflight refuse to start. Repeated `kill_launch.sh` SIGINTs did not remove
them; their empty child lists, exact commands and container cgroups were
verified before sending SIGKILL through pinned pidfds. The route suites were
then rerun. The full MMM transmit-queue case also needed its normal LaunchTree
SIGKILL fallback to stop the intentionally stalled bridge.

Renode/board execution, other parked speed/path cells, broad localization drift
and CV accuracy suites were not run for this change. These hosted checks do not
establish hardware timing or combat accuracy.


## Nightly parked baseline — round 2

The unchanged workspace `origin/nightly` at
`bce1754696ff9376491be081221577e0fb4792d2` reproduces zero counted physical
shots. This failure predates the common +y heading change. Both baseline runs
failed their native log checks; neither is accepted as a combat accuracy result.

Exact relevant workspace gitlinks (not the package branches' moving heads):

- sim: `38b886865ae795d4118e2ffd62374983a1e50910`
- firmware/MCBV3: `143b63f601b5eab2243c13acf5a891aa6e4e3df0`
- thornbots_pkg: `f0e1756b7eb69e5649dfad4b79911c3dabce14df`
- sentry_localization: `1bb9a8524f0fc1e8e57e765b3b295e169a1abf6b`
- ros2_dji_serial_bridge: `3abf41ce0a96adc7ded76a1ca6adccaa3db549b9`

The detached baseline worktree had every workspace submodule checked out at its
nightly gitlink, with recursive firmware dependencies initialized. All packages
up to sim and the checked-out hosted firmware were built in own container
`isaac-agent-parked-nightly`, capped at eight CPUs, ROS domain 135, private
partition `agent-parked-nightly-135`. The smoke check passed, verified the local
sim/operator/localization overlays, NVIDIA libraries, and an empty ROS graph.
No display was available, matching round 1. No builds overlapped measurements.
Raw host evidence, including all pins, setup logs and launch/stack logs, is at
`/tmp/parked-nightly-evidence`.

Both runs used the identical command, changing only the evidence directory:

```sh
src/sim/tools/run_suite.sh mcb_parked backend:=hosted speeds:=0 paths:=lateral log_dir:=/evidence/parked-baseline
src/sim/tools/run_suite.sh mcb_parked backend:=hosted speeds:=0 paths:=lateral log_dir:=/evidence/parked-repeat
```

- Nightly baseline: 820 fire decisions, 0 physical shots, 0 physical hits;
  1,230 acquired states, median centre error 0.039 m, zero lockstep timeouts;
  failed in 33.27 s.
- Nightly repeat: 799 fire decisions, 0 physical shots, 0 physical hits;
  1,199 acquired states, median centre error 0.043 m, zero lockstep timeouts;
  failed in 34.21 s. A passive 100 Hz indexer-motor subscriber was added only
  to this repeat; the sources and launch settings were unchanged.
- Heading branch (round 1, sim `ebcb1d8`, firmware `d2e8294`): 806 fire decisions,
  0 physical shots, 0 physical hits; 1,209 acquired states, median centre error
  0.032 m, zero lockstep timeouts; failed in 32.22 s. All three runs report
  fire-decision hit rate 1.0, which is separate from physical-shot scoring.

### Diagnosis and scope

The likely gate is the simulated chamber-exit counter, rather than an aim-angle
comparison. `ProjectileExit::update` in
`include/sim/mcb_motor.hpp` arms only after ten consecutive 1 ms updates with
at most two encoder counts of travel, at chamber phase 0.20 ± 0.10. It counts
one subsequent forward feed, then requires another rest to rearm. The plugin
publishes physical shots solely from this counter in
`src/mcb_firmware_system.cpp`; hosted firmware indexing requests alone are not
physical exits.

The repeat's `motors-repeat.jsonl` records continuous indexer travel during
scoring: from sim time 16 to 29 s, 1,300 samples cover about 194.87 chamber
increments, with indexer shaft speed between -667.78 and -62.80 rad/s. The
indexer is therefore moving despite zero scored exits. Continuous production
fire requests plausibly prevent the rest required to rearm the counter; the
100 Hz trace is diagnostic evidence, not an exact 1 ms replay of its state.
The existing `ChamberExitFollowsEachShotRequestByTheTimedLatency` firmware/plant
test exercises requests spaced 500 ms apart, which allows settling and does
not cover this continuous cadence.

Firmware `AutoAimAndFireCommand::execute` applies the boot yaw to its aim
setpoint but has no yaw-error firing gate. It requests indexing when targeting,
shot timing, game state and the FIRE flag allow it; `IndexerSubsystem::canShoot`
checks motor availability, heat, referee power, beam state and cooldown.
These checks and the chamber-exit implementation are unchanged by the heading
PR. This is a diagnosis of the pre-existing fixture behavior, not a claim that
hosted combat accuracy or real hardware firing has passed.

No runtime source, acceptance floor or baseline was changed in round 2.
Physics/MMM/drive/match/registered sim/firmware suites were not rerun: the
regression-only fix path was unnecessary, and this update changes only the
record. Host Python 3.12 quality, actionlint and whitespace checks passed for
the documentation update and workspace pin. Final process checks found only
the ROS CLI daemon; the task container and all three volumes were removed.
No physical robot, Renode run, merge, Mac or Colima was used.
