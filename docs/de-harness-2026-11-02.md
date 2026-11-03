# Estimation harness reset and pacing — 2026-11-02

## Scope and diagnosis

Recovered from the local `t3-8f044d4e` checkout without modifying it; applied
to sim nightly `bcf9d86295c4a3ac0542f0cc17dfb1866b52dd16`.
This change does not alter estimator math, limits, fixture acquisition,
scan/map readiness, firmware, or the 0.5 s scored lockstep watchdog.

Track G's retained Mini baseline logs establish invalid harness runs:

- `default-r1` and `diagonal-r1`: native 12/12, rejected for
  `/bench/progress` timeout. Native numeric success is insufficient.
- `radial-r1`: `/bench/case 2127518508` times out after 30 s at the second
  cell; native 11/12, invalid.

Original evidence is under
`/Users/blaise/.t3/worktrees/src/t3-5b49f1f1/artifacts/track-g-2026-11-02/baseline/`.

`SimTimeNode::set_params()` spins its executor while waiting for a service
response. The new `/bench/case` callback can therefore run before it returns.
Previously `start_case()` then cleared that acknowledgement. Clear the sampler
before the request, so either response/callback ordering retains the case.

The scorer remains a publisher while writing cell results, but does not spin
truth callbacks then. Publisher discovery cannot identify this idle period.
The recovered patch used an idle sentinel in `Header.frame_id`; that field
should identify a coordinate frame. The implementation instead acknowledges
an explicit `/bench/pacing` service handoff while truth callbacks are still
serviced. The world ignores delayed progress samples while inactive. Resume
clears stale progress and retains the existing first-sample activation rule.
All tracker/aim gates remain active throughout the handoff.

## Quiet Mini validation — 2026-11-03

Tested sim `53d9a5f32e3005646b73da6b1ceccef76ffe5e84` with workspace
`7095298e511ceb18b2298fb42d584e597bb9e153`, using its pinned packages.
No Season2 foundation candidates or estimator changes were added. The fresh
owned container used 2 CPUs, 4 GiB and one compiler job throughout.

- Hosted firmware and six-package `colcon build --packages-up-to sim` passed.
  The ROS build took 7 min 1 s, including sim's 4 min 4 s.
- Docker-skill smoke passed, including clean graph and package resolution
  through the same entry point used for tests.
- Registered sim tests: colcon reports **238 tests, zero errors, failures
  or skips** (215 testcase elements across 23 XML reports). All reports were
  also checked for skipped/error/failure elements.
  Sim registers native tests plus copyright, flake8 and pep257, not cppcheck.
- Default (`lateral`), radial and diagonal: **three runs each, 12/12 cells
  per run**, with all nine native log checks accepted. Scored duration stayed
  at the default 30 simulated seconds per cell; all numeric and 0.5 s watchdog
  limits remained unchanged. No reset/watchdog failure, launch exception or
  SIGKILL was reported. Post-launch process snapshots contain no installed nodes.
- Host Python 3.12 quality: sim 205 files and workspace 93 files, zero
  diagnostics; actionlint passed in both repositories.

Native suite wall seconds, in repeat order: lateral 40.54 / 40.44 / 43.10;
radial 40.65 / 42.32 / 45.33; diagonal 40.94 / 41.94 / 42.54. These measure
this tracking fixture, not the full Gazebo speed acceptance or a before/after
comparison. The native phase/RTF reports are preserved with each run.

Raw logs, JSONL, process snapshots, XML and exact package refs are retained on
the Mini at
`/Users/blaise/.t3/worktrees/src/t3-135ea0b1/evidence/de-harness-2026-11-02/20261103-021645/`.
Corrected recovery drivers are in that worktree's
`artifacts/de-recovery-2026-11-02/`. Three preceding preparation attempts
stopped before tests: Docker socket selection, nounset while sourcing ROS,
and uninitialized firmware build tools. Each removed only its new owned
container. The successful batch removed its owned container at completion
and explicitly released the quiet slot. Colima remained for other containers.

## Unfinished gates

A different fresh reviewer must approve the final exact refs before G resumes
accuracy measurements. This is implementation validation, not independent
review or final integrated acceptance. Both PRs remain drafts and unmerged.

The recovered broader lifetime patch remains separate, unapplied and
unvalidated. DDS teardown races, SIGKILL-only shutdown, matching-library aarch64
scan-bridge backtrace, public-wrapper Ctrl-C/shell-death stress, named RTF
bottleneck/rendering measurements, integrated exact-SHA regression, and the
Mac–Arch comparison remain open. The comparison cannot run under the Mini-only
authorization. No roadmap item is completed by this finite harness batch.
