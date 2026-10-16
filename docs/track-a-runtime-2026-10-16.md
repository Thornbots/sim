# Track A runtime and hosted parked result — 2026-10-16

This note records isolated runtime provenance, two accepted full hosted
parked runs and their comparison. It does not close T17 or select combat floors.

## Runtime provenance

The first sweep below used the reused Jazzy container. Subsequent controls
use the owned internal-network container `track-a-bench-20261016`, created
from the existing `thornbots-mac` image
`sha256:a5c4b40f792e6abd9d8b4ee3802bfc9c41a1096d48034b1d07f299f9c7df4744`.
No VM or user container was restarted. Only owned `/tmp` trees were copied,
preserving their absolute paths; the fresh worktree and host artifact directory
were bound into the container.

Package-version comparison found identical installed ROS/Gazebo runtimes;
differences were compiler, debugger and support tools. The copied runtime
resolves all five changed packages to its explicit overlay. Building the
nightly Renode adapter succeeded after installing the missing JSON headers.

The fresh-host nightly baseline manifest,
`/Users/blaise/ros2_track_a_artifacts_20261015/nightly-source-20261016.json`,
has SHA-256
`56a6c7811298e2a9bd36e6b2ec71806a3573628355405137c3f632caddf44d94`. Its 186
runtime-source hashes match the pinned Git objects, and both hosted and Renode
firmware ELF hashes match. For the branch run, the fresh-host manifest
`track-a-source-20261016.json` has SHA-256
`4b438580d541e945ec4996f86ec2995a90ea221b962265fd19417006b10a017b`; it pins
workspace `cdf4ec1a` and verifies 201 runtime-source hashes with zero
mismatches. Earlier container-Git manifests pointed at a stale mounted
checkout. These host manifests and runtime hashes supersede them; no files in
that stale checkout were modified.

## Full hosted parked r1

`final-hosted-parked-r1` ran from the pinned branch manifest above. The native
runner and checker exited 0/0; all 12 default cells passed at 20 scored sim
seconds each. MCB scored 368/413 (89.1%); the flag score was 378/423. Per-cell
MCB hits/shots, in speed order stationary/1/2/4, were lateral 40/40, 32/36,
31/36, 22/29; radial 40/40, 35/35, 33/33, 30/31; diagonal 40/40, 29/34,
24/29, 12/30. Lockstep reported zero timeouts and the scorer reported zero
latest-TF fallbacks. Suite wall time was 450.7 s at RTF .65; Gazebo accounted
for the longest wait (414.73 s).

The speed-4 diagonal audit records 12/30 MCB hits. Its median aim-off-panel
distance was .164 m and barrel-off-aim p95 was 1.75 degrees; raw odometry
disagreement stayed near 1e-11 m, target states remained valid, and detection
age stayed at or below 16 ms. These are diagnostics, not a tuning conclusion.
For context, the earlier single nightly parked sweep scored 362/428 (84.6%);
the differing shot denominators and single-run samples do not establish a
three-repeat comparison or a no-regression floor.

`final-hosted-parked-r2` ran in the owned internal-network container and also
passed its native runner and checker (0/0), with
12/12 cells, 368/413 MCB hits/shots, zero lockstep timeouts and zero latest-TF
fallbacks. Suite wall time was 428.6 s at RTF .69; the final coordinator record
was 290.2 sim seconds in 425.5 wall seconds, with Gazebo the longest wait at
385.94 s. Comparing r2 with r1, `compare_runs` first diverged in shots at
record 80 (28.211 s), states at record 1200 (28.1 s), and poses at record 400
(28.1 s). Thus both full sweeps pass their native checks, but the exact-run
comparison gate fails.

The first differing carried input is a target-state center at 25.3 s,
followed by raw odometry at 25.331 s. Detection rows contain counts but not
path/application stamps, so this ordering does not establish a parameter-timer
or path-application race. The following two-cell diagnostics record geometry and path application.
No tracker gain or floor changed.

The complete run artifacts are in
`/Users/blaise/ros2_track_a_artifacts_20261015/runs/final-hosted-parked-r1/`
and `/Users/blaise/ros2_track_a_artifacts_20261015/runs/final-hosted-parked-r2/`;
their container result directories were
`/tmp/track-a-uart-20261015/final-hosted-parked-r1/` and
`/track-a-artifacts/runs/final-hosted-parked-r2/`. Earlier accepted
[stationary Mac gates](track-a-uart-2026-10-15.md#frozen-profile-stationary-mac-gates)
and [default localization regressions](track-a-uart-2026-10-15.md#default-localization-regression-checks)
remain separate evidence. Remaining hosted repeats/stages, the board parked
sweep, and T17 acceptance are still open.


## Path application diagnostics and final scope

Two 10-second, two-cell lateral diagnostics completed with native exits 0/0.
Each scored 37/39 MCB hits: stationary 20/20 and speed 1 at 17/19. R1 took
51.78 s (RTF .66); r2 took 51.12 s (RTF .64). Exact JSON comparison still
fails: first shot difference at record 40 / 18.211 s, state difference at
record 600 / 18.1 s, and pose difference at record 200 / 18.1 s.
`path-trace-host-audit-20261016.json` preserves the host audit; these short
runs are diagnostic samples, not full-stage calibration or gate closure.

The path application trace gives direct evidence of variable phase. R1
selected 126 path updates in their capture step and 1,625 one millisecond
later; r2 selected 120 in their capture step and 1,631 one millisecond later.
The same captured command can therefore enter physics in different steps.
Detection geometry and commanded paths now remain in the input ledger.
This establishes a scheduling defect, but does not prove it is the only
source of the complete-run divergence. No tracker tuning is justified.

A proposal queues commands and makes capture T eligible strictly after T.
It and its regression tests are retained outside the repository as
`unvalidated-step-inputs-proposal-20261016.patch`; they are not shipped.
At finalization the Docker socket was absent and no Colima process was
running. The existing VM was not restarted. The committed runtime sources
match all 201 hashes in the tested diagnostic manifest
`path-trace-source-20261016.json` (SHA-256
`4f7ec781c2765dace960536914551e5bf6415dcb16008f451d4f8dc95d916000`).

The diagnostic build passes 226 registered sim checks, zero failures/errors,
84 skipped, plus host Python 3.12 quality and actionlint. An initial copyright
check encountered Mac tar metadata sidecars; removing those two generated
files and rerunning the suite produced the clean result. No baseline changed.
The earlier five-package result remains 466 checks, zero failures/errors,
95 skips, with coordinator, real bridge and installed EKF checks recorded in
the preceding investigation. Default localization regressions passed there.

## Acceptance and remaining work

These are actual MCB enemy hits/shots. Counts include only native-accepted
runs; accepted repeat count is in parentheses. A dash means no current-profile
sample. Earlier history treatments are superseded and not calibration.

| Backend / stage | Pinned nightly | Current profile |
| --- | --- | --- |
| Hosted parked | 362/428 (1) | 736/826 (2), exact comparison fails |
| Hosted drive | 88/96 (3) | — (0) |
| Hosted match | 60/110 (3) | — (0) |
| Board parked | — (0) | — (0) |
| Board drive | — (0) | — (0) |
| Board match | — (0) | — (0) |

There is no three-repeat current-profile comparison for any complete stage,
so the required no-regression claim is unmet. No new combat floor is selected
or installed; existing floors are unchanged. Earlier proposed floors remain
unenforced. Navigation startup failures and rejected attempts stay in the
preceding archived acceptance records.

- Closed: hosted stationary-lateral Mac repeats, RTF=1 and Gazebo CPU2 exact
  controls; default scan-degraded, drift-correction and EKF regressions.
- Unmet: remaining full-stage repeatability controls; three accepted current
  calibration repeats per stage/backend; additional nightly parked repeats
  and all nightly board controls; board parked sweep; numeric drive/match
  combat floors; original EKF/raw-join failure prerequisites still unproved.
- Next: validate the queued-path proposal, rerun stationary and full parked
  controls, then drive/match, audit stamped inputs, complete board controls
  and derive floors from each worst accepted run with an explicit margin.
- Physical robot and Archlinux are excluded by the brief. Nothing was merged.
  Package and workspace PRs remain draft because acceptance is incomplete.
- The owned benchmark container/network could not be removed through the
  unavailable daemon. They may remain in the stopped VM's storage; no other
  container or VM was stopped, restarted or deleted by this task.
