# Track A runtime and hosted parked result — 2026-10-16

Current status: [reconciled-stack checks, 2026-10-18](track-a-reconciled-2026-10-18.md).
Their common-firmware comparison and run-level uncertainty analysis supersede
the categorical match-regression conclusion in this historical record.

This note records successive isolated profiles. The
[orchestrator follow-up](#orchestrator-follow-up) supersedes the earlier
closeout below; historical attempts remain evidence. T17 and combat floors
are tracked separately from the deterministic-path proof.

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

## Orchestrator follow-up

Fresh worktrees were recreated from the pushed PR heads. The runtime source
commit is sim `4ef58655d1e0dd347eed7577553dff937034092a`, pinned by workspace
`ee94fb9bd92b69f660321d9c965b2c0dadcfc401`; later evidence-only commits do not
change these measured sources. Artifacts are under
`/Users/blaise/ros2_track_a_followup_artifacts_20261016/`.

### Deterministic path proof

The acknowledged opponent fixture queues captures and selects the newest
capture strictly before the current 1 ms physics tick. Receiving the command
still acknowledges queue insertion. Cell transitions additionally wait for
the target driver's publication at the blocked boundary before resetting its
parameters. This removes both same-tick path eligibility and the boundary
publication/parameter race. The unacknowledged fixture keeps its defaults.
Tracker settings, QoS, wait budgets and floors are unchanged.

| Full hosted parked attempt | Native runner/checker | MCB hits/shots | Wall s | Result |
| --- | --- | ---: | ---: | --- |
| branch-parked-proof-r1 | 0/0 | 369/413 | 430.78 | 12/12 cells accepted |
| branch-parked-proof-r2 | 1/1 | 0/0 | startup failure | rejected before scoring |
| branch-parked-proof-r3 | 0/0 | 369/413 | 333.99 | 12/12 cells accepted; exact r1 match |
| branch-parked-proof-r4 (2026-10-17) | 0/0 | 369/413 | 480.6 | 12/12 cells accepted; exact r1/r3 match |

Native `compare_runs` exits zero for r1/r3: 835 shot records (including flag
shots), 14,400 states and 4,800 poses are identical. The path-application logs
are also byte-identical, each containing 17,411 captures, all first applied
exactly 1,000,000 ns after capture. Both accepted runs have zero lockstep
timeouts and zero latest-TF fallbacks despite different wall throughput.
The two-full-sweep proof is closed. The third accepted sweep also has
byte-identical shot, state, pose and path logs, giving three matching
unthrottled full sweeps. This does not close the full-stage RTF/CPU controls
or all T17 gates.
R3's odometry parameter bridge exits -11 during teardown; the native checker
accepts that shutdown note. Native acceptance is not a claim that every child
process exited cleanly.

Before any tracker tuning, the two accepted input streams were audited. Each
contains 17,412 detection messages, 26,118 raw `/odom` and 26,118 EKF inputs,
with carried stamps distinct from arrival simulation time. All 14,400 sampled
scored states are scorer/tracker-valid, with zero maximum detection/tracker
age. One invalid target-state input occurs in the stationary-radial scoring
window without a sampled-state dropout. All 58 periodic tracker diagnostics
report `dropped 0`; no lost-track, stale or target-timeout event is logged.
This ledger records published messages, not internal acceptance decisions;
sampled states do not exclude transitions between samples. The detailed
`newartifacts/stamped-input-audit.{md,json}` preserves those limits. No tracker
tuning was performed.

| Identical payload | SHA-256, both runs |
| --- | --- |
| shots.jsonl | `9cce309bcfa52831d7d34f5e3026ae90b2b3d9588a46d98d7ef91ad8d46fbd70` |
| states.jsonl | `2d5f5e82336297a9e22ca9f7423465fd3a34e562b19790d17709dce9d28cd6a3` |
| poses.jsonl | `57253eb53c4c1cd067d68a7205bd1fd594f73ae8b2b7c220c1bd7b248341183e` |
| path-applied.jsonl | `4b2feabd7d953d894b9ea5f40929e09ec29b491a1ef113781c9ff1cd863ea918` |

### Retained startup rejection

R2 unpaused correctly at simulation zero and its first five MCB ticks each
had a reader. At physics 14 ms the input ledger contains the 11 ms MCB pose
and scorer receipt, but no raw-pose TF or EKF-input receipt. The aimer reports
both muzzle and camera joins missing because frame `odom` does not exist.
The plugin then rejects `raw_pose_tf=missing`, `scorer_pose=present`,
`ekf_input=missing`, each with one receipt publisher; 29 cascading coordinator
timeouts follow. There are no scored cells. The gap is in the shared
pose-to-odometry/TF chain; these logs do not distinguish missing input delivery
from missing downstream output delivery and do not prove a DDS discovery cause.
R3 is one inspected retry with unchanged binaries and budgets, not a repair
of this intermittent startup failure. Original 134 ms EKF prerequisites also
remain unproved.

### Follow-up validation

The sim build and 230 registered checks pass, zero errors/failures, 84 skips.
Host Python 3.12 repository quality and actionlint pass. All 203 runtime source
hashes match the branch manifest. The earlier default scan-degraded,
drift-correction and EKF regressions remain prior-profile evidence at
[their recorded pins](track-a-uart-2026-10-15.md#default-localization-regression-checks);
they were not rerun or relabeled as current-nightly results.

### Hosted pooled comparison

The comparison uses current nightly workspace
`63a245323520d5b257fc732c1627195474d11859`, not the earlier `5e1fd040`
controls. Each run retains its source manifest, actual package prefixes,
firmware hash, complete command, native exits, timing and raw inputs. Nightly's
five packages and hosted firmware were rebuilt from these pins:

| Source | Nightly control | Branch runtime |
| --- | --- | --- |
| sim | `b4be627f6c9d10bb264c5f386e569b7c08f82369` | `4ef58655d1e0dd347eed7577553dff937034092a` |
| thornbots_pkg | `23db221064911b8d730710a368d5392d157bf931` | `2996f522a56c55266c227286ec9b0aa4e0220e9d` |
| bridge | `3bdba2db51e7140299ed2ab67d98baea0466fbdb` | `fcab257749763245c7f536d6ba88e6be2a032163` |
| localization | `955bfda27c5afc3e8cb1b2b3dfaa7f4d690f948c` | `a1fbc249fe772645e01cd10200f078b2430b016b` |
| rf2o | `a2d858a5053790bc2add4edf2daa22d2097d0af5` | `06a2562dc50a474a409210d70c62432fccd5bd66` |
| firmware | `ad13759007491ebfb666c5e8c93087882922d9c3` | `432216d7dfc9e6e4b82643b00de9c27fd3aee1cd` |

The branch hosted ELF SHA-256 is
`70feb397e468906360e12f65ce9f37bc3ccf3f60e53b8c3286bf77b416dbaaad`;
nightly is `650ee666011542e7aaf411c485123398efa17db3d402ff9f708549b385678e49`.
This is a pinned-stack comparison, including firmware differences, rather
than a single-change attribution.

### Driving startup boundary — 2026-10-17

The first three strict-profile drive attempts and first three match attempts
all rejected before scoring: the coordinator timed out at the initial 1 s
scorer hold, then the scorer rejected the advancing clock as a missed exact
tick. No failed attempt contributes to pooled accuracy. A separate TF/aim
capture preserves this rejection with extra diagnostic subscribers.

That capture directly shows nonoverlapping TF histories: `map->nav_odom`
starts at 1.000 s, while `nav_odom->odom` and `odom->root` end at .991 s.
AMCL's configured transform tolerance is 1 s. Fresh `/cv/target` messages
start at .025 s, so missing aim publication does not explain this attempt.
The initial scorer hold prevents the raw TF history from reaching the first
map TF stamp, while setup waits for a localized `map->root` chain.

Driving and match now activate at a fixed 2 s boundary; parked keeps its
1 s reset. This allows TF overlap without changing AMCL, physics, wall-time
budgets, firing rules or acceptance floors. It is a separate driving-startup
fix from the parked path-timing fix. Parked evidence uses sim `4ef58655`;
subsequent drive/match runs retain their new source manifest. The earlier
intermittent 11 ms POSE/TF startup rejection remains unresolved.

### Final hosted pools — 2026-10-17

Each row pools three native-accepted full stages. The parent independently
verified source-manifest hashes, runner/checker exits and raw MCB shot counts.
Drive/match use workspace `23a9bf919fb5ecc45777366a122496ec76b23cf7`,
sim `cab6e94161714a94aa15856e3ff6ce7b72f4d965`; the other package/firmware
pins above are unchanged. Parked retains its original `ee94fb9b` / `4ef58655`
manifest. Runtime hashes differ only in the driving-startup harness change.
The later documentation-only commits do not change measured runtime sources.

| Backend / stage | Nightly hits/shots | Branch hits/shots | Change (pp) |
| --- | ---: | ---: | ---: |
| Hosted mcb_parked | 1090/1286 (84.76%) | 1107/1239 (89.35%) | +4.59 |
| Hosted mcb_drive | 88/96 (91.67%) | 93/96 (96.88%) | +5.21 |
| Hosted mcb_match | 60/109 (55.05%) | 60/114 (52.63%) | -2.41 |
| Board parked/drive/match | unrun | unrun | — |

**The per-stage no-regression gate fails for match.** Accepted match runs
have 20 hits each on both profiles, but branch fires 38 shots per run versus
nightly 37/36/36. This is a measured rate regression, not a claim about its
cause. Different package and firmware pins prevent attributing it solely to
the path or startup fix. No tracker settings changed.

| Stage / profile | Accepted per-run hits/shots |
| --- | --- |
| mcb_parked / nightly | nightly-parked-r1: 370/430, nightly-parked-r2: 359/428, nightly-parked-r3: 361/428 |
| mcb_parked / branch | branch-parked-proof-r1: 369/413, branch-parked-proof-r3: 369/413, branch-parked-proof-r4: 369/413 |
| mcb_drive / nightly | nightly-drive-r1: 29/34, nightly-drive-r2: 30/31, nightly-drive-r3: 29/31 |
| mcb_drive / branch | branch-drive-fixed-r1-retry1: 31/32, branch-drive-fixed-r2: 31/32, branch-drive-fixed-r3: 31/32 |
| mcb_match / nightly | nightly-match-r1: 20/37, nightly-match-r2: 20/36, nightly-match-r3: 20/36 |
| mcb_match / branch | branch-match-fixed-r1: 20/38, branch-match-fixed-r2: 20/38, branch-match-fixed-r3: 20/38 |

All 18 accepted stages have native runner/checker 0/0 and zero coordinator
timeouts. Branch accepted stages also report zero latest-TF fallbacks.
The separate timing audit records suite RTF, final coordinator totals, longest
held chain and shutdown errors for every accepted and rejected attempt.
Gazebo is the longest held component in every accepted run.

| Run | Suite wall s / RTF | Longest held chain |
| --- | ---: | --- |
| branch-drive-fixed-r1-retry1 | 35.2 / 0.90 | gz 28.45 s |
| branch-drive-fixed-r2 | 34.1 / 0.93 | gz 27.48 s |
| branch-drive-fixed-r3 | 36.0 / 0.89 | gz 28.14 s |
| branch-match-fixed-r1 | 48.3 / 0.84 | gz 39.34 s |
| branch-match-fixed-r2 | 52.3 / 0.81 | gz 40.12 s |
| branch-match-fixed-r3 | 49.5 / 0.82 | gz 39.69 s |
| branch-parked-proof-r1 | 430.1 / 0.68 | gz 388.51 s |
| branch-parked-proof-r3 | 333.4 / 0.88 | gz 297.33 s |
| branch-parked-proof-r4 | 479.4 / 0.61 | gz 433.02 s |
| nightly-drive-r1 | 41.1 / 0.89 | gz 36.07 s |
| nightly-drive-r2 | 43.3 / 0.85 | gz 38.64 s |
| nightly-drive-r3 | 44.3 / 0.85 | gz 39.14 s |
| nightly-match-r1 | 55.5 / 0.80 | gz 51.62 s |
| nightly-match-r2 | 47.4 / 1.00 | gz 44.10 s |
| nightly-match-r3 | 46.4 / 1.05 | gz 40.69 s |
| nightly-parked-r1 | 328.5 / 0.92 | gz 322.85 s |
| nightly-parked-r2 | 317.5 / 0.96 | gz 313.01 s |
| nightly-parked-r3 | 305.6 / 1.00 | gz 300.68 s |

Rejected startup evidence remains: parked proof r2 (missing first raw TF /
EKF-input receipt); three initial drive attempts, three initial match attempts
and the separately instrumented TF diagnostic (the 1 s boundary defect);
fixed-drive r1 (map_server/AMCL lifecycle activation timed out at sim zero).
None contributes to pooled accuracy. Child shutdown signal notes, including
the parked bridge exit -11, remain in `newartifacts/timing-audit.{md,json}`.

- Closed: deterministic path eligibility, three identical full unthrottled
  hosted parked sweeps, and three accepted nightly/branch hosted samples per stage.
- Open: full-stage RTF=1/CPU2 controls, board sweeps/controls, original EKF/raw-join
  failure prerequisites, intermittent startup failures and the match rate regression.
- Board repeats and combat-floor calibration were deferred to prioritize the
  required hosted comparison. Full T17 is unmet, so no numeric floor or margin
  is selected; existing floors are unchanged. Physical robot and Archlinux
  runs remain excluded. Nothing was merged.
