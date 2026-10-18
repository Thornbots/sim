# Track A reconciled-stack checks — 2026-10-18

Fresh hosted measurements use the reconciled stack and one firmware executable
on both sides. They do not establish a match accuracy regression or a win.
T17 and combat-floor acceptance remain open; no floor or margin was changed.
The coordinated PRs remain draft. Nothing was merged into nightly or main.

## Provenance and method

Artifacts: `/Users/blaise/ros2_track_a_rebased_artifacts_20261017/`.
The predecessor's complete historical report remains at
`/Users/blaise/ros2_track_a_followup_artifacts_20261016/FINAL_REPORT.md`.

- Baseline workspace: `4f9e73a945c99292eaa869d4c38d44dc2b991d1e`.
- Measured branch workspace: `bb92d8c99a6e670847be0362cdd0ddce0dad90af`.
- Shared firmware source: `3f14d5b4e5c5ae06541229452af4338943d6e8ae`.
- Shared hosted ELF SHA-256:
  `537bf650fef53d8fa4d92a3488273356b590430cba9a4f72c28369095c4a8b8a`.

Each runner verifies runtime source hashes before launch, records package
resolution and firmware hash, and runs the native log checker after completion.
The independent source audit matches 299 branch and 290 baseline source files
against their recorded Git objects. All six match stages have runner/checker
exits 0/0 and zero reported coordinator timeouts. The branch profile enables
`paused_startup` and `lockstep_localization`; the baseline retains its defaults.
Both use hosted execution explicitly, not the board backend.

Later nightly workspace `6f7a16663aab452c068d4b5106cc2a3eb8dcbac9` changes only
CHANGELOG and the common-package pin; that package changes only README files.
Those updates are preserved in the integration branch and do not change the
measured runtime. Full pins and exact commands live in each run's manifest.

## Fresh match comparison

Runs alternate baseline then branch, three pairs, without changing the ELF.

| Pair | Baseline hits/shots | Branch hits/shots |
| --- | ---: | ---: |
| 1 | 20/35 (57.14%) | 20/38 (52.63%) |
| 2 | 20/37 (54.05%) | 20/38 (52.63%) |
| 3 | 20/38 (52.63%) | 20/38 (52.63%) |
| Pooled within this experiment | 60/110 (54.55%) | 60/114 (52.63%) |

Branch minus baseline is **−1.91 percentage points**. Baseline run-to-run range
is 52.63–57.14% (4.51 pp); branch is 52.63% throughout. An exact percentile
bootstrap resampling whole runs independently within each side gives a
descriptive 95% interval of **[−4.51, 0.00] pp**. It enumerates all 729 pairs
of three-run resamples and recomputes pooled hit proportions. Only three runs
per side give limited coverage; the interval does not prove equivalence or
exclude an unobserved scheduler-dependent failure. It does not establish a
regression or improvement. Shots from one game are not independent trials.

The historical pools used different firmware and remain separate:
parked 84.76% versus 89.35%, drive 91.67% versus 96.88%, and match 55.05%
versus 52.63% (−2.41 pp). The earlier categorical match-regression conclusion
is superseded by this uncertainty analysis, not by combining the pools.

## Match diagnosis

The official `enemy_hit` metric requires referee damage. `Referee::apply`
rejects damage to a robot already at zero HP, while the shot still enters
the denominator. The detector/selector continues targeting that robot.

| Run | Enemy damage hits | Enemy panel impacts eligible geometrically | Impacts on already defeated victims | Last damaging shot, sim s |
| --- | ---: | ---: | ---: | ---: |
| Baseline 1 | 20 | 26 | 6 | 36.023 |
| Baseline 2 | 20 | 32 | 12 | 30.772 |
| Baseline 3 | 20 | 33 | 14 | 30.777 |
| Each branch run | 20 | 35 | 16 | 26.984 |

The audit replays every shot in referee application order, carrying HP from
the preceding row. These are supplemental diagnostics; no shots are removed
from the official rate. Earlier damage saturation and longer firing at a
defeated target explain why damage-hit proportion alone cannot diagnose aim
quality. It would be incorrect to infer worse aiming from the pooled −1.91 pp.
Target retirement and a live-target/damage-efficiency acceptance contract
remain work before choosing combat floors. No tracker tuning occurred.

The branch's projection of every MCB shot onto time, enemy-hit, impact,
victim, panel and eligibility is identical across all three runs. Native
`compare_runs`, with its default zero tolerance, nevertheless exits 1 for
r1/r2 and r1/r3. The first shot record at 2.144 s differs in
`selected_target_association_m`; the first state at 2.000 s differs in
prediction-error diagnostics. Route diagnostics diverge at 15.350 s.
These are strict-log failures, not a T17 pass.

The r1/r2 input audit finds identical carried stamps for consumed firmware
ticks and raw-TF/scorer/EKF-input receipts. The first detector payload difference
is in panel corners at capture 2.034 s; target-state centers differ at 2.084 s.
Raw MCB velocity, `/odom` and EKF input differ first at 15.351 s; rf2o differs
at 15.400 s. Initial scan-consumed ledgers differ in whether stamp zero is
recorded. This narrows the divergence but does not establish a physics-only
cause. Arrival-time differences are excluded only in that diagnostic audit,
not from native acceptance.

## Parked and drive confirmations

One fresh complete hosted parked run per side gives baseline 366/432
(84.72%) and branch 369/413 (89.35%), **+4.62 pp**. Both accept all 12 cells.
The branch count matches the historical accepted branch sweep. This confirms
the direction of the earlier gain on the reconciled stack; one run per side
does not provide a run-level noise bound or calibrate floors.

The complete drive confirmations give baseline 30/32 (93.75%) and branch
30/31 (96.77%), **+3.02 pp**. They also retain the earlier direction, with
the same single-run limitation. All ten fresh comparison stages have native
runner/checker exits 0/0, zero coordinator timeouts, and branch stages report
zero latest-TF fallbacks.

## Stationary T17 controls

The current-profile stationary-lateral cell (`speeds:=0 paths:=lateral`)
passes three unthrottled runs, one `real_time_factor:=1` run and one with
Gazebo alone pinned to CPUs 0–1. Each scores 40/40. Native `compare_runs`
reports identical 80 shot records, 1,200 states and 400 poses against r1
for all four comparisons, at zero tolerance. All five runs have native
exits 0/0, zero coordinator timeouts and zero latest-TF fallbacks.
The Gazebo-only affinity receipt contains 301 samples, starts before startup
unpause, and records every observed Gazebo thread on CPUs 0–1.

An earlier, broader CPU2 stress attempt pinned the entire launch. It rejects
before scoring at sim zero: map_server logs a lifecycle-response send timeout;
map_server and AMCL remain inactive until the unchanged 30 s startup deadline.
Runner/checker exit 1/1, with no shots or physics holds. Its 160 affinity
samples confirm CPUs 0–1. It is retained separately, not relabeled as a passing
control or hidden by the narrower run. The later Gazebo-only test is the
condition required by the plan; it does not repair the broader startup failure.
The failed launch also retains EKF abort and lifecycle-manager SIGKILL teardown
notes. The first wrapper stopped after the three successful runs because ROS
setup is incompatible with its shell `nounset`; the corrected wrapper resumed
without rerunning or discarding those runs.

## Remaining gates and evidence

Full-stage speed/CPU controls, board sweeps and controls,
cross-machine comparison, original EKF/raw-join prerequisites and intermittent
startup remain open. The stationary subset is complete; the demonstrated
match strict-log failure prevents whole-track acceptance and floor calibration.
No physical robot run was performed. All six PRs stay draft because the
[done bar](../../docs/E2E_LOCKSTEP_PLAN.md#order-and-done-when) is unmet.

The predecessor's rebuilt reconciled stack passed 233 registered sim checks,
138 thornbots checks and 7/7 lockstep transport checks; host Python 3.12 quality
and actionlint passed in all six repositories. These are recorded checks at
their original heads, not a substitute for the remaining runtime gates.

Reproduction and audits are retained as `continuation_series.sh`,
`analyze_fresh.py`, `fresh-comparison.json`, `diagnose_fresh_match.py`,
`fresh-match-diagnosis.json`, `trace_match_inputs.py`,
`fresh-match-input-divergence.json` and `fresh-match-compare-r1-r{2,3}.log`.
Each run directory retains source manifests, launch/stack/checker logs, native
exits and raw JSONL. Child shutdown errors remain in those logs; native
acceptance does not assert that every child exited cleanly.

`fresh-timing-audit.json` records suite timing, final coordinator totals,
held seconds per chain and shutdown notes for all 16 fresh attempts (15
native-accepted, one rejected CPU2 stress attempt). Gazebo is the longest
held chain in every accepted run.

| Run | Suite wall s / RTF | Longest hold, s |
| --- | ---: | ---: |
| branch-match-fresh1 | 80.7 / 0.50 | gz 67.79 |
| branch-match-fresh2 | 85.5 / 0.46 | gz 73.62 |
| branch-match-fresh3 | 76.4 / 0.52 | gz 64.58 |
| branch-mcb_drive-confirm-fresh1 | 34.0 / 0.96 | gz 26.60 |
| branch-mcb_parked-confirm-fresh1 | 327.6 / 0.90 | gz 290.93 |
| branch-stationary-fresh-cpu2 (rejected) | 41.5 / — | none (sim zero) |
| branch-stationary-fresh-gz-cpu2 | 31.2 / 0.97 | gz 23.44 |
| branch-stationary-fresh-rtf1 | 37.4 / 0.79 | gz 29.19 |
| branch-stationary-fresh1 | 32.0 / 0.98 | gz 23.49 |
| branch-stationary-fresh2 | 30.2 / 0.99 | gz 23.03 |
| branch-stationary-fresh3 | 33.9 / 0.96 | gz 23.04 |
| nightly-match-fresh1 | 71.7 / 0.61 | gz 63.50 |
| nightly-match-fresh2 | 65.8 / 0.66 | gz 61.02 |
| nightly-match-fresh3 | 68.0 / 0.63 | gz 64.08 |
| nightly-mcb_drive-confirm-fresh1 | 31.7 / 1.29 | gz 26.85 |
| nightly-mcb_parked-confirm-fresh1 | 360.0 / 0.84 | gz 354.00 |
