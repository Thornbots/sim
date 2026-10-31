# G.1 radius and spin investigation, 2026-10-15

No accuracy setting is accepted by this investigation. Lower radius process
noise improves the isolated 4 m/s cells, but the full matrix exposes stationary
depth failures and a slow radial spin alias. Keep the existing tracker defaults
and limits. G.1 remains open; G.2, G.3, T15, T7 and T8 have not been completed.
The [historical baseline](cv-bench-results-2026-09-28.md) records the original
outlier observations; the measurements below use fresh `nightly` worktrees.

The proposed package changes provide an initial-radius uncertainty override,
opt-in innovation logs and estimation-launch overrides. They do not change
default process noise, initial radius, validity or the UART contract.
The tracker change is [thornbots_pkg #16](https://github.com/Thornbots/thornbots_pkg/pull/16).

## Revisions and method

Fresh worktrees start at workspace `97560fe`, thornbots_pkg `6d60bb0`, sim
`3d79813`, bridge `740faaa` and firmware `ad13759`, all pinned by workspace
`nightly`. The instrumented tracker is `ba69a01`. The owned arm64 Mac container
uses the existing `thornbots-mac` image, ROS Jazzy, Release builds and
`--symlink-install`. Package prefixes resolve to the isolated workspace.
The Docker skill smoke check passes. RViz remains enabled; its truth,
detection and tracker displays and TF status were inspected.

Wrap-up incorporates the later documentation-only `nightly` revisions:
workspace `6bd0692`, tracker `e54361a` and sim `a769930`. They relocate workspace
references into `docs/`; the measured tracker/bench behavior is unchanged.
The tracker branch retains its tested change through signed merge `719ae20`.

Estimation uses the real ROS selector, tracker and aimer with `bench_world`:
1 ms physics, 5 ms clock steps, default lockstep, seed 0, a 3 s settling
window and 30 s scored per cell. The seven matrices are default, radial,
diagonal, shooter at 1 m/s, 0.3 s blackouts every 2 s, 30 ms camera latency,
and chassis spin at 9 rad/s. Each includes both layouts and all 12 cells,
including the currently unbounded `stationary45` variants. No limit is added
for those cells in this checkpoint.

Before each heavy command, inspect running containers and processes and wait
for other work to become idle. Track A's container is never stopped or reused.
The owned container is `track-g-estimation-20261015`, on ROS domain 47.
Later runs also set a unique gz partition. No physical robot or archlinux run
is performed.

Accept a run only when its native suite and log checker pass. Preserve failed
attempts; exclude lockstep timeouts from numerical comparisons. Full-matrix
tables also show **diagnostic** measurements from clock-valid attempts that
fail native limits; those are not accepted runs.

Raw logs, per-state errors, summaries, commands, binary copies and audits are
retained on the Mac at `/Users/blaise/ros2_track_g_20261015/artifacts/`.
The nightly tracker SHA-256 is
`c15a1419d53a557ef05cefede1fee72d11660bd69ac70b3a99d95b51d38ec0da`;
the instrumented tracker is
`92ed1c1962053d72af23440d989d041e003ed19304b448602684bbf9342fe86e`.

## Matched radius and prior trials

Run only the two 4 m/s cells using the native GTest filter while retaining
the default speed list. This keeps the original 1 Hz spin and case noise key:

```bash
ros2 launch sim estimation.launch.py \
  pytest_args:='--gtest_filter=*speed4*' log_innovations:=true \
  process_noise_radius:=0.005 log_dir:=/tmp/g1-q005-r1
```

Repeat each setting three times in distinct log directories. The default
control is `control4-r1..3`; trial prefixes are `q010`, `q005`, `q000`,
`q005-exactprior` and `q005-tightprior`. Zero-noise repeat 3 times out and is
excluded; repeat 4 supplies its third accepted run. All other matched runs
pass the native suite and checker.

Values below are the worst per-cell p95 over the first three accepted runs,
then the maximum across the two layouts. Distances are metres and spin errors
are rad/s. The prior mean stays 0.27 m and initial standard deviation stays
0.05 m unless the row says otherwise.

| Radius process noise | Prior change | Radius p95 | Spin p95 | Facing-panel p95 |
| ---: | --- | ---: | ---: | ---: |
| 0.02, matched control | none | 0.0744 | 0.3931 | 0.1560 |
| 0.01 | none | 0.0669 | 0.3776 | 0.1501 |
| 0.005 | none | 0.0485 | 0.3638 | 0.1278 |
| 0 | none | 0.0112 | 0.2872 | 0.0987 |
| 0.005 | mean 0.252 m, matching sim truth | 0.0600 | 0.3682 | 0.1378 |
| 0.005 | initial std 0.02 m | 0.0535 | 0.3850 | 0.1375 |

These are screening results, not full-matrix acceptance. These sim trials
do not validate a hardware prior. An exact simulated prior is not a reason
to change the hardware default.

## Full-matrix checks

The nightly baseline has three attempts per matrix. Radial repeat 1 passes
all assertions but records a lockstep timeout and is excluded; repeat 4 passes
and supplies the third clock-valid radial run. Floor failures remain visible.

| Matrix | Nightly native passes / attempts | Clock-valid floor failures |
| --- | ---: | ---: |
| Default | 3/3 | 0 |
| Radial | 3/4 | 0, plus one timeout exclusion |
| Diagonal | 3/3 | 0 |
| Shooter 1 m/s | 0/3 | 3 |
| Blackout | 1/3 | 2 |
| Camera latency | 2/3 | 1 |
| Chassis spin | 0/3 | 3 |

Nightly's repeated failures include staggered stationary center depth with
the moving shooter (8.41–8.42 cm against 6.54 cm), blackouts (6.35 cm against
5.58 cm) and camera latency (5.67 cm against 4.58 cm). Chassis-spin runs also
fail staggered stationary height-offset limits. Default `stationary45`
accuracy is still unbounded, as described in the [bench README](../README.md#run-the-tests).

One full seven-matrix screen rejects `process_noise_radius:=0.005`: the
staggered 4 m/s lateral radius p95 reaches 7.78 cm, versus 6.98 cm in the
three-run nightly full-sequence control. Radial radius p95 reaches 5.05 cm
versus 2.66 cm, over 1.5x; stationary cells still fail existing limits.
Do not promote this setting from its better isolated-cell scores.

Zero radius process noise has three clock-valid runs per matrix. Five timeout
attempts are excluded; repeat 4 replaces radial 3, shooter 1, blackout 1,
camera-latency 2 and chassis-spin 2. The resulting 21 clock-valid runs include
11 native passes and 10 numerical failures; no failed numerical run is replaced
to improve the result. The 0.005 full screen has four native passes and three
numerical failures across seven runs. Together with the nightly and matched
screens, this investigation records 74 bench attempts: 45 native passes,
22 numerical failures and seven excluded timeout attempts.

The table gives the worst per-cell p95 across the first three clock-valid
full runs for each matrix. Numerical failures remain **diagnostic**, not
accepted accuracy data. Radius/facing distances are metres; spin is rad/s.
The zero-noise experiment is not the proposed default.

| Matrix | Nightly radius | Zero-Q radius | Nightly spin | Zero-Q spin | Nightly facing | Zero-Q facing |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Default | 0.0787 | 0.0661 | 0.4185 | 0.3147 | 0.1656 | 0.1053 |
| Radial | 0.0266 | 0.0582 | 0.6533 | 6.0362 | 0.2269 | 0.2705 |
| Diagonal | 0.0819 | 0.0770 | 0.5296 | 0.4620 | 0.2160 | 0.1850 |
| Shooter 1 m/s | 0.0912 | 0.0721 | 0.4531 | 0.4485 | 0.1826 | 0.1673 |
| Blackout | 0.0663 | 0.0763 | 3.5433 | 4.5917 | 0.3070 | 0.3020 |
| Camera latency | 0.0697 | 0.0581 | 0.4292 | 0.4423 | 0.1581 | 0.1619 |
| Chassis spin | 0.0766 | 0.0709 | 0.4122 | 0.3519 | 0.1557 | 0.1146 |

Its first radial run aliases the staggered 0.5 m/s spin: p95 spin error
6.0362 rad/s against 0.7288, facing error 0.2705 m against 0.2006 m, and
radius error 0.0582 m against 0.0484 m. Its first default run also fails
staggered stationary depth, 0.0706 m against 0.0594 m. It cannot be accepted
on the strength of the 4 m/s screen.

Validity is also still unaccepted: in the three native-passing nightly default
runs, 27 of 36 cell runs have facing-panel error above 5 cm in their first
valid state. This is a conservative violation count, not a settling-duration
estimate. The new controls do not change the existing two-update validity
gate; age/residual gating and aiming cost remain G.3.

## Innovation evidence

The logger records capture time, track ID, visible panel count, selected
hypothesis, association, result, pre-update residual/NIS/log determinant,
and post-update spin/radii/radius variance/score. It exposes the chosen lead,
not the rejected hypotheses. Logging is off by default.

Join capture-time logs to same-track publish-time states at approximately
57 ms lag, accepting the nearest record within 45–70 ms; do not interpolate
outside a case. This join is approximate, especially near boundaries.

In `q005-exactprior-r1`, the flat 4 m/s radius error stays above 5 cm for
6.88 s (publish time 15.740–22.605), peaking at 6.3 cm. The joined 630 log
records contain no NIS above 16.3, outlier or reacquisition. Logged pair radii
span 0.189–0.313 m. `control4-r3` has another 5.84 s radius episode with
no rejected innovations and radii spanning 0.179–0.328 m. Position innovation
acceptance therefore does not establish a correct radius estimate.

Across the accepted matched runs, 26 of 81 sustained error episodes have a
gate crossing; none changes the chosen lead during that episode. A wrong lead
could have been selected earlier; this evidence does not establish that all
bank hypotheses were correct. Fresh-track spin transients are distinct from
steady-state p95 and remain part of G.3.

Two existing implementation details need follow-up:

- Radius constraints run in the positional update before the subsequent
  facing-yaw pseudo-update, which can move correlated radius state again.
  Logged radii slightly below the 0.18 m clamp warrant a tested constraint
  ordering fix; this checkpoint does not change it.
- `speeds:=4` changes the target spin from 1 Hz to 2 Hz because spin depends
  on the selected speed range, without changing the cell key. A filter must
  not silently change the test input. Use GTest filtering for the comparisons
  here; the alone-versus-sequence acceptance work remains T8.

## Hosted E2E gate and validation

No pooled hosted E2E result is established by this checkpoint. The firmware
hosted build passes with GCC 14, but building it is not a stage run. The
tracker draft must remain unmerged until the requested comparison passes.

| Hosted stage | Nightly, three-run pooled hits/shots | Branch, three-run pooled hits/shots | Gate |
| --- | --- | --- | --- |
| `mcb_parked` | not run | not run | pending |
| `mcb_drive` | not run | not run | pending |
| `mcb_match` | not run | not run | pending |

All 130 thornbots tests pass, including the two new prior/diagnostic tests;
all 203 sim tests pass. Both registered package test runs report zero errors,
failures and skips. Host Python 3.12 repository quality and actionlint pass
for the changed repositories. No quality baseline or estimation limit is grown.

## Remaining work

G.1 still needs a radius/spin repair that survives all cells and the radial/
blackout guard, three accepted accuracy repeats, tightened limits and hosted
E2E nonregression. Consider a model/regularization repair rather than selecting
the lowest isolated-cell radius score. The initial-prior trials above do not
establish a successful continuous radius prior.

G.2 ray noise and blackout parity, G.3 settling validity and aiming cost, T15
angled stationary repair/limits, T7 acceleration/path-end sweep and T8
alone/sequence parity were not started as separate repairs: the ordered work
stops at this G.1 investigation checkpoint. The fresh-validity tally and speed
filter finding are evidence for later work, not completed fixes. No item is
removed from the workspace roadmap on the strength of these rejected trials.
