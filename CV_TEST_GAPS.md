# CV test gaps

Open gaps found reviewing the CV test suites on 2026-09-09. Spans two
submodules, so it lives here rather than in either one.

Gaps 1, 3, 5, 6 and 7 are closed. Gap 2 is closed in unit tests and waits on the
aim bench's first `shooter_speed` run.
Gap 8, moving-target hit rate, is open.

**The moving lead=on shot-hit cells are red, and that is the intended
state** until shot timing exists (gap 8). Do not relax a threshold to
green them.

Already fixed, not listed below: the URDF-constant pin
(`sim/test/cv/test_urdf_constants.py`), the duplicated docstring summaries,
the in-body settle sleep, and the tautological intercept tests (gap 1).

What's already solid and should stay that way:
`test_random_angles_round_trip`, and in `test_target_tracker.py` the armor
model's `test_spin_in_place_recovers_rate_centre_and_both_radii` and
`test_tracker_bank_recovers_the_spin_after_a_bad_first_second`, all of which
check physics rather than restating the implementation.

On 2026-09-17 the armor-model tracker replaced `SpinDetector`,
`KalmanFilter6D` and `corrected_centre`, and their tests went with them.
Gaps 3 and 6 below describe those removed tests; they are kept as history.

## 1. Tautological intercept tests — FIXED 2026-09-09

`test_point_to_cv_target.py` now checks the intercept condition
(`|aim - shooter| == v_muzzle * t`) and compares both the flight time and
the aim point componentwise against a closed-form quadratic solve, over a
7-geometry table covering crossing, closing, receding and oblique motion.

Two things worth keeping in mind if the tolerances ever need touching:

- The default `iterations=3` leaves a worst-case 0.41% flight-time error
  and 6.2mm aim error over that table, hence `ITER3_REL_TOL = 1e-2` and
  `ITER3_AIM_TOL_M = 0.02`. Both are measured, not guessed. Dropping the
  iteration count to 2 or 1 now fails the suite, which is intended: 3 is
  load-bearing for 1% accuracy.
- The componentwise assertion exists because `|aim|` alone is too blunt.
  Dropping the z lead entirely moves the norm by ~0.06% on a low target —
  invisible at `ITER3_REL_TOL` — while putting the shot 15cm high.

Verified by mutation: flipping the lead sign, doubling the muzzle speed,
dropping the z lead, skipping the solve, and reducing the iteration count
each fail the suite.

## 2. `shooter_pos` covered; `shooter_vel` deliberately still open

`shooter_pos` is closed. `GEOMETRIES` in `test_point_to_cv_target.py` now
carries four off-origin rows — shooter beside, behind and above the target,
plus a fully oblique one — and `_analytic_flight_time` subtracts the shooter
from the closed-form `d`. Deleting `shooter_pos` from the core now fails 5
tests. Worst-case residual over the expanded table is unchanged (0.41%
flight time, 6.2mm aim), so `ITER3_REL_TOL` / `ITER3_AIM_TOL_M` still hold.

`shooter_vel` stays **intentionally unverified** until the moving-robot test
exists. What is pinned today is only that the parameter reaches the math:
`test_shooter_velocity_is_wired_into_the_solve` asserts a chasing shooter
gives a shorter flight time and less lead than a stationary one, so the
parameter can't be quietly deleted (that mutation, and flipping the
`sx + svx*t` sign, both fail). Its *magnitude* is untested.

The framework for closing it is in place, and this was verified rather than
asserted: `_analytic_flight_time` already carries the moving-shooter term
(`w = v - shooter_vel`), `GEOMETRIES` already has a `shooter_vel` column
(all `ORIGIN` today), and `test_intercept_condition_holds` already measures
from the muzzle position at impact (`s + sv*t`). Dropping three moving rows
into the table passes all four table-driven tests unchanged, at 0.076%
flight-time and 0.3mm aim error. So closing this gap is adding rows — no new
math, no test changes.

The aim bench drives the shooter (2026-09-25, not yet run):
`shooter_speed:=` bounces our `root` along y during every case, `/pose`
carries its velocity, and each shot carries it too. `target_path:=radial` and
`diagonal` drive the target down the camera ray. `plan_shot`'s own-motion
correction is unit-tested by flying the shot
(`test_plan_moving_shooter_*`); before it, the node aimed as if still and
would have missed by ~0.15 m at 1 m/s.

## 3. `KalmanFilter6D.predicted()` aliasing — FIXED 2026-09-09

The `dt <= 0` branch returned `self.state` and a `np.diag(self.P)` view, so
a caller mutating either corrupted the filter in place. Now
`return self.state.copy(), np.diag(self.P).copy()`.

`test_kf_predicted_does_not_alias_the_filter_state` writes into both return
values and asserts the filter is unchanged; the old `allclose` test could
not see it, since it compared the returned array against the very array it
aliased. Reverting the fix fails the new test.

## 5. `test_shot_hit.py` smoke assertion — FIXED 2026-09-09

Both upgrades landed, and running them turned up gap 7 below.

- The stationary case now has a hit-**rate** pass condition
  (`test_stationary_hit_rate_meets_floor`, `STATIONARY_MIN_HIT_RATE = 0.5`).
  The old `hits >= 1` would pass a pipeline landing 1 shot in 200 at a
  motionless target.
- The lead feature has its own pass/fail
  (`test_lead_does_not_regress_hit_rate_at_slowest_speed`). It runs both
  legs itself at the slowest speed rather than caching the parameterized
  cells' results, so it holds under `-k`, `--lead` and any case selection.
  The condition is one-sided — `lead_on >= lead_off - 0.15` — because
  pinning "lead is better by X" at 0.5 m/s would be pinning sim noise.

Both discriminate now that gap 7 is fixed. On 2026-09-17 the stationary
rate was 78.6% and the lead comparison at 0.5 m/s was off 5.6%, on 18.2%.

On 2026-09-21 the lead-off leg was dropped: the bench runs lead on only, so
`test_lead_does_not_regress_hit_rate_at_slowest_speed` went with it, and the
stationary floor moved into the stationary case of `test_shot_hit`.

Beyond the two gap-5 upgrades, the moving sweep now has a pass condition
of its own (`MOVING_MIN_HIT_RATE`), because aiming at a moving, spinning
target is what this bench exists for and asserting only `shots_fired > 0`
on those cells left the stated purpose untested. It applies to the
lead=ON cells only — lead=OFF is the control leg and is expected to aim
worse at speed, so holding it to the same floor would be asserting that
the control works.

`MOVING_MIN_HIT_RATE = 0.25` is the one threshold in this suite that is
**not** measured against a working stack. It states the intent in code;
re-derive it once shot timing lands (gap 8). Expect it to go up.

## 6. Smaller coverage gaps — FIXED 2026-09-09

All four closed, each verified by mutating the core and watching the new
test fail.

- **`compute_score`** — one case at `center_weight=0.3`, one at `0.0`, plus
  a check that the priority bonus is unweighted. Moving the weight onto
  confidence, or onto the confidence+centrality sum, now fails.
- **`RobotHysteresis`** — the streak-reset branch is pinned by a five-frame
  sequence where frame 2 drops under `switch_margin`. Without the reset the
  switch lands on frame 4; with it, frame 5. Replacing the reset with
  `pass` fails.
- **`group_panels`** — the boundary pair is a 3-4-5 triple scaled by 0.08
  (0.24, 0.32), so its separation is **exactly** 0.4 in binary floating
  point and `<=` vs `<` is a real distinction. The obvious 2.0-to-2.4 pair
  does not work: it lands a half-ulp short of 0.4 and links under both, and
  the first draft of this test silently pinned nothing because of it. The
  merge tradeoff is now pinned in both directions — two robots bridged at
  0.3 m come back as one cluster, and widening the bridge past the radius
  separates them.
- **`SpinDetector`** — `spin_hz` is checked at 0.5/1.0/2.0 Hz, which pins
  `1/(4*interval)` as a relation rather than a single point; `spin_phase`
  is pinned to 0 at a handoff and `2*pi*elapsed/period` after it, and
  bounded to `[0, 2*pi)`.

Two fixture traps this cost, both worth keeping:

- The spin fixture drives **8** handoffs, not 9, so the last one lands at
  1.75 s against a 1.0 s period. At 2.0 s, absolute time and
  time-since-handoff differ by exactly two revolutions, the phase modulo
  hides the difference, and a `spin_phase` computed from `t_sec` instead of
  `since_last` passes.
- The fixture returns its last `class_id` so a later probe can reuse it.
  Probing with a different id registers a *fresh handoff*, which resets
  `since_last` and reads a phase of 0 no matter what.

## 7. The stack missed a stationary target by a constant 0.884 m — FIXED 2026-09-17

Not a geometry offset. The head never moved. Three bugs stacked:

- `target_tracker` called `lookup_transform(timeout=0.05)` in its detection
  callback. The wait sleeps on wall time, and `/tf` shares the node's
  executor, so at ~60Hz detections the TF buffer fell 0.6-1.7s behind and
  nearly every detection was dropped as "TF behind". A separate listener in
  the same run stayed within 50ms. Lookups are non-blocking now.
- `cv_target_emulator` still published `/cv/panel_detection` (track id 0)
  alongside `target_selector` (track id 1), so the tracker reset on every
  message. The emulator now publishes only the array, like `roi_depth_node`.
- `point_to_cv_target`'s fire trigger ignored the aim, so shots kept firing
  from a head at its rest pose. 0.884 m is that rest pose's miss to the
  nearest panel. It now fires only after a tick that emitted an aim point.

Headless sweep, `--lead both`, 2026-09-17 (8 passed, 4 failed):

```
stationary  lead=off | shots= 17 | hits= 12 (70.6%) | miss mean= 0.024
stationary  lead=on  | shots= 14 | hits= 11 (78.6%) | miss mean= 0.027
0.5 m/s     lead=off | shots= 11 | hits=  0 ( 0.0%) | miss mean= 0.097
0.5 m/s     lead=on  | shots= 18 | hits=  4 (22.2%) | miss mean= 0.125
1.0 m/s     lead=off | shots= 19 | hits=  0 ( 0.0%) | miss mean= 0.232
1.0 m/s     lead=on  | shots= 15 | hits=  1 ( 6.7%) | miss mean= 0.144
2.0 m/s     lead=off | shots= 15 | hits=  0 ( 0.0%) | miss mean= 0.554
2.0 m/s     lead=on  | shots= 19 | hits=  0 ( 0.0%) | miss mean= 0.457
4.0 m/s     lead=off | shots= 17 | hits=  0 ( 0.0%) | miss mean= 1.230
4.0 m/s     lead=on  | shots= 19 | hits=  0 ( 0.0%) | miss mean= 1.011
lead comparison 0.5 m/s: off 1/18 (5.6%), on 2/11 (18.2%)
```

The four failures are the moving lead=on cells against the placeholder
`MOVING_MIN_HIT_RATE` (0.25). What's left is gap 8.

## 8. Moving, spinning targets are mostly missed

Open. The aim point is the chassis centre, and a hit needs the ray within
0.05 m of a panel that faces the shooter. Against a 1-2Hz spin, centre aim
only lands when a panel happens to face the muzzle at impact, and nothing
times shots to the spin phase (firing logic is out of scope). Lead does
help: mean miss drops 18-38% at 1-4 m/s. Samples are 11-19 shots per cell,
so single-cell rates are noisy. Re-derive `MOVING_MIN_HIT_RATE` once shot
timing exists. Latest on `sentry_v2` (`sim/AGENTS.md`): flat 0.5/1.0 m/s
52%/43%, 4 m/s 9%, every staggered moving cell misses, and state leaks
between cases, so a full-run number isn't trustworthy yet.
