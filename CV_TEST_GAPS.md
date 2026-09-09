# CV test gaps

Open gaps found reviewing the CV test suites on 2026-09-09. Spans two
submodules, so it lives here rather than in either one.

Gaps 1, 3, 5 and 6 are closed. Gap 2 is half closed and half deliberately
deferred (the moving-shooter case, with the scaffolding for it in place).
Gap 7 is new: strengthening gap 5's assertions surfaced a real aim bug.

**The shot-hit suite is red, and that is the intended state.** The aiming
and firing code is known-wrong and is being left that way until someone
works on it; these tests describe what it must do, not what it does. Do
not relax a threshold to green them.

Already fixed, not listed below: the URDF-constant pin
(`sim/test/cv/test_urdf_constants.py`), the duplicated docstring summaries,
the in-body settle sleep, and the tautological intercept tests (gap 1).

What's already solid and should stay that way:
`test_lagged_measurement_time_recovers_true_velocity`,
`test_kf_predicted_extrapolates_without_mutating`,
`test_corrected_centre_extends_along_off_axis_bearing`, and
`test_random_angles_round_trip` all encode real past bugs in physics terms
rather than restating the implementation.

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

One fixture trap worth knowing: `n_handoffs` and the `2.0`-second probe time
matter. See gap 6's `SpinDetector` note.

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

Neither does useful work yet, and that is gap 7's fault rather than
theirs. The rate test fails outright. The lead comparison *passes*, but
vacuously — measured 2026-09-09 at 0.5 m/s / 2.0 Hz spin, lead off hit
0/22 and lead on 0/23, so `0.0 >= 0.0 - 0.15` holds and tells you nothing.
It starts discriminating the moment the aim offset is fixed.

Beyond the two gap-5 upgrades, the moving sweep now has a pass condition
of its own (`MOVING_MIN_HIT_RATE`), because aiming at a moving, spinning
target is what this bench exists for and asserting only `shots_fired > 0`
on those cells left the stated purpose untested. It applies to the
lead=ON cells only — lead=OFF is the control leg and is expected to aim
worse at speed, so holding it to the same floor would be asserting that
the control works.

`MOVING_MIN_HIT_RATE = 0.25` is the one threshold in this suite that is
**not** measured. Gap 7 means there is no working baseline to measure
against; it states the intent in code and should be re-derived from a
real sweep once aiming lands. Expect it to go up.

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

## 7. The stack misses a stationary target by a constant 0.884 m

Found 2026-09-09 running gap 5's new assertions, headless, in the dev
container. Not a test gap — an aim bug the tests were previously too weak
to show. **Known and deliberately unfixed**: the aiming and firing code is
being left as-is until it's worked on, so this section is a starting point
for whoever picks that up, not an open action item.

```
stationary, lead=off | shots= 28 | hits= 0 (0.0%) | miss mean= 0.884 max= 0.884 m
stationary, lead=on  | shots= 26 | hits= 0 (0.0%) | miss mean= 0.884 max= 0.884 m
```

`mean == max` across every shot, and the figure is *identical* with lead on
and off. So it is not scatter, not tracking error, and not the intercept
solve — it looks like a fixed geometric offset in the aim path or in the
harness's duplicated FK chain. The target is motionless, so prediction is
not involved at all.

The moving cases *do* scatter (0.5 m/s: miss mean 0.72 m, max 2.26 m), so
the constancy is specific to the motionless target and there is plausibly
a second, motion-dependent error stacked on the fixed one. Fix the
constant offset first and re-measure before chasing that.

Both stationary cells of `test_shot_hit` were already failing on the older
`hits >= 1` assertion before any of this work, so the suite was red on
arrival. Nothing here is marked `xfail`: the miss is a live bug, and
`xfail` would encode it as expected forever.

Worth checking first, in rough order of suspicion: the `corrected_centre`
radius push (`target_tracker_core.py`), the `head_link`/`head_pitch` FK
constants duplicated between `shot_hit_harness.py` and
`cv_target_emulator.py`, and the panel-centre-vs-chassis-centre convention
where `target_selector` hands off to `target_tracker`.

0.884 m matches no single constant in the chain. Searching sums of the five
(`PANEL_RADIUS_X` 0.30, `PANEL_RADIUS_Y` 0.24, `head_link` z 0.252215,
`head_pitch` x 0.1 and z 0.1218) turns up exactly one near-match,
0.8922 m — 8 mm off, and four terms drawn from five is enough freedom that
this is a lead to check, not evidence. Measure the offset direction in the
odom frame before trusting any of it.
