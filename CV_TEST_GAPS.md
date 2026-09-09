# CV test gaps

Open gaps found reviewing the CV test suites on 2026-09-09. Spans two
submodules, so it lives here rather than in either one.

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

## 2. `shooter_pos` and `shooter_vel` have no coverage

`shooter_pos` is `(0, 0, 0)` in all six tests and `shooter_vel` is never
passed at all. Delete both parameters from `point_to_cv_target_core.py`
and the whole file still passes — the chassis-velocity correction is
deliberate work with zero coverage. Needs at least one case with the
shooter off-origin and one with it moving.

## 3. `KalmanFilter6D.predicted()` hands out the live state array

`target_tracker_core.py:168` returns `self.state` (not a copy) on the
`dt <= 0` branch. A caller mutating the returned array corrupts the
filter. `test_kf_predicted_at_or_before_filter_time_is_current_state`
(`test_target_tracker.py:147`) uses `allclose`, so it cannot see this.

Fix is `return self.state.copy(), np.diag(self.P)`, plus a test that
mutates the return and asserts `kf.state` is unchanged. Small, but it is
a real aliasing bug rather than only a test gap.

## 5. `test_shot_hit.py` is a measurement harness with a smoke assertion

The assertions are `shots_fired > 0` (:109) and one hit in the stationary
case (:115). The lead feature — the entire reason for the `lead=both`
parameterization — has no pass/fail, so it could regress from 60% to 5%
and the suite stays green. `shot_hit_harness.py`'s docstring says this
outright, which is honest, but it leaves the CV integration story as "a
human reads the printed table."

Two upgrades that don't require pinning a noisy absolute number:

- Make the stationary assertion a rate (`hits / shots >= 0.5`) instead of
  `>= 1`. A pipeline that lands 1 hit in 200 shots at a motionless target
  is broken and currently passes.
- Assert `lead_on >= lead_off - margin` at the slowest moving speed,
  where run-to-run variance is lowest. Needs the two lead values compared
  within one test rather than across two parameterized cases.

## 6. Smaller coverage gaps

- **`compute_score`** — `center_weight` is `1.0` in both tests
  (`test_target_selector.py:90`, `:101`), so applying the weight to the
  wrong term would pass. One case with `center_weight=0.3` closes it.
- **`RobotHysteresis`** — the streak-reset branch (a challenger appears,
  then its margin lapses before `switch_hold_frames` is reached, so the
  streak must go back to 0) is untested. That is the flicker-resistance
  path.
- **`group_panels`** — neighbours sit at 0.384 m and 3.0 m against a
  0.4 m radius; nothing is near the boundary, and the tradeoff the
  docstring calls out (two robots merging when their nearest panels fall
  inside the radius) is not pinned.
- **`SpinDetector`** — `spin_phase` is unpacked and discarded in all four
  tests, and the 4-panels-per-revolution factor is only exercised at the
  single `spin_hz == 1.0` point.
