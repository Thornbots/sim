# CV bench observations (2026-09-28 to 2026-09-29)

Historical results retained from the retired CV split plan. They rank changes,
not field hit rates. Current commands and bench behavior are in the
[simulation README](../README.md#run-the-tests); open accuracy work is in
[ROADMAP track G](../../ROADMAP.md#g-estimation-accuracy).

- Aiming passed its per-cell floors with chase mode. The perfect-target
  aiming bench has no estimation noise; simulation detection noise was
  0.005 m. Slew limits and target acceleration are estimates.
- Estimation covered 60 cells with limits at 2x the worst of three to six
  runs, floored at 0.02 (2026-09-27). Stationary facing-panel p95 was under
  2 cm; moving was 0.08-0.19 m, with camera latency 0.03 s and our chassis
  at 1 m/s.
- Radial motion at 2-4 m/s doubled along-ray error to 0.18-0.25 m, versus
  0.09-0.11 m. Blackouts (0.3 s every 2 s) were 2-3x worse; staggered
  0.5-1 m/s reached 0.28-0.30 m, versus 0.09 m.
- About a quarter of runs tripped a limit on a spin-rate or radius outlier.
  Bad 4 m/s runs held radius error at 5-12 cm for 10-20 s, versus about
  2 cm normally; spin rate could be misread by 0.6-2 rad/s.
- With blackout, staggered 1-4 m/s cells had z-offset p95 0.012-0.062 m
  (run medians), versus flat's 0.002 m, with stagger 0.095 m. Panel error
  at 2-4 m/s was 1.4-1.6x flat. Without blackout, staggered matched flat:
  panel error within 0.9-1.2x and z-offset under 4 mm.
- Fresh spinner tracks took 0.3-3 s to settle, up to 0.4 m off in the
  first second. Spin-rate variance did not separate settled tracks.
- C++ target_tracker on the Mac bench used 33% of a core at about 27x,
  versus Python's 110% at about 14x (2026-09-28).
- The 12 chassis-spin-9 cells matched their spin-0 twins (2026-09-29),
  including optional bearing drag; they received limits. The aiming
  bench's perfect gimbal makes chassis spin under heading-fixed root a
  no-op. Moving-shooter radial/diagonal floors remained missing.
- Flat 4 m/s misses clustered at unpredicted acceleration changes at path
  ends. This was a known limitation without a separate fix plan.
