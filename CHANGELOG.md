# Changelog

## Unreleased

- Adopt the team's 2026-10-26 low-resolution CAD (#48) and remove
  `simplify_urdf`, its `sentry_v2.yaml` config and vendored mesh decimator;
  edit `urdf/sentry_v2/` directly. `camera` is the CAD's RGB FOV apex
  (detections are RGB-deprojected), not the IR apex.
- Shots fly at the sentry's 22 m/s (was 24 in E2E ballistics, 25 in the
  gz-free aiming bench and ghost lead), matching MCBV3 and `v_muzzle`.
- Match the robot's timed 20 ms indexer request-to-exit: a ball now leaves
  0.045 ball into a feed from the pre-indexed rest (was the next chamber
  boundary, 94 ms), once per feed. E2E stages pass `firmware_latency_s` 0.020.
  Hosted firmware tests pin the 20 ms exit and the held pending shot.
- Record where each scored shot crosses its panel (`panel_right_m`,
  `panel_up_m`, and `crossing_panel` for ballistic stages) in `shots.jsonl`.
  Hosted MCB firmware tests expect the barrel-offset yaw from MCBV3.
- Bound E2E referee parameter requests by a two-second steady-clock deadline.
  Lost responses now release the pending request and resend current match
  state; three consecutive timeouts fail explicitly. This prevents a lost
  bring-up response from leaving driving stages in `drive=stop`. Physical
  robot control is unaffected.
