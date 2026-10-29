# Changelog

## Unreleased

- Record where each scored shot crosses its panel (`panel_right_m`,
  `panel_up_m`, and `crossing_panel` for ballistic stages) in `shots.jsonl`.
  Hosted MCB firmware tests expect the barrel-offset yaw from MCBV3.
- Bound E2E referee parameter requests by a two-second steady-clock deadline.
  Lost responses now release the pending request and resend current match
  state; three consecutive timeouts fail explicitly. This prevents a lost
  bring-up response from leaving driving stages in `drive=stop`. Physical
  robot control is unaffected.
