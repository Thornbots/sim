# Suite names, 2026-11-01

Historical evidence; live actions and acceptance are maintained in
[ROADMAP](../../docs/ROADMAP.md) and [ROBOT_TESTS](../../docs/ROBOT_TESTS.md).
Statements about draft PRs or remaining work below describe the dated checkpoint.

The suite wrapper now accepts `localization_drift`, `ekf`, `aim`, `tracking`,
`mcb_parked`, `mcb_drive` and `mcb_match`. A stubbed ROS command exercised
each selection once and confirmed its launch and stage arguments; this is a
routing check, not a simulation result. The previous wrapper names are no
longer accepted.
