# Suite names, 2026-11-01

The suite wrapper now accepts `localization_drift`, `ekf`, `aim`, `tracking`,
`mcb_parked`, `mcb_drive` and `mcb_match`. A stubbed ROS command exercised
each selection once and confirmed its launch and stage arguments; this is a
routing check, not a simulation result. The previous wrapper names are no
longer accepted.
