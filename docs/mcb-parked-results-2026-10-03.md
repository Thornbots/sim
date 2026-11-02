# Parked firmware results, 2026-10-03

E2 on `position-based-cv` with only the `delay_ms` fix (2026-10-03, container,
lateral, still and 1 m/s, 15 s each): `CV_TARGET` and `RELOCALIZE` frames
are read, none refused, but the gun turns away. One `RELOCALIZE` moved the
parked MCB's odometry to (0.74, 1.86), our `odom`'s numbers, and the head
sat at `head_yaw` 4.13 rad with the opponent straight ahead. Still: no
valid `TargetState`, no shots. 1 m/s: the tracked centre 3.17 m off, 0 of
6 shots hit. `pose_translator` reads POSE's x right, y forward as REP-105,
so our `odom` and the MCB's odometry differ by a turn.

With all three fixes (2026-10-03, container, still lateral cell): the
aim lands within 3 mm of the panel and the barrel 1.55 deg above it, a
24 m/s shot's drop over 3.2 m, so the stages now score shots under gravity.
Then 28 of 40 hit (70%, barrel 0.36 deg off the aim, miss 0.020 m)
on a clean track (centre 0.097 m), 2 of 40 and 1 of 30 on runs where
the tracker read 1.6-1.9 m/s for the still target (T17).


