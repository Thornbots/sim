# Track A input-audit checkpoint, 2026-11-02

This is preparation for T17, not parked repeatability or combat acceptance.
All execution stayed on the Mac Mini in worktree
`/Users/blaise/.t3/worktrees/src/t3-28bbd5b3`. Evidence is saved at
`/Users/blaise/track-a-evidence/2026-11-02-mini/`.

## Candidate and checks

The workspace started at `4eb0da9`, advanced to nightly `a522d7a`, and
recovered the pushed idle-look change from workspace #159 / thornbots_pkg #53.
The build used sim `b2c1acc`, thornbots_pkg `a32303a`, bridge `9845591`,
MCBV3 `a16c9aa`, and localization `f248c55`, plus the reconstructed sim diff.
The owned container was `isaac-agent-track-a-28bb`, capped at two CPUs and
4 GB, with one compiler per build. Hosted and board firmware were built
locally; Renode 1.17.0 was installed for arm64.

| Check | Result | Evidence file |
| --- | --- | --- |
| Workspace/package quality and actionlint | pass | `*-quality.log`, `*-actionlint.log` |
| thornbots_pkg registered tests | 279 pass, no skips | `thornbots-test-results.log` |
| Hosted firmware tests after reconstructed pitch correction | 91 pass | `firmware-tests-after.log` |
| sim registered tests on reconstructed candidate | 234 pass, no skips | `sim-test-results.log` |
| Container smoke / package resolution | pass; clean graph | `smoke.log` |
| Scored campaigns | **not run** | prepared scripts only |

The registered sim results include the now-deferred pitch/acquisition/startup
candidate patches; they are not verification of the final integrated refs.
The old pitch conversion returned 0.494 rad for a 0.2 rad input; the new
round-trip assertion failed before the correction and passed afterward
(`pitch-before.log`, `firmware-tests-after.log`). A pre-existing GCC
`-Wmaybe-uninitialized` warning on `ProjectileExit::rest` remains in the
build log; no warning or acceptance threshold was disabled.

## Ownership and remaining work

The Season 2 implementation owns acquisition, pitch conversion, projectile
counting and its readiness fixtures. Reconstructed overlapping edits were
removed from A's active diff and saved as `deferred-foundational.patch`.
The complete earlier candidate is `reconstructed-all-before-ownership.patch`.
No reconstructed overlapping sim fix was committed or landed by A. thornbots_pkg #53 and the
workspace #159 integration remain on hold pending the owning #52/#63 stack.

A's retained change writes `inputs.jsonl` for empty/nonempty detections,
selected-panel captures, raw odometry, MCB pose, valid/invalid target states
and aim. It records header stamps separately from scorer receipt times.
This is observation, not a consumption acknowledgement or a simulated UART.

The prepared batches and `summarize-inputs.py` remain in the evidence directory.
Rebase onto the merged foundation before running them. Retain every failed
startup and native checker result. Inspect the actual input traces and lost
tracks, finish UART/consumer and deterministic bring-up gates, compare repeated
shot sequences, then calibrate floors from accepted repeated runs. No new
floor or repeatability claim is supported by this checkpoint.

The owned container and its volumes were removed. Colima remained running
because other agents' containers were active; none of those were altered.
