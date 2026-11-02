# Estimation harness reset and pacing — 2026-11-02

## Scope and diagnosis

Recovered from the local `t3-8f044d4e` checkout without modifying it; applied
to sim nightly `bcf9d86295c4a3ac0542f0cc17dfb1866b52dd16`.
This change does not alter estimator math, limits, fixture acquisition,
scan/map readiness, firmware, or the 0.5 s scored lockstep watchdog.

Track G's retained Mini baseline logs establish invalid harness runs:

- `default-r1` and `diagonal-r1`: native 12/12, rejected for
  `/bench/progress` timeout. Native numeric success is insufficient.
- `radial-r1`: `/bench/case 2127518508` times out after 30 s at the second
  cell; native 11/12, invalid.

Original evidence is under
`/Users/blaise/.t3/worktrees/src/t3-5b49f1f1/artifacts/track-g-2026-11-02/baseline/`.

`SimTimeNode::set_params()` spins its executor while waiting for a service
response. The new `/bench/case` callback can therefore run before it returns.
Previously `start_case()` then cleared that acknowledgement. Clear the sampler
before the request, so either response/callback ordering retains the case.

The scorer remains a publisher while writing cell results, but does not spin
truth callbacks then. Publisher discovery cannot identify this idle period.
The recovered patch used an idle sentinel in `Header.frame_id`; that field
should identify a coordinate frame. The implementation instead acknowledges
an explicit `/bench/pacing` service handoff while truth callbacks are still
serviced. The world ignores delayed progress samples while inactive. Resume
clears stale progress and retains the existing first-sample activation rule.
All tracker/aim gates remain active throughout the handoff.

## Validation and unfinished gates

Host quality/actionlint results and exact candidate SHA are recorded in the PR.
No runtime pass is claimed by this document. At recovery, Season2 owns the
quiet Mini slot and D/E is host-only. The prepared finite batch runs default,
radial and diagonal three times each, requires all 12 native cells and native
log acceptance, and stops at the first invalid run. It must run in a new owned
2 CPU / 4 GB container after an explicit capacity grant, with one compiler.

Required before G resumes accuracy measurements: successful registered tests,
nine accepted tracking runs, and approval of exact refs by a different fresh
reviewer. Then run the broader lifetime stress and RTF work separately.
DDS teardown races, SIGKILL-only shutdown, matching-library aarch64 scan-bridge
backtrace, named RTF bottleneck/rendering measurements, integrated exact-SHA
regression, and the Mac–Arch comparison remain open. The comparison cannot run
under the Mini-only authorization. No acceptance limit or baseline is relaxed.
