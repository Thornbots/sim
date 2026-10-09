#!/bin/bash
# Run one integration suite in the container, then check its logs.
#
#   tools/run_suite.sh drift|ekf|shot_hit|estimation [launch args...]
#
# Refuses to start on top of a live stack, tees the launch to
# /tmp/sim_suite_<suite>_<time>.log, and ends with the native check_bench_log
# on that log (plus this run's /tmp/localization_drift_tests/*.log for drift
# and ekf). Exits 0 only if the checker trusts the run and GTest passed.
# Needs a TTY (docker exec -it) for Ctrl-C to reach the stack.
set -uo pipefail

SUITE="${1:-}"; shift || true
case "$SUITE" in
    drift)      LAUNCH=(localization_tests.launch.py) ;;
    ekf)        LAUNCH=(localization_tests.launch.py suite:=ekf) ;;
    shot_hit)   LAUNCH=(shot_hit.launch.py) ;;
    estimation) LAUNCH=(estimation.launch.py) ;;
    mcb_parked|mcb_drive|mcb_match) LAUNCH=(e2e.launch.py "stage:=$SUITE") ;;
    *) echo "usage: $0 drift|ekf|shot_hit|estimation|mcb_parked|mcb_drive|mcb_match [launch args...]" >&2; exit 2 ;;
esac

LIVE=$(ps aux | grep -E 'gz sim|slam_toolbox|amcl|map_server|ekf_filter_node|pose_translator|pose_emulator|ros2 launch' \
    | grep -v -e grep -e run_suite.sh -e defunct)
if [ -n "$LIVE" ]; then
    echo "$LIVE"
    echo "run_suite.sh: a stack is already running; stop it first (kill_launch.sh -l)." >&2
    exit 1
fi
if [[ "$SUITE" == drift || "$SUITE" == ekf ]] && ! command -v gz >/dev/null; then
    echo "run_suite.sh: no gz in this container; run install-sim.sh first." >&2
    exit 1
fi

LOG="/tmp/sim_suite_${SUITE}_$(date +%Y%m%d_%H%M%S).log"
START=$(mktemp); trap 'rm -f "$START"' EXIT
# A handler, not an ignore: Ctrl-C still reaches ros2 launch (handlers reset
# on exec), and this script survives it to run the checker.
trap 'true' INT
ros2 launch sim "${LAUNCH[@]}" "$@" 2>&1 | tee -i "$LOG"
LAUNCH_STATUS=${PIPESTATUS[0]}
trap - INT

LOGS=("$LOG")
if [[ "$SUITE" == drift || "$SUITE" == ekf ]]; then
    mapfile -t DRIFT < <(find /tmp/localization_drift_tests -name '*.log' -newer "$START" 2>/dev/null)
    LOGS+=("${DRIFT[@]}")
fi
echo
echo "== check_bench_log (launch log: $LOG)"
ros2 run sim check_bench_log "${LOGS[@]}"; CHECK=$?
# The native suites preserve the historical summary line for log consumers.
SUMMARY=$(sed 's/\x1b\[[0-9;]*m//g' "$LOG" | grep -E '=+ .* in [0-9.]+s( \([0-9:]+\))? =+' | tail -1)
if [[ "$SUMMARY" != *passed* || "$SUMMARY" =~ failed|error ]]; then
    echo "run_suite.sh: suite did not pass: ${SUMMARY:-no summary line}" >&2
    exit 1
fi
if (( LAUNCH_STATUS != 0 )); then
    exit "$LAUNCH_STATUS"
fi
exit $CHECK
