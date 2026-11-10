#!/bin/bash
# Run one integration suite in the container, then check its logs.
#
#   tools/run_suite.sh localization_drift|ekf|aim|tracking|mcb_parked|mcb_drive|mcb_match [launch args...]
#
# Refuses to start on top of a live stack or without gz, tees the launch to
# /tmp/sim_suite_<suite>_<time>.log, and ends with the native check_bench_log
# on that log (plus this run's /tmp/localization_drift_tests/*.log for
# localization_drift and ekf). Exits 0 only if the checker trusts the run and
# GTest passed.
#
# The launch runs in its own process group, behind guarded_exec. Ctrl-C, SIGTERM
# and SIGHUP here become one SIGINT to the launch leader, which forwards it to
# its children once; a SIGKILLed shell delivers it through the kernel's
# parent-death signal. No TTY is needed.
set -uo pipefail

SUITE="${1:-}"; shift || true
case "$SUITE" in
    localization_drift) LAUNCH=(localization_tests.launch.py) ;;
    ekf)        LAUNCH=(localization_tests.launch.py suite:=ekf) ;;
    aim)        LAUNCH=(shot_hit.launch.py) ;;
    tracking)   LAUNCH=(estimation.launch.py) ;;
    mcb_parked|mcb_drive|mcb_match) LAUNCH=(e2e.launch.py "stage:=$SUITE") ;;
    *) echo "usage: $0 localization_drift|ekf|aim|tracking|mcb_parked|mcb_drive|mcb_match [launch args...]" >&2; exit 2 ;;
esac

LIVE=$(ps aux | grep -E 'gz sim|slam_toolbox|amcl|map_server|ekf_filter_node|pose_translator|pose_emulator|ros2 launch' \
    | grep -v -e grep -e run_suite.sh -e defunct)
if [ -n "$LIVE" ]; then
    echo "$LIVE"
    echo "run_suite.sh: a stack is already running; stop it first (kill_launch.sh -l)." >&2
    exit 1
fi
if ! command -v gz >/dev/null; then
    case "$SUITE" in
        aim|tracking) ;;
        *) echo "run_suite.sh: no gz in this container; run install-sim.sh first." >&2; exit 1 ;;
    esac
fi
GUARD="$(ros2 pkg prefix sim 2>/dev/null)/lib/sim/guarded_exec"
if [ ! -x "$GUARD" ]; then
    echo "run_suite.sh: sim is not built or sourced (no guarded_exec); see README#build." >&2
    exit 1
fi

# A run that follows another within seconds can find the last one's gz-transport advertisements
# still listed; discovery of the new gate's release channel then outlasts the first hold's 2 s.
# Wait for them to expire (the listing empties) before starting.
if command -v gz >/dev/null; then
    for _ in $(seq 1 120); do
        [ -z "$(gz topic -l 2>/dev/null)" ] && break
        sleep 0.5
    done
fi

LOG="/tmp/sim_suite_${SUITE}_$(date +%Y%m%d_%H%M%S).log"
START=$(mktemp); FIFO=$(mktemp -u)
mkfifo "$FIFO"
cleanup() { rm -f "$START" "$FIFO"; }
trap cleanup EXIT

# Job control gives the launch its own process group, so a tty's Ctrl-C reaches
# only this script; the traps below pass it on exactly once.
set -m
tee -i "$LOG" < "$FIFO" &
TEE=$!
"$GUARD" ros2 launch sim "${LAUNCH[@]}" "$@" > "$FIFO" 2>&1 &
LAUNCH_PID=$!
INTERRUPTED=0
forward() { INTERRUPTED=1; kill -INT "$LAUNCH_PID" 2>/dev/null; }
trap forward INT TERM HUP
LAUNCH_STATUS=0
while :; do  # a trapped signal ends `wait` early with status > 128
    wait "$LAUNCH_PID"; rc=$?
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then LAUNCH_STATUS=$rc; break; fi
done
wait "$TEE" 2>/dev/null
trap - INT TERM HUP
set +m

LOGS=("$LOG")
if [[ "$SUITE" == localization_drift || "$SUITE" == ekf ]]; then
    mapfile -t DRIFT < <(find /tmp/localization_drift_tests -name '*.log' -newer "$START" 2>/dev/null)
    LOGS+=("${DRIFT[@]}")
fi
echo
echo "== check_bench_log (launch log: $LOG)"
ros2 run sim check_bench_log "${LOGS[@]}"; CHECK=$?
# The native suites preserve the historical summary line for log consumers.
SUMMARY=$(sed 's/\x1b\[[0-9;]*m//g' "$LOG" | grep -E '=+ .* in [0-9.]+s( \([0-9:]+\))? =+' | tail -1)
if (( INTERRUPTED )); then
    echo "run_suite.sh: interrupted; the launch exited with status $LAUNCH_STATUS." >&2
    exit 130
fi
if [[ "$SUMMARY" != *passed* || "$SUMMARY" =~ failed|error ]]; then
    echo "run_suite.sh: suite did not pass: ${SUMMARY:-no summary line}" >&2
    exit 1
fi
if (( LAUNCH_STATUS != 0 )); then
    exit "$LAUNCH_STATUS"
fi
exit $CHECK
