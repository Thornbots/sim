#!/bin/bash
# Run a suite in the container and finish with the verdict in its summary.json.
# guarded_exec and job control give the launch its own process group and deliver
# one SIGINT if this shell is interrupted or killed, even without a TTY.
set -uo pipefail

SUITE="${1:-}"; shift || true
case "$SUITE" in
    localization_drift) LAUNCH=(localization_tests.launch.py) ;;
    ekf) LAUNCH=(localization_tests.launch.py suite:=ekf) ;;
    aim) LAUNCH=(shot_hit.launch.py) ;;
    tracking) LAUNCH=(estimation.launch.py) ;;
    mcb_parked|mcb_drive|mcb_match) LAUNCH=(e2e.launch.py "stage:=$SUITE") ;;
    *) echo "usage: $0 localization_drift|ekf|aim|tracking|mcb_parked|mcb_drive|mcb_match [launch args...]" >&2; exit 2 ;;
esac

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGS=()
DIR=""
for arg in "$@"; do
    case "$arg" in
        log_dir:=*) DIR=${arg#*:=} ;;
        *) ARGS+=("$arg") ;;
    esac
done
if [ -n "$DIR" ]; then
    mkdir -p -- "$DIR" || exit 1
    DIR="$(cd -- "$DIR" && pwd -P)" || exit 1
else
    DIR="$(mktemp -d "${TMPDIR:-/tmp}/sim_suite_${SUITE}.XXXXXXXX")" || exit 1
fi

LOG="$DIR/launch.log"
# Reserve the run directory before doing anything that can produce evidence.
if [ -e "$DIR/summary.json" ] || ! (set -C; : > "$LOG"); then
    echo "run_suite.sh: use a fresh run directory ($DIR)" >&2
    exit 1
fi
TMP="$(mktemp -d "$DIR/.run.XXXXXXXX")" || exit 1
trap 'rm -rf -- "$TMP"' EXIT
START="$TMP/start"
touch "$START"
if [[ "$SUITE" != localization_drift && "$SUITE" != ekf ]]; then
    ARGS+=("log_dir:=$DIR")
fi

INTERRUPTED=0
LAUNCH_PID=""
forward() {
    if (( ! INTERRUPTED )); then
        INTERRUPTED=1
        if [ -n "$LAUNCH_PID" ]; then kill -INT "$LAUNCH_PID" 2>/dev/null || true; fi
    fi
}
trap forward INT TERM HUP

preflight() {
    local live arg artifact
    for artifact in stack.log scores.jsonl estimation.jsonl segments.json match.json \
        localization-logs .launch-complete; do
        if [ -e "$DIR/$artifact" ]; then
            echo "run directory already contains $artifact"
            return 1
        fi
    done
    for arg in "${ARGS[@]}"; do
        case "$SUITE:${arg%%:=*}" in
            mcb_*:stage) [ "$arg" = "stage:=$SUITE" ] || { echo "stage must match $SUITE"; return 1; } ;;
            ekf:suite) [ "$arg" = suite:=ekf ] || { echo "suite must match ekf"; return 1; } ;;
            localization_drift:suite) [ "$arg" = suite:=drift ] || { echo "suite must match localization_drift"; return 1; } ;;
        esac
    done
    live=$(ps aux | grep -E 'gz sim|gz_server|slam_toolbox|amcl|map_server|ekf_filter_node|pose_translator|pose_emulator|ros2 launch' \
        | grep -v -e grep -e run_suite.sh -e defunct)
    if [ -n "$live" ]; then
        echo "a stack is already running; stop it first (kill_launch.sh -l)"
        return 1
    fi
    case "$SUITE" in
        aim|tracking) ;;
        *) command -v gz >/dev/null || { echo "no gz in this container; run install-sim.sh first"; return 1; } ;;
    esac
}

LAUNCH_STATUS=""
CHECKER=""
FAILURE=$(preflight)
if [ -z "$FAILURE" ]; then
    PREFIX=$(ros2 pkg prefix sim 2>/dev/null)
    GUARD="$PREFIX/lib/sim/guarded_exec"
    CHECKER="$PREFIX/lib/sim/check_bench_log"
    if [ ! -x "$GUARD" ]; then
        FAILURE="sim is not built or sourced (no guarded_exec); see README#build"
    fi
fi

# Give prior runs' stale gz advertisements time to expire.
if [ -z "$FAILURE" ] && command -v gz >/dev/null; then
    for _ in $(seq 1 120); do
        (( INTERRUPTED )) && break
        [ -z "$(gz topic -l 2>/dev/null)" ] && break
        sleep 0.5
    done
fi

LOGS=()
if [ -z "$FAILURE" ] && (( ! INTERRUPTED )); then
    FIFO="$TMP/launch.fifo"
    if mkfifo "$FIFO"; then
        set -m
        tee -i "$LOG" < "$FIFO" &
        TEE=$!
        "$GUARD" ros2 launch sim "${LAUNCH[@]}" "${ARGS[@]}" > "$FIFO" 2>&1 &
        LAUNCH_PID=$!
        if (( INTERRUPTED )); then kill -INT "$LAUNCH_PID" 2>/dev/null || true; fi
        while :; do
            wait "$LAUNCH_PID"; LAUNCH_STATUS=$?
            if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
        done
        LAUNCH_PID=""
        while :; do
            wait "$TEE" 2>/dev/null; TEE_STATUS=$?
            if ! kill -0 "$TEE" 2>/dev/null; then break; fi
        done
        (( TEE_STATUS == 0 )) || FAILURE="launch log stream exited with status $TEE_STATUS"
        LOGS=("$LOG")
    else
        FAILURE="could not create launch log stream"
    fi
fi

if [[ -n "$LAUNCH_STATUS" && ( "$SUITE" == localization_drift || "$SUITE" == ekf ) ]]; then
    while IFS= read -r -d '' path; do
        INNER="$DIR/localization-logs"
        mkdir -p -- "$INNER"
        if cp -- "$path" "$INNER/"; then
            LOGS+=("$INNER/${path##*/}")
        else
            FAILURE="could not archive localization log: $path"
        fi
    done < <(find /tmp/localization_drift_tests -maxdepth 1 -name '*.log' -newer "$START" -print0 2>/dev/null)
elif [[ -n "$LAUNCH_STATUS" && ( "$SUITE" == mcb_* || -f "$DIR/stack.log" ) ]]; then
    LOGS+=("$DIR/stack.log")
fi

CAPTURE=()
if [ "${SIM_SUITE_CAPTURE:-0}" = 1 ]; then
    touch "$DIR/.launch-complete"
    for _ in $(seq 1 450); do
        [ -f "$DIR/.capture-result.json" ] && break
        (( INTERRUPTED )) && break
        sleep 0.1
    done
    CAPTURE=(--capture-result "$DIR/.capture-result.json")
fi

CHECK_ARGS=()
for path in "${LOGS[@]}"; do CHECK_ARGS+=(--log "$path"); done
python3 "$SELF/suite_summary.py" --directory "$DIR" --suite "$SUITE" \
    --launch-status "$LAUNCH_STATUS" --interrupted "$INTERRUPTED" --failure "$FAILURE" \
    --checker "$CHECKER" "${CHECK_ARGS[@]}" "${CAPTURE[@]}" -- "${LAUNCH[@]}" "${ARGS[@]}"
