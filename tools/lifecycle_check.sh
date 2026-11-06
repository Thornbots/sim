#!/bin/bash
# Lifecycle acceptance: end each suite by completion, Ctrl-C, SIGTERM, SIGHUP
# and a SIGKILLed shell, and assert nothing survives and nothing was killed.
#
#   tools/lifecycle_check.sh [--repeat N] [--out DIR] [SUITE:ENDING ...]
#
# SUITE   mcb_parked | mcb_drive | mcb_match | tracking   (shortened cells)
# ENDING  complete | int | term | hup | kill
#         int/hup signal run_suite.sh's whole process group (what a terminal
#         does); term and kill signal the shell alone, kill with SIGKILL.
#         The signal follows the first test starting by AT seconds (default
#         3); AT=bringup signals 12 s after the launch starts instead.
# With no SUITE:ENDING the full matrix runs. Run it in the container with sim
# built and sourced. BACKEND (hosted|renode) and FIRMWARE pick the MCB build;
# SUITE_ARGS_<suite> replaces a suite's launch arguments.
#
# Per run it asserts: run_suite.sh is gone within BOUND seconds of the signal
# (or, for `complete`, ends by itself: exit 0, or exit 1 with the suite's own
# summary line when only its score gate failed, which is counted and reported); no process, gz topic, MCB pty
# link or temporary world is left within GRACE seconds; and no log shows
# SIGKILL, SIGTERM escalation, a died process or an unclean-shutdown note.
# Exit 0 only if every run passed.
set -uo pipefail

REPEAT=1; OUT=/tmp/lifecycle
BOUND=${BOUND:-60}; GRACE=${GRACE:-10}; AT=${AT:-3}; COMPLETE_BOUND=${COMPLETE_BOUND:-1500}
BACKEND=${BACKEND:-hosted}
FIRMWARE=${FIRMWARE:-/workspaces/isaac_ros-dev/src/firmware/MCBV3/MCB-project/build/sim/scons-release/MCB-project.elf}
CASES=()
while (( $# )); do
    case "$1" in
        --repeat) REPEAT=$2; shift 2 ;;
        --out) OUT=$2; shift 2 ;;
        *) CASES+=("$1"); shift ;;
    esac
done
if (( ${#CASES[@]} == 0 )); then
    for suite in mcb_parked mcb_drive mcb_match tracking; do
        for ending in complete int term hup kill; do CASES+=("$suite:$ending"); done
    done
fi

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_SUITE="$SELF/run_suite.sh"
PATTERN='/install/|gz sim|ruby |MCB-project|renode|mcb_type_c|ros2 (launch|run)|guarded_exec|tee -i|e2e_suite|estimation_suite|run_suite'
BAD_LOG='SIGKILLing|did not exit within|outlived the launch|sending signal .SIG(TERM|KILL).|process has died|unclean shutdown'
mkdir -p "$OUT"

suite_args() {
    local var="SUITE_ARGS_$1"
    if [ -n "${!var:-}" ]; then echo "${!var}"; return; fi
    case "$1" in
        mcb_parked) echo "backend:=$BACKEND firmware_binary:=$FIRMWARE speeds:=0 paths:=lateral duration:=10 headless:=true" ;;
        mcb_drive|mcb_match) echo "backend:=$BACKEND firmware_binary:=$FIRMWARE headless:=true" ;;
        tracking) echo "headless:=true foxglove:=false" ;;
    esac
}

live() {  # non-zombie processes that belong to a sim stack, not this script's ancestors
    local skip=" $$ " p=$$
    while [ "${p:-1}" -gt 1 ]; do p=$(ps -o ppid= -p "$p" | tr -d ' '); skip+="$p "; done
    ps -eo pid,ppid,pgid,stat,args | awk -v skip="$skip" 'NR > 1 && $4 !~ /^Z/ && index(skip, " " $1 " ") == 0' \
        | grep -E "$PATTERN" | grep -v -e grep -e lifecycle_check -e "ps -eo" -e "python3 -" -e "bash -lc"
}

leftovers() {  # processes, gz topics, pty links, temporary worlds
    live
    local topics; topics=$(timeout 8 gz topic -l 2>/dev/null)
    [ -z "$topics" ] || echo "gz topics in partition ${GZ_PARTITION:-?}: $(echo "$topics" | head -3 | tr '\n' ' ')"
    ls /tmp/mcb_emulator_pty-* /tmp/sim_world_*.sdf 2>/dev/null
}

# Starts the suite in its own session with default signals, redirects its output
# and stays behind as the reaper that records the exit status.
cat > "$OUT/spawn.py" <<'PY'
import os, signal, sys
pidfile, statusfile, logfile = sys.argv[1:4]
pid = os.fork()
if pid == 0:
    os.setsid()
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT):
        signal.signal(s, signal.SIG_DFL)
    fd = os.open(logfile, os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
    os.dup2(fd, 1); os.dup2(fd, 2); os.close(fd)
    os.execvp(sys.argv[4], sys.argv[4:])
open(pidfile, 'w').write(str(pid))
_, status = os.waitpid(pid, 0)
open(statusfile, 'w').write(str(os.waitstatus_to_exitcode(status)))
PY

now() { date +%s; }
gone() { ! kill -0 "$1" 2>/dev/null; }

run_case() {  # run_case SUITE ENDING INDEX
    local suite=$1 ending=$2 index=$3 dir fail=() pid signalled="" t_signal t_clean
    dir="$OUT/$(printf '%03d' "$index")-$suite-$ending"; mkdir -p "$dir"
    echo "== [$index] $suite:$ending ($BACKEND)"
    if [ -n "$(leftovers)" ]; then
        leftovers > "$dir/preexisting.txt"; echo "   FAIL: leftovers before start: $(head -2 "$dir/preexisting.txt" | tr '\n' '|')"; return 1
    fi
    rm -f "$dir/pid" "$dir/status"
    # shellcheck disable=SC2046
    python3 "$OUT/spawn.py" "$dir/pid" "$dir/status" "$dir/run.log" \
        "$RUN_SUITE" "$suite" $(suite_args "$suite") "log_dir:=$dir" &
    local reaper=$!
    for _ in $(seq 50); do [ -s "$dir/pid" ] && break; sleep 0.1; done
    pid=$(cat "$dir/pid" 2>/dev/null)
    [ -n "$pid" ] || { echo "   FAIL: could not start"; return 1; }
    local t0; t0=$(now)

    if [ "$ending" != complete ]; then
        local trigger='\[ RUN '
        [ "$AT" = bringup ] && trigger='process started with pid'
        until grep -q "$trigger" "$dir/run.log" 2>/dev/null; do
            gone "$pid" && break
            (( $(now) - t0 > 900 )) && break
            sleep 1
        done
        if [ "$AT" = bringup ]; then sleep 12; else sleep "$AT"; fi
        if gone "$pid"; then
            fail+=("suite ended before the signal")
        else
            t_signal=$(now); signalled=1
            case "$ending" in
                int) kill -INT -- "-$pid" ;;
                hup) kill -HUP -- "-$pid" ;;
                term) kill -TERM "$pid" ;;
                kill) kill -KILL "$pid" ;;
            esac
        fi
    fi

    # Wait for the shell (or, after a kill, for the whole stack) to end.
    local limit=$BOUND; [ "$ending" = complete ] && limit=$COMPLETE_BOUND
    local t_wait; t_wait=$(now)
    if [ "$ending" = kill ]; then
        until [ -z "$(live)" ]; do
            (( $(now) - t_wait > limit )) && { fail+=("stack still alive $limit s after SIGKILL of the shell"); break; }
            sleep 1
        done
    else
        until gone "$pid"; do
            (( $(now) - t_wait > limit )) && { fail+=("run_suite.sh still alive $limit s after ${signalled:+the signal}"); break; }
            sleep 1
        done
    fi
    t_clean=$(now)
    # Give the reaper and init a moment, then judge.
    for _ in $(seq "$GRACE"); do [ -z "$(leftovers)" ] && break; sleep 1; done
    leftovers > "$dir/leftovers.txt"
    [ -s "$dir/leftovers.txt" ] && fail+=("leftovers: $(head -3 "$dir/leftovers.txt" | tr '\n' '|')")
    sleep 1; wait "$reaper" 2>/dev/null
    local status; status=$(cat "$dir/status" 2>/dev/null || echo none)
    echo "$status" > "$dir/exit_status"
    if [ "$ending" = complete ] && [ "$status" != 0 ]; then
        # A failed score gate is the suite's verdict, not a lifecycle fault: the
        # run still reached its own end (summary line) and shut down below.
        if [ "$status" = 1 ] && grep -q -E '=== [0-9]+ passed.* in [0-9.]+s ===' "$dir/run.log"; then
            echo "   note: suite verdict failed (exit 1); see $dir/run.log" | tee "$dir/verdict-failed"
        else
            fail+=("suite exited with $status")
        fi
    fi
    if [ "$ending" = int ] || [ "$ending" = term ] || [ "$ending" = hup ]; then
        [ "$status" = 130 ] || fail+=("interrupted run exited with $status, expected 130")
    fi
    local logs=("$dir/run.log")
    [ -f "$dir/stack.log" ] && logs+=("$dir/stack.log")
    # The suite's own exit code is the interrupted test's verdict, not a shutdown fault.
    if grep -E -h "$BAD_LOG" "${logs[@]}" 2>/dev/null | grep -v -E '\[[a-z0-9_]+_tests-[0-9]+\]: process has died' \
            > "$dir/bad-lines.txt" && [ -s "$dir/bad-lines.txt" ]; then
        fail+=("$(wc -l < "$dir/bad-lines.txt") unclean-shutdown log lines, e.g. $(head -1 "$dir/bad-lines.txt" | cut -c1-110)")
    fi
    if (( ${#fail[@]} )); then
        printf '   FAIL: %s\n' "${fail[@]}"
        return 1
    fi
    echo "   ok ($((t_clean - t0)) s total${t_signal:+, ended $((t_clean - t_signal)) s after the signal})"
    return 0
}

total=0; passed=0; failed_cases=()
for ((rep = 1; rep <= REPEAT; rep++)); do
    for c in "${CASES[@]}"; do
        total=$((total + 1))
        if run_case "${c%%:*}" "${c##*:}" "$total"; then passed=$((passed + 1)); else failed_cases+=("$total:$c"); fi
    done
done
echo
verdicts=$(find "$OUT" -name verdict-failed | wc -l)
echo "lifecycle_check: $passed/$total runs passed (backend $BACKEND, $REPEAT repeat(s));" \
     "$verdicts completed run(s) had a failed suite verdict"
(( ${#failed_cases[@]} )) && printf 'failed: %s\n' "${failed_cases[@]}"
exit $(( passed == total ? 0 : 1 ))
