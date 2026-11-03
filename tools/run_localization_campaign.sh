#!/usr/bin/env bash
# Batched H comparisons; run inside the prepared overlay after capacity approval.
set -euo pipefail

if (( $# != 1 && $# != 2 )) || [[ ${2:-} != '' && ${2:-} != --plan ]]; then
    echo "usage: $0 OUTPUT_DIRECTORY [--plan]" >&2
    exit 2
fi
output=$1
plan=${2:-}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ -z $plan ]]; then
    if [[ -e $output ]]; then
        echo "Refusing to overwrite campaign evidence: $output" >&2
        exit 2
    fi
    mkdir -p "$output"
    for repo in sim sentry_localization thornbots_pkg rf2o_laser_odometry; do
        git -C "$script_dir/../../$repo" rev-parse HEAD
        git -C "$script_dir/../../$repo" diff --stat
    done > "$output/source-state.txt"
fi

run_trial() {
    local name=$1
    shift
    if [[ -n $plan ]]; then
        printf '%s' "$name"
        printf ' %q' "$@"
        printf '\n'
        return
    fi
    local trial_dir="$output/$name"
    mkdir "$trial_dir"
    printf '%q ' "$@" > "$trial_dir/command.txt"
    printf '\n' >> "$trial_dir/command.txt"
    local status=0
    "$script_dir/run_suite.sh" localization_drift headless:=true foxglove:=false \
        real_time_factor:=1 "$@" > "$trial_dir/launch.log" 2>&1 || status=$?
    printf '%s\n' "$status" > "$trial_dir/status"
    if [[ -d /tmp/localization_drift_tests ]]; then
        cp -R /tmp/localization_drift_tests "$trial_dir/"
    fi
    if (( status != 0 )); then
        echo "Campaign stopped at $name (status $status); inspect native logs before resuming." >&2
        return "$status"
    fi
}

for trial in 1 2 3; do
    for variant in amcl008 amcl015 amcl020 fused raw; do
        args=(backend:=amcl)
        case "$variant" in
            amcl008) args+=(amcl_sigma_hit:=0.08) ;;
            amcl015) args+=(amcl_sigma_hit:=0.15) ;;
            amcl020) args+=(amcl_sigma_hit:=0.20) ;;
            fused) args=(backend:=mapping) ;;
            raw) args=(backend:=mapping slam_input_frame:=odom) ;;
        esac
        for scenario in drift_correction moving_obstacles real_accel; do
            run_trial "$variant-$scenario-$trial" "${args[@]}" "scenario:=$scenario"
        done
    done
done
run_trial wall-impact backend:=none scenario:=wall_impact
