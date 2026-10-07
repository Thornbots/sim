#!/usr/bin/env bash
# Run this checkout even when T3's worktree is outside the container's bind mount.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$ROOT/isaac_ros_common/scripts/container.sh"
DEXEC="$ROOT/isaac_ros_common/scripts/dexec.sh"
if ! container_running "$CONTAINER"; then
    echo "Start $CONTAINER before running the MCB fixture." >&2
    exit 1
fi
if [[ ! -f "$ROOT/firmware/MCBV3/MCB-project/src/hosted/main.cpp" ||
      ! -d "$ROOT/firmware/MCBV3/taproot-scripts/scons-tools" ]]; then
    echo 'Initialize firmware/MCBV3 and its taproot-scripts submodule first.' >&2
    exit 1
fi
if [[ "${1:-}" != --build-only ]]; then
    LIVE="$("$ROOT/isaac_ros_common/scripts/kill_launch.sh" -l)"
    if [[ $(printf '%s\n' "$LIVE" | wc -l) -gt 1 ]]; then
        echo "A ROS launch is already running. Stop it before starting the MCB fixture:" >&2
        printf '%s\n' "$LIVE" >&2
        exit 1
    fi
fi
TAG="$(printf '%s' "$ROOT" | shasum | cut -c1-12)"
CACHE="/tmp/mcb-worktree-$TAG"
"$DEXEC" -- mkdir -p "$CACHE/firmware/MCBV3/MCB-project" "$CACHE/sim/launch" "$CACHE/sim/tools"
# Remove mirrored sources so deleted firmware files cannot linger in a later build.
# Keep the isolated build directory and SCons signatures for incremental rebuilds.
"$DEXEC" -- rm -rf "$CACHE/firmware/MCBV3/MCB-project/src" \
    "$CACHE/firmware/MCBV3/MCB-project/taproot" "$CACHE/firmware/MCBV3/taproot-scripts" \
    "$CACHE/sim/sim"
docker cp "$ROOT/firmware/MCBV3/MCB-project/." "$CONTAINER:$CACHE/firmware/MCBV3/MCB-project"
docker cp "$ROOT/firmware/MCBV3/taproot-scripts" "$CONTAINER:$CACHE/firmware/MCBV3/"
docker cp "$ROOT/sim/sim" "$CONTAINER:$CACHE/sim/"
docker cp "$ROOT/sim/launch/mcb.launch.py" "$CONTAINER:$CACHE/sim/launch/"
docker cp "$ROOT/sim/tools/build_mcb_firmware.sh" "$CONTAINER:$CACHE/sim/tools/"
"$DEXEC" -- bash "$CACHE/sim/tools/build_mcb_firmware.sh"
if [[ "${1:-}" == --build-only ]]; then
    exit 0
fi
"$DEXEC" -d -- bash -c '
    MCB_RUNTIME_ROOT="$1"; shift
    export PYTHONPATH="$MCB_RUNTIME_ROOT/sim:$PYTHONPATH"
    exec ros2 launch "$MCB_RUNTIME_ROOT/sim/launch/mcb.launch.py" "$@"
' bash "$CACHE" "$@"
