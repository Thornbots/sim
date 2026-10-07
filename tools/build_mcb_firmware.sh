#!/usr/bin/env bash
# Build the checked-out MCB code, including its hosted hardware fixture.
set -euo pipefail
if [[ "$(uname -s)" != Linux ]] || ! command -v scons >/dev/null || ! command -v g++-11 >/dev/null; then
    echo 'Build in the Linux ROS container with scons and g++-11 installed.' >&2
    exit 1
fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MCB_PROJECT="${MCB_PROJECT:-$ROOT/firmware/MCBV3/MCB-project}"
if [[ ! -f "$MCB_PROJECT/SConstruct" ]]; then
    echo 'Initialize the firmware first: git submodule update --init --checkout firmware/MCBV3' >&2
    exit 1
fi
if [[ ! -d "$MCB_PROJECT/../taproot-scripts/scons-tools" ]]; then
    echo 'Initialize its build tools: git -C firmware/MCBV3 submodule update --init taproot-scripts' >&2
    exit 1
fi
cd "$MCB_PROJECT"
scons build-sim robot=sentry profile=release -j"${MCB_BUILD_JOBS:-4}"
echo "MCB firmware: $MCB_PROJECT/build/sim/scons-release/MCB-project.elf"
