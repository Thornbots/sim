#!/bin/bash
# Install the pinned emulator in an isolated directory inside Linux.
set -euo pipefail
# Renode cleans /tmp/renode-* as stale per-process directories.
DEST="${1:-/tmp/mcb-renode-1.17.0}"
case "$(uname -m)" in
  aarch64) FILE=renode-1.17.0.linux-arm64-portable.tar.gz
    SHA=e9051578411441bfb47b77de0381cfba3badce03663367fab561472fd7807f1f ;;
  x86_64) FILE=renode-1.17.0.linux-portable.tar.gz
    SHA=4ba7c68b59e2447f188ef4b4b112fcccd2802582c460beedceb63b84e5605a5f ;;
  *) echo 'Renode installer requires Linux arm64 or x86_64.' >&2; exit 2 ;;
esac
[[ "$(uname -s)" == Linux ]] || { echo 'Run this installer in Linux.' >&2; exit 2; }
mkdir -p "$DEST"
ARCHIVE="$DEST/$FILE"
curl --fail --location --retry 3 "https://github.com/renode/renode/releases/download/v1.17.0/$FILE" -o "$ARCHIVE"
printf '%s  %s\n' "$SHA" "$ARCHIVE" | sha256sum --check -
tar -xzf "$ARCHIVE" -C "$DEST"
ROOT="$DEST/renode_1.17.0-portable"
cmake -S "$ROOT/tools/external_control_client/lib" -B "$ROOT/client-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-14
cmake --build "$ROOT/client-build" -j4
echo "Renode root: $DEST/renode_1.17.0-portable"
