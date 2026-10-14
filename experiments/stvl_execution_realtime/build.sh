#!/usr/bin/env bash
set -euo pipefail
REALTIME_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$REALTIME_ROOT/build/realtime_cache"
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$REALTIME_ROOT/build/realtime_cache:/check:rw" -w "$REALTIME_ROOT" \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; cmake -S experiments/stvl_execution_realtime -B /check/small_build -DCMAKE_BUILD_TYPE=Release; cmake --build /check/small_build -j2'
