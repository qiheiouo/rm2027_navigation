#!/usr/bin/env bash
set -euo pipefail
HOLD_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$HOLD_ROOT/build/lifecycle_cache:/check:rw" -w "$HOLD_ROOT" \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; cmake -S experiments/stvl_lifecycle_hold -B /check/small_build -DCMAKE_BUILD_TYPE=Release; cmake --build /check/small_build -j2; /check/small_build/exception_probe'
