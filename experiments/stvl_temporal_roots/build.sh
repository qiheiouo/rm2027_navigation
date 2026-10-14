#!/usr/bin/env bash
set -euo pipefail
ROOTS_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$ROOTS_ROOT/build/roots_cache"
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$ROOTS_ROOT/build/roots_cache:/check:rw" -w "$ROOTS_ROOT" \
 rm2027_navigation:humble -c '
source /opt/ros/humble/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash
cmake -S experiments/stvl_temporal -B /check/plugin_build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/check/research_install
cmake --build /check/plugin_build -j2
cmake --install /check/plugin_build
'
