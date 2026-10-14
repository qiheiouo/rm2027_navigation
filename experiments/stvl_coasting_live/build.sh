#!/usr/bin/env bash
set -euo pipefail
LIVE_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$LIVE_ROOT/build/coasting_live_cache:/studycache:rw" \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash; for variant in strict allow_equal; do cmake -S /studycache/projects/$variant -B /studycache/$variant/plugin_build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/studycache/$variant/research_install; cmake --build /studycache/$variant/plugin_build -j2; cmake --install /studycache/$variant/plugin_build; done'
