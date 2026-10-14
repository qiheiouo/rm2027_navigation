#!/usr/bin/env bash
set -euo pipefail
COAST_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$COAST_ROOT/build/coasting_epoch_cache:/check:rw" -w "$COAST_ROOT" \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash; cmake -S experiments/stvl_coasting_epoch -B /check/build -DCMAKE_BUILD_TYPE=Release; cmake --build /check/build -j2; /check/build/coasting_before experiments/stvl_coasting_epoch/evidence/inputs.json; /check/build/coasting_after experiments/stvl_coasting_epoch/evidence/inputs.json'
