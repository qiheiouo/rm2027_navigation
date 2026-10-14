#!/usr/bin/env bash
set -euo pipefail
EXPORT_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$EXPORT_ROOT/build/observer_export_cache:/check:rw" -w "$EXPORT_ROOT" \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash; cmake -S experiments/stvl_observer_export -B /check/build -DCMAKE_BUILD_TYPE=Release; cmake --build /check/build -j2; /check/build/observer_export experiments/stvl_observer_export/evidence/inputs.json /check > /check/result.json'
