#!/usr/bin/env bash
set -euo pipefail
R4_VALUE_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
R4_VALUE_OUT=${1:-"$R4_VALUE_ROOT/build/r4_rotation_value_20261005"}
mkdir -p "$R4_VALUE_OUT"
python3 "$R4_VALUE_ROOT/experiments/r4_rotation_value/select_inputs.py" "$R4_VALUE_ROOT" "$R4_VALUE_OUT/inputs"
docker run --rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
  --user 1000:1000 --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
  -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro \
  -v "$R4_VALUE_OUT:/check:rw" -w "$R4_VALUE_ROOT" \
  sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3 bash -c '
set -e
source /opt/ros/humble/setup.bash
source build/r4_adapter_humble_check_20261004/install/setup.bash
source build/r4_source_bridge_humble_check_20261005/install/setup.bash
export CMAKE_PREFIX_PATH="$PWD/build/r4_follow_humble_check_20261005/dependencies/install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$PWD/build/r4_follow_humble_check_20261005/dependencies/install/lib:$LD_LIBRARY_PATH"
build/r4_follow_humble_check_20261005/build/rm_r4_prediction_consumption/follow_probe > /check/fixed_before.json
cmake -S src/rm_r4_prediction_consumption -B /check/library -DCMAKE_BUILD_TYPE=Release -DRM_R4_BUILD_FOLLOW=ON -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/check/install
cmake --build /check/library -j2
cmake --install /check/library
export CMAKE_PREFIX_PATH="/check/install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="/check/install/lib:$LD_LIBRARY_PATH"
cmake -S experiments/r4_rotation_value -B /check/probes -DCMAKE_BUILD_TYPE=Release
cmake --build /check/probes -j2
/check/probes/fixed_probe > /check/fixed_after.json
/check/probes/probe /check/probe.csv
/check/probes/replay build/r4_corrected_runtime_shadow_20261005 /check/inputs /check
' > "$R4_VALUE_OUT/run.log" 2>&1
