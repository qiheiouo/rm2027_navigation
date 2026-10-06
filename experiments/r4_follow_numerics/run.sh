#!/usr/bin/env bash
set -euo pipefail
R4_NUM_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
R4_NUM_OUT=${1:-"$R4_NUM_ROOT/build/r4_follow_endpoint_clock64_20261006"}
R4_NUM_SCOPE=${2:-all}
case "$R4_NUM_SCOPE" in all|value) ;; *) exit 2 ;; esac
mkdir -p "$R4_NUM_OUT"
R4_NUM_OUT=$(cd "$R4_NUM_OUT" && pwd)
mkdir -p "$R4_NUM_OUT/baseline_source"
git -C "$R4_NUM_ROOT" show 0374f4b3:src/rm_r4_prediction_consumption/src/follow.cpp > "$R4_NUM_OUT/baseline_source/follow.cpp"
git -C "$R4_NUM_ROOT" show 0374f4b3:src/rm_r4_prediction_consumption/src/digest.hpp > "$R4_NUM_OUT/baseline_source/digest.hpp"
python3 "$R4_NUM_ROOT/experiments/r4_rotation_value/select_inputs.py" "$R4_NUM_ROOT" "$R4_NUM_OUT/inputs" native
docker run --rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
  --user 1000:1000 --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
  -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$R4_NUM_OUT:/check:rw" \
  -e R4_NUM_SCOPE="$R4_NUM_SCOPE" -w "$R4_NUM_ROOT" sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3 bash -c '
set -e
source /opt/ros/humble/setup.bash
source build/r4_adapter_humble_check_20261004/install/setup.bash
source build/r4_source_bridge_humble_check_20261005/install/setup.bash
export CMAKE_PREFIX_PATH="$PWD/build/r4_follow_humble_check_20261005/dependencies/install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$PWD/build/r4_follow_humble_check_20261005/dependencies/install/lib:$LD_LIBRARY_PATH"
cmake -S src/rm_r4_prediction_consumption -B /check/library -DCMAKE_BUILD_TYPE=Release -DRM_R4_BUILD_FOLLOW=ON -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/check/install
cmake --build /check/library -j2
cmake --install /check/library
export CMAKE_PREFIX_PATH="/check/install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="/check/install/lib:$LD_LIBRARY_PATH"
cmake -S experiments/r4_follow_numerics -B /check/diagnostics -DCMAKE_BUILD_TYPE=Release -DR4_BASELINE_SOURCE=/check/baseline_source/follow.cpp
cmake --build /check/diagnostics -j2
if [ "$R4_NUM_SCOPE" = all ]; then
  /check/diagnostics/endpoint /check/baseline baseline feedback
  for mode in zero_primal interval10 rho_at42; do /check/diagnostics/endpoint "/check/$mode" "$mode"; done
  for mode in rho_ratio15 equivalent_rows strict_polish reduced_polish reduced_refine; do
    /check/diagnostics/endpoint "/check/$mode" "$mode" feedback
  done
fi
/check/diagnostics/feedback /check/value world_value feedback
cmake -S experiments/r4_world_xy -B /check/probes -DCMAKE_BUILD_TYPE=Release
cmake --build /check/probes -j2
/check/probes/fixed_probe > /check/fixed_after.json
/check/probes/probe /check/probe.csv
/check/probes/replay build/r4_corrected_runtime_shadow_20261005 /check/inputs /check experiments/r4_corrected_runtime_shadow/evidence
' > "$R4_NUM_OUT/run.log" 2>&1
python3 "$R4_NUM_ROOT/experiments/r4_world_xy/analyze.py" "$R4_NUM_ROOT" "$R4_NUM_OUT" > "$R4_NUM_OUT/world_analysis.txt"
if [ "$R4_NUM_SCOPE" = all ]; then
  python3 "$R4_NUM_ROOT/experiments/r4_follow_numerics/analyze.py" "$R4_NUM_ROOT" "$R4_NUM_OUT" "$R4_NUM_OUT" > "$R4_NUM_OUT/endpoint_analysis.txt"
fi
