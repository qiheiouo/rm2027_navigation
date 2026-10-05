#!/usr/bin/env bash
# Reuse the A18 library; build only the changed research caller.
set -euo pipefail
R4_BOUND_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
R4_BOUND_MODE=${2:-dynamic}
case "$R4_BOUND_MODE" in probe|dynamic) ;; *) exit 2;; esac
R4_BOUND_OUT=${1:-"$R4_BOUND_ROOT/build/r4_rotation_locked_${R4_BOUND_MODE}_20261005"}
R4_BOUND_LIBRARY="$R4_BOUND_ROOT/build/r4_rotation_value_20261005"
test -f "$R4_BOUND_LIBRARY/install/lib/librm_r4_prediction_consumption_follow.so"
mkdir -p "$R4_BOUND_OUT"
R4_BOUND_OUT=$(cd "$R4_BOUND_OUT" && pwd)
if [ "$R4_BOUND_MODE" = dynamic ]; then
  python3 "$R4_BOUND_ROOT/experiments/r4_rotation_value/select_inputs.py" "$R4_BOUND_ROOT" "$R4_BOUND_OUT/inputs" dynamic
fi
docker run --rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges \
  --user 1000:1000 --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
  -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro \
  -v "$R4_BOUND_LIBRARY:/check:ro" -v "$R4_BOUND_OUT:/result:rw" \
  -w "$R4_BOUND_ROOT" -e R4_BOUND_MODE="$R4_BOUND_MODE" \
  sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3 bash -c '
set -e
source /opt/ros/humble/setup.bash
source build/r4_adapter_humble_check_20261004/install/setup.bash
source build/r4_source_bridge_humble_check_20261005/install/setup.bash
export CMAKE_PREFIX_PATH="/check/install:$PWD/build/r4_follow_humble_check_20261005/dependencies/install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="/check/install/lib:$PWD/build/r4_follow_humble_check_20261005/dependencies/install/lib:$LD_LIBRARY_PATH"
cmake -S experiments/r4_rotation_value -B /result/probes -DCMAKE_BUILD_TYPE=Release
if [ "$R4_BOUND_MODE" = probe ]; then
  cmake --build /result/probes --target probe -j2
  /result/probes/probe /result/probe.csv
else
  cmake --build /result/probes --target replay -j2
  /result/probes/replay build/r4_corrected_runtime_shadow_20261005 /result/inputs /result locked
fi
' > "$R4_BOUND_OUT/run.log" 2>&1
python3 "$R4_BOUND_ROOT/experiments/r4_rotation_value/analyze.py" "$R4_BOUND_OUT"
