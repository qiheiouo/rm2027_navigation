#!/usr/bin/env bash
set -euo pipefail
HOLD_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
HOLD_KIND=${1:?timing/reference/brake/brake_delayed}
[[ "$HOLD_KIND" =~ ^(timing|reference|brake|brake_delayed)$ ]] || exit 2
python3 - "$HOLD_ROOT" <<'PY'
import pathlib,sys,shutil
p=pathlib.Path(sys.argv[1]);assert shutil.disk_usage(p).free>3.12*1024**3
used=sum(x.stat().st_size for x in (p/'build/lifecycle_cache').rglob('*') if x.is_file());assert used<70*1024**2
PY
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$HOLD_ROOT/build/lifecycle_cache:/check:rw" -w "$HOLD_ROOT" \
 -e HOLD_KIND="$HOLD_KIND" -e HOLD_LABEL="${HOLD_LABEL:-$HOLD_KIND}" -e ROS_DOMAIN_ID=185 -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION="stvl_hold_$HOLD_KIND" -e IGN_IP=127.0.0.1 \
 -e LIBGL_ALWAYS_SOFTWARE=true -e ROS_LOG_DIR=/check/ros_logs rm2027_navigation:humble -c '
set -e
source /opt/ros/humble/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash
source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_runtime_shadow_20261005/profile_install/setup.bash
python3 experiments/stvl_lifecycle_hold/fixture.py "$HOLD_KIND"
'
