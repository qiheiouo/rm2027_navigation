#!/usr/bin/env bash
set -euo pipefail
WINDOW_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
WINDOW_OFFSET=${1:?0.4 or 0.8}
[[ "$WINDOW_OFFSET" == 0.4 || "$WINDOW_OFFSET" == 0.8 ]] || exit 2
mkdir -p "$WINDOW_ROOT/build/brake_window_cache"
python3 - "$WINDOW_ROOT" <<'PY'
import pathlib,shutil,sys
p=pathlib.Path(sys.argv[1]);assert shutil.disk_usage(p).free>3.12*1024**3
assert sum(f.stat().st_size for f in (p/'build/brake_window_cache').rglob('*') if f.is_file())<30*1024**2
PY
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$WINDOW_ROOT/build/brake_window_cache:/check:rw" -w "$WINDOW_ROOT" \
 -e WINDOW_OFFSET="$WINDOW_OFFSET" -e HOLD_LABEL="brake_late${WINDOW_OFFSET/./}" -e ROS_DOMAIN_ID=186 -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION="stvl_window_${WINDOW_OFFSET/./}" -e IGN_IP=127.0.0.1 -e LIBGL_ALWAYS_SOFTWARE=true -e ROS_LOG_DIR=/check/ros_logs \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash; source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_runtime_shadow_20261005/profile_install/setup.bash; python3 experiments/stvl_brake_window/run_fixture.py "$WINDOW_OFFSET"'
