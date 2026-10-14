#!/usr/bin/env bash
set -euo pipefail
LIVE_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
LIVE_KIND=${1:?strict or allow_equal}
[[ "$LIVE_KIND" == strict || "$LIVE_KIND" == allow_equal ]] || exit 2
LIVE_CACHE="$LIVE_ROOT/build/coasting_live_cache"
LIVE_BASE=/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014
python3 - "$LIVE_CACHE" "$LIVE_KIND" <<'PY'
import pathlib,shutil,sys
p=pathlib.Path(sys.argv[1]);assert shutil.disk_usage(p).free>3.06*1024**3
assert not (p/sys.argv[2]/'runs').exists(), 'do not overwrite or repeat this preregistered start'
assert sum(f.stat().st_size for f in p.rglob('*') if f.is_file())<75*1024**2
PY
LIVE_CONTAINER="rm_stvl_coasting_$LIVE_KIND"
docker run --name "$LIVE_CONTAINER" --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --ulimit core=0 --user 1000:1000 --shm-size 512m --tmpfs /tmp:rw --tmpfs /home/rmnav:rw,uid=1000,gid=1000 \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$LIVE_CACHE:/studycache:ro" -v "$LIVE_CACHE/$LIVE_KIND:/check:rw" -v "$LIVE_CACHE/run_trial.py:$LIVE_BASE/experiments/stvl_c83_endtoend/run_trial.py:ro" -w "$LIVE_BASE" \
 -e STUDY_MODE=narrow:C:83:aligned -e COAST_KIND="$LIVE_KIND" -e ROS_DOMAIN_ID=188 -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION="stvl_coasting_$LIVE_KIND" -e IGN_IP=127.0.0.1 -e LIBGL_ALWAYS_SOFTWARE=true \
 -e R4_RESEARCH_NATIVE_RECOVERY=1 -e ROS_LOG_DIR=/check/ros_logs -e REALTIME_DIAGNOSTIC_CAPTURE=off -e C83_SHADOW=off \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_adapter_humble_check_20261004/install/setup.bash; source /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/r4_runtime_shadow_20261005/profile_install/setup.bash; source /studycache/$COAST_KIND/research_install/share/rm_stvl_temporal_research/local_setup.bash; python3 -c "import warnings,runpy,os,sys;sys.argv=[\"experiments/stvl_c83_endtoend/run_trial.py\",os.environ[\"STUDY_MODE\"]];runpy.run_path(sys.argv[0],run_name=\"__main__\")"' &
LIVE_PID=$!
while kill -0 "$LIVE_PID" 2>/dev/null; do
 sleep 2
 if ! python3 - "$LIVE_CACHE" <<'PY'
import pathlib,shutil,sys
p=pathlib.Path(sys.argv[1]);sys.exit(0 if shutil.disk_usage(p).free>=3.03*1024**3 and sum(f.stat().st_size for f in p.rglob('*') if f.is_file())<75*1024**2 else 1)
PY
 then
  printf '%s\n' 'owned cache/disk guard: stopping current attempt; retain partial raw' > "$LIVE_CACHE/${LIVE_KIND}_resource_abort.txt"
  docker stop --time 10 "$LIVE_CONTAINER"
  break
 fi
done
wait "$LIVE_PID"
