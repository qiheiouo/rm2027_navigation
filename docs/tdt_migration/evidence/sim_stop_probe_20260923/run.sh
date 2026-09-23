#!/usr/bin/env bash
set -eo pipefail
out=${1:?output directory required}
source /opt/ros/humble/setup.bash
source /work/sim-install/setup.bash
source /work/install/setup.bash
export LD_LIBRARY_PATH="/work/deps/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="$out/ros" XDG_RUNTIME_DIR="$out/xdg"
mkdir -p "$ROS_LOG_DIR" "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
launch_pid=
finish() {
  status=$?
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]]; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    sleep 2
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  printf '%s\n' "$status" > "$out/container_exit.txt"
  exit "$status"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 launch rm_simulation phase1_5_gazebo.launch.py use_nav2:=false use_scan_adapter:=false > "$out/launch.log" 2>&1 &
launch_pid=$!
python3 /ws/docs/tdt_migration/evidence/sim_stop_probe_20260923/probe.py "$out/records.jsonl" > "$out/probe.log" 2>&1
