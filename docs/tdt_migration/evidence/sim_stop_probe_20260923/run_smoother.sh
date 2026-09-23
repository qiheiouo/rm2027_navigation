#!/usr/bin/env bash
set -eo pipefail
out=$1
test -n "$out"
source /opt/ros/humble/setup.bash
source /work/sim-install/setup.bash
source /work/install/setup.bash
export LD_LIBRARY_PATH="/work/deps/lib:$LD_LIBRARY_PATH"
export ROS_LOG_DIR="$out/ros" XDG_RUNTIME_DIR="$out/xdg"
mkdir -p "$ROS_LOG_DIR" "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
launch_pid=
smoother_pid=
finish() {
  status=$?
  trap - EXIT INT TERM
  for pid in "$smoother_pid" "$launch_pid"; do
    if [[ -n "$pid" ]]; then
      kill -INT -- "-$pid" 2>/dev/null || true
      sleep 1
      kill -TERM -- "-$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  printf '%s\n' "$status" > "$out/container_exit.txt"
  exit "$status"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 launch rm_simulation phase1_5_gazebo.launch.py use_nav2:=false use_scan_adapter:=false > "$out/launch.log" 2>&1 &
launch_pid=$!
profile=/work/runs/dynamic_map_age_pilot_v1/tdt_astar_1/profile.yaml
setsid ros2 run nav2_velocity_smoother velocity_smoother --ros-args --params-file "$profile" \
  -r cmd_vel:=cmd_vel_nav -r cmd_vel_smoothed:=cmd_vel > "$out/smoother.log" 2>&1 &
smoother_pid=$!
for _ in {1..40}; do
  ros2 lifecycle get /velocity_smoother > "$out/lifecycle_get.txt" 2>&1 && break
  sleep 0.25
done
ros2 lifecycle set /velocity_smoother configure > "$out/lifecycle_configure.txt"
ros2 lifecycle set /velocity_smoother activate > "$out/lifecycle_activate.txt"
python3 /ws/docs/tdt_migration/evidence/sim_stop_probe_20260923/probe_smoother.py "$out/records.jsonl" > "$out/probe.log" 2>&1
