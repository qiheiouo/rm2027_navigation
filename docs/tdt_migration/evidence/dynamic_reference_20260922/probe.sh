#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /work/sim-install/setup.bash
source /work/install/setup.bash
export LD_LIBRARY_PATH="/work/deps/lib:${LD_LIBRARY_PATH:-}"
out=/work/runs/dynamic_fixture_probe_v1
export ROS_LOG_DIR="$out/ros" XDG_RUNTIME_DIR="$out/xdg"
mkdir -p "$ROS_LOG_DIR" "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
finish() {
  status=$?
  trap - EXIT
  kill -INT -- "-$pid" 2>/dev/null || true
  sleep 3
  kill -TERM -- "-$pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  printf '%s\n' "$status" > "$out/exit.txt"
}
trap finish EXIT
setsid ros2 launch /ws/docs/tdt_migration/evidence/dynamic_reference_20260922/dynamic.launch.py params_file:=/work/runs/snapshot_revalidation_v1/profiles/tdt_qp.yaml > "$out/launch.log" 2>&1 &
pid=$!
timeout 35s ign topic -e --json-output -t /world/phase1_omni/pose/info > "$out/gazebo_poses.jsonl" 2> "$out/pose_stderr.log" || test "$?" = 124
ros2 node info /moving_obstacle_controller > "$out/controller_node.txt"
