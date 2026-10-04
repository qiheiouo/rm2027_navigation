#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
repo=$(cd "$(dirname "$0")/../../.." && pwd)
source "$repo/build/temporal_mpc_ros2/install/setup.bash"
set -u
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
output=${1:?absolute output directory required}
strategy=${TEMPORAL_MPC_STRATEGY:-single}
[[ "$strategy" == single || "$strategy" == portfolio ]] || exit 2
mkdir -p "$output"
# Dependencies are enabled only for worker/harness, never for colcon generation.
export PYTHONPATH="$repo/build/temporal_mpc_ros2/python_deps:$repo/experiments/temporal_mpc:${PYTHONPATH:-}"
config="$repo/experiments/temporal_mpc/integration/dual_controller_humble.yaml"
frontend="$repo/build/temporal_mpc_ros2/frontend"
bash "$repo/experiments/temporal_mpc/frontend/build.sh" "$frontend" > "$output/frontend_build.log" 2>&1
python3 "$repo/experiments/temporal_mpc/gazebo/runtime_identity.py" "$output/runtime_identity.json"
pids=()
finish() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep .5
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait || true
}
trap finish EXIT
ros2 run nav2_controller controller_server --ros-args --params-file "$config" > "$output/controller.log" 2>&1 & pids+=("$!")
ros2 run nav2_planner planner_server --ros-args --params-file "$config" > "$output/planner.log" 2>&1 & pids+=("$!")
ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file "$config" \
  -p default_nav_to_pose_bt_xml:="$repo/experiments/temporal_mpc/integration/runtime_tree.xml" \
  -p default_nav_through_poses_bt_xml:="$repo/experiments/temporal_mpc/integration/runtime_tree.xml" > "$output/navigator.log" 2>&1 & pids+=("$!")
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true \
  -p node_names:="['controller_server','planner_server','bt_navigator']" > "$output/lifecycle.log" 2>&1 & pids+=("$!")
python3 "$repo/experiments/temporal_mpc/integration/worker_node.py" --ros-args -p frontend:="$frontend" -p strategy:="$strategy" > "$output/worker.log" 2>&1 & worker_pid=$!; pids+=("$worker_pid")
export MPC_WORKER_PID="$worker_pid"
python3 "$repo/experiments/temporal_mpc/integration/selector_node.py" > "$output/selector.log" 2>&1 & pids+=("$!")
python3 "$repo/experiments/temporal_mpc/integration/ros_harness.py" --output "$output"
