#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
repo=$(cd "$(dirname "$0")/../../.." && pwd)
source "$repo/build/temporal_mpc_ros2/install/setup.bash"
set -u
output=${1:?absolute fresh output directory required}
if [[ -e "$output/events.jsonl" ]]; then echo 'Refuse existing evidence' >&2; exit 2; fi
mkdir -p "$output"
tar --exclude=__pycache__ --exclude=.pytest_cache -czf "$output/source_snapshot.tar.gz" -C "$repo" experiments/temporal_mpc
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$repo/build/temporal_mpc_ros2/python_deps:$repo/experiments/temporal_mpc:${PYTHONPATH:-}"
setsid python3 "$repo/experiments/temporal_mpc/integration/execution_guard_node.py" > "$output/guard.log" 2>&1 & guard_pid=$!
finish() {
  kill -INT -- "-$guard_pid" 2>/dev/null || true
  sleep .2
  kill -TERM -- "-$guard_pid" 2>/dev/null || true
  wait || true
}
trap finish EXIT
python3 "$repo/experiments/temporal_mpc/integration/execution_guard_harness.py" "$output" > "$output/harness.log" 2>&1
cat "$output/harness.log"
