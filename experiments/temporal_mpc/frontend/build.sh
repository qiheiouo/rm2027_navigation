#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd "$(dirname "$0")" && pwd)"
task_output="${1:?supply an output executable path}"
c++ -std=c++20 -O2 -DRM_TDT_NO_OPENCV -I/usr/include/eigen3 \
  -I"$task_root/vendor/tdt_nav" "$task_root/bridge.cpp" \
  "$task_root/vendor/tdt_nav/YAstar/yastar.cpp" \
  "$task_root/vendor/tdt_nav/MinimumSnapOsqp/sfcSquare.cpp" \
  -pthread -o "$task_output"
