#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /tmp/aud003_install/setup.bash
set -u
export AUD003_INSTALL=/tmp/aud003_install
export ROS_LOG_DIR=/tmp/aud003_ros_logs
mkdir -p "$ROS_LOG_DIR"

result_root=/tmp/aud003_final_motion_results
rm -rf "$result_root"
mkdir -p "$result_root"

domain=210
for case_name in unauthorized normal mission_cancel localization_lost authority_timeout source_timeout source_nonfinite injection recovery
do
  export ROS_DOMAIN_ID=$domain
  domain=$((domain + 1))
  python3 tests/aud003_final_motion/probe.py --case "$case_name" --output "$result_root/$case_name"
done
