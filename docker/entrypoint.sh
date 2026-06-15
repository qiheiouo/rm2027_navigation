#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /workspace/rm2027_navigation/install/setup.bash ]; then
  source /workspace/rm2027_navigation/install/setup.bash
fi

exec "$@"
