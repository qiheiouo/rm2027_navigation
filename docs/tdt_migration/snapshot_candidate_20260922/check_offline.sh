#!/usr/bin/env bash
# Offline review only: no install, ROS node launch or candidate plugin loading.
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
candidate=/ws/docs/tdt_migration/snapshot_candidate_20260922
pkg=/ws/experiments/tdt_planner/rm_tdt_planner
output=/work/snapshot_candidate_review_20260922
c++ -std=c++17 -O2 -g -Wall -Wextra -Wpedantic \
  -I"$candidate/include" -I"$pkg/include" \
  "$candidate/snapshot_guard.cpp" "$candidate/test_snapshot_guard.cpp" \
  /work/build/rm_tdt_planner/librm_tdt_core.a /work/build/rm_tdt_planner/libtdt_nav_core.a \
  -L/work/deps/lib -Wl,-rpath,/work/deps/lib -lOsqpEigen -losqp -lgtest_main -lgtest -lpthread \
  -o "$output/test_snapshot_guard"
"$output/test_snapshot_guard" --gtest_output="xml:$output/tests.xml"
# Compile the proposed adapter object with the actual installed ROS include ABI.
# Do not link or install a plugin, and do not modify the normal build tree.
python3 - <<'PY'
from pathlib import Path
import shlex,subprocess
root=Path('/ws/docs/tdt_migration/snapshot_candidate_20260922')
flags=Path('/work/build/rm_tdt_planner/CMakeFiles/rm_tdt_global_planner.dir/flags.make').read_text()
args=['/usr/bin/c++']
for line in flags.splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):args.extend(shlex.split(line.split('=',1)[1]))
args+=['-I'+str(root/'include'),'-c',str(root/'nav2_plugin.cpp'),'-o','/work/snapshot_candidate_review_20260922/proposed_nav2_plugin.o']
subprocess.run(args,check=True)
print('Proposed adapter object compiles; not linked, loaded or installed.')
PY
