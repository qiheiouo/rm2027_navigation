#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /workspace
task_build=/workspace/build/fastlio_validation
source "$task_build/install_driver/setup.bash"
export ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="$task_build/ros_logs"
ulimit -c 0
if [ "${1:-all}" = all ]; then
  python3 tests/fastlio_deterministic/ros_probe.py --binary "$task_build/backend/fast_lio_multi/laserMapping_bundle" --output "$task_build/ros_debug" > "$task_build/ros_debug.log" 2>&1
fi
cmake -S tests/fastlio_deterministic -B "$task_build/original_fixture" -DFASTLIO_SOURCE=/workspace/build/original_fast_lio -DFASTLIO_LIBRARY="$task_build/backend/fast_lio_multi/libfastlio_preprocess.a" -DFASTLIO_REFERENCE="$task_build/reference_measurement.inc" > "$task_build/original_configure.log" 2>&1
cmake --build "$task_build/original_fixture" --target measurement_fixture -j1 > "$task_build/original_build.log" 2>&1
gcc -shared -fPIC -O0 -g0 tests/fastlio_deterministic/abort_trace.c -ldl -o "$task_build/libabort_trace.so"
python3 - <<'PY'
import json,os,re,subprocess
from pathlib import Path
b=Path('/workspace/build/fastlio_validation'); cases=[]
for name in ['matched','unmatched']:
    env=dict(os.environ);env['LD_PRELOAD']=str(b/'libabort_trace.so')
    p=subprocess.run([str(b/'original_fixture/measurement_fixture'),name],capture_output=True,text=True,timeout=10,env=env)
    (b/('original_'+name+'.stdout')).write_text(p.stdout);(b/('original_'+name+'.stderr')).write_text(p.stderr)
    cases.append({'case':name,'exit':p.returncode})
    assert p.returncode==(-6 if name=='matched' else 0)
    if name=='matched':
        offsets=re.findall(r'measurement_fixture\(\+(0x[0-9a-fA-F]+)\)',p.stderr)
        assert 'AUDIT_ABORT_TRACE_BEGIN' in p.stderr and offsets
        stack=subprocess.check_output(['addr2line','-f','-C','-e',str(b/'original_fixture/measurement_fixture'),*offsets],text=True)
        (b/'original_abort_stack.txt').write_text(stack)
        assert 'h_share_model' in stack and 'std::vector<pcl::PointXYZINormal' in stack
(b/'original_result.json').write_text(json.dumps({'scope':'Unchanged entire original bundle TU, original real ESKF/IKD/PCL, _GLIBCXX_ASSERTIONS. Same unmodified preprocess library; no ROS graph.', 'cases':cases},indent=2)+'\n')
PY
cmake -S src/fast_lio_multi -B "$task_build/release" > "$task_build/release_configure.log" 2>&1
cmake --build "$task_build/release" --target laserMapping_bundle -j1 > "$task_build/release_build.log" 2>&1
python3 tests/fastlio_deterministic/ros_probe.py --binary "$task_build/release/laserMapping_bundle" --output "$task_build/ros_release" > "$task_build/ros_release.log" 2>&1
printf '%s\n' 'ROS_AND_ORIGINAL_TESTS_PASS'
