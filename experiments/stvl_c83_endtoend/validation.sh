#!/usr/bin/env bash
set -euo pipefail
C83_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$C83_ROOT/build/c83_cache/validation"
docker run --rm --entrypoint bash --network none --read-only --cap-drop ALL --security-opt no-new-privileges --user 1000:1000 --tmpfs /tmp:rw \
 -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro -v "$C83_ROOT/build/c83_cache/validation:/check:rw" -w "$C83_ROOT" \
 rm2027_navigation:humble -c 'set -e; source /opt/ros/humble/setup.bash; python3 - <<"PY"
import pathlib,subprocess,json
old=pathlib.Path("/home/qihei/rm2027_navigation/build/stvl_joint_execution_20261014/build/joint_cache")
prior=pathlib.Path("/home/qihei/rm2027_navigation/build/stvl_execution_realtime_20261014/build/realtime_cache")
exe=prior/"small_build/joint_bench";out=[]
for folder in [old/"inputs",old/"fresh_inputs",old/"unit_inputs",prior/"updates"]:
 for p in sorted(folder.glob("*.json")):
  r=json.loads(subprocess.check_output(["taskset","-c","0",str(exe),str(p),"1","classify"],text=True));assert r["equality"];out.append(dict(case=r["case"],equality=r["equality"],trajectories=len(r["predicate_witnesses"])))
assert len(out)==33
pathlib.Path("/check/33_validation.json").write_text(json.dumps(out,indent=2)+"\n");print("33/33 fixed reference predicates match")
PY'
