#!/usr/bin/env python3
"""Focused checks and preservation audit; writes a fresh result once."""
import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
RUNS=ROOT/'build/tdt_p2b/runs'
DIRS=[
 ('sweep_v1','dynamic_sweep_routing_20260923'),
 ('sweep_v2','dynamic_sweep_routing_v2_20260923'),
 ('directional_offline','dynamic_directional_guard_20260924'),
 ('scaled','dynamic_scaled_guard_20260924'),
 ('horizon','dynamic_guard_horizon_20260924'),
 ('viability','dynamic_viability_guard_20260924')]
SERIES=[
 'dynamic_sweep_routing_pilot_v1','dynamic_sweep_routing_pilot_v2',
 'dynamic_scaled_guard_pilot_v1','dynamic_guard_horizon_pilot_v1',
 'dynamic_guard_horizon_pilot_v2','dynamic_viability_guard_pilot_v1']

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    tests={}
    for label,name in DIRS:
        proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',
                             str(HERE.parent/name),'-p','test_*.py','-v'],
                            cwd=ROOT,capture_output=True,text=True)
        tests[label]={'exit':proc.returncode,'output':proc.stdout+proc.stderr}
        assert proc.returncode==0,label
    old=HERE.parent/'dynamic_guard_pilot_20260923/manifest.json'
    manifest=json.loads(old.read_text())
    for name,digest in manifest['files'].items():
        assert sha(ROOT/name)==digest,name
    series={}
    for name in SERIES:
        p=RUNS/name
        m=json.loads((p/'inputs.json').read_text())
        for file,digest in m['files'].items():
            assert sha(ROOT/file)==digest,(name,file)
        assert sha(p/'profile.yaml')==m['profile_sha256']
        series[name]={'inputs_sha256':sha(p/'inputs.json'),
                      'profile_sha256':m['profile_sha256'],
                      'hashed_file_count':len(m['files'])}
    src_diff=subprocess.check_output(['git','diff',
        'a419654a8fed1fc8321c234fb212abd0a6cabe04','--','src','experiments'],
        cwd=ROOT,text=True)
    assert not src_diff
    result={'schema':'tdt_dynamic_viability_checks/v1',
            'runtime_source_unchanged':True,
            'old_manifest_sha256':sha(old),
            'old_manifest_files_preserved':len(manifest['files']),
            'series':series,'focused_tests':tests,
            'full_stack_sanitizer_this_round':False,
            'new_nav2_ctest_this_round':False,
            'accepted_for_deployment':False}
    with (HERE/'checks.json').open('x') as f:
        json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({k:{'exit':v['exit'],'tests':next((line.strip() for line in
        v['output'].splitlines() if line.startswith('Ran ')),'unknown')}
        for k,v in tests.items()},indent=2))
    print('old_manifest_files_preserved',len(manifest['files']))

if __name__=='__main__':main()
