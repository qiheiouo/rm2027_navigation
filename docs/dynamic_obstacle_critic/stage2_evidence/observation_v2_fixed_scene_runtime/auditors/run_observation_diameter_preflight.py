#!/usr/bin/env python3
"""Execute the complete nominal/v2 preflight on the pinned installed SDK.

Build separately, then use this driver for both initial evidence and independent
reexecution. Historical archives are verified and read only.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from replay_observation_anchor_preflight import verify, xml_counts

ANCHOR_MANIFEST='1370e1ac0d0fc6653a30183bd2d6e06afa4922851469633a07d1d8c4e41b91ac'


def main():
    parser=argparse.ArgumentParser();parser.add_argument('new_output',type=Path)
    for name in ['build','install','v1-archive','anchor-archive']:parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--expected-provenance',type=Path);args=parser.parse_args()
    args.new_output.mkdir(parents=True,exist_ok=False);out=args.new_output.resolve()
    package=Path(__file__).resolve().parents[1];tools=package/'tools'
    tracker=package.parent/'rm_dynamic_obstacle_tracking';build=args.build/'rm_dynamic_obstacle_critic'
    anchor=args.anchor_archive.resolve();verify(anchor)
    if hashlib.sha256((anchor/'manifest.json').read_bytes()).hexdigest()!=ANCHOR_MANIFEST:raise ValueError('registered anchor reference required')
    checks={};env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
    env['PYTHONPATH']=str(tracker)+':'+str(tools)+':'+env.get('PYTHONPATH','')
    def run(command,label):
        with (out/(label+'.log')).open('x') as log:
            subprocess.run(list(map(str,command)),stdout=log,stderr=subprocess.STDOUT,env=env,check=True)
        print(label+' completed',flush=True)
    def python(script,arguments,label):run([sys.executable,'-B',tools/script,*arguments],label)
    native=0
    fixture=out/'cpp_fixtures';fixture.mkdir();env['RM_DYNAMIC_CONSUMPTION_FIXTURE_DIR']=str(fixture)
    for name in ['test_model','test_plugin','test_cost_accumulator_evidence','test_cost_accumulator_plugin']:
        run([build/name,'--gtest_output=xml:'+str(out/(name+'.xml'))],name)
        native+=xml_counts(out/(name+'.xml'))
    checks['57_actual_native_tests']=native==57
    for label,target in [('visible','V2VisibleActualScoreAndBudgetArePreserved'),('nominal','V2NominalActualScoreAndBudgetArePreserved')]:
        for extension in ['json','bin']:
            checks['actual_cpp_fixture:'+label+'.'+extension]=(fixture/target/('score_0.'+extension)).read_bytes()==(
                package/'test/fixtures/observation_consumption'/label/('score_0.'+extension)).read_bytes()
    env.pop('RM_DYNAMIC_CONSUMPTION_FIXTURE_DIR')
    for script,count in [('test_dynamic_consumption_io.py',20),('test_observation_consumption_io.py',11),
                         ('test_observation_profile_selection.py',5),('test_conditional_cv_support.py',8),
                         ('test_conditional_observation_support.py',6)]:
        python(script,[],script)
        content=(out/(script+'.log')).read_text()
        checks['python:'+script]=f'Ran {count} tests in ' in content and content.rstrip().endswith('OK')
    python('test_observation_consumption_tools.py',[build,out/'consumption_tools'],'consumption_tools')
    tools_report=json.loads((out/'consumption_tools/summary.json').read_text())
    checks['actual_tools']=tools_report['status']=='PASS' and len(tools_report['checks'])==18 and all(tools_report['checks'].values())
    python('test_observation_profile_preflight.py',[out/'profiles'],'profiles')
    profiles=json.loads((out/'profiles/result.json').read_text())
    checks['actual_profiles']=profiles['verdict']=='PASS' and len(profiles['cases'])==20
    for label,arguments in [('visible',[]),('nominal',['--nominal-diameter','.8','--prior-source','synthetic:guard-fixture-only'])]:
        python('test_observation_anchor_guard.py',[out/('guard_'+label),*arguments],'guard_'+label)
        report=json.loads((out/('guard_'+label)/'summary.json').read_text())
        checks['actual_guard:'+label]=report['status']=='PASS' and len(report['cases'])==8 and all(row['passed'] for row in report['cases'])
    python('audit_observation_v1_regression.py',[args.v1_archive,out/'v1_regression',build/'replay_dynamic_consumption'],'v1_regression')
    old=json.loads((out/'v1_regression/summary.json').read_text())
    checks['actual_v1_regression']=old['status']=='PASS' and len(old['checks'])==5 and all(old['checks'].values())
    run([sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider',tracker/'test',
         '--basetemp='+str(out/'ros_fixtures'),'--junitxml='+str(out/'tracker.xml')],'tracker')
    checks['54_tracker_tests']=xml_counts(out/'tracker.xml')==54
    run([sys.executable,'-B',tracker/'tools/audit_observation_anchor_core.py',anchor/'reference/core_41e96399.py',
         out/'default_core.json'],'default_core')
    checks['default_core_byte_exact']=(out/'default_core.json').read_bytes()==(anchor/'verification/default_core_regression.json').read_bytes()
    run([sys.executable,'-B',tracker/'tools/audit_default_prediction_wire.py',anchor/'reference/node_41e96399.py',
         out/'default_wire.json'],'default_wire')
    wire=json.loads((out/'default_wire.json').read_text());old_wire=json.loads((anchor/'verification/default_wire_regression.json').read_text())
    checks['default_wire_semantic_exact']=all(wire[key]==old_wire[key] for key in
        ['status','canonical_wire_trace_sha256','frames','coasting_tracks','truncated_frames','serialized_fields','structural_padding_bytes_examined'])
    paths=['rm_dynamic_obstacle_critic/lib/librm_dynamic_obstacle_critic.so',
           'rm_dynamic_obstacle_critic/lib/rm_dynamic_obstacle_critic/dynamic_safety_guard',
           'rm_dynamic_obstacle_critic/lib/rm_dynamic_obstacle_critic/native_command_observer',
           'rm_dynamic_obstacle_critic/lib/librm_native_cost_accumulator_observer.so']
    runtime={name:hashlib.sha256((args.install/name).read_bytes()).hexdigest() for name in paths}
    helpers=tools_report['helper_sha256']
    checks['sdk_libraries_unchanged']=all(hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest for name,digest in {
        '/opt/ros/humble/lib/libmppi_controller.so':'fc259b61103fe1cc8ce6e5fa517460074aecf4b8d5c1075c50fad0ab0eaff88b',
        '/opt/ros/humble/lib/libmppi_critics.so':'cb4f73ca0ac36be85c37a7bb1eafcca94120d8300d7c290c52a35f89fe4f31ef'}.items())
    if args.expected_provenance:
        expected=json.loads(args.expected_provenance.read_text())
        checks['independent_runtime_elf_exact']=runtime==expected['runtime_elf_sha256']
        checks['independent_helper_elf_exact']=helpers==expected['helper_sha256']
    report=dict(schema=1,status='PASS' if all(checks.values()) else 'FAILED',checks=checks,native_tests=native,
                runtime_elf_sha256=runtime,helper_sha256=helpers,
                dependency_manifests={'anchor':ANCHOR_MANIFEST,'v1':old['verified_dependency_manifest']},
                scope='Actual preflight/reexecution only; processed fixture repeats raw paths. No Gazebo physical/task acceptance, real e/B unknown.')
    (out/'summary.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n');print(json.dumps(report,sort_keys=True))
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
