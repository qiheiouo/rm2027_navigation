#!/usr/bin/env python3
"""A16 extra input/stop diagnostics over completed runs; no control or prediction."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
import pathlib
import shutil


def module(path):
    spec=importlib.util.spec_from_file_location(path.stem+'_recorded_helpers',path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=pathlib.Path);args=p.parse_args()
    out=args.output.resolve();root=pathlib.Path(__file__).resolve().parents[2]
    inputs=module(root/'experiments/r4_input_applicability_audit/analyze.py')
    stop=module(root/'experiments/r4_native_stop_audit/analyze.py')
    results={};original_summary=json.loads((out/'summary.json').read_text())
    for scene in ['S0','S1','S2']:
        cycles=stop.rows(out/scene/'cycles.csv');odom=stop.rows(out/'decoded'/(scene+'_odometry.csv'))
        tf=stop.rows(out/'decoded'/(scene+'_tf.csv'));commands=stop.rows(out/'decoded'/(scene+'_commands.csv'))
        refs=stop.rows(out/scene/'references.csv')
        events={e['kind']:e for e in json.loads((out/scene/'events.json').read_text())['events']}
        start=events['goal_accepted']['ROS_ns'];end=events['observation_end']['ROS_ns']
        goal=events.get('native_goal_result',{}).get('ROS_ns')
        active=[r for r in cycles if start<=int(r['acquire_ros_ns'])<=end];assert len(active)==400
        navigation=[r for r in active if goal is None or int(r['acquire_ros_ns'])<=goal]
        after=[r for r in active if goal is not None and int(r['acquire_ros_ns'])>goal]
        assert len(navigation)==original_summary[scene]['native_motion_window']['cycles']
        v=dict(goal=inputs.window_summary(active),native_navigation=inputs.window_summary(navigation),after_native_goal=inputs.window_summary(after))
        if scene!='S0':
            clear=events['clear_target']['ROS_ns']
            v['dynamic_event']=inputs.window_summary([r for r in active if start+10**9<=int(r['acquire_ros_ns'])<clear])
        by_stamp={int(r['source_ns']):r for r in odom};tf_by_stamp={int(r['source_ns']):r for r in tf if r['frame']=='odom'}
        mismatches=[]
        for r in active:
            if not r['pose_source_ns']:continue
            ns=int(r['pose_source_ns']);o=by_stamp.get(ns);t=tf_by_stamp.get(ns)
            if o is None or t is None or any(abs(float(r[k])-float(o[q]))>1e-10 for k,q in [('x','x'),('y','y'),('yaw','yaw'),('measured_vx','vx'),('measured_vy','vy'),('measured_wz','wz')]) or any(abs(float(o[k])-float(t[k]))>1e-10 for k in ['x','y','yaw']):
                mismatches.append(r['cycle'])
        assert not mismatches,(scene,mismatches)
        topics={t:[r for r in commands if r['topic']==t] for t in ['/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel']}
        for topic,kind in [('/cmd_vel_nav','mppi_upstream'),('/cmd_vel','actual_output')]:
            assert [stop.command(r) for r in topics[topic]]==[stop.command(r) for r in refs if r['kind']==kind]
        a=topics['/cmd_vel'];b=topics['/simulation/chassis/cmd_vel']
        assert [stop.command(r) for r in b[:len(a)]]==[stop.command(r) for r in a]
        assert all(stop.zero(r) for r in b[len(a):])
        nz=[i for i,r in enumerate(b) if not stop.zero(r)];j=nz[-1]+1 if nz else 0
        permanent=b[j] if j<len(b) else None
        v['permanent_zero_recorder_bracket_ns']=[int(permanent['recorder_clock_before_ns']),int(permanent['recorder_clock_after_ns'])] if permanent else None
        actual=[r for r in odom if goal is not None and goal<=int(r['source_ns'])<=end]
        v['native_goal_elapsed_seconds']=(goal-start)/1e9 if goal else None
        v['native_goal_to_end_displacement_m']=math.hypot(float(actual[-1]['x'])-float(actual[0]['x']),float(actual[-1]['y'])-float(actual[0]['y'])) if len(actual)>1 else None
        post=[r for r in actual if permanent and int(r['source_ns'])>=max(goal,int(permanent['recorder_clock_after_ns']))+10**9]
        v['post_zero_plus_1s_measured_planar_speed']=stop.stats(math.hypot(float(r['vx']),float(r['vy'])) for r in post)
        v['post_zero_plus_1s_absolute_wz']=stop.stats(abs(float(r['wz'])) for r in post)
        v['caller_odom_source_TF_mismatch_cycles']=mismatches
        v['command_reference_relay_value_mismatch']=0
        v['extra_relay_zero_rows']=len(b)-len(a)
        results[scene]=v
    record=dict(stage='A16 corrected original-profile runtime shadow',scenes=results,
        verdict='FAILED_INPUT_APPLICABILITY; dynamic behavior INCONCLUSIVE; closed-loop NOT_ELIGIBLE',
        interpretation='Three finite original scenes with restored friction-frame literal. No controlled random-seed pairing with A13; no physical deployment or production lease certificate. Post-goal proposals do not establish active navigation behavior.')
    (out/'input_stop_summary.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
    evidence=root/'experiments/r4_corrected_runtime_shadow/evidence';evidence.mkdir(exist_ok=True)
    shutil.copyfile(out/'input_stop_summary.json',evidence/'input_stop_summary.json')
    for scene in ['S0','S1','S2']:
        for suffix in ['odometry.csv','commands.csv']:
            shutil.copyfile(out/'decoded'/(scene+'_'+suffix),evidence/(scene+'_'+suffix))
    for name in ['sdf_semantics.tsv','decoding.log']:
        shutil.copyfile(out/name,evidence/name)
    provenance=dict(stage='A16 input/stop analysis',script_sha256=stop.sha(pathlib.Path(__file__)),
        helper_sources={str(p.relative_to(root)):stop.sha(p) for p in [root/'experiments/r4_input_applicability_audit/analyze.py',root/'experiments/r4_native_stop_audit/analyze.py']},
        bag_input_manifest=json.loads((out/'decoded/decode_manifest.json').read_text()),
        evidence={name:stop.sha(evidence/name) for name in ['input_stop_summary.json','sdf_semantics.tsv','decoding.log']+[scene+'_'+suffix for scene in ['S0','S1','S2'] for suffix in ['odometry.csv','commands.csv']]},
        new_calls=dict(Follow=0,prediction=0,ROS_nodes=0),checks='recorded source/TF and ROS relay consistency PASS')
    # The command decoder owns decode_manifest.json; Odometry raw input hashes are identical bags.
    (evidence/'input_stop_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
