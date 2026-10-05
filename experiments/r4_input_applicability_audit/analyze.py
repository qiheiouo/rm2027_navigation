#!/usr/bin/env python3
"""Diagnose frozen model applicability from recorded data; never call Follow."""
import argparse
import bisect
import csv
import hashlib
import json
import math
import pathlib
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_csv(path):
    with path.open() as f:return list(csv.DictReader(f))


def stats(values):
    a=[x for x in values if x is not None and math.isfinite(x)]
    return dict(n=len(a),median=float(np.median(a)) if a else None,
                p95=float(np.percentile(a,95)) if a else None,
                max=max(a) if a else None,min=min(a) if a else None)


def longest_run(rows,predicate):
    longest=count=0
    for r in rows:
        count=count+1 if predicate(r) else 0
        longest=max(longest,count)
    return longest


def window_summary(rows):
    have=[r for r in rows if r['measured_wz']]
    compat=lambda r:bool(r['measured_wz']) and abs(float(r['measured_wz']))<=1e-6
    return dict(cycles=len(rows),measured_cycles=len(have),yaw_compatible=sum(compat(r) for r in rows),
                valid=sum(r['valid']=='1' for r in rows),
                longest_consecutive_yaw_compatible_cycles=longest_run(rows,compat),
                longest_consecutive_valid_cycles=longest_run(rows,lambda r:r['valid']=='1'),
                absolute_wz=stats(abs(float(r['measured_wz'])) for r in have),
                cold_seed_cycles=sum(r['seed_kind']=='shadow_virtual_cold_zero' for r in rows),
                cold_seed_valid=sum(r['seed_kind']=='shadow_virtual_cold_zero' and r['valid']=='1' for r in rows),
                warm_proposals=sum(r['used_warm']=='1' and r['valid']=='1' for r in rows))


def main():
    p=argparse.ArgumentParser();p.add_argument('decoded',type=pathlib.Path);p.add_argument('a13',type=pathlib.Path)
    p.add_argument('output',type=pathlib.Path);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    result={};derived=[];fig,axes=plt.subplots(3,3,figsize=(15,10))
    for i,scene in enumerate(['S0','S1','S2']):
        odom=read_csv(args.decoded/(scene+'_odometry.csv'));tf=read_csv(args.decoded/(scene+'_tf.csv'))
        cycles=read_csv(args.a13/(scene+'_cycles.csv'));events=json.loads((args.a13/(scene+'_events.json')).read_text())['events']
        e={x['kind']:x for x in events};start=e['goal_accepted']['ROS_ns'];end=e['observation_end']['ROS_ns']
        rows=[r for r in cycles if start<=int(r['acquire_ros_ns'])<=end]
        native_end=e.get('native_goal_result',{}).get('ROS_ns',end)
        windows={'goal':window_summary(rows),
                 'native_navigation':window_summary([r for r in rows if int(r['acquire_ros_ns'])<=native_end]),
                 'after_native_goal':window_summary([r for r in rows if int(r['acquire_ros_ns'])>native_end])}
        if scene!='S0':
            clear=e['clear_target']['ROS_ns']
            windows['dynamic_event']=window_summary([r for r in rows if start+10**9<=int(r['acquire_ros_ns'])<clear])
            windows['after_clear_target']=window_summary([r for r in rows if int(r['acquire_ros_ns'])>=clear])
        by_stamp={int(r['source_ns']):r for r in odom};tf_by_stamp={int(r['source_ns']):r for r in tf if r['frame']=='odom'}
        odom_mismatches=[];tf_mismatches=[];map_tf=[r for r in tf if r['frame']=='map']
        for r in rows:
            if not r['pose_source_ns']:continue
            stamp=int(r['pose_source_ns']);o=by_stamp.get(stamp);t=tf_by_stamp.get(stamp)
            if o is None or any(abs(float(r[k])-float(o[kk]))>1e-10 for k,kk in [('x','x'),('y','y'),('yaw','yaw'),('measured_vx','vx'),('measured_vy','vy'),('measured_wz','wz')]):
                odom_mismatches.append(r['cycle'])
            if t is None or o is None or any(abs(float(o[k])-float(t[k]))>1e-10 for k in ['x','y','yaw']):
                tf_mismatches.append(r['cycle'])
        body_changes=path_changes=rebuilds=body_only_rebuilds=0;near_zero_body_changes=0
        for a,b in zip(rows,rows[1:]):
            if not a['body_digest'] or not b['body_digest']:continue
            changed=a['body_digest']!=b['body_digest'];same_path=a['path_revision']==b['path_revision']
            body_changes+=changed;path_changes+=not same_path;rebuilds+=b['corridor_rebuilt']=='1'
            body_only_rebuilds+=changed and same_path and b['static_revision']==a['static_revision'] and b['corridor_rebuilt']=='1'
            near_zero_body_changes+=changed and abs(float(a['measured_wz']))<=1e-6 and abs(float(b['measured_wz']))<=1e-6
        odom_goal=[r for r in odom if start<=int(r['source_ns'])<=end];times=[int(r['source_ns']) for r in odom]
        derivatives=[];yaw_deltas={.05:[],1.5:[]};horizon_rows=[]
        for a,b in zip(odom,odom[1:]):
            ns=int(b['source_ns']);dt=(ns-int(a['source_ns']))/1e9
            if not (start<=ns<=end and 0<dt<=.1):continue
            derivative=math.remainder(float(b['yaw'])-float(a['yaw']),2*math.pi)/dt
            reported=.5*(float(a['wz'])+float(b['wz']))
            derivatives.append((ns,derivative,reported))
        # Recorded future pose is used only for retrospective model-difference measurement.
        for o in odom_goal:
            t=int(o['source_ns'])
            for horizon in [.05,1.5]:
                target=t+int(horizon*1e9)
                if target>end:continue
                j=bisect.bisect_left(times,target)
                candidates=[k for k in [j-1,j] if 0<=k<len(odom)]
                k=min(candidates,key=lambda k:abs(times[k]-target)) if candidates else None
                if k is None or abs(times[k]-target)>30000000:continue
                delta=abs(math.remainder(float(odom[k]['yaw'])-float(o['yaw']),2*math.pi))
                yaw_deltas[horizon].append(delta)
                row=dict(scene=scene,source_ns=t,matched_future_source_ns=times[k],horizon_seconds=horizon,
                         yaw_delta_rad=delta,body_vertex_rotation_displacement_m=2*math.hypot(.30,.25)*math.sin(delta/2))
                horizon_rows.append(row);derived.append(row)
        after=[o for o in odom_goal if int(o['source_ns'])>=native_end]
        drift=math.hypot(float(after[-1]['x'])-float(after[0]['x']),float(after[-1]['y'])-float(after[0]['y'])) if len(after)>1 else None
        result[scene]=dict(windows=windows,raw_odometry_frames=sorted({(r['frame'],r['child']) for r in odom}),
                           odom_messages=len(odom),source_stamps_strictly_increasing=all(b>a for a,b in zip(times,times[1:])),
                           caller_odom_value_mismatch_cycles=odom_mismatches,odom_tf_mismatch_cycles=tf_mismatches,
                           max_map_odom_planar_offset=max((math.hypot(float(r['x']),float(r['y'])) for r in map_tf),default=None),
                           max_map_odom_absolute_yaw=max((abs(float(r['yaw'])) for r in map_tf),default=None),
                           pose_derived_absolute_wz=stats(abs(r[1]) for r in derivatives),
                           derivative_reported_midpoint_absolute_error=stats(abs(r[1]-r[2]) for r in derivatives),
                           body_changes=body_changes,path_changes=path_changes,body_only_rebuilds=body_only_rebuilds,
                           near_zero_rate_body_changes=near_zero_body_changes,
                           recorded_yaw_delta_rad={str(k):stats(v) for k,v in yaw_deltas.items()},
                           vertex_rotation_displacement_m={str(h):stats(r['body_vertex_rotation_displacement_m'] for r in horizon_rows if r['horizon_seconds']==h) for h in [.05,1.5]},
                           native_goal_to_window_end_measured_displacement_m=drift)
        t=[(int(r['source_ns'])-start)/1e9 for r in odom_goal]
        axes[i,0].plot(t,[float(r['wz']) for r in odom_goal],label='reported odom wz',linewidth=1)
        axes[i,0].plot([(r[0]-start)/1e9 for r in derivatives],[r[1] for r in derivatives],label='pose yaw derivative',alpha=.5,linewidth=.8)
        axes[i,0].set_ylabel(scene+' angular rate (rad/s)')
        axes[i,1].plot(t,[float(r['yaw']) for r in odom_goal],label='recorded yaw')
        axes[i,1].set_ylabel('yaw (rad)')
        axes[i,2].scatter([(int(r['acquire_ros_ns'])-start)/1e9 for r in rows],
                          [1 if r['valid']=='1' else 0 for r in rows],s=4,label='proposal available')
        axes[i,2].plot([(int(r['acquire_ros_ns'])-start)/1e9 for r in rows],
                      [float(r['seed_vx']) if r['seed_vx'] else np.nan for r in rows],linewidth=.7,label='virtual seed vx (m/s)')
        axes[i,2].set_ylabel('availability / seed vx')
        for ax in axes[i]:
            ax.set_xlim(0,20);ax.grid(alpha=.2);ax.legend(fontsize=7)
            if scene!='S0':
                ax.axvspan(1,(e['clear_target']['ROS_ns']-start)/1e9,color='orange',alpha=.1)
            if native_end<end:ax.axvline((native_end-start)/1e9,color='gray',linestyle='--',alpha=.5)
    for ax in axes[-1]:ax.set_xlabel('ROS seconds since native goal accepted')
    fig.suptitle('A14 retrospective input audit — unchanged A13 bags; no new runtime / no yaw substitution')
    fig.tight_layout();fig.savefig(args.output/'input_applicability.png',dpi=140);plt.close(fig)
    with (args.output/'recorded_yaw_differences.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(derived[0]),lineterminator='\n');w.writeheader();w.writerows(derived)
    (args.output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    root=pathlib.Path(__file__).resolve().parents[2]
    sources=['src/rm_r4_prediction_consumption/src/follow.cpp','src/rm_r4_prediction_consumption/src/consumption.cpp',
             'experiments/r4_runtime_shadow/shadow.cpp','src/rm_simulation/worlds/phase1_omni.sdf',
             'src/rm_simulation/config/ros_gz_bridge.yaml','src/rm_localization_adapters/src/lio_adapter.cpp',
             'src/rm_localization_adapters/config/lio_adapter.yaml','src/rm_localization_adapters/launch/localization_adapters.launch.py',
             'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml']
    historical=json.loads((args.a13/'provenance.json').read_text())
    source_hashes={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in sources}
    caller='experiments/r4_runtime_shadow/shadow.cpp'
    source_hashes[caller]=historical['runs']['S0']['sources'][caller]
    provenance=dict(stage='A14 recorded-input analysis only; seed replay is separately indexed',
                    base_commit='0266f2ba1b7e708e42e888f6b68e4dfaf8ea5867',algorithm_baseline='e137635e',
                    source_files=source_hashes,
                    historical_source_files={caller:'A13 executed caller at 0266f2ba, before A14 patch'},
                    analysis_script_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
                    decoded_manifest=json.loads((args.decoded/'decode_manifest.json').read_text()),
                    meaning='Pose derivatives and recorded future yaw are offline diagnostics, not new runtime inputs/predictions or physical safety bounds.',
                    execution=dict(ROS_nodes_started=0,new_scenarios=0,Follow_calls=0,algorithm_changes=False),
                    results_sha256={name:hashlib.sha256((args.output/name).read_bytes()).hexdigest() for name in ['input_applicability.png','summary.json','recorded_yaw_differences.csv']})
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    evidence=pathlib.Path(__file__).parent/'evidence';evidence.mkdir(exist_ok=True)
    for name in ['summary.json','input_applicability.png','recorded_yaw_differences.csv','provenance.json']:
        shutil.copyfile(args.output/name,evidence/name)


if __name__=='__main__':main()
