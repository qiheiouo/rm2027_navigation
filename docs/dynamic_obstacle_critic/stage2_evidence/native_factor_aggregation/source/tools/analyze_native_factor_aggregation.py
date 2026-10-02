#!/usr/bin/env python3
"""Full fixed-context labels for four independently reweighted SDK paths."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct
import numpy as np
import yaml
from analyze_trial import load_truth_rows
from audit_mechanical_footprint import components
from audit_raw_rollout_objective import geometry
from audit_scan_geometry import fixture
from native_factor_io import read_paths
from native_snapshot_io import read_snapshot
from prepare_processed_cv import verify
from verify_raw_rollout_geometry import verify as verify_scalar
from witness_geometry import rotate, polygon_distance, circle_box_gap, interpolation_lower, raw_clearance_lower, travel


def full_context(source,native,blocks,record,truth,scene,shapes,epoch,policy,dynamic):
    times=np.array([r['t'] for r in truth]);dt=record['model_dt'];grid=epoch+np.arange(31)*dt
    samples=np.unique(np.concatenate((grid,times[(times>epoch)&(times<grid[-1])])))
    poses=np.empty((4,len(samples),3))
    for row in range(4):
        for axis,name in enumerate(['x','y','yaw']):
            initial=native['pose'][axis] if axis<2 else record['initial_yaw']
            values=np.concatenate(([initial],record['blocks'][name]['values'][row*30:(row+1)*30]))
            if axis==2:values=np.unwrap(values)
            poses[row,:,axis]=np.interp(samples,grid,values)
    sx,sy=scene['static_dimensions']
    static=rotate(np.array(scene['static_pose']),np.array([[-sx/2,-sy/2],[sx/2,-sy/2],[sx/2,sy/2],[-sx/2,sy/2]]))
    body=np.array(next(s['points'] for s in shapes if s['kind']=='polygon' and s['name']=='base_link/base_collision'))
    profile=yaml.safe_load((source/'profile.yaml').read_text());cm=profile['local_costmap']['local_costmap']['ros__parameters']
    pad=cm['footprint_padding'];configured=np.array([[v+(pad if v>0 else -pad if v<0 else 0) for v in p] for p in yaml.safe_load(cm['footprint'])])
    metrics={};body_radius=np.linalg.norm(body,axis=1).max();mechanical_radius=body_radius
    for name,points in [('body',body),('padded_configured',configured),('padded_native',np.array(native['padded_footprint']))]:
        gap=polygon_distance(rotate(poses,points),static)
        metrics[name]={'static_sample_min':gap.min(axis=1),'static_lower':interpolation_lower(gap,poses,np.linalg.norm(points,axis=1).max())}
        if name=='body':mechanical=gap.copy()
    for shape in shapes:
        if shape['kind']=='circle':
            centers=rotate(poses,np.array([shape['center']]))[...,0,:]
            mechanical=np.minimum(mechanical,circle_box_gap(centers,shape['radius'],np.array(scene['static_pose']),scene['static_dimensions']))
            mechanical_radius=max(mechanical_radius,np.linalg.norm(shape['center'])+shape['radius'])
    metrics['mechanical']={'static_sample_min':mechanical.min(axis=1),'static_lower':interpolation_lower(mechanical,poses,mechanical_radius)}
    actor_values=np.array([r['obstacle'] for r in truth]);actor_values[:,2]=np.unwrap(actor_values[:,2])
    obstacle=np.stack([np.interp(samples,times,actor_values[:,i]) for i in range(3)],axis=-1)
    ax,ay=scene['actor_dimensions'][:2];actor=np.array([[-ax/2,-ay/2],[ax/2,-ay/2],[ax/2,ay/2],[-ax/2,ay/2]])
    gap=polygon_distance(rotate(poses,configured),rotate(obstacle,actor))
    metrics['padded_configured'].update(dynamic_sample_min=gap.min(axis=1),dynamic_lower=interpolation_lower(gap,poses,np.linalg.norm(configured,axis=1).max(),obstacle,np.linalg.norm(actor,axis=1).max()))
    footprint=np.array(native['padded_footprint']);movement=travel(poses[:,:-1],poses[:,1:],np.linalg.norm(footprint,axis=1).max())
    reserve=np.zeros((4,len(samples)));reserve[:,:-1]=.5*movement;reserve[:,1:]=np.maximum(reserve[:,1:],.5*movement)
    raw={**{k:native['map'][k] for k in ['resolution','origin','size']},'data':blocks['raw_costmap']['values']}
    raw_margin=(raw_clearance_lower(poses,footprint,raw,reserve,policy['raw_costmap_threshold'])-reserve).min(axis=1)
    follow=profile['controller_server']['ros__parameters']['FollowPath'];tol=policy['bounds_tolerance']
    rounded=struct.unpack('<f',struct.pack('<f',native['pose'][0]))[0];results=[]
    for row in range(4):
        combined={name:{key:float(value[row]) for key,value in values.items()} for name,values in metrics.items()}
        for name in ['body','mechanical','padded_native']:
            for new,old in [('dynamic_sample_min','sample_min'),('dynamic_lower','interval_lower')]:
                combined[name][new]=dynamic['rows'][row]['geometry'][name][old]
        arrays={n:np.array(record['blocks'][n]['values'][row*30:(row+1)*30]) for n in ['cvx','cvy','cwz']}
        bounded=bool(np.all((arrays['cvx']>=follow['vx_min']-tol)&(arrays['cvx']<=follow['vx_max']+tol)&
            (np.abs(arrays['cvy'])<=follow['vy_max']+tol)&(np.abs(arrays['cwz'])<=follow['wz_max']+tol)))
        end=float(record['blocks']['x']['values'][row*30+29]);progress=end-native['pose'][0]
        gates={'body_clearance':min(combined['body']['dynamic_lower'],combined['body']['static_lower'])>=policy['body_clearance'],
               'full_mechanical_clearance':min(combined['mechanical']['dynamic_lower'],combined['mechanical']['static_lower'])>=policy['body_clearance'],
               'padded_no_contact':all(min(combined[k]['dynamic_lower'],combined[k]['static_lower'])>0 for k in ['padded_configured','padded_native']),
               'frozen_native_raw203_with_interval_reserve':bool(raw_margin[row]>1e-9),
               'SG_full_sequence_and_return_bounds':bounded,'strict_positive_progress':progress>0,'progress_beyond_source_float_rounding':end>rounded}
        results.append({'gates':gates,'conditional_safe_control_with_progress':all(gates.values()),'geometry':combined,
                        'raw_interval_margin_lower':float(raw_margin[row]),'progress_x':progress})
    return results


def analyze(source,witness,factors,prepared,native_paths,native_json,score_json):
    for root in [source,witness,factors]:verify(root)
    identity=json.loads((prepared/'input_identity.json').read_text());record=read_paths(native_paths)
    if hashlib.sha256((prepared/'aggregation_input.bin').read_bytes()).hexdigest()!=identity['aggregation_input_sha256']:
        raise ValueError('private aggregation input identity')
    native,blocks=read_snapshot(source/'native_cycles/cycle_298.json')
    data=[json.loads(l) for l in native_json.read_text().splitlines()]
    if (len(data)!=5 or data[0]['actual_baseline_bit_exact_mask']!=127 or not data[0]['measured_prefix_exact_all_four'] or
            [r['factor'] for r in data[1:]]!=identity['factor_names'] or data[0]['native_library']!='/opt/ros/humble/lib/libmppi_controller.so'):
        raise ValueError('complete actual native baseline/four factor rows required')
    names=identity['factor_names'];costs=[json.loads(l) for l in score_json.read_text().splitlines()]
    if (len(costs)!=1217 or costs[0]['processed_rows']!=4 or costs[0]['source_ordinal']!=298 or
            costs[0]['actual_raw_risk_bit_exact_rows']!=300 or costs[0]['actual_raw_after_bit_exact_rows']!=300 or
            costs[0]['model_dt']!=record['model_dt']):
        raise ValueError('four aggregate scoring matrix budget')
    indexed={(r['factor'],r['path_kind'],r['row']):r for r in costs[1:]}
    if set(indexed)!={(f,k,i) for f in names for k,n in [('raw',300),('processed',4)] for i in range(n)}:
        raise ValueError('complete unique aggregate scoring entries')
    for r in indexed.values():
        if (not math.isfinite(r['risk']) or r['risk']<0 or r['minimum_clearance'] is None or not math.isfinite(r['minimum_clearance']) or
                r['collision_time'] is not None and not 0<r['collision_time']<=30*record['model_dt']):
            raise ValueError('finite aggregate risk/clearance/TTC required')
    frozen=[json.loads(l) for l in gzip.open(factors/'factors.jsonl.gz','rt')][1:]
    old={(r['factor'],r['path_kind'],r['row']):r for r in frozen}
    for key,r in indexed.items():
        if key[1]=='raw' or key[2]==0:
            for field in ['risk','minimum_clearance','collision_time']:
                a,b=r[field],old[key][field]
                if not (a is None and b is None or a is not None and b is not None and struct.pack('<d',a)==struct.pack('<d',b)):
                    raise ValueError('frozen raw/baseline aggregate score bits changed')
    truth=load_truth_rows(source,json.loads((source/'execution.json').read_text()));scene=fixture(source/'scene_inputs');shapes=components(source/'scene_inputs')
    policy=json.loads((source/'policy.json').read_text());epochs={'actual_score_clock':identity['score_clock'],'source_pose_clock_sensitivity':identity['source_pose_clock']}
    paths={k:record['blocks'][k] for k in ['x','y','yaw']}
    dynamic={name:geometry(native,paths,record['initial_yaw'],truth,shapes,scene,epoch,policy) for name,epoch in epochs.items()}
    full={name:full_context(source,native,blocks,record,truth,scene,shapes,epoch,policy,dynamic[name]) for name,epoch in epochs.items()}
    legacy=json.loads(gzip.open(witness/'selected/analysis.json.gz','rb').read())['records'][0]['proposals'][0]
    baseline=full['source_pose_clock_sensitivity'][0];errors=[]
    for name,values in legacy['geometry'].items():
        for key,value in values.items():errors.append(abs(value-baseline['geometry'][name][key]))
    if max(errors)!=0 or baseline['gates']!=legacy['gates'] or baseline['progress_x']!=legacy['progress_x'] or baseline['raw_interval_margin_lower']!=legacy['raw_interval_margin_lower']:
        raise ValueError('baseline full conditional source-pose witness changed')
    rows=[]
    for i,name in enumerate(names):
        n=data[i+1];weights=n['native_softmax_weights']
        if (n['row']!=i or len(weights)!=300 or not all(math.isfinite(v) and v>=0 for v in weights) or not any(v>0 for v in weights) or
                not n['weights_reaggregate_bounded_mean_bit_exact'] or len(n['post_gamma_costs'])!=300 or
                not all(math.isfinite(v) for v in n['post_gamma_costs']) or len(n['mean_before_SG'])!=90 or len(n['mean_after_SG'])!=90):
            raise ValueError('native finite/reaggregated weight shape')
        for axis,field in enumerate(['cvx','cvy','cwz']):
            values=record['blocks'][field]['values'][i*30:(i+1)*30]
            if struct.pack('<30f',*values)!=struct.pack('<30f',*n['mean_after_SG'][axis*30:(axis+1)*30]):
                raise ValueError('native JSON/control tensor join mismatch')
        if struct.pack('<3d',*n['returned_control'])!=struct.pack('<3d',*[record['blocks'][field]['values'][i*30+1] for field in ['cvx','cvy','cwz']]):
            raise ValueError('native returned control join mismatch')
        rows.append({'native_factor_row':i,'factor':name,'role':'actual_baseline_replica' if i==0 else 'fixed_context_counterfactual',
            'returned_control':n['returned_control'],'matching_factor_model_score':indexed[(name,'processed',i)],
            'all_factor_model_scores':{f:indexed[(f,'processed',i)] for f in names},
            'physical_dynamic_labels':{epoch:values['rows'][i] for epoch,values in dynamic.items()},
            'full_conditional_labels':{epoch:values[i] for epoch,values in full.items()},
            'weight_summary':{'sum_float64':math.fsum(weights),'largest_weight':max(weights),'largest_weight_raw_row':max(range(300),key=lambda j:weights[j]),'effective_sample_size':1/math.fsum(v*v for v in weights)}})
    summaries={epoch:{'epoch':values['epoch'],'horizon':values['horizon'],'samples':values['samples'],'max_sample_interval':values['max_sample_interval'],
        'padded_contact_on_aggregate_grid_rows':sum(r['physical_dynamic_labels'][epoch]['geometry']['padded_native']['original_rollout_grid_min']==0 for r in rows),
        'full_conditional_safe_progress_rows':sum(r['full_conditional_labels'][epoch]['conditional_safe_control_with_progress'] for r in rows)} for epoch,values in dynamic.items()}
    report={'schema':1,'native_source_ordinal':298,'baseline_original_full_witness_geometry_max_error':max(errors),
        'input_identity_sha256':hashlib.sha256((prepared/'input_identity.json').read_bytes()).hexdigest(),
        'native_paths_sha256':hashlib.sha256(native_paths.read_bytes()).hexdigest(),'native_CPP_sha256':hashlib.sha256(native_json.read_bytes()).hexdigest(),'pure_score_CPP_sha256':hashlib.sha256(score_json.read_bytes()).hexdigest(),
        'epoch_summaries':summaries,'rows':rows,
        'scope':'Four independent fixed-context native gamma/softmax/constraints/actual SG history hypotheses, full original conditional three-second gates.',
        'limits':['Oracle source/future states are inadmissible online; no new actual score or command events.',
            'All labels are conditional model paths with offline actor truth, pose interpolation and captured frozen raw map, not closed-loop or hardware execution.',
            'Full-trace task/raw203/age FAILED and witness NOT ESTABLISHED remain; raw-map source and actual odometry callback stamp unavailable.',
            'No new sampler, CA, custom ranking, guard change or deployment acceptance. Physical support bounds remain unknown.']}
    scalar=verify_scalar(source,report,paths,'native_factor_row',0,'padded_contact_on_aggregate_grid_rows')
    return report,scalar


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','factors','prepared','native_paths','native_json','score_json','report','scalar_report']:parser.add_argument(name,type=Path)
    args=parser.parse_args();report,scalar=analyze(args.source.resolve(),args.witness.resolve(),args.factors.resolve(),args.prepared,args.native_paths,args.native_json,args.score_json)
    for path,value in [(args.report,report),(args.scalar_report,scalar)]:
        with path.open('x') as stream:stream.write(json.dumps(value,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'epochs':report['epoch_summaries'],'rows':[{'factor':r['factor'],'model':r['matching_factor_model_score'],'full_labels':r['full_conditional_labels']} for r in report['rows']]}),flush=True)
    print(json.dumps(scalar),flush=True)
