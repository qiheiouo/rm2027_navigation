#!/usr/bin/env python3
"""Conditional full-horizon offline witnesses, never closed-loop acceptance."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import struct
import numpy as np
import yaml
from analyze_trial import load_truth_rows
from audit_mechanical_footprint import components
from audit_scan_geometry import fixture
from native_snapshot_io import read_snapshot
from native_witness_io import records
from witness_geometry import rotate,polygon_distance,circle_box_gap,interpolation_lower,raw_clearance_lower,travel


def analyze(root,witness_file,identity_file):
    identity=json.loads(identity_file.read_text());policy=json.loads((root/'policy.json').read_text())
    config=yaml.safe_load((root/'profile.yaml').read_text());cm=config['local_costmap']['local_costmap']['ros__parameters']
    configured=yaml.safe_load(cm['footprint']);pad=cm['footprint_padding']
    padded=np.array([[v+(pad if v>0 else -pad if v<0 else 0) for v in point] for point in configured])
    shapes=components(root/'scene_inputs');scene=fixture(root/'scene_inputs');dimensions=scene['actor_dimensions']
    actor=np.array([[-dimensions[0]/2,-dimensions[1]/2],[dimensions[0]/2,-dimensions[1]/2],
                    [dimensions[0]/2,dimensions[1]/2],[-dimensions[0]/2,dimensions[1]/2]])
    sx,sy=scene['static_dimensions'];static=rotate(np.array(scene['static_pose']),np.array([[-sx/2,-sy/2],[sx/2,-sy/2],[sx/2,sy/2],[-sx/2,sy/2]]))
    body=np.array(next(s['points'] for s in shapes if s['kind']=='polygon' and s['name']=='base_link/base_collision'))
    body_radius=np.linalg.norm(body,axis=1).max();actor_radius=np.linalg.norm(actor,axis=1).max()
    mechanical_radius=max([body_radius]+[np.linalg.norm(s['center'])+s['radius'] for s in shapes if s['kind']=='circle'])
    execution=json.loads((root/'execution.json').read_text());truth=load_truth_rows(root,execution)
    times=np.array([r['t'] for r in truth]);physical_actor=np.array([r['obstacle'] for r in truth]);physical_robot=np.array([r['robot'] for r in truth])
    physical_actor[:,2]=np.unwrap(physical_actor[:,2]);physical_robot[:,2]=np.unwrap(physical_robot[:,2])
    def interpolate(values,query):return np.stack([np.interp(query,times,values[:,i]) for i in range(3)],axis=-1)
    results=[];counts=Counter();first_witnesses={};integrity=True
    for record in records(witness_file):
        ordinal=record['ordinal']
        if ordinal>=identity['cycles'] or identity['records'][ordinal]['ordinal']!=ordinal:raise ValueError('witness cycle identity')
        source_ordinal=identity['records'][ordinal].get('source_ordinal',ordinal)
        meta,blocks=read_snapshot(root/'native_cycles'/f'cycle_{source_ordinal}.json')
        if not (record['raw_velocity_pose_exact'] and record['aggregate_SG_exact'] and record['actual_command_double_bit_exact']):
            raise ValueError('native model or actual SG integrity failed; no safety analysis permitted')
        if meta['ordinal']!=source_ordinal:raise ValueError('witness source cycle identity')
        count=record['row_count'];steps=record['steps'];dt=record['model_dt'];horizon=steps*dt
        if count not in (11,11+identity['original_grid'][0]):raise ValueError('unregistered proposal count')
        arrays={name:np.asarray(values,dtype=np.float32).reshape(count,steps) for name,values in record['blocks'].items()}
        prefix=all(all(struct.pack('<f',value)==struct.pack('<f',meta['speed'][index]) for value in arrays[name][:,0])
                   for name,index in [('vx',0),('vy',1),('wz',5)])
        if not prefix:raise ValueError('actual measured prefix was replaced')
        start=meta['pose_stamp_sec']+meta['pose_stamp_nanosec']*1e-9;end=start+horizon
        if start<times[0] or end>times[-1]:raise ValueError('missing full-horizon actor truth bracket')
        grid=start+np.arange(steps+1)*dt
        sample_times=np.unique(np.concatenate((grid,times[(times>start)&(times<end)])))
        poses=np.empty((count,len(sample_times),3))
        for row in range(count):
            for axis,name in enumerate(['x','y','yaw']):
                initial=meta['pose'][axis] if axis<2 else record['initial_yaw']
                values=np.concatenate(([initial],arrays[name][row].astype(float)))
                if axis==2:values=np.unwrap(values)
                poses[row,:,axis]=np.interp(sample_times,grid,values)
        obstacle=interpolate(physical_actor,sample_times);actor_polygons=rotate(obstacle,actor)
        metrics={};gaps_by_kind={}
        def polygon_metrics(points):
            polygons=rotate(poses,points);dynamic=polygon_distance(polygons,actor_polygons);static_gap=polygon_distance(polygons,static)
            radius=np.linalg.norm(points,axis=1).max()
            return dynamic,static_gap,{'dynamic_sample_min':dynamic.min(axis=1),'static_sample_min':static_gap.min(axis=1),
              'dynamic_lower':interpolation_lower(dynamic,poses,radius,obstacle,actor_radius),
              'static_lower':interpolation_lower(static_gap,poses,radius)}
        for name,points in [('body',body),('padded_configured',padded),('padded_native',np.array(meta['padded_footprint']))]:
            dynamic,static_gap,metrics[name]=polygon_metrics(points);gaps_by_kind[name]=(dynamic,static_gap)
        dynamic,static_gap=[a.copy() for a in gaps_by_kind['body']]
        static_pose=np.asarray(scene['static_pose']);static_dimensions=scene['static_dimensions']
        for shape in shapes:
            if shape['kind']!='circle':continue
            centers=rotate(poses,np.array([shape['center']]))[...,0,:]
            dynamic=np.minimum(dynamic,circle_box_gap(centers,shape['radius'],obstacle,dimensions))
            static_gap=np.minimum(static_gap,circle_box_gap(centers,shape['radius'],static_pose,static_dimensions))
        metrics['mechanical']={'dynamic_sample_min':dynamic.min(axis=1),'static_sample_min':static_gap.min(axis=1),
          'dynamic_lower':interpolation_lower(dynamic,poses,mechanical_radius,obstacle,actor_radius),
          'static_lower':interpolation_lower(static_gap,poses,mechanical_radius)}
        native_footprint=np.array(meta['padded_footprint']);radius=np.linalg.norm(native_footprint,axis=1).max()
        movement=travel(poses[:,:-1],poses[:,1:],radius);reserve=np.zeros((count,len(sample_times)))
        reserve[:,:-1]=.5*movement;reserve[:,1:]=np.maximum(reserve[:,1:],.5*movement)
        raw={'resolution':meta['map']['resolution'],'origin':meta['map']['origin'],'size':meta['map']['size'],'data':blocks['raw_costmap']['values']}
        raw_lower=raw_clearance_lower(poses,native_footprint,raw,reserve,policy['raw_costmap_threshold'])
        raw_margin=(raw_lower-reserve).min(axis=1)
        endpoint=arrays['x'][:,-1].astype(float);progress=endpoint-meta['pose'][0]
        rounded_source=struct.unpack('<f',struct.pack('<f',meta['pose'][0]))[0]
        physical_source=interpolate(physical_robot,start)
        source_error=np.linalg.norm(np.array(meta['pose'][:2])-physical_source[:2])
        proposals=[]
        for row in range(count):
            name=identity['proposal_names'][row] if row<11 else f'sampler_{row-11}_bounds_SG_counterfactual'
            body_clear=min(metrics['body']['dynamic_lower'][row],metrics['body']['static_lower'][row])>=policy['body_clearance']
            full_clear=min(metrics['mechanical']['dynamic_lower'][row],metrics['mechanical']['static_lower'][row])>=policy['body_clearance']
            padded_clear=all(min(metrics[k]['dynamic_lower'][row],metrics[k]['static_lower'][row])>0 for k in ['padded_configured','padded_native'])
            tol=policy['bounds_tolerance'];vx=arrays['cvx'][row];vy=arrays['cvy'][row];wz=arrays['cwz'][row]
            bounds=bool(np.all((vx>=-.5-tol)&(vx<=.8+tol)&(np.abs(vy)<=.5+tol)&(np.abs(wz)<=1.2+tol)))
            gates={'body_clearance':bool(body_clear),'full_mechanical_clearance':bool(full_clear),'padded_no_contact':bool(padded_clear),
                   'frozen_native_raw203_with_interval_reserve':bool(raw_margin[row]>1e-9),'SG_full_sequence_and_return_bounds':bounds,
                   'strict_positive_progress':bool(progress[row]>0),'progress_beyond_source_float_rounding':bool(endpoint[row]>rounded_source)}
            geometric=all(v for k,v in gates.items() if k not in ('strict_positive_progress','progress_beyond_source_float_rounding'))
            passed=all(gates.values());counts[name+':safe_context']+=geometric;counts[name+':safe_progress']+=passed
            result={'name':name,'gates':gates,'conditional_safe_control_with_progress':passed,'progress_x':float(progress[row]),
                    'geometry':{kind:{key:float(value[row]) for key,value in values.items()} for kind,values in metrics.items()},
                    'raw_interval_margin_lower':float(raw_margin[row]),'returned_control':[float(arrays[k][row,1]) for k in ['cvx','cvy','cwz']]}
            proposals.append(result)
            if passed and name not in first_witnesses:first_witnesses[name]={'ordinal':ordinal,'source_ordinal':source_ordinal,'pose_stamp':start,**result}
        results.append({'ordinal':ordinal,'source_ordinal':source_ordinal,'source_pose_stamp':start,'capture_stamp':meta['capture_stamp'],'horizon':horizon,
                        'geometry_samples':len(sample_times),'max_geometry_interval':float(np.diff(sample_times).max()),
                        'actual_measured_prefix_exact':prefix,'source_native_vs_physical_xy_error':float(source_error),
                        'proposals':proposals})
        if ordinal%25==0:print(json.dumps({'processed':ordinal+1,'safe_progress_proposal_kinds':len(first_witnesses)}),flush=True)
    if len(results)!=identity['cycles']:raise ValueError('incomplete witness set')
    return {'verdict':'CONDITIONAL WITNESSES FOUND' if first_witnesses else 'NO REGISTERED SAFE PROGRESS WITNESS',
            'scope':'offline native full horizon under measured prefix, verified reconstructed history, actual fixture actor labels, linear pose interpolation and captured frozen native map',
            'cycles':len(results),'actual_measured_prefix_exact':integrity,'counts':dict(counts),'first_witnesses':first_witnesses,'records':results,
            'original_trial_verdict':'FAILED','limits':'No closed-loop/hardware acceptance. Actual dynamic consumer input and map source stamp unavailable. Source pose/capture epochs differ. Geometry is conditional planar native rollout, not guaranteed physical execution; actor truth is offline only. Frozen raw context cannot certify future map messages. Individual bounded/SG sampler rows, if present, are counterfactual controls, not actual MPPI weighted outputs. No sampler responsibility, CA/ranking restoration or CV support certificate.'}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('witnesses',type=Path)
    parser.add_argument('input_identity',type=Path);parser.add_argument('report',type=Path);args=parser.parse_args()
    result=analyze(args.archive,args.witnesses,args.input_identity)
    with args.report.open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    summary={k:v for k,v in result.items() if k not in ('records','first_witnesses','counts')}
    summary['registered_counts']={k:v for k,v in result['counts'].items() if not k.startswith('sampler_')}
    summary['sampler_safe_progress']=sum(v for k,v in result['counts'].items() if k.startswith('sampler_') and k.endswith(':safe_progress'))
    print(json.dumps(summary))


if __name__=='__main__':main()
