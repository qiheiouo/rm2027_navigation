#!/usr/bin/env python3
"""Reproduce private captured map/TF/members and the assignments actually used."""
from dataclasses import asdict
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rm_dynamic_obstacle_tracking.core import (
    MultiObjectTracker, OccupancyMap, Point2D, cluster_detections_with_members,
    dynamic_candidates, filter_detections_near_static, planar_rotation_matrix,
)
from rm_dynamic_obstacle_tracking.observed_surface_geometry import extract_observed_surface
from rm_dynamic_obstacle_tracking.surface_member_evidence import SurfaceMemberEvidence


def exact(a,b):
    return json.dumps(a,sort_keys=True,separators=(',',':'),allow_nan=False)==json.dumps(b,sort_keys=True,separators=(',',':'),allow_nan=False)


def decode_range(value):
    if type(value) in (int,float):return float(value)
    if value=='NaN':return math.nan
    if value=='Infinity':return math.inf
    if value=='-Infinity':return -math.inf
    raise ValueError('unknown measured return encoding')


def audit(root,out):
    root=root.resolve()
    summary=json.loads((root/'summary.json').read_text())
    if (root/'incomplete.json').exists() or summary['schema']!=SurfaceMemberEvidence.SCHEMA or summary['complete'] is not True:
        raise ValueError('incomplete member stream is not positive evidence')
    paths=sorted([*root.glob('map_*.json'),*root.glob('scan_*.json')],key=lambda p:int(p.stem.split('_')[1]))
    if not paths or len(paths)!=summary['records_written'] or any(p.is_symlink() for p in root.rglob('*')):
        raise ValueError('full closed regular-file stream required')
    if {p.name for p in root.iterdir()}!={'summary.json',*(p.name for p in paths)}:
        raise ValueError('unexpected member stream file')
    if [r['ordinal'] for r in summary['write_timings']]!=list(range(len(paths))):raise ValueError('write timing completeness')
    out.mkdir(parents=True,exist_ok=False)
    current_map=None;tracker=None;tracker_parameters=None;latest={};observation_ns={}
    counts={'map_inputs':0,'accepted_scans':0,'rejected_scans':0,'detections':0,'endpoints':0,'public_truncations':0}
    payload=out/'actual_surfaces.jsonl'
    with payload.open('x') as stream:
        for ordinal,path in enumerate(paths):
            record=json.loads(path.read_text())
            prefix='map' if record['status']=='map' else 'scan'
            if record['schema']!=SurfaceMemberEvidence.SCHEMA or record['ordinal']!=ordinal or path.name!=f'{prefix}_{ordinal:06d}.json':
                raise ValueError('actual input event identity gap')
            if record['status']=='map':
                current_map=record;identity=record['identity']
                if hashlib.sha256(bytes(v&255 for v in record['data'])).hexdigest()!=identity['signed_int8_data_sha256']:
                    raise ValueError('actual map identity mismatch')
                counts['map_inputs']+=1;continue
            if record['status']=='rejected':
                if not record['reason']:raise ValueError('missing rejected source reason')
                counts['rejected_scans']+=1;continue
            if record['status']!='accepted' or current_map is None:raise ValueError('missing actual map context')
            if not exact(record['map_identity'],current_map['identity']):raise ValueError('used map differs from captured input')
            source=record['source_stamp_ns'];tf=record['source_tf'];scan=record['scan'];parameters=record['extraction_parameters']
            if type(source) is not int or source<=0 or tf['requested_stamp_ns']!=source or tf['child_frame']!=record['scan_frame'] or tf['frame']!=record['map_identity']['frame']:
                raise ValueError('source frame/epoch mismatch')
            matrix=planar_rotation_matrix(tuple(tf['quaternion']))
            if not exact(list(matrix),tf['used_xy_rotation']):raise ValueError('actual TF XY block mismatch')
            ranges=[decode_range(v) for v in scan['ranges']];points=[];source_indices=[];angle=scan['angle_min']
            for i,distance in enumerate(ranges):
                if math.isfinite(distance) and scan['range_min']<=distance<=scan['range_max']:
                    x,y=distance*math.cos(angle),distance*math.sin(angle)
                    points.append(Point2D(tf['translation'][0]+matrix[0]*x+matrix[1]*y,
                                          tf['translation'][1]+matrix[2]*x+matrix[3]*y));source_indices.append(i)
                angle+=scan['angle_increment']
            projected=[[i,p.x,p.y] for i,p in zip(source_indices,points)]
            if not exact(projected,record['projected_endpoints']):raise ValueError('projected actual return bits differ')
            identity=current_map['identity'];qx,qy,qz,qw=identity['origin_quaternion']
            yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
            occupancy=OccupancyMap(*identity['size'],identity['resolution'],*identity['origin_xy'],yaw,current_map['data'],parameters['occupied_threshold'])
            candidates=dynamic_candidates(points,occupancy,parameters['static_distance_threshold'],parameters['require_known_free'])
            beam_by_point={id(p):i for p,i in zip(points,source_indices)}
            if [beam_by_point[id(p)] for p in candidates]!=record['candidate_source_indices']:raise ValueError('actual subtraction membership differs')
            members=cluster_detections_with_members(candidates,parameters['cluster_tolerance'],parameters['cluster_min_points'],parameters['cluster_max_extent'])
            kept={id(d) for d in filter_detections_near_static([d for d,m in members],occupancy,parameters['detection_static_distance_threshold'])}
            members=[(d,m) for d,m in members if id(d) in kept]
            if tracker is None:
                tracker_parameters=record['tracker_parameters'];tracker=MultiObjectTracker(**tracker_parameters)
            if not exact(tracker_parameters,record['tracker_parameters']):raise ValueError('unregistered changed tracker policy')
            kwargs={'source_stamp_ns':source} if tracker.public_anchor_mode=='last_observation_cv' else {}
            update=tracker.update([d for d,m in members],source/1e9,capture_assignments=True,**kwargs)
            expected=[{'detection_index':i,'value':asdict(d),'candidate_indices':list(m),
                       'source_indices':[beam_by_point[id(candidates[j])] for j in m],'track_id':update.detection_track_ids[i]}
                      for i,(d,m) in enumerate(members)]
            if not exact(expected,record['detections']) or not exact(asdict(update),record['tracker_update']):
                raise ValueError('actual cluster/assignment/filter snapshot differs')
            live={t.track_id for t in update.tracks};latest={k:v for k,v in latest.items() if k in live};observation_ns={k:v for k,v in observation_ns.items() if k in live}
            for track_id,(d,indices) in zip(update.detection_track_ids,members):
                beam_ids=[beam_by_point[id(candidates[j])] for j in indices]
                latest[track_id]=asdict(extract_observed_surface([(candidates[j].x,candidates[j].y) for j in indices],beam_ids,
                    ranges=ranges,source_stamp_ns=source,range_min=scan['range_min'],range_max=scan['range_max']))
                observation_ns[track_id]=source
            public=record['public_prediction'];order={'confirmed':0,'coasting':1,'tentative':2}
            selected=sorted(update.tracks,key=lambda t:(order[t.state.value],t.track_id))[:record['prediction_max_tracks']]
            expected_public={'schema':'rm_dynamic_obstacle_predictions/'+('v2_observation_anchor' if tracker.public_anchor_mode=='last_observation_cv' else 'v1'),
                'authority':'shadow_only','source_stamp_ns':source,'processing_stamp_ns':public['processing_stamp_ns'],'frame':identity['frame'],
                'complete':len(update.tracks)<=record['prediction_max_tracks'],'total_track_count':len(update.tracks),
                'prediction_dt':tracker.prediction_dt,'prediction_steps':tracker.prediction_steps,
                'tracks':[{'id':t.track_id,'state':{'tentative':1,'confirmed':2,'coasting':3}[t.state.value],
                    'xy':[t.position.x,t.position.y],'vxy':[t.velocity.x,t.velocity.y],'size_xy':[t.size_x,t.size_y],
                    'position_z':0.,'velocity_z':0.,'size_z':0.,
                    'last_observation_stamp_ns':observation_ns[t.track_id] if tracker.public_anchor_mode=='last_observation_cv' else round(t.last_observation_timestamp*1e9),
                    'observation_count':t.observations,'miss_count':t.misses,
                    'prediction_xyz':[[p.x,p.y,0.] for p in t.prediction]} for t in selected]}
            if not exact(expected_public,public):raise ValueError('captured public prediction differs from actual member/filter context')
            stream.write(json.dumps({'source_stamp_ns':source,'public_complete':public['complete'],
                'tracks':[{'track_id':t.track_id,'state':t.state.value,'velocity':[t.velocity.x,t.velocity.y],
                    'last_observed_surface':latest[t.track_id]} for t in update.tracks]},separators=(',',':'),allow_nan=False)+'\n')
            counts['accepted_scans']+=1;counts['endpoints']+=len(points);counts['detections']+=len(members);counts['public_truncations']+=not public['complete']
    result={'verdict':'EXACT CAPTURED MEMBER/TF/ASSOCIATION/PUBLIC CONTEXT PASS','counts':counts,
        'input_files_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [*paths,root/'summary.json']},
        'actual_surface_payload_sha256':hashlib.sha256(payload.read_bytes()).hexdigest(),
        'limits':'Actual source TF, map, members and assignment metadata reproduced. Surface chords/hidden geometry/CV remain uncertified. Private offline output is not a ROS control contract or C trial. DDS fixture is not MID360/Gazebo/hardware acceptance. Processing timing is captured metadata; no actual response/brake guarantee.'}
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('evidence',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    print(json.dumps(audit(args.evidence,args.output)))
