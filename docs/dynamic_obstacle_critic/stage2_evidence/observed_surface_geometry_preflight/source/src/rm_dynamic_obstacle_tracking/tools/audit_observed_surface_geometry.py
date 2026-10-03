#!/usr/bin/env python3
"""Extract every replay cluster before consulting independent actor truth labels."""
from collections import Counter
from dataclasses import asdict
import argparse,hashlib,json,math
from pathlib import Path
import sys
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'rm_dynamic_obstacle_critic/tools'))
from rm_dynamic_obstacle_tracking.core import Point2D,Detection,cluster_point_indices,cluster_points,dynamic_candidates,filter_detections_near_static
from rm_dynamic_obstacle_tracking.observed_surface_geometry import SurfaceConfig,extract_observed_surface
from audit_visible_box_geometry import load_static_map
from audit_surface_reveal import validate
from audit_scan_geometry import fixture,interpolate,first_box,stats
from analyze_trial import compose,load_truth_rows
from trial_io import rows


def label_return(measured_range, expected_hit, residual_limit):
    """Offline nearest-box agreement label; never a runtime selection rule."""
    if expected_hit is None:
        return {'status':'NO_EXPECTED_BOX', 'nearest_box':None, 'range_residual':None}
    expected_range,kind=expected_hit
    residual=measured_range-expected_range
    return {'status':kind if abs(residual)<=residual_limit else 'RANGE_MISMATCH_UNKNOWN',
            'nearest_box':kind,'range_residual':residual}


def audit(root,out):
    root=root.resolve();manifest=validate(root);out.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load((root/'installed_inputs/tracker_cv.yaml').read_text())['dynamic_obstacle_tracker_shadow']['ros__parameters']
    occupancy=load_static_map(root/'installed_inputs/course_static.yaml',cfg['occupied_threshold'])
    execution=json.loads((root/'execution.json').read_text());scene=fixture(root/'scene_inputs');policy=SurfaceConfig()
    source=list(rows(root,'observations.jsonl'));odom=[]
    for r in sorted((r for r in source if r['kind']=='canonical_odom'),key=lambda r:r['stamp']):
        if odom and r['stamp']<=odom[-1][0]:continue
        x,y,qx,qy,qz,qw=r['pose']
        if r['frame']!='odom' or r['child_frame']!='base_link' or abs(qx)>1e-3 or abs(qy)>1e-3 or abs(qz*qz+qw*qw-1)>1e-3:raise ValueError('unsupported source odometry')
        odom.append((r['stamp'],(x,y,math.atan2(2*qw*qz,1-2*qz*qz))))
    times=[t for t,p in odom];poses=[p for t,p in odom];truth=load_truth_rows(root,execution)
    truth_times=[r['t'] for r in truth];actors=[r['obstacle'] for r in truth];robots=[r['robot'] for r in truth]
    publication_epochs={r['source_stamp_ns'] for r in source if r['kind']=='obstacles'}
    counts=Counter();skips=Counter();label_missing=Counter();lengths=[];records=[]
    scans=[r for r in source if r['kind']=='scan' and execution['start_sim']<=r['stamp']<=execution['last_sim']]
    with (out/'clusters.jsonl').open('x') as stream:
        for scan in scans:
            stamp=scan['stamp']
            if scan['frame']!='sim_lidar_link' or scan['time_increment']!=0:skips['unsupported_frame_or_motion_timing']+=1;continue
            if not -cfg['max_future_scan_sec']<=scan['receive_sim']-stamp<=cfg['max_scan_age_sec']:skips['observer_source_age_gate']+=1;continue
            robot=interpolate(times,poses,stamp,.04)
            if robot is None:skips['missing_canonical_source_pose_bracket']+=1;continue
            sensor=compose(robot,scene['sensor_xy_yaw']);ranges=[float(v) for v in scan['ranges']]
            endpoints=[];original_indices=[];angle=scan['angle_min']
            for i,r in enumerate(ranges):
                if math.isfinite(r) and scan['range_min']<=r<=scan['range_max']:
                    world_angle=sensor[2]+angle;endpoints.append(Point2D(sensor[0]+r*math.cos(world_angle),sensor[1]+r*math.sin(world_angle)));original_indices.append(i)
                angle+=scan['angle_increment']
            candidates=dynamic_candidates(endpoints,occupancy,cfg['static_distance_threshold'],cfg['require_known_free'])
            # Point2D object identities preserve source beam membership even if coordinates duplicate.
            beam_by_object={id(p):i for p,i in zip(endpoints,original_indices)}
            clusters=[];legacy=[]
            for members in cluster_point_indices(candidates,cfg['cluster_tolerance'],cfg['cluster_min_points'],cfg['cluster_max_extent']):
                points=[candidates[i] for i in members];xs=[p.x for p in points];ys=[p.y for p in points]
                detection=Detection(Point2D(sum(xs)/len(xs),sum(ys)/len(ys)),max(max(xs)-min(xs),cfg['cluster_tolerance']),max(max(ys)-min(ys),cfg['cluster_tolerance']),len(points))
                if not filter_detections_near_static([detection],occupancy,cfg['detection_static_distance_threshold']):continue
                indices=[beam_by_object[id(p)] for p in points]
                extracted=extract_observed_surface([(p.x,p.y) for p in points],indices,ranges=ranges,
                    source_stamp_ns=round(stamp*1e9),range_min=scan['range_min'],range_max=scan['range_max'],config=policy)
                clusters.append({'source_indices':sorted(indices),'surface':asdict(extracted),'legacy_visible_detection':asdict(detection)})
                legacy.append(detection)
            expected=filter_detections_near_static(cluster_points(candidates,cfg['cluster_tolerance'],cfg['cluster_min_points'],cfg['cluster_max_extent']),occupancy,cfg['detection_static_distance_threshold'])
            if legacy!=expected:raise ValueError('original cluster order or visible detection changed')
            counts['accepted_source_pose_scans']+=1;counts['legacy_detection_equal_scans']+=1
            # Fit/retain ALL selected measured surfaces BEFORE truth labeling.
            actor=interpolate(truth_times,actors,stamp,.04);physical_robot=interpolate(truth_times,robots,stamp,.04)
            missing='missing_truth_bracket' if actor is None or physical_robot is None else ('world_odom_alignment' if math.dist(robot[:2],physical_robot[:2])>1e-4 or abs(math.remainder(robot[2]-physical_robot[2],2*math.pi))>1e-4 else None)
            if missing:label_missing[missing]+=1
            for cluster in clusters:
                counts['clusters']+=1;surface=cluster['surface'];represented={i for element in surface['elements'] for i in element['source_indices']}
                if represented!=set(cluster['source_indices']):raise ValueError('an observed endpoint was lost')
                counts['measured_endpoints_preserved']+=len(represented);counts['elements']+=len(surface['elements']);counts['source_gap_breaks']+=sum('source_index_gap' in b[2] for b in surface['breaks'])
                for element in surface['elements']:
                    counts[element['kind']]+=1;lengths.append(math.dist(element['start'],element['end']))
                if missing:cluster['truth_label_status']=missing;continue
                labels=Counter();verified=Counter();point_labels={}
                for i in cluster['source_indices']:
                    hit=first_box(sensor,sensor[2]+scan['angle_min']+i*scan['angle_increment'],actor,scene,scan['range_min'],scan['range_max'])
                    labels[hit[1] if hit else 'no_expected_box']+=1
                    label=label_return(ranges[i],hit,3*scene['noise_stddev'])
                    point_labels[i]=label;verified[label['status']]+=1
                cluster['offline_nearest_box_labels']=dict(labels)
                cluster['offline_range_agreement_labels']=dict(verified)
                cluster['offline_point_labels']=point_labels
                if labels['actor']:
                    counts['actor_clusters']+=1;counts['actor_clusters_mixed_with_other_labels']+=len(labels)>1
                if verified['actor']:
                    counts['range_agreement_actor_clusters']+=1
                    counts['range_agreement_actor_clusters_with_other_or_unknown']+=len(verified)>1
                for kind,n in verified.items():counts['endpoint_range_agreement:'+kind]+=n
                for element in surface['elements']:
                    kinds={point_labels[i]['status'] for i in element['source_indices']}
                    if len(kinds)>1:counts['nominal_chord_mixed_or_unknown_endpoint_labels']+=1
            record={'source_stamp':stamp,'source_epoch_public_receipt_seen':round(stamp*1e9) in publication_epochs,'clusters':clusters,
                    'actual_tracker_cluster_membership':'UNKNOWN: source-time odometry replay, not captured internal TF/members'}
            stream.write(json.dumps(record,separators=(',',':'),allow_nan=False)+'\n');records.append({'source_stamp':stamp,'clusters':len(clusters)})
    result={'schema':'observed_surface_geometry_audit/v1','verdict':'NOMINAL OBSERVED CHAIN PROTOTYPE; SUPPORT/CONTROL SAFETY NOT CERTIFIED',
            'source_manifest_sha256':manifest,'surface_policy':asdict(policy),'offline_label_range_residual_limit_m':3*scene['noise_stddev'],'counts':dict(counts),'skipped_scans':dict(skips),
            'missing_truth_labels':dict(label_missing),'element_length_m':stats(lengths),'cluster_payload_sha256':hashlib.sha256((out/'clusters.jsonl').read_bytes()).hexdigest(),
            'limits':['Every replay cluster is extracted before truth labels. Canonical source-time interpolation is not exact internal tracker TF/membership capture.',
                      'Legacy cluster detections remain exactly equal for replay inputs. Endpoint observations preserved; chords are a local continuity assumption.',
                      'No centroid/full-object completion, hidden-space freeing, ROS/TF/commands, optimizer/SG changes or moving-surface C run.',
                      'Gap, sparse, mixed and never-seen geometry remains unverified; this is a fixed single-layer planar LaserScan fixture, not MID360 acceptance.']}
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    print(json.dumps(audit(args.archive,args.output)))
