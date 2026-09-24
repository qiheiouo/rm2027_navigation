#!/usr/bin/env python3
"""Replay frozen fixture scans through the existing tracker core; no navigation run."""
import bisect
import hashlib
import json
import math
import statistics
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
TRIAL=ROOT/'build/tdt_p2b/runs/dynamic_reference_pilot_v2/tdt_qp_1'
sys.path.insert(0,str(ROOT/'src/rm_dynamic_obstacle_tracking'))
from rm_dynamic_obstacle_tracking.core import (OccupancyMap, Point2D, MultiObjectTracker,
    dynamic_candidates, cluster_points, filter_detections_near_static, TrackState)
sys.path.insert(0,str(ROOT/'docs/tdt_migration/evidence/dynamic_reference_20260922'))
from dynamic_metrics import rows_from_transport

MAP_RES=.05
MAP_ORIGIN=(-6.5,-6.0)
MAP_DIMS=(260,240)
FIRST_BODY_OVERLAP_S=28.114  # Frozen original QP report; post-contact future is not an unbiased forecast target.


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read_rows(p):return [json.loads(x) for x in p.open()]


def static_map():
    world=ET.parse(ROOT/'src/rm_simulation/worlds/phase1_omni.sdf')
    block=world.find(".//model[@name='center_block']")
    bp=[float(x) for x in block.find('pose').text.split()]
    bs=[float(x) for x in block.find('.//collision/geometry/box/size').text.split()]
    wall=ET.parse(ROOT/'src/rm_simulation/models/course_wall.sdf')
    ws=[float(x) for x in wall.find('.//collision/geometry/box/size').text.split()]
    rectangles=[(bp[0],bp[1],bs[0],bs[1]),(3,.525,ws[0],ws[1]),(3,-.525,ws[0],ws[1])]
    cells=[]
    for iy in range(MAP_DIMS[1]):
        y=MAP_ORIGIN[1]+(iy+.5)*MAP_RES
        for ix in range(MAP_DIMS[0]):
            x=MAP_ORIGIN[0]+(ix+.5)*MAP_RES
            cells.append(100 if any(abs(x-cx)<=sx/2 and abs(y-cy)<=sy/2
                                    for cx,cy,sx,sy in rectangles) else 0)
    return OccupancyMap(*MAP_DIMS,MAP_RES,*MAP_ORIGIN,0,cells),rectangles


def interpolator(poses):
    times=[x['t'] for x in poses]
    def at(t):
        i=bisect.bisect_right(times,t)
        if i<=0 or i>=len(times):return None
        a,b=poses[i-1],poses[i]
        ratio=(t-a['t'])/(b['t']-a['t'])
        result={}
        for key in ('robot','obstacle'):
            x,y,yaw=a[key];u,v,ang=b[key]
            result[key]=(x+ratio*(u-x),y+ratio*(v-y),yaw+ratio*math.remainder(ang-yaw,2*math.pi))
        return result
    return at


def scan_points(scan,robot):
    x,y,yaw=robot
    points=[]
    for i,r in enumerate(scan['ranges']):
        if not isinstance(r,(int,float)) or not math.isfinite(r):continue
        if not scan['range_min']<=r<=scan['range_max']:continue
        angle=yaw+scan['angle_min']+i*scan['angle_increment']
        points.append(Point2D(x+r*math.cos(angle),y+r*math.sin(angle)))
    return points


def main():
    occ,rectangles=static_map()
    poses=rows_from_transport(TRIAL/'gazebo_poses.jsonl')
    at=interpolator(poses)
    scans=read_rows(TRIAL/'observation/scans.jsonl')
    # Config matches the existing shadow profile except the requested V1 CV
    # forecast uses velocity_decay_tau=0; tracking/association is unchanged.
    tracker=MultiObjectTracker(association_gate=.6,process_noise=3.,
        measurement_noise=.08,initial_variance=1.,min_hits_to_confirm=3,
        min_displacement_to_confirm=.15,use_global_assignment=False,
        tentative_max_misses=1,max_coast_time_sec=.6,
        prediction_steps=10,prediction_dt=.1,velocity_decay_tau=0.,
        max_prediction_speed=3.)
    output=[]
    for scan in scans:
        t=scan['t']
        if not 16<=t<=32:continue
        actual=at(t)
        if actual is None:continue
        assert scan['frame']=='sim_lidar_link'
        points=scan_points(scan,actual['robot'])
        candidates=dynamic_candidates(points,occ,.25,True)
        detections=cluster_points(candidates,.20,3,1.5)
        detections=filter_detections_near_static(detections,occ,.35)
        update=tracker.update(detections,t)
        confirmed=[x for x in update.tracks if x.state==TrackState.CONFIRMED]
        box=actual['obstacle'];near=sorted(confirmed,key=lambda x:math.hypot(x.position.x-box[0],x.position.y-box[1]))
        chosen=near[0] if near and math.hypot(near[0].position.x-box[0],near[0].position.y-box[1])<1 else None
        row={'t':t,'scan_points':len(points),'candidates':len(candidates),
             'detections':len(detections),'confirmed_count':len(confirmed),
             'confirmed_near_box':bool(chosen),'actual_obstacle_xy':box[:2]}
        if chosen:
            row.update(track_id=chosen.track_id,
                       estimated_xy=[chosen.position.x,chosen.position.y],
                       estimated_velocity_xy=[chosen.velocity.x,chosen.velocity.y],
                       estimated_visible_cluster_size_xy=[chosen.size_x,chosen.size_y],
                       last_observation_stamp=chosen.last_observation_timestamp,
                       centre_error_m=math.hypot(chosen.position.x-box[0],chosen.position.y-box[1]),
                       predicted_1s_xy=[chosen.prediction[9].x,chosen.prediction[9].y])
            row['prediction_errors_before_contact_m']={}
            row['age_compensated_errors_before_contact_m']={}
            for index in (1,3,5,7,9):
                horizon=(index+1)*.1
                if t+horizon<FIRST_BODY_OVERLAP_S:
                    future=at(t+horizon)
                    if future:
                        ox,oy,_=future['obstacle']
                        row['prediction_errors_before_contact_m'][str(round(horizon,1))]=math.hypot(
                            chosen.prediction[index].x-ox,chosen.prediction[index].y-oy)
                age=.15
                if t+age+horizon<FIRST_BODY_OVERLAP_S:
                    future=at(t+age+horizon)
                    if future:
                        ox,oy,_=future['obstacle']
                        px=chosen.position.x+chosen.velocity.x*(age+horizon)
                        py=chosen.position.y+chosen.velocity.y*(age+horizon)
                        row['age_compensated_errors_before_contact_m'][str(round(horizon,1))]=math.hypot(px-ox,py-oy)
        output.append(row)
    before=[r for r in output if 26<=r['t']<=28.1]
    near=[r for r in before if r['confirmed_near_box']]
    horizon_summary={}
    for key in ('0.2','0.4','0.6','0.8','1.0'):
        values=[r['prediction_errors_before_contact_m'][key] for r in near
                if key in r['prediction_errors_before_contact_m']]
        aged=[r['age_compensated_errors_before_contact_m'][key] for r in near
               if key in r['age_compensated_errors_before_contact_m']]
        horizon_summary[key]={'valid_precontact_samples':len(values),
                              'prediction_error_median_m':statistics.median(values) if values else None,
                              'age_015_compensated_valid_samples':len(aged),
                              'age_015_compensated_error_median_m':statistics.median(aged) if aged else None}
    result={'schema':'rm_dynamic_prediction_v1_tracker_probe/v2',
            'scope':'Offline replay of frozen QP failure scans; no controller or navigation candidate run.',
            'source_trial':str(TRIAL.relative_to(ROOT)),
            'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in
                [TRIAL/'observation/scans.jsonl',TRIAL/'gazebo_poses.jsonl',
                 ROOT/'src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py',
                 ROOT/'src/rm_simulation/worlds/phase1_omni.sdf',
                 ROOT/'src/rm_simulation/models/course_wall.sdf']},
            'static_map_rectangles_m':rectangles,
            'scan_count':len(output),
            'confirmed_near_box_total':sum(r['confirmed_near_box'] for r in output),
            'precollision_window_scan_count':len(before),
            'precollision_confirmed_near_box':len(near),
            'precollision_centre_error_median':statistics.median(r['centre_error_m'] for r in near) if near else None,
            'first_body_overlap_s':FIRST_BODY_OVERLAP_S,
            'precollision_visible_cluster_size_y_median':statistics.median(r['estimated_visible_cluster_size_xy'][1] for r in near) if near else None,
            'precollision_horizon_error':horizon_summary,
            'rows':output,
            'accepted_for_deployment':False}
    with (HERE/'probe_v2.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','rows')},indent=2))

if __name__=='__main__':main()
