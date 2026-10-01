#!/usr/bin/env python3
"""Offline truth labels for recorded tracker centers and CV occupied disks; never controller input."""
import argparse
import bisect
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
import yaml
from analyze_trial import compose,pose,rotation
from trial_io import rows as log_rows


def main():
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path);args=p.parse_args();root=args.trial
    execution=json.loads((root/'execution.json').read_text());config=yaml.safe_load((root/'profile.yaml').read_text())
    limits=config['controller_server']['ros__parameters']['FollowPath']['DynamicObstacleCritic']
    truth=[]
    for msg in log_rows(root,'gazebo_poses.jsonl'):
        by={x['name']:x for x in msg.get('pose',[]) if x.get('name') in ('moving_obstacle','obstacle_link')}
        if len(by)!=2:continue
        stamp=msg['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
        if truth and t<=truth[-1][0]:continue
        truth.append((t,compose(pose(by['moving_obstacle']),pose(by['obstacle_link']))))
    times=[x[0] for x in truth]
    def actual(t):
        if not times or t<times[0] or t>times[-1]:return None
        j=bisect.bisect_right(times,t)
        if j==len(times):return truth[-1][1]
        a,b=truth[j-1],truth[j];u=(t-a[0])/(b[0]-a[0]);return tuple(x+(y-x)*u for x,y in zip(a[1],b[1]))
    supports=defaultdict(list);position_errors=[];vx=[];radii=[];matched=0;unmatched=0
    for row in log_rows(root,'observations.jsonl'):
        if row['kind']!='obstacles' or execution['start_sim'] is None or not execution['start_sim']<=row['receive_sim']<=execution['end_sim']:continue
        if not row['complete'] or row['frame']!=limits['input_frame']:continue
        source=row['stamp'];a=actual(source)
        if a is None:continue
        candidates=[t for t in row['tracks'] if t['state'] in (2,3) and row['receive_sim']-t['observed']<=limits['max_observation_age']]
        if not candidates:unmatched+=1;continue
        track=min(candidates,key=lambda t:math.hypot(t['xy'][0]-a[0],t['xy'][1]-a[1]))
        error=math.hypot(track['xy'][0]-a[0],track['xy'][1]-a[1])
        if error>1.0:unmatched+=1;continue
        matched+=1;position_errors.append(error);vx.append(abs(track['vxy'][0]));radius=max(limits['minimum_obstacle_radius'],.5*math.hypot(*track['size']));radii.append(radius)
        for horizon in (0.,1.,2.,3.):
            future=actual(source+horizon)
            if future is None:continue
            predicted=[track['xy'][i]+track['vxy'][i]*horizon for i in (0,1)]
            corners=rotation(future,[(-.225,-.275),(.225,-.275),(.225,.275),(-.225,.275)])
            maximum=max(math.hypot(x-predicted[0],y-predicted[1]) for x,y in corners)
            supports[str(horizon)].append({'covered':maximum<=radius,'center_error':math.hypot(predicted[0]-future[0],predicted[1]-future[1]),'support_deficit':max(0,maximum-radius)})
    result={'scope':'observer-recorded tracker states; offline actual actor-box labels, not exact consumer snapshots or proof of the failing MPPI candidate',
        'matched_confirmed_or_fresh_coasting_frames':matched,'no_usable_actor_track_frames':unmatched,
        'source_center_error_median':statistics.median(position_errors) if position_errors else None,
        'source_center_error_max':max(position_errors) if position_errors else None,
        'abs_estimated_vx_median':statistics.median(vx) if vx else None,'physical_actor_x_is_constant':4.9,
        'radius_median':statistics.median(radii) if radii else None,'CV_support':{}}
    for horizon,rows in supports.items():
        result['CV_support'][horizon]={'frames':len(rows),'full_physical_box_covered':sum(x['covered'] for x in rows),
            'center_error_median':statistics.median(x['center_error'] for x in rows),'center_error_max':max(x['center_error'] for x in rows),
            'support_deficit_max':max(x['support_deficit'] for x in rows)}
    (root/'prediction_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
