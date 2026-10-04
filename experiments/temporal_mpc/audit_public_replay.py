#!/usr/bin/env python3
"""Audit captured public input without inventing omitted authority/TF/odometry."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import math


def audit(path):
    records=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    missing=Counter();frames=Counter();schemas=Counter();states=Counter();invalid=Counter()
    delays=[];regressions=0;previous=None;sample=[]
    for index,p in enumerate(records):
        for name in ('authority','source_t','processing_t','frame','schema','complete','total_track_count','prediction_dt','prediction_steps','tracks'):
            if name not in p:missing[name]+=1
        frames[str(p.get('frame'))]+=1;schemas[str(p.get('schema'))]+=1
        source=p.get('source_t')
        if not isinstance(source,(int,float)) or not math.isfinite(source):invalid['source']+=1
        else:
            if previous is not None and source<=previous:regressions+=1
            previous=source
            for name in ('receive_ros_t','processing_t'):
                value=p.get(name)
                if isinstance(value,(int,float)) and math.isfinite(value):delays.append((name,value-source))
        tracks=p.get('tracks',[])
        if p.get('total_track_count')!=len(tracks):invalid['count']+=1
        if p.get('complete') is not True:invalid['incomplete']+=1
        for t in tracks:
            states[str(t.get('state'))]+=1
            for name in ('xy','vxy','size_xy','last_observation_t','future_xy'):
                if name not in t:missing['track.'+name]+=1
        if index<3:sample.append(p)
    transport={}
    for name in ('receive_ros_t','processing_t'):
        values=[t for n,t in delays if n==name]
        if values:transport[name]={'count':len(values),'min_s':min(values),'max_s':max(values),'mean_s':sum(values)/len(values)}
    blockers=[]
    if missing['authority']:blockers.append('recorded JSONL omitted authority; cannot reconstruct it')
    if set(frames)-{'map'}:blockers.append('source frame is not map; captured source-epoch TF required')
    if set(schemas)-{'rm_dynamic_obstacle_predictions/v2_observation_anchor'}:
        blockers.append('v1 capture requires explicitly configured v1 intake, not v2 relabeling')
    blockers.append('JSONL alone lacks synchronized measured ROS odometry and source-epoch TF; Gazebo poses are truth')
    return {'source':str(path.resolve()),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'records':len(records),'frames':dict(frames),'schemas':dict(schemas),'states':dict(states),
            'missing_fields':dict(missing),'invalid_counts':dict(invalid),'nonincreasing_sources':regressions,
            'recorded_delays':transport,'samples':sample,'live_consumer_acceptance':False,
            'blockers':blockers,'scope':'real public input audit only; no fabricated authority/TF/odom/geometry'}


def main():
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=audit(a.source);a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))


if __name__=='__main__':main()
