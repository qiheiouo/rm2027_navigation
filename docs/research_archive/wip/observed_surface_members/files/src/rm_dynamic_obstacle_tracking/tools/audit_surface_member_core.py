#!/usr/bin/env python3
"""Frozen core numerical parity, including actual optional assignment capture."""
from dataclasses import asdict
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rm_dynamic_obstacle_tracking import core


def audit(reference):
    spec=importlib.util.spec_from_file_location('frozen_surface_reference',reference)
    old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
    checks=0;cluster_checks=0
    for mode in ('filtered','last_observation_cv'):
        for global_assignment in (False,True):
            for min_hits in (1,3):
                settings=dict(public_anchor_mode=mode,use_global_assignment=global_assignment,
                              min_hits_to_confirm=min_hits,prediction_steps=30,
                              velocity_decay_tau=0.,max_prediction_speed=0.)
                legacy=old.MultiObjectTracker(**settings)
                plain=core.MultiObjectTracker(**settings)
                captured=core.MultiObjectTracker(**settings)
                random_source=random.Random(90817)
                for frame in range(300):
                    stamp_ns=19_000_000_001+(frame%113)*100_000_000
                    points=[]
                    if frame%37 not in range(9):
                        for target in range(3):
                            if (frame+target)%11==0:continue
                            cx=target*.75+.35*math.sin(frame*.09+target)
                            cy=target*.25+.3*math.cos(frame*.05+target)
                            for beam in range(4):
                                points.append((cx+beam*.03+random_source.uniform(-.003,.003),cy+random_source.uniform(-.002,.002)))
                    old_d=old.cluster_points([old.Point2D(*p) for p in points],.2,3,1.5)
                    current_p=[core.Point2D(*p) for p in points]
                    current_d=core.cluster_points(current_p,.2,3,1.5)
                    members=core.cluster_detections_with_members(current_p,.2,3,1.5)
                    if [asdict(d) for d in old_d]!=[asdict(d) for d in current_d] or [d for d,m in members]!=current_d:
                        raise ValueError('cluster arithmetic/order changed')
                    cluster_checks+=1
                    kwargs={'source_stamp_ns':stamp_ns} if mode=='last_observation_cv' else {}
                    baseline=asdict(legacy.update(old_d,stamp_ns/1e9,**kwargs))
                    for tracker,capture in ((plain,False),(captured,True)):
                        result=asdict(tracker.update(current_d,stamp_ns/1e9,capture_assignments=capture,**kwargs))
                        assignments=result.pop('detection_track_ids')
                        invalid=(len(assignments)!=len(current_d) or len(set(assignments))!=len(assignments)
                                 or any(v<=0 for v in assignments)) if capture else bool(assignments)
                        if invalid:
                            raise ValueError('incomplete or duplicate actual assignment metadata')
                        if json.dumps(result,sort_keys=True)!=json.dumps(baseline,sort_keys=True):raise ValueError('old numeric snapshot changed')
                        checks+=1
    return {'verdict':'PASS','numeric_update_checks':checks,'cluster_checks':cluster_checks,
            'frozen_source_sha256':hashlib.sha256(reference.read_bytes()).hexdigest(),
            'current_source_sha256':hashlib.sha256(Path(core.__file__).read_bytes()).hexdigest(),
            'scope':'Exact old numerical fields across two anchors, assignment policies, confirmation/coasting/reset; capture assignments are metadata only. No DDS/control acceptance.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('reference',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    result=audit(args.reference)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
