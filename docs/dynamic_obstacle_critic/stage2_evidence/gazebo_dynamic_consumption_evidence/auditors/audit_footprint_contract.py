#!/usr/bin/env python3
"""Offline experiment isolation and whole-circle plant containment checks."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import yaml
from audit_mechanical_footprint import components


def changes(a,b,path=()):
    if isinstance(a,dict) and isinstance(b,dict):
        return [v for key in sorted(set(a)|set(b)) for v in
                changes(a.get(key),b.get(key),path+(key,))]
    return [] if a==b else [{'path':'.'.join(path),'before':a,'after':b}]


def analyze(package,scene):
    paths={name:package/'config'/name for name in ('nav2_cv_soft_map_clearance.yaml',
        'nav2_cv_mechanical_footprint.yaml','guard.yaml','guard_mechanical_footprint.yaml')}
    cfg={name:yaml.safe_load(path.read_text()) for name,path in paths.items()}
    profile=cfg['nav2_cv_mechanical_footprint.yaml']
    cm=profile['local_costmap']['local_costmap']['ros__parameters']
    fp=yaml.safe_load(cm['footprint']); padded=[(math.copysign(abs(x)+cm['footprint_padding'],x),
        math.copysign(abs(y)+cm['footprint_padding'],y)) for x,y in fp]
    flat=cfg['guard_mechanical_footprint.yaml']['dynamic_safety_guard']['ros__parameters']['footprint']
    guard=list(zip(flat[::2],flat[1::2])); shapes=components(scene)
    bounds=(min(p[0] for p in fp),max(p[0] for p in fp),min(p[1] for p in fp),max(p[1] for p in fp))
    containment=[]
    for shape in shapes:
        if shape['kind']=='circle':
            x,y=shape['center'];r=shape['radius'];s=(x-r,x+r,y-r,y+r)
        else:
            points=shape['points'];s=(min(p[0] for p in points),max(p[0] for p in points),
                                    min(p[1] for p in points),max(p[1] for p in points))
        margins=[s[0]-bounds[0],bounds[1]-s[1],s[2]-bounds[2],bounds[3]-s[3]]
        containment.append({'component':shape['name'],'whole_shape_bounds':s,'margins':margins,
                            'contained':min(margins)>=-1e-12})
    pc=changes(cfg['nav2_cv_soft_map_clearance.yaml'],profile)
    gc=changes(cfg['guard.yaml'],cfg['guard_mechanical_footprint.yaml'])
    matching=(len(guard)==len(padded)==4 and max(min(math.dist(p,g) for g in guard) for p in padded)<=1e-12)
    gates={'nav2_only_local_global_footprints':{v['path'] for v in pc}=={
        'local_costmap.local_costmap.ros__parameters.footprint','global_costmap.global_costmap.ros__parameters.footprint'},
        'guard_only_footprint':len(gc)==1 and gc[0]['path']=='dynamic_safety_guard.ros__parameters.footprint',
        'all_physical_projections_contained':len(shapes)==5 and all(v['contained'] for v in containment),
        'guard_equals_padded_planning_envelope':matching}
    return {'verdict':'PASS' if all(gates.values()) else 'FAILED','scope':'fixed planar self-plant and configuration contract only',
            'gates':gates,'profile_changes':pc,'guard_changes':gc,'containment':containment,
            'planning_bounds':bounds,'padded_points':padded,'guard_points':guard,
            'source_sha256':{name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in paths.items()},
            'scene_sha256':hashlib.sha256((scene/'phase1_omni.sdf').read_bytes()).hexdigest()}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args()
    package=Path(__file__).resolve().parents[1]
    result=analyze(package,package.parent/'rm_simulation/worlds')
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    return 0 if result['verdict']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
