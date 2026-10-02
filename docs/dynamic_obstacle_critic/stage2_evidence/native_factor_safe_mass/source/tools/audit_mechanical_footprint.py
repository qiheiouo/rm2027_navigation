#!/usr/bin/env python3
"""Frozen-fixture planar collision union: base box and wheel spheres, offline only."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from analyze_trial import distance, load_truth_rows, rotation, travel
from audit_scan_geometry import fixture


def components(scene):
    robot = ET.parse(scene/'phase1_omni.sdf').getroot().find("world/model[@name='rm_sentry_2027']")
    if robot is None:
        raise ValueError('frozen fixture robot missing')
    base = robot.find("link[@name='base_link']")
    if base is None:
        raise ValueError('frozen fixture base_link missing')
    base_pose = [float(x) for x in (base.findtext('pose') or '0 0 0 0 0 0').split()]
    if len(base_pose) != 6 or any(v != 0 for v in base_pose):
        raise ValueError('projection audit requires model frame coincident with base_link')
    shapes = []
    for link in robot.findall('link'):
        lp = [float(x) for x in (link.findtext('pose') or '0 0 0 0 0 0').split()]
        link_pose = link.find('pose')
        if (len(lp) != 6 or not all(math.isfinite(v) for v in lp)
                or (link_pose is not None and link_pose.attrib.get('relative_to') not in
                    (None, '__model__', 'base_link'))):
            raise ValueError('unsupported fixture link frame or pose')
        for collision in link.findall('collision'):
            cp = [float(x) for x in (collision.findtext('pose') or '0 0 0 0 0 0').split()]
            collision_pose = collision.find('pose')
            if (len(cp) != 6 or not all(math.isfinite(v) for v in cp)
                    or (collision_pose is not None and collision_pose.attrib.get('relative_to') not in
                        (None, link.attrib['name']))):
                raise ValueError('unsupported fixture collision frame or pose')
            name = link.attrib['name'] + '/' + collision.attrib['name']
            sphere = collision.find('geometry/sphere')
            if sphere is not None:
                if any(cp[i] != 0 for i in (0, 1, 2)):
                    raise ValueError('audit supports fixture wheel spheres at link origin only')
                radius = float(sphere.findtext('radius'))
                if not math.isfinite(radius) or radius <= 0:
                    raise ValueError('invalid fixture sphere radius')
                shapes.append({'name': name, 'kind': 'circle', 'center': lp[:2], 'radius': radius})
            else:
                if any(abs(v) > 1e-12 for v in [lp[3],lp[4],cp[3],cp[4]]) or collision.find('geometry/box') is None:
                    raise ValueError('unsupported fixture collision geometry')
                x, y, z = [float(v) for v in collision.findtext('geometry/box/size').split()]
                if not all(math.isfinite(v) and v > 0 for v in (x,y,z)):
                    raise ValueError('invalid fixture box size')
                center = rotation((lp[0], lp[1], lp[5]), [cp[:2]])[0]
                points = rotation((*center,lp[5]+cp[5]), [(-x/2,-y/2),(x/2,-y/2),(x/2,y/2),(-x/2,y/2)])
                shapes.append({'name': name, 'kind': 'polygon', 'points': points})
    return shapes


def circle_box_gap(center, radius, box_pose, dimensions):
    x, y = center[0]-box_pose[0], center[1]-box_pose[1]
    c, s = math.cos(box_pose[2]), math.sin(box_pose[2])
    local = (c*x+s*y, -s*x+c*y)
    return max(0., math.hypot(max(abs(local[0])-dimensions[0]/2,0.),
                               max(abs(local[1])-dimensions[1]/2,0.))-radius)


def analyze(root, include_static=False):
    execution = json.loads((root/'execution.json').read_text())
    if execution['execution'] != 'PASS': return {'verdict': 'FAILED', 'scope': 'infrastructure_before_goal'}
    shapes = components(root/'scene_inputs'); scene = fixture(root/'scene_inputs')
    dimensions = scene['actor_dimensions']; x, y = dimensions
    actor_shape = [(-x/2,-y/2),(x/2,-y/2),(x/2,y/2),(-x/2,y/2)]
    truth = load_truth_rows(root, execution); values = []
    for r in truth:
        actor = rotation(r['obstacle'], actor_shape); gaps = []
        for shape in shapes:
            if shape['kind'] == 'polygon': gap = distance(rotation(r['robot'],shape['points']),actor)
            else: gap = circle_box_gap(rotation(r['robot'],[shape['center']])[0],shape['radius'],r['obstacle'],dimensions)
            gaps.append((gap,shape['name']))
        gap, component = min(gaps); values.append({'time':r['t'],'gap':gap,'component':component})
    support = []
    for shape in shapes:
        if shape['kind'] == 'polygon': support.extend(shape['points'])
        else:
            cx, cy = shape['center']; rad = shape['radius']; support.extend([(cx-rad,cy),(cx+rad,cy),(cx,cy-rad),(cx,cy+rad)])
    robot_radius = max(math.hypot(*p) for p in support)
    # Circumradius of a circle is center norm + radius, not just its axis extrema.
    robot_radius = max([robot_radius]+[math.hypot(*s['center'])+s['radius'] for s in shapes if s['kind']=='circle'])
    actor_radius = math.hypot(x,y)/2; lower = min(v['gap'] for v in values)
    for i,(a,b) in enumerate(zip(truth,truth[1:])):
        lower = min(lower,min(values[i]['gap'],values[i+1]['gap'])-.5*(travel(a['robot'],b['robot'],robot_radius)+travel(a['obstacle'],b['obstacle'],actor_radius)))
    policy = json.loads((root/'policy.json').read_text()); profile = yaml.safe_load((root/'profile.yaml').read_text())
    cm = profile['local_costmap']['local_costmap']['ros__parameters']; fp = yaml.safe_load(cm['footprint']); pad = cm['footprint_padding']
    padded = [tuple(v+(pad if v>0 else -pad if v<0 else 0) for v in p) for p in fp]
    result = {'scope':'offline planar union of frozen fixture collision projections; not 3D contacts or hardware',
            'verdict':'CONDITIONAL PASS' if lower>=policy['body_clearance'] else 'FAILED',
            'collision_components':shapes, 'projected_union_xy_bounds':[min(p[0] for p in support),max(p[0] for p in support),min(p[1] for p in support),max(p[1] for p in support)],
            'configured_padded_xy_bounds':[min(p[0] for p in padded),max(p[0] for p in padded),min(p[1] for p in padded),max(p[1] for p in padded)],
            'dynamic_sample_min':min(v['gap'] for v in values),'dynamic_linear_interpolation_bound':lower,
            'first_margin_violation':next((v for v in values if v['gap']<policy['body_clearance']),None),
            'first_projected_contact':next((v for v in values if v['gap']==0),None),
            'limits':'Exact circle/box distances under fixed planar poses. Projection is conservative in height; does not reconstruct engine contacts. Linear interpolation bounds are conditional. Static/raw/task gates remain separate; no earlier archive verdict is overwritten.'}
    if include_static:
        sx,sy=scene['static_dimensions'];static_pose=scene['static_pose']
        static=rotation(static_pose,[(-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)])
        static_gaps=[]
        for r in truth:
            gaps=[]
            for shape in shapes:
                if shape['kind']=='polygon':gap=distance(rotation(r['robot'],shape['points']),static)
                else:gap=circle_box_gap(rotation(r['robot'],[shape['center']])[0],shape['radius'],static_pose,(sx,sy))
                gaps.append(gap)
            static_gaps.append(min(gaps))
        static_lower=min(static_gaps)
        for i,(a,b) in enumerate(zip(truth,truth[1:])):
            static_lower=min(static_lower,min(static_gaps[i:i+2])-.5*travel(a['robot'],b['robot'],robot_radius))
        result['static_sample_min']=min(static_gaps);result['static_linear_interpolation_bound']=static_lower
        result['verdict']='CONDITIONAL PASS' if min(lower,static_lower)>=policy['body_clearance'] else 'FAILED'
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);parser.add_argument('--include-static',action='store_true');args=parser.parse_args();result=analyze(args.trial,args.include_static)
    (args.trial/'mechanical_footprint_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
