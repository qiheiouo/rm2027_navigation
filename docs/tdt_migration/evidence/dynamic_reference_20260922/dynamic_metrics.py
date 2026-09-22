"""Independent moving polygon oracle using synchronized Gazebo model/link poses.

Reuses the existing convex SAT distance; bounds assume linear planar pose
interpolation, not arbitrary unobserved accelerations between samples.
"""
from pathlib import Path
import json
import math
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'docs/tdt_migration/evidence/new_car_geometry_audit_20260917'))
from audit_geometry import placed, padded, radius, gap
from simulation_geometry import polygon_distance


def stamp(message):
    s=message['header']['stamp']
    return float(s.get('sec',0))+float(s.get('nsec',0))*1e-9


def pose(item):
    p=item['position'];q=item['orientation'];x,y,z,w=[q.get(k,0.) for k in ('x','y','z','w')]
    if abs(x)>1e-3 or abs(y)>1e-3:
        raise ValueError('nonplanar pose requires 3D projection audit')
    return (p.get('x',0.),p.get('y',0.),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def compose(parent,child):
    x,y,a=parent;u,v,b=child
    return (x+math.cos(a)*u-math.sin(a)*v,y+math.sin(a)*u+math.cos(a)*v,math.remainder(a+b,2*math.pi))


def rows_from_transport(path):
    rows=[];required=('rm_sentry_2027','base_link','moving_obstacle','obstacle_link')
    for line in path.open():
        message=json.loads(line);by={}
        for p in message['pose']:
            if p['name'] in required:
                if p['name'] in by:raise ValueError('ambiguous named pose')
                by[p['name']]=p
        if not all(n in by for n in required):continue  # Before successful spawn.
        row={'t':stamp(message),'robot':compose(pose(by['rm_sentry_2027']),pose(by['base_link'])),
             'obstacle':compose(pose(by['moving_obstacle']),pose(by['obstacle_link']))}
        if not all(math.isfinite(v) for v in (row['t'],*row['robot'],*row['obstacle'])):raise ValueError('nonfinite pose')
        if rows and row['t']<=rows[-1]['t']:raise ValueError('nonmonotonic transport timestamps')
        rows.append(row)
    if len(rows)<2:raise ValueError('missing actual obstacle motion')
    return rows


def obstacle_polygon():
    sdf=ET.parse(ROOT/'src/rm_simulation/models/moving_obstacle.sdf')
    x,y,z=map(float,sdf.find('.//link[@name="obstacle_link"]/collision/geometry/box/size').text.split())
    return [(-x/2,-y/2),(x/2,-y/2),(x/2,y/2),(-x/2,y/2)]


def travel(a,b,rad):
    return math.hypot(b[0]-a[0],b[1]-a[1])+rad*abs(math.remainder(b[2]-a[2],2*math.pi))


def geometry_metrics(rows,polygon,padding,obstacle):
    shapes=[polygon,padded(polygon,padding)];result={};r_obstacle=radius(obstacle)
    for name,poly in zip(('body','padded'),shapes):
        static=[gap(poly,r['robot']) for r in rows]
        moving=[polygon_distance(placed(poly,r['robot']),placed(obstacle,r['obstacle'])) for r in rows]
        sb=min(static);mb=min(moving);rad=radius(poly)
        for i,(a,b) in enumerate(zip(rows,rows[1:])):
            robot_motion=travel(a['robot'],b['robot'],rad)
            relative_motion=robot_motion+travel(a['obstacle'],b['obstacle'],r_obstacle)
            sb=min(sb,min(static[i:i+2])-.5*robot_motion)
            mb=min(mb,min(moving[i:i+2])-.5*relative_motion)
        index=min(range(len(rows)),key=moving.__getitem__)
        result[name]={'static_sample_min_m':min(static),'moving_sample_min_m':min(moving),
            'static_interpolation_bound_m':sb,'moving_interpolation_bound_m':mb,
            'overall_interpolation_bound_m':min(sb,mb),'sampled_moving_overlap':min(moving)==0.,
            'moving_min_witness':{**rows[index],'gap_m':moving[index]}}
    result['body_clearance_at_least_005m']=result['body']['overall_interpolation_bound_m']>=.05
    result['padded_footprint_no_contact']=result['padded']['overall_interpolation_bound_m']>0.
    return result
