#!/usr/bin/env python3
"""Independent planar physical geometry audit. Truth is used only after the trial."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import yaml
from trial_io import rows as log_rows


def rotation(pose,points):
    x,y,a=pose;c=math.cos(a);s=math.sin(a)
    return [(x+c*u-s*v,y+s*u+c*v) for u,v in points]


def segment(a,b,p):
    dx=b[0]-a[0];dy=b[1]-a[1];den=dx*dx+dy*dy
    t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def distance(a,b):
    separated=False
    for shape in (a,b):
        for i,p in enumerate(shape):
            q=shape[(i+1)%len(shape)];axis=(p[1]-q[1],q[0]-p[0])
            aa=[x*axis[0]+y*axis[1] for x,y in a];bb=[x*axis[0]+y*axis[1] for x,y in b]
            separated|=max(aa)<min(bb) or max(bb)<min(aa)
    if not separated:return 0.
    return min(segment(shape[i],shape[(i+1)%len(shape)],p) for shape,other in ((a,b),(b,a)) for i in range(len(shape)) for p in other)


def pose(entry):
    p=entry['position'];q=entry['orientation'];x,y,z,w=[q.get(k,0.) for k in ('x','y','z','w')]
    if abs(x)>1e-3 or abs(y)>1e-3:raise ValueError('nonplanar truth pose')
    return (p.get('x',0.),p.get('y',0.),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def compose(a,b):
    xy=rotation(a,[(b[0],b[1])])[0];return (*xy,a[2]+b[2])


def travel(a,b,radius):
    return math.hypot(b[0]-a[0],b[1]-a[1])+radius*abs(math.remainder(b[2]-a[2],2*math.pi))


def load_truth_rows(root, execution):
    rows=[]
    for msg in log_rows(root,'gazebo_poses.jsonl'):
        by={p['name']:p for p in msg.get('pose',[]) if p.get('name') in ('rm_sentry_2027','base_link','moving_obstacle','obstacle_link')}
        if len(by)!=4:continue
        stamp=msg['header']['stamp'];t=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
        if t<execution['start_sim'] or t>execution['last_sim']:continue
        if rows and t<=rows[-1]['t']:continue
        rows.append({'t':t,'robot':compose(pose(by['rm_sentry_2027']),pose(by['base_link'])),'obstacle':compose(pose(by['moving_obstacle']),pose(by['obstacle_link']))})
    return rows


def analyze(root):
    execution=json.loads((root/'execution.json').read_text());policy=json.loads((root/'policy.json').read_text())
    if execution['execution']!='PASS':return {'verdict':'FAILED','layer':'test_execution','execution':execution}
    config=yaml.safe_load((root/'profile.yaml').read_text());cm=config['local_costmap']['local_costmap']['ros__parameters'];body=yaml.safe_load(cm['footprint']);pad=cm['footprint_padding']
    padded=[tuple(v+(pad if v>0 else -pad if v<0 else 0) for v in xy) for xy in body]
    obstacle=[(-.225,-.275),(.225,-.275),(.225,.275),(-.225,.275)]
    static=rotation((1.4,0,0),[(-.175,-.55),(.175,-.55),(.175,.55),(-.175,.55)])
    rows=load_truth_rows(root,execution)
    if len(rows)<2:return {'verdict':'FAILED','layer':'ground_truth','reason':'missing physical actor/robot poses','execution':execution}
    metrics={}
    for name,poly in [('body',body),('padded',padded)]:
        dynamic=[distance(rotation(r['robot'],poly),rotation(r['obstacle'],obstacle)) for r in rows]
        static_gap=[distance(rotation(r['robot'],poly),static) for r in rows];dynamic_lower=min(dynamic);static_lower=min(static_gap)
        rr=max(math.hypot(*p) for p in poly);ro=max(math.hypot(*p) for p in obstacle)
        for i,(a,b) in enumerate(zip(rows,rows[1:])):
            motion=travel(a['robot'],b['robot'],rr);relative=motion+travel(a['obstacle'],b['obstacle'],ro)
            dynamic_lower=min(dynamic_lower,min(dynamic[i:i+2])-.5*relative)
            static_lower=min(static_lower,min(static_gap[i:i+2])-.5*motion)
        witness=min(range(len(rows)),key=dynamic.__getitem__)
        metrics[name]={'dynamic_sample_min':min(dynamic),'dynamic_linear_interpolation_bound':dynamic_lower,
            'static_sample_min':min(static_gap),'static_linear_interpolation_bound':static_lower,'witness':rows[witness],
            'sampled_overlap':min(dynamic)==0.}
    counts=Counter();critic=Counter();violations=0;commands=0
    for row in log_rows(root,'observations.jsonl'):
        if row['receive_sim']<execution['start_sim']:continue
        if row['kind']=='guard':counts.update(s['reason'] for s in row['statuses'])
        if row['kind']=='critic':critic.update(s['reason'] for s in row['statuses'])
        if row['kind']=='final_cmd':
            commands+=1;vx,vy,wz=row['velocity'];tol=policy['bounds_tolerance']
            violations+=vx<-.5-tol or vx>.8+tol or abs(vy)>.5+tol or abs(wz)>1.2+tol
    gates={'body_clearance':min(metrics['body']['dynamic_linear_interpolation_bound'],metrics['body']['static_linear_interpolation_bound'])>=policy['body_clearance'],
           'padded_no_contact':min(metrics['padded']['dynamic_linear_interpolation_bound'],metrics['padded']['static_linear_interpolation_bound'])>0,
           'output_bounds':commands>0 and violations==0,'goal_success':execution['goal_status']==4,
           'strict_positive_progress':rows[-1]['robot'][0]>rows[0]['robot'][0]}
    result={'verdict':'PASS' if all(gates.values()) else 'FAILED','scope':'single full physical trial; no deployment acceptance',
            'gates':gates,'geometry':metrics,'truth_rows':len(rows),'max_truth_interval':max(b['t']-a['t'] for a,b in zip(rows,rows[1:])),
            'commands':commands,'bounds_violations':violations,'goal_progress_x':rows[-1]['robot'][0]-rows[0]['robot'][0],
            'guard_reasons':dict(counts),'critic_statuses':dict(critic),'execution':execution,
            'limits':'Interpolation bounds assume linear pose interpolation. No per-candidate coverage/SG history capture; cannot attribute sampler or optimizer from closed loop alone.'}
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path);args=p.parse_args();result=analyze(args.trial)
    (args.trial/'analysis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0
if __name__=='__main__':raise SystemExit(main())
