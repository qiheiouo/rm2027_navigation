#!/usr/bin/env python3
"""Read-only recomputation using existing mesh vertices and frozen P2B witnesses.

The 127 mm construction is sensitivity analysis, never silently substituted for
an existing footprint. Outputs a new file and never changes runtime inputs.
"""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO/'experiments/tdt_planner/rm_tdt_planner/tools'))
from audit_endpoint_witness import audit_file
from simulation_geometry import OBSTACLES, polygon_distance

SOURCE_COMMIT = 'bd6cf689d3d5aca5c8edd430ff760ca5ceb27496'
MESH = 'src/rm_description/meshes/new_car_octagonal_chassis.obj'
DOG_CONFIG = 'src/rm_dog_hole/config/dog_hole_sim.yaml'
WORLD = 'src/rm_simulation/worlds/phase1_omni.sdf'


def source(path):
    return subprocess.check_output(['git', 'show', f'{SOURCE_COMMIT}:{path}'], cwd=REPO)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def radius(polygon, centre=(0., 0.)):
    # Norm is convex along each boundary segment, so a maximum lies at a
    # vertex. Valid for arbitrary polygon boundaries, including concave ones.
    if len(polygon)<3 or not all(math.isfinite(v) for p in polygon for v in p):
        raise ValueError('invalid polygon')
    return max(math.dist(p, centre) for p in polygon)


def padded(polygon, padding):
    # Nav2's per-coordinate sign padding; not a circular Minkowski offset.
    def sign(x): return 1. if x>0 else -1. if x<0 else 0.
    return [(x+sign(x)*padding, y+sign(y)*padding) for x,y in polygon]


def placed(polygon, pose):
    x,y,yaw=pose; c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in polygon]


def gap(polygon, pose, boxes=OBSTACLES):
    # Existing independent oracle uses SAT: all polygons in this audit must be
    # convex. Do not advertise this particular distance routine for concave CAD.
    turns=[]
    for i,a in enumerate(polygon):
        b=polygon[(i+1)%len(polygon)]; c=polygon[(i+2)%len(polygon)]
        turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not (all(t>0 for t in turns) or all(t<0 for t in turns)):
        raise ValueError('SAT oracle requires a strictly convex polygon')
    return min(polygon_distance(placed(polygon,pose),[(a,b),(c,b),(c,d),(a,d)]) for a,b,c,d in boxes)


def record(polygon, padding):
    pp=padded(polygon,padding)
    return {'vertices_m':polygon,'padding_m':padding,
            'axis_extents_m':[max(p[k] for p in polygon)-min(p[k] for p in polygon) for k in (0,1)],
            'edge_lengths_m':[math.dist(a,b) for a,b in zip(polygon,polygon[1:]+polygon[:1])],
            'spin_centre_m':[0.,0.], 'R_spin_body_m':radius(polygon),
            'R_spin_body_plus_physical_clearance_005_m':radius(polygon)+.05,
            'R_spin_nav2_padded_m':radius(pp),'planner_clearance_m':.02,
            'R_spin_nav2_padded_plus_planner_clearance_m':radius(pp)+.02,
            'nominal_exact_fixture_body_gap_yaw0_m':gap(polygon,(4.3,0.,0.)),
            'nominal_exact_fixture_spin_body_margin_above_005_m':.5-radius(polygon)-.05,
            'nominal_exact_fixture_spin_padded_margin_above_002_m':.5-radius(pp)-.02}


def main(output):
    mesh=source(MESH)
    vertices=[tuple(map(float,line.split()[1:])) for line in mesh.decode().splitlines() if line.startswith('v ')]
    octagon=[p[:2] for p in vertices if p[2]==min(v[2] for v in vertices)]
    dog=yaml.safe_load(source(DOG_CONFIG))['dog_hole_manager']['ros__parameters']['robot.footprint']
    assert octagon==list(zip(dog[::2],dog[1::2])) and len(octagon)==8
    sdf=ET.fromstring(source(WORLD))
    polyline=sdf.find(".//model[@name='rm_sentry_2027']/link[@name='base_link']/collision/geometry/polyline")
    assert octagon==[tuple(map(float,p.text.split())) for p in polyline.findall('point')]
    baseline=yaml.safe_load((REPO/'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml').read_text())
    maps=[baseline[n][n]['ros__parameters'] for n in ('local_costmap','global_costmap')]
    assert maps[0]['footprint']==maps[1]['footprint'] and maps[0]['footprint_padding']==maps[1]['footprint_padding']==.03
    old=[tuple(p) for p in yaml.safe_load(maps[0]['footprint'])]
    a=(.382+math.sqrt(2)*.127)/2; b=.382/2
    alternative=[(a,b),(b,a),(-b,a),(-a,b),(-a,-b),(-b,-a),(b,-a),(a,-b)]
    models={'p2a_old_car_rectangle':record([(-.32,-.27),(-.32,.27),(.32,.27),(.32,-.27)],.02),
            'p2b_simulation_rectangle':record(old,.03),
            'existing_new_car_126mm':record(octagon,.03),
            'unconfirmed_127mm_sensitivity_only':record(alternative,.03)}
    witnesses=[]; replays=[]
    for variant in ('baseline','path_heading_follow'):
        trial=REPO/'build/tdt_p2b/runs/heading_follow_v1'/variant/'tdt_qp_1'
        audit=audit_file(trial/'observation/events.jsonl')
        assert audit['audited']
        for item in audit['entries']:
            w=item['goal']
            if w['status']!='blocked': continue
            assert w['world']==[4.3,0.] and w['cost']>=254
            comparisons={}
            for name,m in models.items():
                poly=m['vertices_m']; pp=padded(poly,m['padding_m']); d=w['distance_m']
                comparisons[name]={'spin_margin_m':d-radius(pp)-.02,
                    'still_rejected_by_this_cell':d<=radius(pp)+.02+1e-7,
                    'body_gap_at_requested_yaw_m':gap(poly,(*w['world'],w['yaw']),[w['world_box']]),
                    'padded_gap_at_requested_yaw_m':gap(pp,(*w['world'],w['yaw']),[w['world_box']])}
            witnesses.append({'variant':variant,'event_line':item['line'],'t':item['t'],
                              'historical_goal_witness':w,'counterfactual_models':comparisons})
        rows=[json.loads(line) for line in (trial/'observation/trajectory.jsonl').read_text().splitlines()]
        for name,m in models.items():
            body=[gap(m['vertices_m'],(r['x'],r['y'],r['yaw'])) for r in rows]
            pp=padded(m['vertices_m'],m['padding_m'])
            pad=[gap(pp,(r['x'],r['y'],r['yaw'])) for r in rows]
            bounds=[min(body),min(pad)]
            for i,(u,v) in enumerate(zip(rows,rows[1:])):
                travel=math.hypot(v['x']-u['x'],v['y']-u['y'])
                angle=abs(math.remainder(v['yaw']-u['yaw'],2*math.pi))
                for k,(dist,rad) in enumerate(((body,radius(m['vertices_m'])),(pad,radius(pp)))):
                    bounds[k]=min(bounds[k],min(dist[i],dist[i+1])-.5*(travel+rad*angle))
            replays.append({'historical_variant':variant,'counterfactual_geometry':name,
                'trajectory_sha256':digest((trial/'observation/trajectory.jsonl').read_bytes()),
                'samples':len(rows),'min_body_gap_m':min(body),'min_padded_gap_m':min(pad),
                'linear_interpolation_body_bound_m':bounds[0],'linear_interpolation_padded_bound_m':bounds[1],
                'body_005_and_padded_no_contact':bounds[0]>=.05 and bounds[1]>0,
                'scope':'Same historical poses only; not a new navigation trial or prediction of action/recoveries.'})
    wheels=[]
    for link in sdf.findall(".//model[@name='rm_sentry_2027']/link"):
        if not link.attrib['name'].endswith('_wheel'): continue
        x,y,*_=map(float,link.find('pose').text.split())
        wheel_radius=float(link.find('collision/geometry/sphere/radius').text)
        overhang=(abs(x)+abs(y)-octagon[0][0]-octagon[0][1])/math.sqrt(2)+wheel_radius
        wheels.append({'link':link.attrib['name'],'centre_xy_m':[x,y],'placeholder_sphere_radius_m':wheel_radius,
                       'sweep_radius_m':math.hypot(x,y)+wheel_radius,'chamfer_overhang_m':overhang})
    assert len(witnesses)==26
    assert all(w['counterfactual_models']['p2b_simulation_rectangle']['still_rejected_by_this_cell'] and
               not w['counterfactual_models']['existing_new_car_126mm']['still_rejected_by_this_cell'] and
               not w['counterfactual_models']['unconfirmed_127mm_sensitivity_only']['still_rejected_by_this_cell'] for w in witnesses)
    # Analytic cross-checks: rectangle, arbitrary centre offset, polygon vs yaw.
    rectangle=[(3.,4.),(-3.,4.),(-3.,-4.),(3.,-4.)]
    assert radius(rectangle)==5.
    assert abs(radius(rectangle,(1.,0.))-math.sqrt(32))<1e-12
    assert gap(old,(4.3,0.,0.),[(4.,.35,4.05,.4)])>.05
    corner_yaw=math.atan2(.35,-.25)-math.atan2(.25,-.30)
    rotated_gap=gap(old,(4.3,0.,corner_yaw),[(4.,.35,4.05,.4)])
    assert abs(rotated_gap-(math.hypot(.25,.35)-math.hypot(.30,.25)))<1e-12
    assert rotated_gap<.05
    for m in (models['existing_new_car_126mm'],models['unconfirmed_127mm_sensitivity_only']):
        assert m['nominal_exact_fixture_spin_body_margin_above_005_m']>0
        assert m['nominal_exact_fixture_spin_padded_margin_above_002_m']>0
    report={'schema':'rm_tdt_planner/new_car_geometry_audit/v1',
        'development_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        'new_car_data_commit':SOURCE_COMMIT,'source_sha256':{p:digest(source(p)) for p in (MESH,DOG_CONFIG,WORLD)},
        'existing_mesh_yaml_sdf_vertices_identical':True,
        'centre_confirmation':'User confirmed base_link XY origin = geometric centre = actual Spin centre; no new vertical-origin or extrinsic claim.',
        'dimension_selection':'Pending 126 mm existing source versus 127 mm approximate description. Both evaluated; neither silently promoted to final CAD.',
        'models':models,'frozen_goal_witness_count':len(witnesses),'witnesses':witnesses,
        'exact_fixture_nominal_nearest_obstacle_distance_m':.5,
        'full_raw_snapshot_nominal_feasibility':'Not certified; one sufficient rejecting cell per witness, not all raw snapshot cells.',
        'historical_trajectory_counterfactuals':replays,'new_branch_wheel_proxy_mismatch':wheels,
        'new_static_trials':0,'accepted_for_deployment':False}
    with output.open('x') as f:
        json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print('26/26 historical goal witnesses cease to reject either new-car octagon.')
    print('New static trials: 0. Full raw-map feasibility and final dimensions not asserted.')

if __name__=='__main__': main(Path(sys.argv[1]))
