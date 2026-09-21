#!/usr/bin/env python3
"""Single footprint-variable pilot. Original SDF is an explicit surrogate carrier.

The existing observer's rectangle summary remains untouched and audited as such.
A separate polygon-at-measured-yaw result evaluates the target platform. This
pilot does not claim to implement a polygon-aware T-DT search/endpoint contract.
"""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'new_car_geometry_audit_20260917'))
from audit_geometry import REPO, MESH, SOURCE_COMMIT, source, digest, padded, radius, gap
from summarize_simulation import inspect_trial, read_rows
from heading_ab import yaw_metrics

HERE=Path(__file__).resolve().parent
WORK=REPO/'build/tdt_p2b'
ROOT=WORK/'runs/new_car_geometry_reference_v2'
PLANNERS=('tdt_astar','tdt_qp')
FIXTURES=['src/rm_simulation/worlds/phase1_omni.sdf','src/rm_simulation/models/course_wall.sdf',
          'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml',
          'experiments/tdt_planner/rm_tdt_planner/launch/simulation_comparison.launch.py']


def write_new(path,data):
    with path.open('x') as f:json.dump(data,f,indent=2,allow_nan=False);f.write('\n')


def prepare():
    ROOT.mkdir(parents=True,exist_ok=False); (ROOT/'profiles').mkdir()
    audit=json.loads((HERE.parent/'new_car_geometry_audit_20260917/audit.json').read_text())
    polygon=audit['models']['existing_new_car_126mm']['vertices_m']
    profile_hashes={}; baseline_hashes={}
    for name in PLANNERS:
        file=WORK/'profiles_p2b'/f'{name}.yaml'; base=yaml.safe_load(file.read_text()); candidate=copy.deepcopy(base)
        for key in ('local_costmap','global_costmap'):
            candidate[key][key]['ros__parameters']['footprint']=json.dumps(polygon)
        output=ROOT/'profiles'/f'{name}.yaml'
        output.write_text(yaml.safe_dump(candidate,sort_keys=False))
        profile_hashes[name]=digest(output.read_bytes());baseline_hashes[name]=digest(file.read_bytes())
    write_new(ROOT/'manifest.json',{'schema':'rm_tdt_planner/new_car_geometry_reference/v1',
        'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        'footprint_source_commit':SOURCE_COMMIT,'footprint_source_path':MESH,'footprint_source_sha256':digest(source(MESH)),
        'body_polygon_m':polygon,'padding_m':.03,'physical_clearance_m':.05,'planner_clearance_m':.02,
        'dimension_status':'Existing 382/126 mm reference; 127 mm selection pending, not final CAD acceptance.',
        'centre_confirmation':'User confirmed base_link/geometric/Spin centres coincide.',
        'profiles':profile_hashes,'baseline_profiles':baseline_hashes,
        'source_sha256':{str(p.relative_to(REPO)):digest(p.read_bytes()) for p in HERE.iterdir() if p.suffix in ('.py','.sh')},
        'only_profile_changes':['local_costmap.local_costmap.ros__parameters.footprint','global_costmap.global_costmap.ros__parameters.footprint'],
        'diagnostic_change':'Only planner_server DEBUG; planner rosout records captured before stdout flushing. V1 is frozen; this series repairs missing QP observability.',
        'world':'Unchanged Phase1.5 surrogate carrier. Target polygon safety is separate from carrier and old-car physical oracle.',
        'planner_geometry':'Unchanged T-DT conservative circumscribed-circle backend derived from new polygon; no SE(2) search implemented.',
        'fixture_sha256':{f:digest((REPO/f).read_bytes()) for f in FIXTURES}})


def verify():
    m=json.loads((ROOT/'manifest.json').read_text())
    for p,h in {**m['source_sha256'],**m['fixture_sha256']}.items():
        if digest((REPO/p).read_bytes())!=h:raise ValueError('frozen input changed: '+p)
    for name in PLANNERS:
        file=WORK/'profiles_p2b'/f'{name}.yaml'; candidate=ROOT/'profiles'/f'{name}.yaml'
        if digest(file.read_bytes())!=m['baseline_profiles'][name] or digest(candidate.read_bytes())!=m['profiles'][name]:
            raise ValueError('profile hash mismatch')
        expected=yaml.safe_load(file.read_text()); actual=yaml.safe_load(candidate.read_text())
        for key in ('local_costmap','global_costmap'):
            expected[key][key]['ros__parameters']['footprint']=json.dumps(m['body_polygon_m'])
        if actual!=expected:raise ValueError('non-footprint variable changed')
    return m


def model_metrics(rows,poly,padding):
    pp=padded(poly,padding)
    body=[gap(poly,(r['x'],r['y'],r['yaw'])) for r in rows]
    pad=[gap(pp,(r['x'],r['y'],r['yaw'])) for r in rows]
    bounds=[min(body),min(pad)]
    for i,(a,b) in enumerate(zip(rows,rows[1:])):
        travel=math.hypot(b['x']-a['x'],b['y']-a['y']);angle=abs(math.remainder(b['yaw']-a['yaw'],2*math.pi))
        for k,(dist,rad) in enumerate(((body,radius(poly)),(pad,radius(pp)))):
            bounds[k]=min(bounds[k],min(dist[i],dist[i+1])-.5*(travel+rad*angle))
    return {'min_body_clearance_m':min(body),'min_padded_clearance_m':min(pad),
            'linear_pose_interpolation_body_bound_m':bounds[0],'linear_pose_interpolation_padded_bound_m':bounds[1],
            'body_clearance_at_least_005m':bounds[0]>=.05,'padded_footprint_no_contact':bounds[1]>0}


def endpoint(raw,poly,padding,pose):
    grid=raw['map']; w,h=grid['width'],grid['height']; r=grid['resolution'];ox,oy=grid['origin']
    if w*h!=len(grid['data']) or not 0<r<=1 or not all(math.isfinite(v) for v in (r,ox,oy)):
        raise ValueError('invalid raw snapshot')
    x,y,yaw=pose;inside=ox<x<ox+w*r and oy<y<oy+h*r
    nearest=None; ordinary=None; centre_253=False
    for index,cost in enumerate(grid['data']):
        ix,iy=index%w,index//w; box=(ox+ix*r,oy+iy*r,ox+(ix+1)*r,oy+(iy+1)*r)
        boundary=ix in (0,w-1) or iy in (0,h-1)
        if cost==253 and box[0]-1e-7<=x<=box[2]+1e-7 and box[1]-1e-7<=y<=box[3]+1e-7:centre_253=True
        if cost<254 and not boundary:continue
        near=math.hypot(x-min(max(x,box[0]),box[2]),y-min(max(y,box[1]),box[3]))
        if nearest is None or near<nearest['distance_m']:
            nearest={'cell':[ix,iy],'cost':cost,'boundary':boundary,'world_box':box,'distance_m':near}
        distance=gap(padded(poly,padding),pose,[box])
        if ordinary is None or distance<ordinary['distance_m']:
            ordinary={'cell':[ix,iy],'cost':cost,'boundary':boundary,'world_box':box,'distance_m':distance}
    # Raw service resolution is float32. The 1e-6 numerical guard conservatively
    # exceeds accumulated coordinate rounding at this fixture size, not a relaxed clearance.
    required=radius(padded(poly,padding))+.02
    return {'pose':pose,'inside_map':inside,'centre_253':centre_253,'nearest_raw_obstacle':nearest,
        'ordinary_padded_polygon_witness':ordinary,'ordinary_required_clearance_m':.02,
        'ordinary_polygon_safe':bool(inside and not centre_253 and ordinary['distance_m']>.020001),
        'derived_spin_required_m':required,'spin_safe':bool(inside and not centre_253 and nearest['distance_m']>required+1e-6),
        'scope':'Complete post-navigation raw map at capture time, lethal/unknown/boundary squares; 253 centre exclusion. Not dynamic or high-speed Spin admission.'}


def analyze_trial(path):
    m=verify(); original=inspect_trial(path)
    if not original.get('raw_summary_matches'):raise ValueError('original observer audit failed: '+str(original.get('audit_error')))
    rows=read_rows(path/'observation/trajectory.jsonl'); summary=original['summary']
    raw=json.loads((path/'runtime_geometry.json').read_text())
    for key in ('local_costmap','global_costmap'):
        actual=raw['runtime'][key]['actual']
        if yaml.safe_load(actual['footprint'])!=m['body_polygon_m'] or actual['footprint_padding']!=.03:
            raise ValueError('runtime geometry differs from frozen source')
    target=model_metrics(rows,m['body_polygon_m'],.03)
    checks=dict(summary['checks']);checks.update({k:target[k] for k in ('body_clearance_at_least_005m','padded_footprint_no_contact')})
    old_car=model_metrics(rows,[(-.32,-.27),(-.32,.27),(.32,.27),(.32,-.27)],.02)
    events=read_rows(path/'observation/events.jsonl')
    params=[e for e in events if e.get('event')=='runtime_parameters']
    if len(params)!=2 or any(e['actual']!=e['expected'] for e in params):raise ValueError('planner/controller runtime mismatch')
    diagnostics=read_rows(path/'observation/planner_diagnostics.jsonl')
    logs='\n'.join(e['message'] for e in diagnostics)
    successes=[e for e in diagnostics if e['message'].startswith('GridBased: ')]
    from audit_endpoint_witness import audit_file
    wa=audit_file(path/'observation/events.jsonl')
    if wa['missing_witnesses']:raise ValueError('missing endpoint rejection evidence')
    report={'planner':original['planner'],'trial':original['trial'],'reference_geometry':'existing_new_car_126mm',
        'original_rectangle_observer_audited':True,'target_static_checks':checks,
        'target_static_pass':bool(summary['evidence_valid'] and all(checks.values())),
        'action_status':summary['action_status'],'recoveries':summary['recoveries'],
        'final_xy_error_m':summary['final_xy_error_m'],'final_yaw_error_rad':summary['final_yaw_error_rad'],
        'cross_track_rms_m':summary['cross_track_rms_m'],'preflight':summary['preflight'],
        'target_geometry_metrics':target,'old_car_rectangle_physical_oracle':old_car,
        'yaw_metrics':yaw_metrics(path),'endpoint_rejections':wa,
        'nominal_after_navigation':endpoint(raw,m['body_polygon_m'],.03,(4.3,0.,0.)),
        'actual_final_after_navigation':endpoint(raw,m['body_polygon_m'],.03,summary['final_pose']),
        'planner_success_diagnostic_count':len(successes),'published_plan_messages':summary['plan_messages'],
        'planner_success_diagnostics_complete':len(successes)==summary['plan_messages'],
        'qp_adopted_debug_messages':logs.count('validated minimum-jerk polyline'),
        'validated_astar_fallback_debug_messages':logs.count('validated A* fallback:'),
        'snapshot_rejection_observed':summary['snapshot_rejection_observed'],
        'source_commit':original['source_commit'],'image_id':original['image_id'],
        'profile_sha256':original['profile_sha256'],'accepted_for_deployment':False,
        'artifacts_sha256':{str(p.relative_to(path)):digest(p.read_bytes()) for p in sorted(path.rglob('*')) if p.is_file() and 'ros' not in p.relative_to(path).parts}}
    write_new(path/'target_geometry_summary.json',report)
    return report


def run(name,trial):
    m=verify()
    if name not in PLANNERS or trial not in range(1,6):raise ValueError('invalid planner/trial')
    for n in range(1,trial):
        prior=json.loads((ROOT/f'{name}_{n}'/'target_geometry_summary.json').read_text())
        if not prior['target_static_pass']:raise ValueError('first failed group stays stopped')
    if subprocess.check_output(['git','status','--porcelain','--','experiments/tdt_planner/rm_tdt_planner','src/rm_simulation','src/rm_nav_config','src/rm_description'],cwd=REPO):
        raise ValueError('relevant runtime sources dirty')
    target=ROOT/f'{name}_{trial}';target.mkdir(exist_ok=False)
    (target/'profile.yaml').write_bytes((ROOT/'profiles'/f'{name}.yaml').read_bytes())
    image=subprocess.check_output(['docker','image','inspect','rm2027_navigation:humble','--format','{{.Id}}'],text=True).strip()
    write_new(target/'metadata.json',{'planner':name,'trial':trial,'source_commit':m['source_commit'],
        'image_id':image,'profile_sha256':m['profiles'][name],'fixture_sha256':m['fixture_sha256'],
        'scope':'Reference new-car algorithm footprint on unchanged surrogate simulation; original observer uses legacy rectangle. See target_geometry_summary.json.',
        'performance_is_not_algorithm_rejection':True})
    relative='/work/'+str(target.relative_to(WORK))
    args=['docker','run','--rm','--init','--network','none','--user',f'{os.getuid()}:{os.getgid()}',
        '--entrypoint','bash','-v',f'{REPO}:/ws:ro','-v',f'{WORK}:/work','--cidfile',str(target/'container_id')]
    for value in ('ROS_DOMAIN_ID=174','ROS_LOCALHOST_ONLY=1','PYTHONDONTWRITEBYTECODE=1','TMPDIR=/work/tmp',
        'LIBGL_ALWAYS_SOFTWARE=true','QT_QPA_PLATFORM=offscreen','TDT_HEADING_AB=1',f'IGN_PARTITION=tdt_new_geometry_{name}_{trial}'):
        args+=['-e',value]
    args+=['rm2027_navigation:humble','/ws/'+str(HERE.relative_to(REPO))+'/run_reference_trial.sh',relative,relative+'/profile.yaml']
    status=subprocess.call(args);(target/'docker_exit.txt').write_text(str(status)+'\n')
    report=analyze_trial(target)
    print(json.dumps({k:report[k] for k in ('planner','trial','target_static_pass','action_status','recoveries','final_xy_error_m','final_yaw_error_rad','qp_adopted_debug_messages','validated_astar_fallback_debug_messages')}),flush=True)
    return 0 if report['target_static_pass'] else 1

if __name__=='__main__':
    if sys.argv[1]=='prepare':prepare()
    elif sys.argv[1]=='run':sys.exit(run(sys.argv[2],int(sys.argv[3])))
    else:raise ValueError('expected prepare or run')
