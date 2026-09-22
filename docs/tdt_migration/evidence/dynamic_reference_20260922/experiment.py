"""Frozen first-stage A*/QP moving-fixture pilot; never overwrite a trial."""
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
WORK=REPO/'build/tdt_p2b'
RUNS=WORK/'runs/dynamic_reference_pilot_v1'
sys.path.insert(0,str(HERE.parent/'snapshot_revalidation_20260922'))
from reference_experiment import inspect_trial, read_rows, yaw_metrics
from dynamic_metrics import rows_from_transport, obstacle_polygon, geometry_metrics


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def write_new(p,data):
    with p.open('x') as f:json.dump(data,f,indent=2,allow_nan=False);f.write('\n')

def prepare():
    RUNS.mkdir(exist_ok=False);(RUNS/'profiles').mkdir()
    base=WORK/'runs/snapshot_revalidation_v1';inputs=json.loads((base/'manifest.json').read_text())
    for name in ('tdt_astar','tdt_qp'):
        (RUNS/'profiles'/f'{name}.yaml').write_bytes((base/'profiles'/f'{name}.yaml').read_bytes())
    files=[p for p in HERE.iterdir() if p.suffix in ('.py','.sh')]
    files+=[REPO/f for f in inputs['fixture_sha256']]
    files+=[REPO/f for f in ('src/rm_simulation/models/moving_obstacle.sdf','src/rm_simulation/src/moving_obstacle_controller.cpp','src/rm_simulation/config/course_dynamic_bridge.yaml')]
    write_new(RUNS/'inputs.json',{'schema':'rm_tdt_planner/dynamic_reference_pilot_inputs/v1',
        'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        'runtime_source_commit':'a419654a8fed1fc8321c234fb212abd0a6cabe04',
        'profiles':inputs['profiles'],'body_polygon_m':inputs['body_polygon_m'],
        'files':{str(p.relative_to(REPO)):sha(p) for p in files},
        'goal':[5.6,0.,0.],'obstacle':'Unchanged Phase1.5D 0.45 x 0.55 m box at model x=4.9; joint amplitude .9 m, period 8 s; actual link motion is independently recorded.',
        'pilot_trials':[['tdt_astar',1,0.],['tdt_qp',1,0.]],
        'phase_policy':'Prespecified phase zero after >=16 simulated seconds; same for both. No choice based on successful opening. Pilot only, not ten varied-phase acceptance trials.',
        'gates':'Existing .05 m body and padded no-contact, .15m XY/.20rad yaw, stopped command, record recovery; no relaxation.',
        'scope':'Reference target polygon on surrogate carrier. No controller/planner tuning, no hardware, no full dynamic acceptance.'})

def verify():
    m=json.loads((RUNS/'inputs.json').read_text())
    for name,h in m['files'].items():assert sha(REPO/name)==h,name
    for name,h in m['profiles'].items():assert sha(RUNS/'profiles'/f'{name}.yaml')==h,name
    return m

def analyze(target,m):
    raw=inspect_trial(target);s=raw['summary'];assert raw['raw_summary_matches']
    poses=rows_from_transport(target/'gazebo_poses.jsonl')
    events=read_rows(target/'observation/events.jsonl')
    start=next(e['t'] for e in events if e.get('event')=='preflight')
    end=next(e['t'] for e in events if e.get('event')=='navigation_result')
    times=[p['t'] for p in poses];lo=max(0,bisect.bisect_right(times,start)-1);hi=bisect.bisect_left(times,end)
    covered=times[0]<=start and times[-1]>=end
    rows=poses[lo:hi+1];max_gap=max(b['t']-a['t'] for a,b in zip(rows,rows[1:]))
    box=obstacle_polygon()
    models={'new_car_reference':(m['body_polygon_m'],.03),
            'old_car_physical_rectangle':([(-.32,-.27),(.32,-.27),(.32,.27),(-.32,.27)],.02),
            'surrogate_body_rectangle':([(-.3,-.25),(.3,-.25),(.3,.25),(-.3,.25)],.03)}
    geom={name:geometry_metrics(rows,*args,box) for name,args in models.items()}
    # Verify the independent Gazebo model/link composition against ROS ground truth.
    differences=[]
    for r in read_rows(target/'observation/trajectory.jsonl'):
        i=bisect.bisect_right(times,r['t']);a,b=poses[i-1],poses[i];f=(r['t']-a['t'])/(b['t']-a['t'])
        x=a['robot'][0]+f*(b['robot'][0]-a['robot'][0]);y=a['robot'][1]+f*(b['robot'][1]-a['robot'][1])
        differences.append(math.hypot(x-r['x'],y-r['y']))
    continuity={'gazebo_covers_navigation':covered,'max_gazebo_gap_s':max_gap,
        'max_gazebo_ros_position_difference_m':max(differences),
        'max_scan_gap_sim_s':s['max_scan_gap_sim_s'],'max_odom_gap_sim_s':s['max_odom_gap_sim_s'],
        'obstacle_y_range_m':[min(r['obstacle'][1] for r in rows),max(r['obstacle'][1] for r in rows)]}
    evidence=bool(s['evidence_valid'] and covered and max_gap<=.20 and max(differences)<.02 and continuity['obstacle_y_range_m'][1]-continuity['obstacle_y_range_m'][0]>.5)
    checks=dict(s['checks']);checks.update({k:geom['new_car_reference'][k] for k in ('body_clearance_at_least_005m','padded_footprint_no_contact')})
    diagnostics=read_rows(target/'observation/planner_diagnostics.jsonl')
    admissions=[json.loads(e['message'].split('snapshot_admission_v1=',1)[1]) for e in diagnostics if e['message'].startswith('snapshot_admission_v1=')]
    log='\n'.join(e['message'] for e in diagnostics)
    result={'schema':'rm_tdt_planner/dynamic_reference_pilot/v1','planner':s.get('planner',target.name.rsplit('_',1)[0]),
        'evidence_valid':evidence,'original_static_summary_matches':True,'limited_dynamic_geometry_and_goal_pass':bool(evidence and all(checks.values())),
        'checks':checks,'action_status':s['action_status'],'recoveries':s['recoveries'],
        'final_xy_error_m':s['final_xy_error_m'],'final_yaw_error_rad':s['final_yaw_error_rad'],
        'cross_track_rms_m':s['cross_track_rms_m'],'preflight':s['preflight'],'navigation_interval_sim_s':[start,end],
        'geometry':geom,'continuity':continuity,'yaw_metrics':yaw_metrics(target),
        'snapshot_admissions':admissions,'planner_failures':[e for e in diagnostics if 'failed to plan' in e['message']],
        'qp_adopted':log.count('validated minimum-jerk polyline'),'validated_astar_fallbacks':log.count('validated A* fallback:'),
        'full_dynamic_gate_pass':False,'accepted_for_deployment':False,
        'uncovered':['ten clean varied-phase runs per planner','complete marking/clearing and TF ownership audit','full moving-vehicle physical envelope','high speed Spin','hardware'],
        'scope':'Actual synchronized Gazebo model/link planar poses, convex polygon distance, linear interpolation of both bodies. Static summary alone is not dynamic safety.',
        'artifacts_sha256':{str(p.relative_to(target)):sha(p) for p in target.rglob('*') if p.is_file() and 'ros' not in p.relative_to(target).parts}}
    write_new(target/'dynamic_summary.json',result)
    return result

def run(name):
    m=verify();assert name in ('tdt_astar','tdt_qp')
    assert not subprocess.check_output(['git','status','--porcelain','--','experiments','src'],text=True)
    target=RUNS/f'{name}_1';target.mkdir(exist_ok=False);(target/'profile.yaml').write_bytes((RUNS/'profiles'/f'{name}.yaml').read_bytes())
    image=subprocess.check_output(['docker','image','inspect','rm2027_navigation:humble','--format','{{.Id}}'],text=True).strip()
    write_new(target/'metadata.json',{'planner':name,'trial':1,'source_commit':m['source_commit'],'runtime_source_commit':m['runtime_source_commit'],
        'image_id':image,'profile_sha256':m['profiles'][name],'fixture_sha256':{n:h for n,h in m['files'].items() if n.startswith('src/') or n.endswith('simulation_comparison.launch.py')},
        'scope':'Dynamic reference pilot, Phase1.5D goal (5.6,0,0), no hardware','performance_is_not_algorithm_rejection':True})
    out='/work/'+str(target.relative_to(WORK));args=['docker','run','--rm','--init','--network','none','--user',f'{os.getuid()}:{os.getgid()}',
        '--entrypoint','bash','-v',f'{REPO}:/ws:ro','-v',f'{WORK}:/work','--cidfile',str(target/'container_id')]
    for e in ['ROS_DOMAIN_ID=174','ROS_LOCALHOST_ONLY=1','PYTHONDONTWRITEBYTECODE=1','TMPDIR=/work/tmp','LIBGL_ALWAYS_SOFTWARE=true','QT_QPA_PLATFORM=offscreen','TDT_HEADING_AB=1','TDT_PHASE_SECONDS=0',f'IGN_PARTITION=tdt_dynamic_pilot_v1_{name}']:args+=['-e',e]
    args+=['rm2027_navigation:humble','/ws/'+str(HERE.relative_to(REPO))+'/run_dynamic_trial.sh',out,out+'/profile.yaml']
    status=subprocess.call(args);(target/'docker_exit.txt').write_text(str(status)+'\n')
    s=analyze(target,m);print(json.dumps({k:s[k] for k in ['planner','evidence_valid','limited_dynamic_geometry_and_goal_pass','action_status','recoveries','checks','continuity','geometry']},indent=2),flush=True)
    return 0 if s['limited_dynamic_geometry_and_goal_pass'] else 1
if __name__=='__main__':
    if sys.argv[1]=='prepare':prepare()
    elif sys.argv[1]=='run':sys.exit(run(sys.argv[2]))
    elif sys.argv[1]=='analyze':analyze(RUNS/sys.argv[2],verify())
