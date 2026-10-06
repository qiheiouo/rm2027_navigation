"""A24 configuration and empty-field progress comparison, no ROS control."""
import csv,json,math,pathlib,sys
import yaml
from analyze import stats
root=pathlib.Path(sys.argv[1])
def flatten(value,prefix=''):
    if isinstance(value,dict):
        return {key:item for name,child in value.items() for key,item in flatten(child,prefix+'.'+name if prefix else name).items()}
    return {prefix:value}
def differences(a,b):
    a=flatten(a);b=flatten(b)
    return {key:{'before':a.get(key),'after':b.get(key)} for key in sorted(a.keys()|b.keys()) if a.get(key)!=b.get(key)}
configs={mode:yaml.safe_load((root/f'assets/matched_{mode}_nav2.yaml').read_text()) for mode in ('B0','R4')}
common=yaml.safe_load((root/'assets/common_nav2.yaml').read_text())
trial_config_errors=[]
for path in sorted((root/'runs').iterdir()):
    if not (path/'manifest.json').exists(): continue
    m=json.loads((path/'manifest.json').read_text())
    if m['phase']!='finite': continue
    config=yaml.safe_load((path/'nav2.yaml').read_text())
    diff=differences(configs[m['mode']],config)
    extra={k:v for k,v in diff.items() if k not in ('controller_server.ros__parameters.FollowPath.research_mode','controller_server.ros__parameters.FollowPath.research_log')}
    if extra: trial_config_errors.append(dict(run=path.name,unexpected_differences=extra))
def rows(path): return list(csv.DictReader(path.open()))
def empty(path):
    e=json.loads((path/'events.json').read_text());start=e['goal_ns'];end=e['end_ns']
    od=[r for r in rows(path/'odometry.csv') if start<=int(r['source_ns'])<=end]
    return dict(run=path.name,arrival_s=(end-start)/1e9,
        times_to_x_s={str(x):next(((int(r['source_ns'])-start)/1e9 for r in od if float(r['x'])>=x),None) for x in (1.,1.5,2.,3.)},
        mean_world_vx_2_to_8_s=sum(math.cos(float(r['yaw']))*float(r['vx'])-math.sin(float(r['yaw']))*float(r['vy']) for r in od if 2<=(int(r['source_ns'])-start)/1e9<=8)/max(1,sum(2<=(int(r['source_ns'])-start)/1e9<=8 for r in od)))
empty_runs=[empty(p) for p in sorted((root/'runs').iterdir()) if p.name.startswith('S0_') and (p/'events.json').exists() and json.loads((p/'events.json').read_text())['goal_ns'] is not None]
finite={mode:[r for r in empty_runs if r['run'].startswith(f'S0_{mode}_') and int(r['run'].rsplit('_',1)[1])>=100] for mode in ('B0','R4')}
medians={mode:stats([r['arrival_s'] for r in group]) for mode,group in finite.items()}
speed={mode:stats([r['mean_world_vx_2_to_8_s'] for r in group]) for mode,group in finite.items()}
waypoints={mode:{str(x):stats([r['times_to_x_s'][str(x)] for r in group if r['times_to_x_s'][str(x)] is not None]) for x in (1.,1.5,2.,3.)} for mode,group in finite.items()}
relative=medians['B0']['p50']/medians['R4']['p50']-1 if all(medians.values()) else None
groups=json.loads((root/'summary.json').read_text())['groups']
report=dict(protocol=json.loads((root/'protocol.json').read_text()),freeze=json.loads((root/'freeze.json').read_text()),empty_runs=empty_runs,finite_empty_arrival_s=medians,finite_empty_world_vx_2_to_8_s=speed,finite_empty_waypoint_s=waypoints,finite_empty_arrival_relative_difference=relative,finite_empty_match_pass=abs(relative)<=.05 if relative is not None else False,
    semantic_config_differences={mode:differences(common,c) for mode,c in configs.items()},between_group_config_differences=differences(configs['B0'],configs['R4']),trial_config_errors=trial_config_errors,
    native_geometry='32-gon inradius equals R4 circle; extra support <=2.123mm; both groups local inflation .50m',
    config_differences='A23->A24 common local/global footprint, padding and local inflation. A24 B0 vs R4: only research mode/log and native vx_max; frozen R4 limits remain original.',
    invalid_trials=[r for r in json.loads((root/'summary.json').read_text())['runs'] if r['startup']],
    geometry_pose_sampling_limits='Nav2 5cm grid and polygon border checks are not an exact continuous circle collision certificate.')
(root/'matched_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'empty_match':relative,'groups':[{k:g[k] for k in ('scene','mode','n','success','contacts','arrival_s','min_clearance_m','forward_reversals','maximum_backtrack_m')} for g in groups]},indent=2))
