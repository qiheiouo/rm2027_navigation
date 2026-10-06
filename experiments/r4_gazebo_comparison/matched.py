"""A24 host-side protocol only; reuse the A23 binary, scenes and observers."""
import json,math,pathlib,shutil,subprocess,sys
import yaml
root=pathlib.Path(__file__).resolve().parents[2]
command=sys.argv[1];out=pathlib.Path(sys.argv[2]).resolve()
def profile(speed=None):
    config=yaml.safe_load((out/'assets/common_nav2.yaml').read_text())
    radius=math.hypot(.32,.27)+.02
    # Circumscribed regular polygon: every support contains the frozen R4 circle.
    outer=radius/math.cos(math.pi/32)
    polygon=[[outer*math.cos(2*math.pi*i/32),outer*math.sin(2*math.pi*i/32)] for i in range(32)]
    for scope in ('local_costmap','global_costmap'):
        cm=config[scope][scope]['ros__parameters']
        cm.update(footprint=json.dumps(polygon),footprint_padding=0.)
    config['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['inflation_radius']=.5
    if speed is not None: config['controller_server']['ros__parameters']['FollowPath']['vx_max']=speed
    return config
def write_profile(name,speed):
    for mode in ('B0','R4'):
        config=profile(speed if mode=='B0' else None)
        (out/f'assets/{name}_{mode}_nav2.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
def run(spec,name):
    scene,mode,repeat=spec.split(':')
    directory=out/'runs'/f'{scene}_{mode}_{int(repeat):02d}'
    if directory.exists():
        if not (directory/'events.json').exists(): raise RuntimeError(f'incomplete preserved run: {spec}')
    else:
        with (out/f'{spec.replace(":","_")}_runner.log').open('w') as log:
            subprocess.run(['bash',str(root/'experiments/r4_gazebo_comparison/run.sh'),str(out),spec,name],cwd=root,stdout=log,stderr=subprocess.STDOUT,check=False)
    events=json.loads((directory/'events.json').read_text())
    print(spec,events['finished'],'status',events['result_status'],'reason',events['controller_failure'],flush=True)
    if events['goal_ns'] is None: raise RuntimeError('startup failure preserved; classify before replacement')
    return events
if command=='prepare':
    previous=root/'build/r4_finite_comparison_20261006'
    out.mkdir(exist_ok=False)
    for name in ('assets','research_install'): shutil.copytree(previous/name,out/name,symlinks=True)
    write_profile('matched',None)
    plan=dict(algorithm_commit='ccd3eac4',harness_reference='c3257026',hypothesis='A23 safety gain survives matched empty arrival and circular support',counts={'S0':3,'S1':5,'S2':5},circle_radius=math.hypot(.32,.27)+.02,native_circle_vertices=32,native_circle_outer_radius=(math.hypot(.32,.27)+.02)/math.cos(math.pi/32),local_inflation_radius=.5,calibration_only_scene='S0',calibration_max_candidates=3,empty_arrival_relative_tolerance=.05,finite_speed_frozen_before_dynamic=True,algorithm_changed=False)
    (out/'protocol.json').write_text(json.dumps(plan,indent=2)+'\n')
elif command=='calibrate':
    index=int(sys.argv[3]);speed=float(sys.argv[4]);assert 1<=index<=3 and .1<=speed<=.5
    name=f'calibration_{index}';write_profile(name,speed)
    run(f'S0:B0:{index}',name)
elif command=='reference':
    run('S0:R4:101','matched')
elif command=='freeze':
    index=int(sys.argv[3]);candidate=out/'runs'/f'S0_B0_{index:02d}'
    b=json.loads((candidate/'events.json').read_text());r=json.loads((out/'runs/S0_R4_101/events.json').read_text())
    assert b['result_status']==4 and r['result_status']==4 and not b['contacts'] and not r['contacts']
    bt=(b['end_ns']-b['goal_ns'])/1e9;rt=(r['end_ns']-r['goal_ns'])/1e9
    assert abs(bt/rt-1)<=.05,(bt,rt)
    config=yaml.safe_load((candidate/'nav2.yaml').read_text());speed=config['controller_server']['ros__parameters']['FollowPath']['vx_max']
    write_profile('matched',speed)
    (out/'freeze.json').write_text(json.dumps(dict(candidate=candidate.name,vx_max=speed,calibration_arrival_s=bt,reference_arrival_s=rt,relative_difference=bt/rt-1),indent=2)+'\n')
elif command=='batch':
    assert (out/'freeze.json').exists(),'freeze on S0 evidence before dynamic runs'
    replacements=json.loads((out/'replacement_runs.json').read_text()) if (out/'replacement_runs.json').exists() else {}
    schedule=[f'{scene}:{mode}:{repeat}' for scene,count in [('S0',3),('S1',5),('S2',5)] for repeat in range(101,101+count) for mode in (('B0','R4') if repeat%2 else ('R4','B0'))]
    schedule=[replacements.get(spec,spec) for spec in schedule]
    (out/'schedule.json').write_text(json.dumps(schedule,indent=2)+'\n')
    for spec in schedule: run(spec,'matched')
    print('matched finite schedule complete',flush=True)
else: raise ValueError(command)
