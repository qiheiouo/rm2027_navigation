"""A26 scene preparation and finite host schedule; frozen controller binary."""
import copy,json,pathlib,shutil,subprocess,sys,xml.etree.ElementTree as ET
import yaml
from analyze import trial

root=pathlib.Path(__file__).resolve().parents[2]
command=sys.argv[1];out=pathlib.Path(sys.argv[2]).resolve()

def wall(world,index,bounds):
    x0,x1,y0,y1=bounds
    m=ET.SubElement(world,'model',name=f'corridor_wall_{index}')
    ET.SubElement(m,'static').text='true'
    ET.SubElement(m,'pose').text=f'{(x0+x1)/2} {(y0+y1)/2} .5 0 0 0'
    link=ET.SubElement(m,'link',name='wall_link')
    for tag in ('collision','visual'):
        element=ET.SubElement(link,tag,name=f'wall_{tag}')
        ET.SubElement(ET.SubElement(ET.SubElement(element,'geometry'),'box'),'size').text=f'{x1-x0} {y1-y0} 1'
    sensor=ET.SubElement(link,'sensor',name='wall_contact',type='contact')
    ET.SubElement(sensor,'update_rate').text='100'
    contact=ET.SubElement(sensor,'contact');ET.SubElement(contact,'collision').text='wall_collision'
    ET.SubElement(contact,'topic').text='/simulation/oracle/contacts'
    ET.SubElement(sensor,'always_on').text='1'

def run(scene,mode,repeat,profile='corridor'):
    directory=out/'runs'/f'{scene}_{mode}_{repeat:02d}'
    if not directory.exists():
        with (out/f'{directory.name}_runner.log').open('w') as log:
            subprocess.run(['bash',str(root/'experiments/r4_gazebo_comparison/run.sh'),str(out),f'{scene}:{mode}:{repeat}',profile],cwd=root,stdout=log,stderr=subprocess.STDOUT,check=False)
    if not (directory/'events.json').exists(): raise RuntimeError(f'incomplete evidence preserved: {directory.name}')
    r=trial(directory);print(directory.name,r['finished'],r.get('arrival_s'),r.get('min_dynamic_clearance_m'),flush=True)
    classifications=json.loads((out/'failure_classifications.json').read_text()) if (out/'failure_classifications.json').exists() else {}
    if (r['startup'] or not r['success']) and directory.name not in classifications:
        raise RuntimeError(f'first failure preserved; classify before continuing: {directory.name}')
    return r

if command=='prepare':
    reference=root/'build/r4_matched_comparison_20261006';out.mkdir(exist_ok=False)
    for name in ('assets','research_install'): shutil.copytree(reference/name,out/name,symlinks=True)
    assets=out/'assets'
    config=yaml.safe_load((root/'build/r4_pareto_comparison_20261006/assets/pareto_B0_nav2.yaml').read_text())
    config['controller_server']['ros__parameters']['FollowPath']['vx_max']=.5
    (assets/'corridor_B0_nav2.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    shutil.copy2(reference/'assets/matched_R4_nav2.yaml',assets/'corridor_R4_nav2.yaml')
    walls=[]
    for a,b in ((-1.2,1.5),(2.5,5.2)):
        walls.extend([[a,b,.7,.9],[a,b,-.9,-.7]])
    walls.extend([[-1.2,-1.,-.9,.9],[5.,5.2,-.9,.9]])
    for a,b in ((-2.2,-.9),(.9,2.2)):
        walls.extend([[1.3,1.5,a,b],[2.5,2.7,a,b]])
    walls.extend([[1.3,2.7,-2.2,-2.],[1.3,2.7,2.,2.2]])
    specification=dict(stage='A26 final Research',goal=[4,0,0],corridor_width_m=1.4,gate_x=2.,gate_pocket_x=[1.5,2.5],side_arms_dead_ends=True,walls=walls,motion={s:dict(start_y=-1.4,end_y=1.4,begin_s=1.,speed=v) for s,v in [('S3',.3),('S4',.45)]},controller_receives_schedule=False)
    (assets/'scenario.json').write_text(json.dumps(specification,indent=2)+'\n')
    # The map and physical walls share one geometry; inaccessible outside is occupied.
    width,height=240,160;resolution=.05;ox,oy=-2.,-4.
    raster=bytearray()
    for row in range(height-1,-1,-1):
        y=oy+(row+.5)*resolution
        for col in range(width):
            x=ox+(col+.5)*resolution
            free=(-1.<x<5. and -.7<y<.7) or (1.5<x<2.5 and -2.<y<2.)
            raster.append(254 if free else 0)
    (assets/'empty.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+raster)
    ET.register_namespace('ignition','http://ignitionrobotics.org/schema')
    for scene in ('S0','S3','S4'):
        tree=ET.parse(reference/f"assets/{'S0' if scene=='S0' else 'S2'}.sdf");world=tree.getroot().find('world')
        if scene=='S0': ET.SubElement(world,'plugin',filename='ignition-gazebo-contact-system',name='gz::sim::systems::Contact')
        for index,bounds in enumerate(walls): wall(world,index,bounds)
        if scene!='S0':
            model=world.find("model[@name='moving_obstacle']")
            limit=model.find("joint[@name='slider_joint']/axis/limit")
            limit.find('lower').text='-1.6';limit.find('upper').text='1.6'
        tree.write(assets/f'{scene}.sdf',encoding='utf-8',xml_declaration=True)
    (out/'protocol.json').write_text(json.dumps(dict(stage='A26 last Research',base_commit='89f9035b',algorithm_commit='ccd3eac4',primary_scene='S3',optional_variant='S4 .45m/s only if a repeatable primary Go needs robustness information',n_initial=5,n_max=10,preflight='one empty corridor trial per mode, no parameter tuning',baseline_from='A25 .60m/factor6; only vx_max .32 to .5 for same limits',r4_a24_unchanged=True,shared_speed_limits=True,first_failure='preserve and classify before completing remaining planned samples; never replace a task failure',go_efficiency='R4 median arrival >=10% faster with >=4/5 or 8/10 paired wins, no worse success/contact and no >.05m median or worst dynamic-clearance loss; temporal opening visible in received CV and response',go_safety='R4 all success/zero contact with >=2/5 or 4/10 repeated baseline task failures or contact, without >10% arrival penalty; requires a fair fixture and causal evidence',reactive_challenge='Only on apparent Go: existing A24 radius .50/factor6 baseline at identical limits, five fixed new samples; if it removes advantage, Stop. No search.',extension='Only n5 close (<10% median arrival difference) or inconsistent paired/safety signals that could change Go/Stop; unchanged profiles to n10',stop='No explicit reproducible independent advantage: freeze current low-level prediction-consumption route; no production plumbing',raw_limit_mib=100),indent=2)+'\n')
elif command=='preflight':
    assert not (out/'schedule.json').exists()
    for mode in ('B0','R4'): run('S0',mode,1)
    (out/'preflight_pass.json').write_text(json.dumps(dict(empty_both_pass=True,parameters_changed=False))+'\n')
elif command=='batch':
    assert (out/'preflight_pass.json').exists(),'fair-fixture empty pass required'
    n=int(sys.argv[3]) if len(sys.argv)>3 else 5;assert n in (5,10)
    if n==10:
        extension=json.loads((out/'extension.json').read_text());assert extension['informative'] and not extension['parameters_changed']
    schedule=[f'S3:{mode}:{repeat}' for repeat in range(101,101+n) for mode in (('B0','R4') if repeat%2 else ('R4','B0'))]
    (out/'schedule.json').write_text(json.dumps(schedule,indent=2)+'\n')
    for spec in schedule:
        scene,mode,repeat=spec.split(':');run(scene,mode,int(repeat))
    print('frozen primary comparison complete',flush=True)
else: raise ValueError(command)
