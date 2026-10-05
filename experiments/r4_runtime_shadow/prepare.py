#!/usr/bin/env python3
"""Prepare scene-only derivatives; original ROS/physics/algorithm sources frozen."""
import argparse, copy, hashlib, json, pathlib, xml.etree.ElementTree as ET
import yaml

p=argparse.ArgumentParser();p.add_argument('root',type=pathlib.Path);p.add_argument('output',type=pathlib.Path)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
source=a.root/'src/rm_simulation/worlds/phase1_omni.sdf'
for scene in ['S0','S1','S2']:
    tree=ET.parse(source);world=tree.getroot().find('world')
    for model in list(world.findall('model')):
        if model.get('name')=='center_block':world.remove(model)
    if scene!='S0':
        model=copy.deepcopy(ET.parse(a.root/'src/rm_simulation/models/moving_obstacle.sdf').getroot().find('model'))
        ET.SubElement(model,'pose').text='2 0 0 0 0 0';world.append(model)
    tree.write(a.output/f'{scene}.sdf',encoding='utf-8',xml_declaration=True)
# Empty-plane scenario raw map, not an added costmap/occupancy pipeline.
w,h=240,160
(a.output/'empty.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+bytes([254])*(w*h))
(a.output/'map.yaml').write_text(yaml.safe_dump(dict(image='empty.pgm',mode='trinary',resolution=.05,origin=[-2.,-4.,0.],negate=0,occupied_thresh=.65,free_thresh=.25)))
params=yaml.safe_load((a.root/'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml').read_text())
ctl=params['controller_server']['ros__parameters']['FollowPath'];sm=params['velocity_smoother']['ros__parameters'];cm=params['local_costmap']['local_costmap']['ros__parameters']
fp=json.loads(cm['footprint']);flat=[x for point in fp for x in point]
shadow=dict(footprint=flat,padding=cm['footprint_padding'],lower=[ctl['vx_min'],-ctl['vy_max']],upper=[ctl['vx_max'],ctl['vy_max']],rate=[min(sm['max_accel'][i],-sm['max_decel'][i]) for i in range(2)])
(a.output/'shadow_profile.yaml').write_text(yaml.safe_dump({'r4_runtime_shadow':{'ros__parameters':shadow}}))
(a.output/'preregistration.json').write_text(json.dumps(dict(baseline='e137635e',profile='unchanged nav2_phase1_5_mppi.yaml',goal=[4,0,0],ROS_run_seconds=20,wall_cap_seconds=90,shadow_period_ms=50,cruise=.4,progress_cap=.5,wait_speed=.02,forward_speed=.05,reversal_axis_threshold=.02,resume_consecutive_forward=3,scenes={'S0':'empty','S1':'-0.9 at t<=1, cross to +0.9 over t=1..3','S2':'-0.9 to 0 t=1..2, hold through t=7, release to +0.9 through t=9'},source_world_sha256=hashlib.sha256(source.read_bytes()).hexdigest()),indent=2)+'\n')
