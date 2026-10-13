"""Scene-only isolation. Native STVL / Navfn / MPPI / Nav2 state handling."""
import copy,json,pathlib,sys,xml.etree.ElementTree as ET
import yaml
root,out=map(pathlib.Path,sys.argv[1:3]);out.mkdir(parents=True,exist_ok=True)
ET.register_namespace('ignition','http://ignitionrobotics.org/schema')
scenes=('empty','crossing','head_on','narrow','wait_release','sudden_stop','stale')
for scene in scenes:
 tree=ET.parse(root/'experiments/stvl_temporal/vendor/phase1_omni.sdf');world=tree.getroot().find('world')
 for model in list(world.findall('model')):
  if model.get('name')=='center_block':world.remove(model)
 if scene!='empty':
  actor=copy.deepcopy(ET.parse(root/'src/rm_simulation/models/moving_obstacle.sdf').getroot().find('model'))
  ET.SubElement(actor,'pose').text='2 0 0 0 0 0' if scene!='head_on' else '3.5 0 0 0 0 0'
  actor.find("joint[@name='slider_joint']/axis/xyz").text='0 1 0' if scene!='head_on' else '1 0 0'
  actor.find("joint[@name='slider_joint']/axis/limit/lower").text='-5.0'
  actor.find("joint[@name='slider_joint']/axis/limit/upper").text='5.0'
  world.append(actor)
 if scene in ('narrow','wait_release','stale'):
  for side in (-1,1):
   model=ET.SubElement(world,'model',name=f'corridor_{side}');ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=f'2 {side*.8} .5 0 0 0'
   link=ET.SubElement(model,'link',name='wall')
   for kind in ('collision','visual'):
    item=ET.SubElement(link,kind,name='wall_'+kind);ET.SubElement(ET.SubElement(ET.SubElement(item,'geometry'),'box'),'size').text='7 .2 1'
 for model in world.findall('model'):
  name=model.get('name')
  if name in ('rm_sentry_2027','moving_obstacle'):
   plugin=ET.SubElement(model,'plugin',filename='ignition-gazebo-pose-publisher-system',name='gz::sim::systems::PosePublisher')
   for key,value in dict(publish_link_pose='true',publish_model_pose='true',publish_collision_pose='false',publish_visual_pose='false',publish_nested_model_pose='true',use_pose_vector_msg='true',update_frequency='50').items():ET.SubElement(plugin,key).text=value
  if name=='moving_obstacle':
   sensor=ET.SubElement(model.find("link[@name='obstacle_link']"),'sensor',name='actor_contact',type='contact');ET.SubElement(sensor,'update_rate').text='100';contact=ET.SubElement(sensor,'contact');ET.SubElement(contact,'collision').text='obstacle_collision';ET.SubElement(contact,'topic').text='/simulation/oracle/contacts';ET.SubElement(sensor,'always_on').text='1';ET.SubElement(world,'plugin',filename='ignition-gazebo-contact-system',name='gz::sim::systems::Contact')
 tree.write(out/f'{scene}.sdf',encoding='utf-8',xml_declaration=True)
 calibration=copy.deepcopy(tree);calibration.getroot().find("world/model[@name='rm_sentry_2027']/pose").text='0 3 0 0 0 0'
 calibration.write(out/f'{scene}_CAL.sdf',encoding='utf-8',xml_declaration=True)
 # Same .05m global static map; corridor wall extents match the physical scene.
 width,height=240,160;pixels=bytearray([254])*(width*height)
 if scene in ('narrow','wait_release','stale'):
  for row in range(height):
   for col in range(width):
    x=-2+(col+.5)*.05;y=-4+(height-row-.5)*.05
    if -1.5<=x<=5.5 and .7<=abs(y)<=.9:pixels[row*width+col]=0
 (out/f'{scene}.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pixels)
 (out/f'{scene}.yaml').write_text(yaml.safe_dump(dict(image=str(out/f'{scene}.pgm'),mode='trinary',resolution=.05,origin=[-2,-4,0],negate=0,occupied_thresh=.65,free_thresh=.25)))
config=yaml.safe_load((root/'experiments/stvl_temporal/old_car_baseline.yaml').read_text())
def simtime(value):
 if isinstance(value,dict):
  for key,v in value.items():
   if key=='use_sim_time':value[key]=True
   else:simtime(v)
simtime(config)
controller=config['controller_server']['ros__parameters'];controller['FollowPath']['plugin']='rm_stvl_temporal_research/Controller'
standard=controller['FollowPath']['critics'];traces=['InputTraceCritic']
for name in standard:traces.extend((name,name.replace('Critic','TraceCritic')))
controller['FollowPath']['critics']=traces
stvl=config['local_costmap']['local_costmap']['ros__parameters']['stvl_layer']
for key in ('left_mid360_mark','left_mid360_clear'):stvl[key].update(topic='/scan',data_type='LaserScan')
stvl['left_mid360_mark']['obstacle_min_range']=.12
stvl['left_mid360_clear'].update(horizontal_fov_angle=6.28318530718,vertical_fov_angle=.1,vertical_fov_offset=0.)
(out/'common_nav2.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
bridge=yaml.safe_load((root/'src/rm_simulation/config/ros_gz_bridge.yaml').read_text())+yaml.safe_load((root/'src/rm_simulation/config/course_dynamic_bridge.yaml').read_text())
for model in ('rm_sentry_2027','moving_obstacle'):bridge.append(dict(ros_topic_name=f'/simulation/oracle/{model}',gz_topic_name=f'/model/{model}/pose',ros_type_name='tf2_msgs/msg/TFMessage',gz_type_name='ignition.msgs.Pose_V',direction='GZ_TO_ROS'))
bridge.append(dict(ros_topic_name='/simulation/oracle/contacts',gz_topic_name='/simulation/oracle/contacts',ros_type_name='ros_gz_interfaces/msg/Contacts',gz_type_name='ignition.msgs.Contacts',direction='GZ_TO_ROS'))
(out/'bridge.yaml').write_text(yaml.safe_dump(bridge))
(out/'scenario.json').write_text(json.dumps({'scenes':scenes,'actor_size':[.45,.55,.8],'source_profile':'4541e375:nav2_old_car_2026_left_stvl.yaml','control_hz':10,'same_controller':'installed native MPPI, observers no cost mutation','phase_seeds':{'13':-.2,'29':0,'47':.2}}))
