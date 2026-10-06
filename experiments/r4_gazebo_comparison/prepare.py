"""Only Gazebo assets and a common derived baseline configuration."""
import copy,json,pathlib,subprocess,sys,xml.etree.ElementTree as ET
import yaml
root,out=map(pathlib.Path,sys.argv[1:3]);out.mkdir(parents=True,exist_ok=True)
subprocess.run([sys.executable,str(root/'experiments/r4_runtime_shadow/prepare.py'),str(root),str(out)],check=True)
ET.register_namespace('ignition','http://ignitionrobotics.org/schema')
for scene in ('S0','S1','S2'):
 tree=ET.parse(out/f'{scene}.sdf');world=tree.getroot().find('world')
 for name in ('rm_sentry_2027','moving_obstacle'):
  model=world.find(f"model[@name='{name}']")
  if model is None: continue
  plugin=ET.SubElement(model,'plugin',filename='ignition-gazebo-pose-publisher-system',name='gz::sim::systems::PosePublisher')
  for key,val in dict(publish_link_pose='true',publish_model_pose='true',publish_collision_pose='false',publish_visual_pose='false',publish_nested_model_pose='true',use_pose_vector_msg='true',update_frequency='50').items(): ET.SubElement(plugin,key).text=val
  if name=='moving_obstacle':
   sensor=ET.SubElement(model.find("link[@name='obstacle_link']"),'sensor',name='actor_contact',type='contact');ET.SubElement(sensor,'update_rate').text='100'
   contact=ET.SubElement(sensor,'contact');ET.SubElement(contact,'collision').text='obstacle_collision';ET.SubElement(contact,'topic').text='/simulation/oracle/contacts';ET.SubElement(sensor,'always_on').text='1'
   ET.SubElement(world,'plugin',filename='ignition-gazebo-contact-system',name='gz::sim::systems::Contact')
 tree.write(out/f'{scene}.sdf',encoding='utf-8',xml_declaration=True)
# Baseline configuration from main, never the old obstacle-layer shadow profile.
config=yaml.safe_load(subprocess.check_output(['git','-C',str(root),'show','main:src/rm_nav_config/config/nav2_old_car_2026_left_stvl.yaml']))
def simtime(x):
 if isinstance(x,dict):
  for k,v in x.items():
   if k=='use_sim_time': x[k]=True
   else: simtime(v)
simtime(config);ctrl=config['controller_server']['ros__parameters'];ctrl['controller_frequency']=20.
ctrl['FollowPath']['plugin']='r4_gazebo_comparison/Controller'
stvl=config['local_costmap']['local_costmap']['ros__parameters']['stvl_layer']
for key in ('left_mid360_mark','left_mid360_clear'):
 stvl[key]['topic']='/scan';stvl[key]['data_type']='LaserScan'
stvl['left_mid360_mark'].update(obstacle_min_range=.12)
stvl['left_mid360_clear'].update(horizontal_fov_angle=6.28318530718,vertical_fov_angle=.1,vertical_fov_offset=0.)
(out/'common_nav2.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
bridge=yaml.safe_load((root/'src/rm_simulation/config/ros_gz_bridge.yaml').read_text())+yaml.safe_load((root/'src/rm_simulation/config/course_dynamic_bridge.yaml').read_text())
for model in ('rm_sentry_2027','moving_obstacle'):
 bridge.append(dict(ros_topic_name=f'/simulation/oracle/{model}',gz_topic_name=f'/model/{model}/pose',ros_type_name='tf2_msgs/msg/TFMessage',gz_type_name='ignition.msgs.Pose_V',direction='GZ_TO_ROS'))
bridge.append(dict(ros_topic_name='/simulation/oracle/contacts',gz_topic_name='/simulation/oracle/contacts',ros_type_name='ros_gz_interfaces/msg/Contacts',gz_type_name='ignition.msgs.Contacts',direction='GZ_TO_ROS'))
(out/'bridge.yaml').write_text(yaml.safe_dump(bridge))
(out/'comparison.json').write_text(json.dumps(dict(baseline_commit='ccd3eac4',baseline_profile='main:nav2_old_car_2026_left_stvl.yaml',goal=[4,0,0],counts={'S0':5,'S1':10,'S2':10},cap_s=30,controller_hz=20,body_half_extent=[.32,.27],padding=.02,cruise=.4,progress_cap=.5,algorithm_changed=False,oracle_reuse='frozen R3 physical_projection/polygon_distance only; no control chain',mppi_noise_seed=None),indent=2)+'\n')
