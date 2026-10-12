"""One isolated entry for CAD geometry, omni contacts, field/map and dynamic TF."""
import importlib.util
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    root=Path(get_package_share_directory('rm_new_car_simulation'))
    script=Path(get_package_prefix('rm_new_car_simulation'))/'lib/rm_new_car_simulation/baseline_scene.py'
    spec=importlib.util.spec_from_file_location('baseline_scene',script);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    field=LaunchConfiguration('field').perform(context);lidar=LaunchConfiguration('lidar').perform(context)
    profile=LaunchConfiguration('profile').perform(context)
    sensors=LaunchConfiguration('sensors').perform(context)=='true'
    output=Path(LaunchConfiguration('output').perform(context))
    requested_spawn=LaunchConfiguration('spawn').perform(context)
    spawn=([-10,0,.002,0,0,0] if field=='competition2025_reference' else [0,0,.002,0,0,0]) if requested_spawn=='auto' else list(map(float,requested_spawn.split(',')))
    if len(spawn)!=6: raise RuntimeError('spawn requires x,y,z,roll,pitch,yaw in metres and radians')
    manifest=m.generate(root,output,field,lidar,profile,sensors,spawn,
                        LaunchConfiguration('physics_config').perform(context) or None,
                        LaunchConfiguration('tunnel_section').perform(context),
                        LaunchConfiguration('field_tolerance').perform(context),
                        float(LaunchConfiguration('ground_bank_deg').perform(context)))
    bridge=[]
    pairs=[('/model/new_car/world_odometry','/simulation/ground_truth/odom','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS'),
      ('/model/new_car/sensor_world_odometry','/simulation/sensor_truth','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS'),
      ('/model/new_car/joint_state','/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model','GZ_TO_ROS'),
      ('/model/new_car/cmd_vel','/cmd_vel','geometry_msgs/msg/Twist','gz.msgs.Twist','ROS_TO_GZ'),
      ('/model/new_car/gimbal_velocity','/simulation/gimbal_velocity','std_msgs/msg/Float64','gz.msgs.Double','ROS_TO_GZ'),
      ('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock','GZ_TO_ROS')]
    if sensors:
        pairs += [('/model/new_car/lidar/points','/points/simulation_raw','sensor_msgs/msg/PointCloud2','gz.msgs.PointCloudPacked','GZ_TO_ROS')]
    for name in m.NAMES:
        pairs.append((f'/model/new_car/contact/{name}',f'/simulation/contact/{name}',
                      'ros_gz_interfaces/msg/Contacts','gz.msgs.Contacts','GZ_TO_ROS'))
    for gz,ros,rt,gt,d in pairs:
        bridge.append(dict(gz_topic_name=gz,ros_topic_name=ros,ros_type_name=rt,gz_type_name=gt,direction=d))
    (output/'bridge.yaml').write_text(yaml.safe_dump(bridge))
    args=['gz','sim','-r','-v','2',str(output/'world.sdf')]
    if LaunchConfiguration('headless').perform(context)=='true': args+=['-s']
    if sensors: args+=['--headless-rendering']
    env={'GZ_SIM_SYSTEM_PLUGIN_PATH':str(Path(get_package_prefix('rm_new_car_simulation'))/'lib')+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')}
    nodes=[LogInfo(msg=f'field_revision={manifest["revision"]}; CAD provisional; physics contacts; full-pose simulation oracle; no hardware'),
      ExecuteProcess(cmd=args,additional_env=env,output='screen'),
      Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(output/'bridge.yaml'),'use_sim_time':True}],output='screen'),
      Node(package='rm_new_car_simulation',executable='simulation_tf.py',parameters=[{'use_sim_time':True,
        'publish_truth_tf':LaunchConfiguration('publish_truth_tf').perform(context)=='true'}],output='screen'),
      Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(output/'robot.urdf').read_text(),
        'use_sim_time':True,'publish_frequency':100.}],output='screen')]
    if LaunchConfiguration('use_nav2').perform(context)=='true':
        config=Path(get_package_share_directory('rm_nav_config'))/'config/nav2_phase1_5_mppi.yaml'
        data=yaml.safe_load(config.read_text())
        cad,_=m.load_config(root)
        for name in ['local_costmap','global_costmap']:
            cost=data[name][name]['ros__parameters'];cost['footprint']=str(manifest['navigation_footprint_m']);cost['footprint_padding']=0.
            cost['plugins']=['static_layer','inflation_layer']
            cost['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
            cost['global_frame']='map' if name=='global_costmap' else 'odom'
        data['global_costmap']['global_costmap']['ros__parameters']['rolling_window']=False
        for name,block in data.items():
            if 'ros__parameters' in block:
                block['ros__parameters']['use_sim_time']=True
                block['ros__parameters']['enable_stamped_cmd_vel']=False
                if 'odom_topic' in block['ros__parameters']: block['ros__parameters']['odom_topic']='/odometry/simulation'
        data['controller_server']['ros__parameters']['progress_checker_plugins']=['progress_checker']
        data['controller_server']['ros__parameters']['odom_topic']='/odometry/simulation'
        def update_plugins(tree):
            for key,value in tree.items():
                if isinstance(value,dict): update_plugins(value)
                elif key=='plugin' and isinstance(value,str) and value.startswith('nav2_'):
                    tree[key]=value.replace('/','::')
        update_plugins(data)
        data['map_server']={'ros__parameters':{'use_sim_time':True,'yaml_filename':str(output/'field.yaml')}}
        params=output/'nav2.yaml';params.write_text(yaml.safe_dump(data))
        # The physics oracle owns TF; no additional localization source is started.
        names=['map_server','controller_server','planner_server','behavior_server','velocity_smoother','bt_navigator']
        packages=['nav2_map_server','nav2_controller','nav2_planner','nav2_behaviors','nav2_velocity_smoother','nav2_bt_navigator']
        for name,package in zip(names,packages):
            remap=[('odom','/odometry/simulation')]
            if name in ('controller_server','behavior_server'):remap += [('cmd_vel','/cmd_vel_nav')]
            if name=='velocity_smoother':remap += [('cmd_vel','/cmd_vel_nav'),('cmd_vel_smoothed','/cmd_vel')]
            nodes.append(Node(package=package,executable=name,name=name,parameters=[str(params)],
                remappings=remap,output='screen'))
        nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='new_car_navigation_lifecycle',
            parameters=[{'use_sim_time':True,'autostart':True,'node_names':names}],output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
      DeclareLaunchArgument('field',default_value='rules2026_course',choices=['flat','rules2026_course','dog_slope','banked_ground','competition2025_reference']),
      DeclareLaunchArgument('lidar',default_value='A',choices=['A','B']),
      DeclareLaunchArgument('profile',default_value='deformed',choices=['deformed','undeformed']),
      DeclareLaunchArgument('sensors',default_value='false'),
      DeclareLaunchArgument('headless',default_value='true'),
      DeclareLaunchArgument('use_nav2',default_value='false'),
      DeclareLaunchArgument('publish_truth_tf',default_value='true'),
      DeclareLaunchArgument('physics_config',default_value=''),
      DeclareLaunchArgument('tunnel_section',default_value='B',choices=['A','B']),
      DeclareLaunchArgument('field_tolerance',default_value='nominal',choices=['nominal','lower_clearance']),
      DeclareLaunchArgument('ground_bank_deg',default_value='1.5'),
      DeclareLaunchArgument('spawn',default_value='auto'),
      DeclareLaunchArgument('output',default_value='/tmp/rm_new_car_simulation_baseline'),OpaqueFunction(function=setup)])
