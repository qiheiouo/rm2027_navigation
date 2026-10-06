"""Gazebo-only composition. Reuse existing Nav2 lifecycle and unique output."""
import pathlib
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory as share

def start(context):
 if LaunchConfiguration('enabled').perform(context)!='true': return []
 out=pathlib.Path(LaunchConfiguration('output').perform(context));scene=LaunchConfiguration('scene').perform(context);seed=LaunchConfiguration('seed').perform(context)
 assets=out.parent.parent/'assets';params=str(out/'nav2.yaml')
 configured=ParameterFile(RewrittenYaml(source_file=params,root_key='',param_rewrites={'use_sim_time':'true','autostart':'true'},convert_types=True),allow_substs=True)
 def include(package,file,**kwargs):
  return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pathlib.Path(share(package))/'launch'/file)),launch_arguments={k:str(v) for k,v in kwargs.items()}.items())
 return [ExecuteProcess(cmd=['ign','gazebo','-r','-s','--headless-rendering',str(assets/f'{scene}.sdf')],output='screen'),
  Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(assets/'bridge.yaml')}]),
  Node(package='rm_simulation',executable='scan_frame_adapter',parameters=[dict(use_sim_time=True,input_topic='/simulation/scan_raw',output_topic='/scan',output_frame='sim_lidar_link')]),
  include('rm_description','description.launch.py',use_sim_time='true',use_sim_lidar='true'),
  include('rm_localization_adapters','localization_adapters.launch.py',use_sim_time='true',raw_odom_topic='/simulation/ground_truth/odom'),
  Node(package='rm_chassis_interface',executable='chassis_interface_stub',parameters=[dict(use_sim_time=True,cmd_vel_topic='/cmd_vel',mock_output_cmd_vel_topic='/simulation/chassis/cmd_vel')]),
  Node(package='rclcpp_components',executable='component_container_isolated',name='nav2_container',parameters=[configured,{'autostart':True}],remappings=[('/tf','tf'),('/tf_static','tf_static')],output='screen'),
  include('nav2_bringup','navigation_launch.py',use_sim_time='true',params_file=params,autostart='true',use_composition='True'),
  Node(package='nav2_map_server',executable='map_server',name='research_static_map',parameters=[dict(use_sim_time=True,yaml_filename=str(assets/'map.yaml'))]),
  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='research_map_lifecycle',parameters=[dict(use_sim_time=True,autostart=True,node_names=['research_static_map'])]),
  Node(package='rm_dynamic_obstacle_tracking',executable='dynamic_obstacle_tracker_node',name='dynamic_obstacle_tracker_shadow',parameters=[str(pathlib.Path(share('rm_dynamic_obstacle_tracking'))/'config/dynamic_obstacle_tracking_shadow.yaml'),{'use_sim_time':True,'scan_topic':'/scan','prediction.anchor_mode':'last_observation_cv','prediction.velocity_decay_tau':0.,'prediction.max_speed':0.,'observed_members.enabled':True}])]
def generate_launch_description():
 return LaunchDescription([DeclareLaunchArgument('enabled',default_value='false'),DeclareLaunchArgument('scene'),DeclareLaunchArgument('seed'),DeclareLaunchArgument('output'),OpaqueFunction(function=start)])
