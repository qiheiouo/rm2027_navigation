"""Opt-in isolated scene launch using existing Nav2 and chassis responsibility."""
import pathlib
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory as share

def start(context):
    if LaunchConfiguration('enabled').perform(context).lower()!='true':return []
    out=pathlib.Path(LaunchConfiguration('output').perform(context));scene=LaunchConfiguration('scene').perform(context)
    assets=out.parent/'assets';root=pathlib.Path(__file__).resolve().parents[2]
    def include(package,file,**kwargs):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pathlib.Path(share(package))/'launch'/file)),launch_arguments={k:str(v) for k,v in kwargs.items()}.items())
    sim=pathlib.Path(share('rm_simulation'))
    params=str(pathlib.Path(share('rm_nav_config'))/'config/nav2_phase1_5_mppi.yaml')
    configured=ParameterFile(RewrittenYaml(source_file=params,root_key='',param_rewrites={'use_sim_time':'true','autostart':'true'},convert_types=True),allow_substs=True)
    return [
        ExecuteProcess(cmd=['ign','gazebo','-r','-s','--headless-rendering',str(assets/f'{scene}.sdf')],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='simulation_bridge',parameters=[{'config_file':str(sim/'config/ros_gz_bridge.yaml')}]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='course_dynamic_bridge',parameters=[{'config_file':str(sim/'config/course_dynamic_bridge.yaml')}]) if scene!='S0' else ExecuteProcess(cmd=['true']),
        Node(package='rm_simulation',executable='scan_frame_adapter',parameters=[dict(use_sim_time=True,input_topic='/simulation/scan_raw',output_topic='/scan',output_frame='sim_lidar_link')]),
        include('rm_description','description.launch.py',use_sim_time='true',use_sim_lidar='true'),
        include('rm_localization_adapters','localization_adapters.launch.py',use_sim_time='true',raw_odom_topic='/simulation/ground_truth/odom'),
        Node(package='rm_chassis_interface',executable='chassis_interface_stub',parameters=[dict(use_sim_time=True,cmd_vel_topic='/cmd_vel',mock_output_cmd_vel_topic='/simulation/chassis/cmd_vel')]),
        # navigation_launch loads components; its standard parent creates this container.
        Node(package='rclcpp_components',executable='component_container_isolated',name='nav2_container',parameters=[configured,{'autostart':True}],remappings=[('/tf','tf'),('/tf_static','tf_static')],output='screen'),
        include('nav2_bringup','navigation_launch.py',use_sim_time='true',params_file=params,autostart='true',use_composition='True'),
        Node(package='nav2_map_server',executable='map_server',name='shadow_static_map_server',parameters=[dict(use_sim_time=True,yaml_filename=str(assets/'map.yaml'))]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='shadow_map_lifecycle',parameters=[dict(use_sim_time=True,autostart=True,node_names=['shadow_static_map_server'])]),
        Node(package='rm_dynamic_obstacle_tracking',executable='dynamic_obstacle_tracker_node',name='dynamic_obstacle_tracker_shadow',parameters=[str(pathlib.Path(share('rm_dynamic_obstacle_tracking'))/'config/dynamic_obstacle_tracking_shadow.yaml'),{'use_sim_time':True,'scan_topic':'/scan','prediction.anchor_mode':'last_observation_cv','prediction.velocity_decay_tau':0.,'prediction.max_speed':0.,'observed_members.enabled':True}]),
        ExecuteProcess(cmd=[str(out.parent/'caller_build/r4_shadow'),'--ros-args','--params-file',str(assets/'shadow_profile.yaml'),'-p','use_sim_time:=true','-p',f'output_directory:={out}','-p',f'run_id:={scene}'],output='screen'),
    ]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('enabled',default_value='false'),DeclareLaunchArgument('scene',default_value='S0'),DeclareLaunchArgument('output'),OpaqueFunction(function=start)])
