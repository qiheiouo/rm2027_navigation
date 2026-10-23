"""Laptop-only alternative global backend; existing bridge owns public TF."""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    backend_launch = os.path.join(share('backend'), 'launch', 'backend.launch.py')
    spec = importlib.util.spec_from_file_location('pinned_zlocalization_launch', backend_launch)
    upstream = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(upstream)
    parameters = upstream.load_params(os.path.join(share('backend'), 'config', 'backend.yaml'))
    upstream.merge(parameters, upstream.load_params(value('config_file')))
    mode = value('mode')
    if mode not in ('localization', 'mapping'):
        raise ValueError('mode must be localization or mapping (odometry-only smoke)')
    parameters['prior_map']['enabled'] = mode == 'localization'
    if mode == 'localization':
        path = value('prior_map') or parameters['prior_map']['directory']
        if not path or not os.path.isdir(path):
            raise ValueError('localization requires prior_map:=/absolute/map_bundle')
        parameters['prior_map']['directory'] = path
    sim = upstream.launch_bool(value('use_sim_time'))
    actions = [
        Node(package='backend', executable='backend_node', name='zlocalization_backend',
             parameters=[parameters, {'use_sim_time': sim}], output='screen',
             remappings=[('/tf', '/zlocalization/_quarantine/tf'),
                         ('/tf_static', '/zlocalization/_quarantine/tf_static'),
                         ('~/localization_pose', '/zlocalization/sensor_pose'),
                         ('~/localization_state', '/zlocalization/state')]),
        Node(package='rm_zlocalization_candidate', executable='candidate_adapter.py',
             parameters=[{'use_sim_time': sim}], output='screen'),
        Node(package='rm_relocalization_bridge', executable='map_odom_from_global_pose',
             parameters=[{'use_sim_time': sim,
                          'upstream_valid_topic': '/localization/zlocalization_backend_valid',
                          'max_pose_odom_dt_sec': .001,
                          'max_global_pose_age_sec': .5}], output='screen'),
    ]
    if upstream.launch_bool(value('start_lio_adapter')):
        actions.append(Node(package='rm_localization_adapters', executable='lio_adapter',
            parameters=[os.path.join(share('rm_localization_adapters'), 'config', 'lio_adapter.yaml'),
                {'use_sim_time': sim, 'backend_child_frame_alias_enabled': True,
                 'raw_odom_parent_frame_mode': 'sensor_initial',
                 'sensor_initial_reference_mode': 'first_source_sample',
                 'twist_mode': 'finite_difference'}], output='screen'))
    if upstream.launch_bool(value('start_fastlio')):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share('rm_lio_bringup'), 'launch', 'fast_lio_multi_phase2a.launch.py')),
            launch_arguments={'use_backend': 'true', 'scan_publish_en': 'true',
                'zero_start_pose': 'false', 'update_method': 'bundle',
                'use_sim_time': value('use_sim_time')}.items()))
    return actions


def generate_launch_description():
    defaults = {
        'config_file': os.path.join(share('rm_zlocalization_candidate'), 'config', 'z_localization_candidate.yaml'),
        'prior_map': '', 'mode': 'localization', 'use_sim_time': 'false',
        'start_lio_adapter': 'true', 'start_fastlio': 'false',
    }
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=setup)])
