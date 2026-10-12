"""Bounded new-car software entry; every active mode is explicitly no-hardware."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    mode=LaunchConfiguration('mode').perform(context)
    enabled=LaunchConfiguration('enable_test').perform(context).lower()=='true'
    if mode=='inventory':
        return [LogInfo(msg='New-car SOFTWARE inventory: no nodes. bundle/AUD-001 integrated; motion authority Modify; real calibration/profile/MCU stop pending.')]
    if not enabled:raise RuntimeError('active software mode requires enable_test:=true; no hardware mode is available')
    def include(package,name,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package))/'launch'/name)),launch_arguments=args.items())
    if mode=='bundle_no_input':
        if os.environ.get('ROS_DISTRO')!='humble':raise RuntimeError('fixed FAST-LIO build/runtime is Humble')
        return [LogInfo(msg='NO HARDWARE: bundle initialization wait only; no driver, adapter/TF owner, Nav2, serial or fake valid source.'),
            include('rm_lio_bringup','fast_lio_multi_phase2a.launch.py',{'use_backend':'true','update_method':'bundle','sensor_mode':'single','selected_side':'left','publish_tf_results':'false','scan_publish_en':'false'})]
    if mode=='v2_protocol':
        if os.environ.get('ROS_DISTRO')!='humble':raise RuntimeError('fixed V2 protocol test environment is Humble')
        return [LogInfo(msg='ISOLATED CODEC TEST: dry-run and MCU fixture; no Nav2/Goal chain, no task authorization, no physical IO. Never combine with navigation.'),
            include('rm_serial_driver','competition_v2_no_hardware_test.launch.py',{'publish_operator_target':'false'})]
    if mode=='s2':
        if os.environ.get('ROS_DISTRO')!='jazzy':raise RuntimeError('S2 uses existing Jazzy/Gazebo8; never mix with Humble localization/serial graph')
        return [LogInfo(msg='S2 SIMULATION ORACLE: provisional geometry and physics; not hardware calibration or localization quality. No serial/Mission.'),
            include('rm_new_car_simulation','baseline.launch.py',{'field':'flat','sensors':'false','headless':'true','use_nav2':'true','publish_truth_tf':'true','spawn':'-3,-3,.002,0,0,0','output':LaunchConfiguration('output').perform(context)})]
    raise RuntimeError('unsupported software mode; physical motion is unavailable')


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='inventory',choices=['inventory','bundle_no_input','v2_protocol','s2']),
        DeclareLaunchArgument('enable_test',default_value='false'),DeclareLaunchArgument('output',default_value='/tmp/rm2027_newcar_software_s2'),OpaqueFunction(function=setup)])
