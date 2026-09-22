"""Reuse Phase 1.5D moving fixture; no new robot or controller configuration."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, LogInfo, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    moving=Node(package='ros_gz_sim',executable='create',name='spawn_moving_obstacle',output='screen',
        arguments=['-world','phase1_omni','-file','/ws/src/rm_simulation/models/moving_obstacle.sdf',
                   '-name','moving_obstacle','-x','4.9','-y','0.0','-z','0.0'])
    def finished(event,context):
        result=[LogInfo(msg=f'[TDT_DYNAMIC] spawn_moving_obstacle exit={event.returncode}')]
        if event.returncode:result.append(EmitEvent(event=Shutdown(reason='dynamic fixture spawn failed')))
        return result
    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource('/ws/docs/tdt_migration/evidence/snapshot_revalidation_20260922/reference.launch.py'),
            launch_arguments={'params_file':LaunchConfiguration('params_file')}.items()),
        RegisterEventHandler(OnProcessExit(target_action=moving,on_exit=finished)),
        TimerAction(period=3.,actions=[moving,Node(package='ros_gz_bridge',executable='parameter_bridge',name='course_dynamic_bridge',output='screen',
            parameters=[{'config_file':'/ws/src/rm_simulation/config/course_dynamic_bridge.yaml'}])]),
        TimerAction(period=4.,actions=[Node(package='rm_simulation',executable='moving_obstacle_controller',name='moving_obstacle_controller',output='screen',
            parameters=[{'use_sim_time':True,'amplitude':.9,'period':8.0}])])])
