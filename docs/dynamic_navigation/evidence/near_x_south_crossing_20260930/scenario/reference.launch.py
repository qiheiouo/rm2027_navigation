"""Reference geometry only; reuse the original world and controller configuration."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        # Existing plugin reports adopted QP versus validated fallback at DEBUG.
        SetLaunchConfiguration('log_level','planner_server:=debug'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            '/work/dynamic_prediction_near_x_south_crossing_20260930/comparison.launch.py'),
            launch_arguments={'params_file':LaunchConfiguration('params_file')}.items())])
