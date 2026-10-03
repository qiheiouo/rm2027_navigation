from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_launch = PathJoinSubstitution([
        FindPackageShare("rm_description"),
        "launch",
        "description.launch.py",
    ])
    lio_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "lio_adapter.yaml",
    ])
    gimbal_state_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "gimbal_state_adapter.yaml",
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
        ),
        Node(
            package="rm_localization_adapters",
            executable="map_odom_stub",
            name="map_odom_stub",
            output="screen",
        ),
        Node(
            package="rm_localization_adapters",
            executable="gimbal_state_adapter",
            name="gimbal_state_adapter",
            output="screen",
            parameters=[gimbal_state_adapter_config],
        ),
        Node(
            package="rm_localization_adapters",
            executable="lio_adapter",
            name="lio_adapter",
            output="screen",
            parameters=[lio_adapter_config],
        ),
        Node(
            package="rm_localization_adapters",
            executable="fake_lio_odom_publisher",
            name="fake_lio_odom_publisher",
            output="screen",
        ),
    ])
