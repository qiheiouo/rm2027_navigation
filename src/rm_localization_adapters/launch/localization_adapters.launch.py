from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Phase 1 skeleton only. FAST-LIO is intentionally not launched here.
    return LaunchDescription([
        Node(
            package="rm_localization_adapters",
            executable="map_odom_stub",
            name="map_odom_stub",
            output="screen",
        ),
        Node(
            package="rm_localization_adapters",
            executable="lio_adapter",
            name="lio_adapter",
            output="screen",
        ),
    ])
