from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
    imu_frame_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "imu_frame_adapter.yaml",
    ])

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
            parameters=[lio_adapter_config],
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
            executable="imu_frame_adapter",
            name="imu_frame_adapter",
            output="screen",
            parameters=[imu_frame_adapter_config],
        ),
    ])
