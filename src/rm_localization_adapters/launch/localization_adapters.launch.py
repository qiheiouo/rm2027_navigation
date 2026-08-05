from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_map_odom_stub = LaunchConfiguration("use_map_odom_stub")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    default_lio_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "lio_adapter.yaml",
    ])
    lio_adapter_config = LaunchConfiguration("lio_adapter_config")
    default_gimbal_state_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "gimbal_state_adapter.yaml",
    ])
    gimbal_state_adapter_config = LaunchConfiguration("gimbal_state_adapter_config")
    imu_frame_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "config",
        "imu_frame_adapter.yaml",
    ])

    # Phase 1 skeleton only. FAST-LIO is intentionally not launched here.
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_map_odom_stub", default_value="true"),
        DeclareLaunchArgument(
            "raw_odom_topic",
            default_value="/odometry/fast_lio_raw",
        ),
        DeclareLaunchArgument(
            "lio_adapter_config",
            default_value=default_lio_adapter_config,
        ),
        DeclareLaunchArgument(
            "gimbal_state_adapter_config",
            default_value=default_gimbal_state_adapter_config,
        ),
        Node(
            condition=IfCondition(use_map_odom_stub),
            package="rm_localization_adapters",
            executable="map_odom_stub",
            name="map_odom_stub",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rm_localization_adapters",
            executable="lio_adapter",
            name="lio_adapter",
            output="screen",
            parameters=[
                lio_adapter_config,
                {
                    "use_sim_time": use_sim_time,
                    "raw_odom_topic": raw_odom_topic,
                },
            ],
        ),
        Node(
            package="rm_localization_adapters",
            executable="gimbal_state_adapter",
            name="gimbal_state_adapter",
            output="screen",
            parameters=[
                gimbal_state_adapter_config,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="rm_localization_adapters",
            executable="imu_frame_adapter",
            name="imu_frame_adapter",
            output="screen",
            parameters=[
                imu_frame_adapter_config,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
