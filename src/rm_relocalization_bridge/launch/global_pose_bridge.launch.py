from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    default_config = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "map_odom_from_global_pose.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        Node(
            package="rm_relocalization_bridge",
            executable="map_odom_from_global_pose",
            name="map_odom_from_global_pose",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
    ])
