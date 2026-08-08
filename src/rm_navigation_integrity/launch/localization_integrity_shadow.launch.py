from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("rm_navigation_integrity"),
        "config",
        "localization_integrity_shadow.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("profile", default_value="generic"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("metrics_output_path", default_value=""),
        Node(
            condition=IfCondition(LaunchConfiguration("enabled")),
            package="rm_navigation_integrity",
            executable="localization_integrity_node",
            name="localization_integrity",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "profile": LaunchConfiguration("profile"),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                    "metrics_output_path": LaunchConfiguration(
                        "metrics_output_path"
                    ),
                },
            ],
        ),
    ])
