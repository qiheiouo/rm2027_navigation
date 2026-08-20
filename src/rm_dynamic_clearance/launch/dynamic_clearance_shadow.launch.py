from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("rm_dynamic_clearance"),
        "config",
        "dynamic_clearance_shadow.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            condition=IfCondition(LaunchConfiguration("enabled")),
            package="rm_dynamic_clearance",
            executable="dynamic_clearance_node",
            name="dynamic_clearance_shadow",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                },
            ],
        ),
    ])
