from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enabled = LaunchConfiguration("enable_chassis_mode_gate")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("enable_chassis_mode_gate", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            condition=IfCondition(enabled),
            package="rm_chassis_interface",
            executable="chassis_mode_gate_node",
            name="chassis_mode_gate",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
