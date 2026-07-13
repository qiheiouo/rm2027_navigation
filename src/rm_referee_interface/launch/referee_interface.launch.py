from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enabled = LaunchConfiguration("enable_referee_interface")
    use_mock = LaunchConfiguration("use_mock")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("enable_referee_interface", default_value="false"),
        DeclareLaunchArgument("use_mock", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            condition=IfCondition(enabled),
            package="rm_referee_interface",
            executable="referee_state_gate",
            name="referee_state_gate",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            condition=IfCondition(PythonExpression([
                "'", enabled, "'.lower() in ['1','true','yes','on'] and '",
                use_mock, "'.lower() in ['1','true','yes','on']",
            ])),
            package="rm_referee_interface",
            executable="referee_state_mock",
            name="referee_state_mock",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
