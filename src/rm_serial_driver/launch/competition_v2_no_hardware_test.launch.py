from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    publish_operator_target = LaunchConfiguration("publish_operator_target")
    posture_transition_responses = LaunchConfiguration(
        "posture_transition_responses"
    )
    return LaunchDescription([
        DeclareLaunchArgument("publish_operator_target", default_value="false"),
        DeclareLaunchArgument("posture_transition_responses", default_value="2"),
        LogInfo(msg=[
            "[competition_v2_test] dry-run transport and mock lower controller; ",
            "no serial device, TF, odometry or navigation goal is used.",
        ]),
        Node(
            package="rm_serial_driver",
            executable="competition_v2_transport_node",
            name="serial_transport_node",
            output="screen",
            parameters=[{
                "dry_run": True,
                "required_remote_capabilities": 31,
            }],
        ),
        Node(
            package="rm_serial_driver",
            executable="competition_v2_mock_lower_node",
            name="competition_v2_mock_lower_node",
            output="screen",
            parameters=[{
                "publish_operator_target": ParameterValue(
                    publish_operator_target, value_type=bool
                ),
                "posture_transition_responses": ParameterValue(
                    posture_transition_responses, value_type=int
                ),
            }],
        ),
        Node(
            package="rm_localization_adapters",
            executable="gimbal_state_adapter",
            name="gimbal_state_adapter",
            output="screen",
            parameters=[{"use_input": True}],
        ),
    ])
