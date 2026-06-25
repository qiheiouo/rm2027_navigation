from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    protocol_profile = LaunchConfiguration("protocol_profile")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    cmd_vel_timeout_sec = LaunchConfiguration("cmd_vel_timeout_sec")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    mock_tx_topic = LaunchConfiguration("mock_tx_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "protocol_profile",
            default_value="legacy_v1_no_crc",
            choices=["legacy_v1_no_crc", "hpm_crc_v1"],
        ),
        DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("cmd_vel_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("mock_tx_topic", default_value="/serial/mock_tx"),
        LogInfo(msg=[
            "[serial_dry_run] No serial device will be opened. profile=",
            protocol_profile,
        ]),
        Node(
            package="rm_serial_driver",
            executable="serial_dry_run_node",
            name="serial_dry_run_node",
            output="screen",
            parameters=[{
                "protocol_profile": protocol_profile,
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                "cmd_vel_timeout_sec": ParameterValue(cmd_vel_timeout_sec, value_type=float),
                "cmd_vel_topic": cmd_vel_topic,
                "mock_tx_topic": mock_tx_topic,
            }],
        ),
    ])
