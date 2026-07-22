from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    protocol_profile = LaunchConfiguration("protocol_profile")
    device = LaunchConfiguration("device")
    baudrate = LaunchConfiguration("baudrate")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    cmd_vel_timeout_sec = LaunchConfiguration("cmd_vel_timeout_sec")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    max_vx = LaunchConfiguration("max_vx")
    max_vy = LaunchConfiguration("max_vy")
    max_wz = LaunchConfiguration("max_wz")
    referee_rx_enabled = LaunchConfiguration("referee_rx_enabled")
    referee_raw_topic = LaunchConfiguration("referee_raw_topic")
    read_poll_rate_hz = LaunchConfiguration("read_poll_rate_hz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "protocol_profile",
            default_value="legacy_v1_no_crc",
            choices=["legacy_v1_no_crc", "hpm_crc_v1"],
        ),
        DeclareLaunchArgument("device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("cmd_vel_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("max_vx", default_value="0.15"),
        DeclareLaunchArgument("max_vy", default_value="0.15"),
        DeclareLaunchArgument("max_wz", default_value="0.30"),
        DeclareLaunchArgument("referee_rx_enabled", default_value="false"),
        DeclareLaunchArgument("referee_raw_topic", default_value="/referee/state_raw"),
        DeclareLaunchArgument("read_poll_rate_hz", default_value="200.0"),
        LogInfo(msg=[
            "[serial_transport] REAL serial writer requested. device=",
            device,
            " profile=",
            protocol_profile,
            ". Use only with wheels off-ground and remote/manual stop ready.",
        ]),
        Node(
            package="rm_serial_driver",
            executable="serial_transport_node",
            name="serial_transport_node",
            output="screen",
            parameters=[{
                "protocol_profile": protocol_profile,
                "device": device,
                "baudrate": ParameterValue(baudrate, value_type=int),
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                "cmd_vel_timeout_sec": ParameterValue(cmd_vel_timeout_sec, value_type=float),
                "cmd_vel_topic": cmd_vel_topic,
                "max_vx": ParameterValue(max_vx, value_type=float),
                "max_vy": ParameterValue(max_vy, value_type=float),
                "max_wz": ParameterValue(max_wz, value_type=float),
                "referee_rx_enabled": ParameterValue(
                    referee_rx_enabled, value_type=bool
                ),
                "referee_raw_topic": referee_raw_topic,
                "read_poll_rate_hz": ParameterValue(
                    read_poll_rate_hz, value_type=float
                ),
            }],
        ),
    ])
