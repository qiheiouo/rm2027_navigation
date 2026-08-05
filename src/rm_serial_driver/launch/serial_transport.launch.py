from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    operator_goal_rx_enabled = LaunchConfiguration("operator_goal_rx_enabled")
    operator_goal_raw_topic = LaunchConfiguration("operator_goal_raw_topic")
    operator_goal_coordinate_system = LaunchConfiguration(
        "operator_goal_coordinate_system"
    )
    read_poll_rate_hz = LaunchConfiguration("read_poll_rate_hz")
    competition_v2_dry_run = LaunchConfiguration("competition_v2_dry_run")
    posture_request_topic = LaunchConfiguration("posture_request_topic")
    posture_state_topic = LaunchConfiguration("posture_state_topic")
    gimbal_state_topic = LaunchConfiguration("gimbal_state_topic")
    chassis_heading_topic = LaunchConfiguration("chassis_heading_topic")
    connection_state_topic = LaunchConfiguration("connection_state_topic")
    connection_timeout_sec = LaunchConfiguration("connection_timeout_sec")
    gimbal_timeout_sec = LaunchConfiguration("gimbal_timeout_sec")
    chassis_heading_timeout_sec = LaunchConfiguration(
        "chassis_heading_timeout_sec"
    )
    posture_timeout_sec = LaunchConfiguration("posture_timeout_sec")
    posture_rate_hz = LaunchConfiguration("posture_rate_hz")
    heartbeat_rate_hz = LaunchConfiguration("heartbeat_rate_hz")
    required_remote_capabilities = LaunchConfiguration(
        "required_remote_capabilities"
    )
    use_competition_v2 = IfCondition(
        PythonExpression(["'", protocol_profile, "' == 'competition_v2'"])
    )
    use_legacy_transport = UnlessCondition(
        PythonExpression(["'", protocol_profile, "' == 'competition_v2'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "protocol_profile",
            default_value="legacy_v1_no_crc",
            choices=["legacy_v1_no_crc", "hpm_crc_v1", "competition_v2"],
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
        DeclareLaunchArgument("operator_goal_rx_enabled", default_value="false"),
        DeclareLaunchArgument(
            "operator_goal_raw_topic",
            default_value="/operator/navigation_target_raw",
        ),
        DeclareLaunchArgument(
            "operator_goal_coordinate_system",
            default_value="unknown",
            choices=["unknown", "referee_field", "map"],
        ),
        DeclareLaunchArgument("read_poll_rate_hz", default_value="200.0"),
        DeclareLaunchArgument("competition_v2_dry_run", default_value="false"),
        DeclareLaunchArgument(
            "posture_request_topic", default_value="/robot/posture/request"
        ),
        DeclareLaunchArgument(
            "posture_state_topic", default_value="/robot/posture/state"
        ),
        DeclareLaunchArgument("gimbal_state_topic", default_value="/gimbal/state"),
        DeclareLaunchArgument(
            "chassis_heading_topic", default_value="/chassis/heading"
        ),
        DeclareLaunchArgument(
            "connection_state_topic", default_value="/serial/connection_state"
        ),
        DeclareLaunchArgument("connection_timeout_sec", default_value="0.75"),
        DeclareLaunchArgument("gimbal_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("chassis_heading_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("posture_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("posture_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("heartbeat_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("required_remote_capabilities", default_value="1"),
        LogInfo(msg=[
            "[serial_transport] requested. device=",
            device,
            " profile=",
            protocol_profile,
            " competition_v2_dry_run=",
            competition_v2_dry_run,
            ". Legacy profiles always open the device; competition_v2 opens it only ",
            "when dry_run=false.",
        ]),
        Node(
            package="rm_serial_driver",
            executable="serial_transport_node",
            name="serial_transport_node",
            output="screen",
            condition=use_legacy_transport,
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
                "operator_goal_rx_enabled": ParameterValue(
                    operator_goal_rx_enabled, value_type=bool
                ),
                "operator_goal_raw_topic": operator_goal_raw_topic,
                "operator_goal_coordinate_system": operator_goal_coordinate_system,
                "read_poll_rate_hz": ParameterValue(
                    read_poll_rate_hz, value_type=float
                ),
            }],
        ),
        Node(
            package="rm_serial_driver",
            executable="competition_v2_transport_node",
            name="serial_transport_node",
            output="screen",
            condition=use_competition_v2,
            parameters=[{
                "device": device,
                "baudrate": ParameterValue(baudrate, value_type=int),
                "dry_run": ParameterValue(
                    competition_v2_dry_run, value_type=bool
                ),
                "chassis_rate_hz": ParameterValue(
                    publish_rate_hz, value_type=float
                ),
                "cmd_vel_timeout_sec": ParameterValue(
                    cmd_vel_timeout_sec, value_type=float
                ),
                "cmd_vel_topic": cmd_vel_topic,
                "max_vx": ParameterValue(max_vx, value_type=float),
                "max_vy": ParameterValue(max_vy, value_type=float),
                "max_wz": ParameterValue(max_wz, value_type=float),
                "referee_raw_topic": referee_raw_topic,
                "operator_goal_raw_topic": operator_goal_raw_topic,
                "read_poll_rate_hz": ParameterValue(
                    read_poll_rate_hz, value_type=float
                ),
                "posture_request_topic": posture_request_topic,
                "posture_state_topic": posture_state_topic,
                "gimbal_state_topic": gimbal_state_topic,
                "chassis_heading_topic": chassis_heading_topic,
                "connection_state_topic": connection_state_topic,
                "connection_timeout_sec": ParameterValue(
                    connection_timeout_sec, value_type=float
                ),
                "gimbal_timeout_sec": ParameterValue(
                    gimbal_timeout_sec, value_type=float
                ),
                "chassis_heading_timeout_sec": ParameterValue(
                    chassis_heading_timeout_sec, value_type=float
                ),
                "posture_timeout_sec": ParameterValue(
                    posture_timeout_sec, value_type=float
                ),
                "posture_rate_hz": ParameterValue(
                    posture_rate_hz, value_type=float
                ),
                "heartbeat_rate_hz": ParameterValue(
                    heartbeat_rate_hz, value_type=float
                ),
                "required_remote_capabilities": ParameterValue(
                    required_remote_capabilities, value_type=int
                ),
            }],
        ),
    ])
