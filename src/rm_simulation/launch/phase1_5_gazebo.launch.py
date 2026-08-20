from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")
    use_scan_adapter = LaunchConfiguration("use_scan_adapter")
    use_chassis_heading_fusion = LaunchConfiguration(
        "use_chassis_heading_fusion"
    )
    gimbal_use_input = LaunchConfiguration("gimbal_use_input")
    gimbal_input_topic = LaunchConfiguration("gimbal_input_topic")
    gimbal_yaw = LaunchConfiguration("gimbal_yaw")
    gimbal_motion_mode = LaunchConfiguration("gimbal_motion_mode")
    gimbal_amplitude = LaunchConfiguration("gimbal_amplitude")
    gimbal_frequency = LaunchConfiguration("gimbal_frequency")
    gimbal_angular_velocity = LaunchConfiguration("gimbal_angular_velocity")
    gimbal_joint_height = LaunchConfiguration("gimbal_joint_height")
    sim_lidar_x = LaunchConfiguration("sim_lidar_x")
    sim_lidar_y = LaunchConfiguration("sim_lidar_y")
    sim_lidar_z = LaunchConfiguration("sim_lidar_z")
    heading_world_offset_rad = LaunchConfiguration(
        "heading_world_offset_rad"
    )
    heading_timestamp_offset_sec = LaunchConfiguration(
        "heading_timestamp_offset_sec"
    )
    heading_publish_divider = LaunchConfiguration("heading_publish_divider")
    use_localization_disturbance = LaunchConfiguration(
        "use_localization_disturbance"
    )
    localization_reference_yaw = LaunchConfiguration(
        "localization_reference_yaw"
    )
    localization_lateral_noise_std_m = LaunchConfiguration(
        "localization_lateral_noise_std_m"
    )
    localization_yaw_noise_std_rad = LaunchConfiguration(
        "localization_yaw_noise_std_rad"
    )
    localization_delay_sec = LaunchConfiguration("localization_delay_sec")
    localization_lateral_drift_amplitude_m = LaunchConfiguration(
        "localization_lateral_drift_amplitude_m"
    )
    localization_yaw_drift_amplitude_rad = LaunchConfiguration(
        "localization_yaw_drift_amplitude_rad"
    )
    localization_drift_frequency_hz = LaunchConfiguration(
        "localization_drift_frequency_hz"
    )
    localization_random_seed = LaunchConfiguration(
        "localization_random_seed"
    )
    use_chassis_disturbance = LaunchConfiguration(
        "use_chassis_disturbance"
    )
    chassis_forward_scale = LaunchConfiguration("chassis_forward_scale")
    chassis_lateral_positive_scale = LaunchConfiguration(
        "chassis_lateral_positive_scale"
    )
    chassis_lateral_negative_scale = LaunchConfiguration(
        "chassis_lateral_negative_scale"
    )
    chassis_angular_scale = LaunchConfiguration("chassis_angular_scale")
    chassis_lateral_time_constant_sec = LaunchConfiguration(
        "chassis_lateral_time_constant_sec"
    )
    chassis_angular_time_constant_sec = LaunchConfiguration(
        "chassis_angular_time_constant_sec"
    )
    nav2_params = LaunchConfiguration("nav2_params")
    rviz_config = LaunchConfiguration("rviz_config")

    default_world = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "worlds",
        "phase1_omni.sdf",
    ])
    world = LaunchConfiguration("world")
    bridge_config = PathJoinSubstitution([
        FindPackageShare("rm_simulation"),
        "config",
        "ros_gz_bridge.yaml",
    ])
    description_launch = PathJoinSubstitution([
        FindPackageShare("rm_description"),
        "launch",
        "description.launch.py",
    ])
    localization_launch = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "launch",
        "localization_adapters.launch.py",
    ])
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
    gimbal_state_adapter_config = LaunchConfiguration(
        "gimbal_state_adapter_config"
    )
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    ])
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    ])
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_phase1_5_gazebo.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "rviz",
        "phase1.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_scan_adapter", default_value="true"),
        DeclareLaunchArgument(
            "use_chassis_heading_fusion", default_value="false"
        ),
        DeclareLaunchArgument("gimbal_use_input", default_value="true"),
        DeclareLaunchArgument("gimbal_input_topic", default_value="/gimbal/state"),
        DeclareLaunchArgument("gimbal_yaw", default_value="0.0"),
        DeclareLaunchArgument("gimbal_motion_mode", default_value="fixed"),
        DeclareLaunchArgument("gimbal_amplitude", default_value="0.0"),
        DeclareLaunchArgument("gimbal_frequency", default_value="0.0"),
        DeclareLaunchArgument("gimbal_angular_velocity", default_value="0.0"),
        DeclareLaunchArgument("gimbal_joint_height", default_value="0.115"),
        DeclareLaunchArgument("sim_lidar_x", default_value="0.12"),
        DeclareLaunchArgument("sim_lidar_y", default_value="0.0"),
        DeclareLaunchArgument("sim_lidar_z", default_value="0.065"),
        DeclareLaunchArgument("heading_world_offset_rad", default_value="0.0"),
        DeclareLaunchArgument(
            "heading_timestamp_offset_sec", default_value="0.0"
        ),
        DeclareLaunchArgument("heading_publish_divider", default_value="1"),
        DeclareLaunchArgument(
            "lio_adapter_config", default_value=default_lio_adapter_config
        ),
        DeclareLaunchArgument(
            "gimbal_state_adapter_config",
            default_value=default_gimbal_state_adapter_config,
        ),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument(
            "use_localization_disturbance", default_value="false"
        ),
        DeclareLaunchArgument(
            "localization_reference_yaw", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "localization_lateral_noise_std_m", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "localization_yaw_noise_std_rad", default_value="0.0"
        ),
        DeclareLaunchArgument("localization_delay_sec", default_value="0.0"),
        DeclareLaunchArgument(
            "localization_lateral_drift_amplitude_m", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "localization_yaw_drift_amplitude_rad", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "localization_drift_frequency_hz", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "localization_random_seed", default_value="20270728"
        ),
        DeclareLaunchArgument("use_chassis_disturbance", default_value="false"),
        DeclareLaunchArgument("chassis_forward_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "chassis_lateral_positive_scale", default_value="1.0"
        ),
        DeclareLaunchArgument(
            "chassis_lateral_negative_scale", default_value="1.0"
        ),
        DeclareLaunchArgument("chassis_angular_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "chassis_lateral_time_constant_sec", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "chassis_angular_time_constant_sec", default_value="0.0"
        ),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        LogInfo(msg=[
            "[phase1_5_gazebo] Gazebo scan + ground truth -> Nav2 -> ",
            "/cmd_vel -> chassis_interface_stub -> Gazebo. No Gazebo TF is bridged.",
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=IfCondition(headless),
            launch_arguments={
                "gz_args": ["-r -s --headless-rendering ", world],
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=UnlessCondition(headless),
            launch_arguments={"gz_args": ["-r ", world]}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="simulation_bridge",
            output="screen",
            condition=UnlessCondition(use_chassis_disturbance),
            parameters=[{"config_file": bridge_config}],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="simulation_bridge",
            output="screen",
            condition=IfCondition(use_chassis_disturbance),
            parameters=[{"config_file": bridge_config}],
            remappings=[
                (
                    "/simulation/chassis/cmd_vel",
                    "/simulation/chassis/cmd_vel_applied",
                )
            ],
        ),
        Node(
            package="rm_simulation",
            executable="chassis_command_disturbance",
            name="chassis_command_disturbance",
            output="screen",
            condition=IfCondition(use_chassis_disturbance),
            parameters=[{
                "input_topic": "/simulation/chassis/cmd_vel",
                "output_topic": "/simulation/chassis/cmd_vel_applied",
                "forward_scale": ParameterValue(
                    chassis_forward_scale, value_type=float
                ),
                "lateral_positive_scale": ParameterValue(
                    chassis_lateral_positive_scale, value_type=float
                ),
                "lateral_negative_scale": ParameterValue(
                    chassis_lateral_negative_scale, value_type=float
                ),
                "angular_scale": ParameterValue(
                    chassis_angular_scale, value_type=float
                ),
                "lateral_time_constant_sec": ParameterValue(
                    chassis_lateral_time_constant_sec, value_type=float
                ),
                "angular_time_constant_sec": ParameterValue(
                    chassis_angular_time_constant_sec, value_type=float
                ),
            }],
        ),
        Node(
            package="rm_simulation",
            executable="sim_gimbal_state_publisher",
            name="sim_gimbal_state_publisher",
            output="screen",
            condition=IfCondition(
                PythonExpression([
                    "'", gimbal_use_input, "' == 'true' and '",
                    use_chassis_heading_fusion, "' != 'true'",
                ])
            ),
            parameters=[{
                "use_sim_time": True,
                "state_topic": gimbal_input_topic,
                "motion_mode": gimbal_motion_mode,
                "offset_rad": ParameterValue(gimbal_yaw, value_type=float),
                "amplitude_rad": ParameterValue(
                    gimbal_amplitude, value_type=float
                ),
                "frequency_hz": ParameterValue(
                    gimbal_frequency, value_type=float
                ),
                "angular_velocity_rad_s": ParameterValue(
                    gimbal_angular_velocity, value_type=float
                ),
            }],
        ),
        Node(
            package="rm_simulation",
            executable="sim_chassis_heading_lio_source",
            name="sim_chassis_heading_lio_source",
            output="screen",
            condition=IfCondition(use_chassis_heading_fusion),
            parameters=[{
                "use_sim_time": True,
                "ground_truth_topic": "/simulation/ground_truth/odom",
                "raw_lio_topic": "/odometry/fast_lio_raw",
                "chassis_heading_topic": "/chassis/heading",
                "fused_odom_topic": "/odometry/lio",
                "derived_gimbal_topic": "/gimbal/state_derived",
                "motion_mode": gimbal_motion_mode,
                "gimbal_offset_rad": ParameterValue(
                    gimbal_yaw, value_type=float
                ),
                "gimbal_amplitude_rad": ParameterValue(
                    gimbal_amplitude, value_type=float
                ),
                "gimbal_frequency_hz": ParameterValue(
                    gimbal_frequency, value_type=float
                ),
                "gimbal_angular_velocity_rad_s": ParameterValue(
                    gimbal_angular_velocity, value_type=float
                ),
                "gimbal_center_in_base.z": ParameterValue(
                    gimbal_joint_height, value_type=float
                ),
                "sensor_offset_from_gimbal.x": ParameterValue(
                    sim_lidar_x, value_type=float
                ),
                "sensor_offset_from_gimbal.y": ParameterValue(
                    sim_lidar_y, value_type=float
                ),
                "sensor_offset_from_gimbal.z": ParameterValue(
                    sim_lidar_z, value_type=float
                ),
                "heading_world_offset_rad": ParameterValue(
                    heading_world_offset_rad, value_type=float
                ),
                "heading_timestamp_offset_sec": ParameterValue(
                    heading_timestamp_offset_sec, value_type=float
                ),
                "heading_publish_divider": ParameterValue(
                    heading_publish_divider, value_type=int
                ),
            }],
        ),
        Node(
            package="rm_simulation",
            executable="scan_frame_adapter",
            name="scan_frame_adapter",
            output="screen",
            condition=IfCondition(use_scan_adapter),
            parameters=[{
                "use_sim_time": True,
                "input_topic": "/simulation/scan_raw",
                "output_topic": "/scan",
                "output_frame": "sim_lidar_link",
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                "use_sim_time": "true",
                "use_sim_lidar": "true",
                "gimbal_joint_height": gimbal_joint_height,
                "sim_lidar_x": sim_lidar_x,
                "sim_lidar_y": sim_lidar_y,
                "sim_lidar_z": sim_lidar_z,
            }.items(),
        ),
        Node(
            package="rm_simulation",
            executable="localization_disturbance",
            name="localization_disturbance",
            output="screen",
            condition=IfCondition(
                PythonExpression([
                    "'", use_chassis_heading_fusion, "' != 'true' and '",
                    use_localization_disturbance, "' == 'true'",
                ])
            ),
            parameters=[{
                "use_sim_time": True,
                "input_topic": "/simulation/ground_truth/odom",
                "output_topic": "/simulation/localization/odom",
                "reference_yaw": ParameterValue(
                    localization_reference_yaw, value_type=float
                ),
                "lateral_noise_std_m": ParameterValue(
                    localization_lateral_noise_std_m, value_type=float
                ),
                "yaw_noise_std_rad": ParameterValue(
                    localization_yaw_noise_std_rad, value_type=float
                ),
                "delay_sec": ParameterValue(
                    localization_delay_sec, value_type=float
                ),
                "lateral_drift_amplitude_m": ParameterValue(
                    localization_lateral_drift_amplitude_m,
                    value_type=float,
                ),
                "yaw_drift_amplitude_rad": ParameterValue(
                    localization_yaw_drift_amplitude_rad,
                    value_type=float,
                ),
                "drift_frequency_hz": ParameterValue(
                    localization_drift_frequency_hz, value_type=float
                ),
                "random_seed": ParameterValue(
                    localization_random_seed, value_type=int
                ),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            condition=IfCondition(
                PythonExpression([
                    "'", use_chassis_heading_fusion, "' != 'true' and '",
                    use_localization_disturbance, "' != 'true'",
                ])
            ),
            launch_arguments={
                "use_sim_time": "true",
                "raw_odom_topic": "/simulation/ground_truth/odom",
                "gimbal_use_input": gimbal_use_input,
                "gimbal_input_topic": gimbal_input_topic,
                "lio_adapter_config": lio_adapter_config,
                "gimbal_state_adapter_config": gimbal_state_adapter_config,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            condition=IfCondition(
                PythonExpression([
                    "'", use_chassis_heading_fusion, "' != 'true' and '",
                    use_localization_disturbance, "' == 'true'",
                ])
            ),
            launch_arguments={
                "use_sim_time": "true",
                "raw_odom_topic": "/simulation/localization/odom",
                "gimbal_use_input": gimbal_use_input,
                "gimbal_input_topic": gimbal_input_topic,
                "lio_adapter_config": lio_adapter_config,
                "gimbal_state_adapter_config": gimbal_state_adapter_config,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            condition=IfCondition(use_chassis_heading_fusion),
            launch_arguments={
                "use_sim_time": "true",
                "raw_odom_topic": "/odometry/fast_lio_raw",
                "gimbal_use_input": "true",
                "gimbal_input_topic": "/gimbal/state_derived",
                "lio_adapter_config": lio_adapter_config,
                "gimbal_state_adapter_config": gimbal_state_adapter_config,
            }.items(),
        ),
        Node(
            package="rm_chassis_interface",
            executable="chassis_interface_stub",
            name="chassis_interface_stub",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "cmd_vel_topic": "/cmd_vel",
                "mock_output_cmd_vel_topic": "/simulation/chassis/cmd_vel",
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(use_nav2),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": nav2_params,
                "autostart": "true",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
        ),
    ])
