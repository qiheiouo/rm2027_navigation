from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _validate_runtime_modes(context, *args, **kwargs):
    use_serial_dry_run = (
        LaunchConfiguration("use_serial_dry_run").perform(context).strip().lower()
        in TRUE_VALUES
    )
    use_real_serial = (
        LaunchConfiguration("use_real_serial").perform(context).strip().lower()
        in TRUE_VALUES
    )
    referee_rx_enabled = (
        LaunchConfiguration("serial_referee_rx_enabled")
        .perform(context)
        .strip()
        .lower()
        in TRUE_VALUES
    )
    serial_profile = LaunchConfiguration("serial_protocol_profile").perform(context)
    use_mapping = (
        LaunchConfiguration("use_mapping").perform(context).strip().lower()
        in TRUE_VALUES
    )
    record_ray_observations = (
        LaunchConfiguration("mapping_record_ray_observations")
        .perform(context)
        .strip()
        .lower()
        in TRUE_VALUES
    )
    use_nav2 = (
        LaunchConfiguration("use_nav2").perform(context).strip().lower()
        in TRUE_VALUES
    )
    use_driver = (
        LaunchConfiguration("use_driver").perform(context).strip().lower()
        in TRUE_VALUES
    )
    use_lio_backend = (
        LaunchConfiguration("use_lio_backend").perform(context).strip().lower()
        in TRUE_VALUES
    )
    use_map_odom_stub = (
        LaunchConfiguration("use_map_odom_stub").perform(context).strip().lower()
        in TRUE_VALUES
    )
    selected_side = LaunchConfiguration("selected_side").perform(context).strip().lower()
    driver_mode = LaunchConfiguration("driver_mode").perform(context).strip().lower()
    if driver_mode not in {"single", "dual"}:
        raise RuntimeError("driver_mode must be 'single' or 'dual'")
    if driver_mode == "dual" and selected_side != "left":
        raise RuntimeError(
            "old-car dual driver currently requires selected_side:=left so the "
            "verified left MID360 remains the LIO/IMU source"
        )
    if use_serial_dry_run and use_real_serial:
        raise RuntimeError(
            "use_serial_dry_run and use_real_serial must not be true at the same time."
        )
    if referee_rx_enabled and not use_real_serial:
        raise RuntimeError("serial referee receive requires use_real_serial:=true")
    if referee_rx_enabled and serial_profile != "hpm_crc_v1":
        raise RuntimeError(
            "the currently flashed referee upload requires "
            "serial_protocol_profile:=hpm_crc_v1"
        )
    if use_mapping and (use_nav2 or use_serial_dry_run or use_real_serial):
        raise RuntimeError(
            "old-car mapping mode cannot run Nav2 or either serial mode. "
            "Build the map under manual control only."
        )
    if use_mapping and not (use_driver and use_lio_backend and use_map_odom_stub):
        raise RuntimeError(
            "old-car integrated mapping requires use_driver:=true, "
            "use_lio_backend:=true, and use_map_odom_stub:=true."
        )
    if use_mapping and selected_side != "left":
        raise RuntimeError(
            "old-car integrated mapping currently requires selected_side:=left "
            "because its verified self-filter profile is left-lidar specific."
        )
    if record_ray_observations and not use_mapping:
        raise RuntimeError(
            "mapping_record_ray_observations requires use_mapping:=true"
        )
    return []


def generate_launch_description():
    selected_side = LaunchConfiguration("selected_side")
    driver_mode = LaunchConfiguration("driver_mode")
    driver_publish_freq = LaunchConfiguration("driver_publish_freq")
    lio_input_mode = LaunchConfiguration("lio_input_mode")
    use_driver = LaunchConfiguration("use_driver")
    use_lio_backend = LaunchConfiguration("use_lio_backend")
    use_map_odom_stub = LaunchConfiguration("use_map_odom_stub")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_serial_dry_run = LaunchConfiguration("use_serial_dry_run")
    use_real_serial = LaunchConfiguration("use_real_serial")
    serial_protocol_profile = LaunchConfiguration("serial_protocol_profile")
    serial_device = LaunchConfiguration("serial_device")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    serial_max_vx = LaunchConfiguration("serial_max_vx")
    serial_max_vy = LaunchConfiguration("serial_max_vy")
    serial_max_wz = LaunchConfiguration("serial_max_wz")
    serial_cmd_vel_topic = LaunchConfiguration("serial_cmd_vel_topic")
    serial_referee_rx_enabled = LaunchConfiguration("serial_referee_rx_enabled")
    serial_referee_raw_topic = LaunchConfiguration("serial_referee_raw_topic")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    update_method = LaunchConfiguration("update_method")
    nav2_params = LaunchConfiguration("nav2_params")
    navigate_to_pose_action = LaunchConfiguration("navigate_to_pose_action")
    goal_pose_topic = LaunchConfiguration("goal_pose_topic")
    pointcloud_filter_enabled = LaunchConfiguration("pointcloud_filter_enabled")
    pointcloud_filter_input_topic = LaunchConfiguration("pointcloud_filter_input_topic")
    pointcloud_filter_output_topic = LaunchConfiguration("pointcloud_filter_output_topic")
    local_scan_enabled = LaunchConfiguration("local_scan_enabled")
    local_scan_input_topic = LaunchConfiguration("local_scan_input_topic")
    local_scan_output_topic = LaunchConfiguration("local_scan_output_topic")
    use_mapping = LaunchConfiguration("use_mapping")
    publish_transformed_registered_cloud = LaunchConfiguration(
        "publish_transformed_registered_cloud"
    )
    mapping_output_root = LaunchConfiguration("mapping_output_root")
    mapping_map_id = LaunchConfiguration("mapping_map_id")
    mapping_revision = LaunchConfiguration("mapping_revision")
    mapping_record_ray_observations = LaunchConfiguration(
        "mapping_record_ray_observations"
    )
    mapping_ray_sample_period_sec = LaunchConfiguration(
        "mapping_ray_sample_period_sec"
    )
    mapping_ray_min_range = LaunchConfiguration("mapping_ray_min_range")
    mapping_ray_max_range = LaunchConfiguration("mapping_ray_max_range")
    mapping_ray_voxel_size = LaunchConfiguration("mapping_ray_voxel_size")
    mapping_ray_max_frames = LaunchConfiguration("mapping_ray_max_frames")
    mapping_ray_max_rays_per_frame = LaunchConfiguration(
        "mapping_ray_max_rays_per_frame"
    )
    mapping_ray_max_total_rays = LaunchConfiguration(
        "mapping_ray_max_total_rays"
    )
    mapping_ray_max_bytes = LaunchConfiguration("mapping_ray_max_bytes")

    old_description_launch = PathJoinSubstitution([
        FindPackageShare("rm_description"),
        "launch",
        "old_car_2026_description.launch.py",
    ])
    single_driver_launch = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "launch",
        "single_mid360_driver.launch.py",
    ])
    dual_driver_launch = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "launch",
        "dual_mid360_driver.launch.py",
    ])
    lio_backend_launch = PathJoinSubstitution([
        FindPackageShare("rm_lio_bringup"),
        "launch",
        "fast_lio_multi_phase2a.launch.py",
    ])
    old_fast_lio_config = PathJoinSubstitution([
        FindPackageShare("rm_lio_bringup"),
        "config",
        "fast_lio_multi_old_car_2026.yaml",
    ])
    localization_launch = PathJoinSubstitution([
        FindPackageShare("rm_localization_adapters"),
        "launch",
        "localization_adapters.launch.py",
    ])
    old_lio_adapter_config = PathJoinSubstitution([
        FindPackageShare("rm_lio_bringup"),
        "config",
        "lio_adapter_old_car_2026.yaml",
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    ])
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_old_car_2026_left.yaml",
    ])
    serial_dry_run_launch = PathJoinSubstitution([
        FindPackageShare("rm_serial_driver"),
        "launch",
        "serial_dry_run.launch.py",
    ])
    serial_transport_launch = PathJoinSubstitution([
        FindPackageShare("rm_serial_driver"),
        "launch",
        "serial_transport.launch.py",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "rviz",
        "old_car_2026.rviz",
    ])
    old_car_pointcloud_filter_config = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "config",
        "old_car_pointcloud_filter.yaml",
    ])
    old_car_pointcloud_to_scan_config = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "config",
        "old_car_pointcloud_to_laserscan.yaml",
    ])
    mapping_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "mapping.launch.py",
    ])
    registered_cloud_enabled = PythonExpression([
        "'", use_mapping, "'.lower() in ['1', 'true', 'yes', 'on'] or '",
        publish_transformed_registered_cloud,
        "'.lower() in ['1', 'true', 'yes', 'on']",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("selected_side", default_value="left"),
        DeclareLaunchArgument(
            "driver_mode",
            default_value="single",
            description="Use one left/right driver or one dual-device SDK instance.",
        ),
        DeclareLaunchArgument(
            "driver_publish_freq",
            default_value="50.0",
            description="MID360 packet publication frequency in Hz.",
        ),
        DeclareLaunchArgument(
            "lio_input_mode",
            default_value="native_custom",
            description=(
                "Dual-driver LIO input timing source. native_custom preserves "
                "the driver's uint32 per-point offsets; reconstructed_custom "
                "is retained only for controlled A/B."
            ),
        ),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument("use_map_odom_stub", default_value="true"),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument(
            "use_mapping",
            default_value="false",
            description=(
                "Start the generic map exporter and OctoMap projection. "
                "Mutually exclusive with Nav2 and serial modes."
            ),
        ),
        DeclareLaunchArgument("use_serial_dry_run", default_value="false"),
        DeclareLaunchArgument("use_real_serial", default_value="false"),
        DeclareLaunchArgument(
            "serial_protocol_profile",
            default_value="legacy_v1_no_crc",
            choices=["legacy_v1_no_crc", "hpm_crc_v1"],
        ),
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("serial_max_vx", default_value="0.50"),
        DeclareLaunchArgument("serial_max_vy", default_value="0.50"),
        DeclareLaunchArgument("serial_max_wz", default_value="1.20"),
        DeclareLaunchArgument(
            "serial_cmd_vel_topic",
            default_value="/cmd_vel",
            description=(
                "Final velocity topic consumed by dry-run or real serial. "
                "Override only when an explicit single-owner safety gate is active."
            ),
        ),
        DeclareLaunchArgument("serial_referee_rx_enabled", default_value="false"),
        DeclareLaunchArgument(
            "serial_referee_raw_topic", default_value="/referee/state_raw"
        ),
        DeclareLaunchArgument("update_method", default_value="bundle"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        DeclareLaunchArgument(
            "navigate_to_pose_action",
            default_value="/navigate_to_pose",
            description=(
                "Action name implemented by Nav2 bt_navigator. Override only "
                "when a public semantic-route proxy owns /navigate_to_pose."
            ),
        ),
        DeclareLaunchArgument("goal_pose_topic", default_value="/goal_pose"),
        DeclareLaunchArgument(
            "pointcloud_filter_enabled",
            default_value="true",
            description=(
                "Filter old-car self/near-field returns before Nav2 costmap. "
                "Set false for pass-through comparison."
            ),
        ),
        DeclareLaunchArgument(
            "pointcloud_filter_input_topic",
            default_value="/livox/left/pointcloud",
        ),
        DeclareLaunchArgument(
            "pointcloud_filter_output_topic",
            default_value="/livox/left/pointcloud_filtered",
        ),
        DeclareLaunchArgument(
            "local_scan_enabled",
            default_value="false",
            description=(
                "Project filtered old-car PointCloud2 to /local_scan for "
                "LaserScan-based local costmap clearing experiments."
            ),
        ),
        DeclareLaunchArgument(
            "local_scan_input_topic",
            default_value="/livox/left/pointcloud_filtered",
        ),
        DeclareLaunchArgument(
            "local_scan_output_topic",
            default_value="/local_scan",
        ),
        DeclareLaunchArgument(
            "mapping_output_root", default_value="/data/rm27_maps"
        ),
        DeclareLaunchArgument("mapping_map_id", default_value="old_car_field"),
        DeclareLaunchArgument("mapping_revision", default_value="auto"),
        DeclareLaunchArgument(
            "mapping_record_ray_observations",
            default_value="false",
            description=(
                "Opt in to bounded offline ray evidence from the verified left "
                "MID360. It remains disabled by default."
            ),
        ),
        DeclareLaunchArgument(
            "mapping_ray_sample_period_sec",
            default_value="0.50",
            description=(
                "Old-car ray evidence sampling period. Tune only after the "
                "two-minute capacity preflight."
            ),
        ),
        DeclareLaunchArgument("mapping_ray_min_range", default_value="0.30"),
        DeclareLaunchArgument("mapping_ray_max_range", default_value="12.0"),
        DeclareLaunchArgument("mapping_ray_voxel_size", default_value="0.10"),
        DeclareLaunchArgument("mapping_ray_max_frames", default_value="10000"),
        DeclareLaunchArgument(
            "mapping_ray_max_rays_per_frame", default_value="10000"
        ),
        DeclareLaunchArgument(
            "mapping_ray_max_total_rays", default_value="10000000"
        ),
        DeclareLaunchArgument(
            "mapping_ray_max_bytes", default_value="536870912"
        ),
        DeclareLaunchArgument(
            "publish_transformed_registered_cloud",
            default_value="false",
            description=(
                "Publish the old-car registered cloud in the corrected odom "
                "basis for mapping or 3D relocalization."
            ),
        ),
        LogInfo(msg=(
            "[old_car_2026_validation] Experiment-only bringup for the 2026 "
            "car. Safe defaults start no real driver, no FAST-LIO, no Nav2, "
            "and no serial transport."
        )),
        OpaqueFunction(function=_validate_runtime_modes),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(old_description_launch),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_driver_launch),
            condition=IfCondition(PythonExpression([
                "'", driver_mode, "' == 'single'",
            ])),
            launch_arguments={
                "use_driver": use_driver,
                "side": selected_side,
                "publish_freq": driver_publish_freq,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_driver_launch),
            condition=IfCondition(PythonExpression([
                "'", driver_mode, "' == 'dual'",
            ])),
            launch_arguments={
                "use_driver": use_driver,
                "lio_imu_source": selected_side,
                "publish_freq": driver_publish_freq,
                "lio_input_mode": lio_input_mode,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lio_backend_launch),
            launch_arguments={
                "use_backend": use_lio_backend,
                "sensor_mode": "single",
                "selected_side": selected_side,
                "update_method": update_method,
                "config_file": old_fast_lio_config,
                "scan_publish_en": registered_cloud_enabled,
                "publish_tf_results": registered_cloud_enabled,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        Node(
            condition=IfCondition(use_driver),
            package="rm_mid360_driver_bridge",
            executable="pointcloud_self_filter_node",
            name="old_car_left_pointcloud_filter",
            output="screen",
            parameters=[
                old_car_pointcloud_filter_config,
                {
                    "input_topic": pointcloud_filter_input_topic,
                    "output_topic": pointcloud_filter_output_topic,
                    "filter_enabled": ParameterValue(
                        pointcloud_filter_enabled,
                        value_type=bool,
                    ),
                },
            ],
        ),
        Node(
            condition=IfCondition(local_scan_enabled),
            package="rm_mid360_driver_bridge",
            executable="pointcloud_to_laserscan_node",
            name="old_car_left_pointcloud_to_laserscan",
            output="screen",
            parameters=[
                old_car_pointcloud_to_scan_config,
                {
                    "input_topic": local_scan_input_topic,
                    "output_topic": local_scan_output_topic,
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "raw_odom_topic": "/odometry/fast_lio_raw",
                "lio_adapter_config": old_lio_adapter_config,
                "use_map_odom_stub": use_map_odom_stub,
            }.items(),
        ),
        GroupAction(
            condition=IfCondition(use_nav2),
            scoped=True,
            actions=[
                # Nav2 Humble remaps controller output through velocity_smoother,
                # but behavior_server otherwise publishes recovery commands
                # directly on /cmd_vel. Keep every Nav2 motion producer behind
                # the same smoothed old-car serial command path.
                SetRemap(src="cmd_vel", dst="cmd_vel_nav"),
                SetRemap(
                    src="/navigate_to_pose",
                    dst=navigate_to_pose_action,
                ),
                # Humble's BtActionServer constructs the action subchannels
                # after node argument parsing, so remap all five explicitly.
                SetRemap(
                    src="/navigate_to_pose/_action/send_goal",
                    dst=[
                        navigate_to_pose_action,
                        "/_action/send_goal",
                    ],
                ),
                SetRemap(
                    src="/navigate_to_pose/_action/get_result",
                    dst=[
                        navigate_to_pose_action,
                        "/_action/get_result",
                    ],
                ),
                SetRemap(
                    src="/navigate_to_pose/_action/cancel_goal",
                    dst=[
                        navigate_to_pose_action,
                        "/_action/cancel_goal",
                    ],
                ),
                SetRemap(
                    src="/navigate_to_pose/_action/feedback",
                    dst=[
                        navigate_to_pose_action,
                        "/_action/feedback",
                    ],
                ),
                SetRemap(
                    src="/navigate_to_pose/_action/status",
                    dst=[
                        navigate_to_pose_action,
                        "/_action/status",
                    ],
                ),
                SetRemap(src="/goal_pose", dst=goal_pose_topic),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "params_file": nav2_params,
                        "autostart": "true",
                    }.items(),
                ),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            condition=IfCondition(use_mapping),
            launch_arguments={
                "enable_mapping": "true",
                "pointcloud_topic": pointcloud_filter_output_topic,
                "registered_cloud_topic": "/lio/cloud_registered_transformed",
                "record_ray_observations": mapping_record_ray_observations,
                "ray_source_frame": "mid360_left_frame",
                "ray_sample_period_sec": mapping_ray_sample_period_sec,
                "ray_min_range": mapping_ray_min_range,
                "ray_max_range": mapping_ray_max_range,
                "ray_voxel_size": mapping_ray_voxel_size,
                "ray_max_frames": mapping_ray_max_frames,
                "ray_max_rays_per_frame": mapping_ray_max_rays_per_frame,
                "ray_max_total_rays": mapping_ray_max_total_rays,
                "ray_max_bytes": mapping_ray_max_bytes,
                "occupancy_topic": "/mapping/projected_map",
                "map_frame": "map",
                "base_frame": "base_link",
                "allow_latest_transform_fallback": "true",
                "output_root": mapping_output_root,
                "map_id": mapping_map_id,
                "revision": mapping_revision,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(serial_dry_run_launch),
            condition=IfCondition(use_serial_dry_run),
            launch_arguments={
                "protocol_profile": serial_protocol_profile,
                "cmd_vel_topic": serial_cmd_vel_topic,
                "mock_tx_topic": "/serial/mock_tx",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(serial_transport_launch),
            condition=IfCondition(use_real_serial),
            launch_arguments={
                "protocol_profile": serial_protocol_profile,
                "device": serial_device,
                "baudrate": serial_baudrate,
                "cmd_vel_topic": serial_cmd_vel_topic,
                "max_vx": serial_max_vx,
                "max_vy": serial_max_vy,
                "max_wz": serial_max_wz,
                "referee_rx_enabled": serial_referee_rx_enabled,
                "referee_raw_topic": serial_referee_raw_topic,
            }.items(),
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
