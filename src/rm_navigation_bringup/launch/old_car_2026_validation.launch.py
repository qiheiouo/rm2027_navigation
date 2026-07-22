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
    use_mapping = (
        LaunchConfiguration("use_mapping").perform(context).strip().lower()
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
    if use_serial_dry_run and use_real_serial:
        raise RuntimeError(
            "use_serial_dry_run and use_real_serial must not be true at the same time."
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
    return []


def generate_launch_description():
    selected_side = LaunchConfiguration("selected_side")
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
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    update_method = LaunchConfiguration("update_method")
    nav2_params = LaunchConfiguration("nav2_params")
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
        DeclareLaunchArgument("update_method", default_value="bundle"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
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
            launch_arguments={
                "use_driver": use_driver,
                "side": selected_side,
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
                "cmd_vel_topic": "/cmd_vel",
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
                "cmd_vel_topic": "/cmd_vel",
                "max_vx": serial_max_vx,
                "max_vy": serial_max_vy,
                "max_wz": serial_max_wz,
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
