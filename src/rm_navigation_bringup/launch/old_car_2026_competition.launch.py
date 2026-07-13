from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in TRUE_VALUES


def _validate_competition_profile(context, *args, **kwargs):
    del args, kwargs
    enabled = _as_bool(context, "enable_competition_stack")
    if not enabled:
        return []

    use_driver = _as_bool(context, "use_driver")
    use_lio = _as_bool(context, "use_lio_backend")
    use_nav2 = _as_bool(context, "use_nav2")
    use_real_serial = _as_bool(context, "use_real_serial")
    use_map_server = _as_bool(context, "use_map_server")
    use_mission = _as_bool(context, "use_mission")
    use_referee = _as_bool(context, "use_referee_interface")
    use_referee_mock = _as_bool(context, "use_referee_mock")
    use_target_mock = _as_bool(context, "use_target_mock")
    use_safety_mock = _as_bool(context, "use_mission_safety_mock")
    use_chassis_mode = _as_bool(context, "use_chassis_mode_interface")
    startup_enabled = _as_bool(context, "mission_startup_enabled")
    use_dual = _as_bool(context, "use_dual_obstacle_fusion")
    use_right_driver = _as_bool(context, "use_right_driver")
    allow_provisional_dual = _as_bool(context, "allow_provisional_dual_extrinsic")
    backend = LaunchConfiguration("relocalization_backend").perform(context)
    policy = LaunchConfiguration("map_acceptance_policy").perform(context)

    if use_driver and not use_lio:
        raise RuntimeError("competition hardware driver requires use_lio_backend:=true")
    if backend != "none" and not use_map_server:
        raise RuntimeError("AMCL/GICP relocalization requires use_map_server:=true")
    if use_nav2 and backend == "none":
        raise RuntimeError(
            "competition Nav2 requires amcl_2d or gicp_3d; identity map->odom is not accepted"
        )
    if use_real_serial and not use_nav2:
        raise RuntimeError("competition real serial requires use_nav2:=true")
    if use_real_serial and policy == "allow_test":
        raise RuntimeError("real serial must not run with synthetic test map assets")
    if use_mission and (not use_nav2 or backend == "none" or not use_referee):
        raise RuntimeError(
            "mission requires Nav2, a real relocalization backend, and referee interface"
        )
    if use_mission and not (use_chassis_mode or use_safety_mock):
        raise RuntimeError(
            "mission requires chassis authority input or explicit no-hardware safety mock"
        )
    if use_real_serial and (use_referee_mock or use_target_mock or use_safety_mock):
        raise RuntimeError("mock competition inputs are forbidden with real serial")
    if use_real_serial and startup_enabled:
        raise RuntimeError(
            "mission_startup_enabled must remain false with real serial; enable by service after checks"
        )
    if use_right_driver and not use_dual:
        raise RuntimeError("use_right_driver requires use_dual_obstacle_fusion:=true")
    if use_dual and use_real_serial and not allow_provisional_dual:
        raise RuntimeError(
            "old-car right-lidar extrinsic is provisional; real motion requires an explicit override"
        )
    return []


def generate_launch_description():
    enable_stack = LaunchConfiguration("enable_competition_stack")
    use_driver = LaunchConfiguration("use_driver")
    use_lio = LaunchConfiguration("use_lio_backend")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_real_serial = LaunchConfiguration("use_real_serial")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_map_server = LaunchConfiguration("use_map_server")
    use_referee = LaunchConfiguration("use_referee_interface")
    use_referee_mock = LaunchConfiguration("use_referee_mock")
    use_pursuit = LaunchConfiguration("use_pursuit")
    use_target_mock = LaunchConfiguration("use_target_mock")
    use_mission = LaunchConfiguration("use_mission")
    use_safety_mock = LaunchConfiguration("use_mission_safety_mock")
    use_chassis_mode = LaunchConfiguration("use_chassis_mode_interface")
    mission_startup_enabled = LaunchConfiguration("mission_startup_enabled")
    use_dual_fusion = LaunchConfiguration("use_dual_obstacle_fusion")
    use_right_driver = LaunchConfiguration("use_right_driver")
    backend = LaunchConfiguration("relocalization_backend")
    map_manifest = LaunchConfiguration("map_bundle_manifest")
    map_policy = LaunchConfiguration("map_acceptance_policy")
    nav2_params = LaunchConfiguration("nav2_params")
    mission_config = LaunchConfiguration("mission_config")
    serial_device = LaunchConfiguration("serial_device")
    serial_baudrate = LaunchConfiguration("serial_baudrate")

    share = FindPackageShare("rm_navigation_bringup")
    old_car_launch = PathJoinSubstitution([share, "launch", "old_car_2026_validation.launch.py"])
    map_launch = PathJoinSubstitution([share, "launch", "map_deployment.launch.py"])
    bridge_launch = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"), "launch", "global_pose_bridge.launch.py"
    ])
    amcl_launch = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"), "launch", "amcl_2d_backend.launch.py"
    ])
    gicp_launch = PathJoinSubstitution([
        FindPackageShare("rm_gicp_relocalization"), "launch", "gicp_3d_backend.launch.py"
    ])
    referee_launch = PathJoinSubstitution([
        FindPackageShare("rm_referee_interface"), "launch", "referee_interface.launch.py"
    ])
    pursuit_launch = PathJoinSubstitution([
        FindPackageShare("rm_pursuit"), "launch", "pursuit.launch.py"
    ])
    mission_launch = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"), "launch", "competition_mission.launch.py"
    ])
    chassis_mode_launch = PathJoinSubstitution([
        FindPackageShare("rm_chassis_interface"), "launch", "chassis_mode_gate.launch.py"
    ])
    right_driver_launch = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"), "launch", "single_mid360_driver.launch.py"
    ])
    dual_fusion_launch = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "launch",
        "dual_pointcloud_obstacle_fusion.launch.py",
    ])
    default_nav2 = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"), "config", "nav2_old_car_2026_left_stvl.yaml"
    ])
    default_map = PathJoinSubstitution([
        FindPackageShare("rm_map_tools"),
        "maps",
        "phase2e_test",
        "phase2e_test.bundle.yaml",
    ])
    default_mission = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"), "config", "mission_safe.yaml"
    ])

    external_localization = PythonExpression(["'", backend, "' != 'none'"])
    amcl_enabled = PythonExpression(["'", backend, "' == 'amcl_2d'"])
    gicp_enabled = PythonExpression(["'", backend, "' == 'gicp_3d'"])
    map_stub_enabled = PythonExpression(["'", backend, "' == 'none'"])

    return LaunchDescription([
        DeclareLaunchArgument("enable_competition_stack", default_value="false"),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("use_real_serial", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_map_server", default_value="false"),
        DeclareLaunchArgument(
            "relocalization_backend",
            default_value="none",
            choices=["none", "amcl_2d", "gicp_3d"],
        ),
        DeclareLaunchArgument("map_bundle_manifest", default_value=default_map),
        DeclareLaunchArgument(
            "map_acceptance_policy",
            default_value="approved_only",
            choices=["approved_only", "allow_candidate", "allow_test"],
        ),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2),
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("use_referee_interface", default_value="false"),
        DeclareLaunchArgument("use_referee_mock", default_value="false"),
        DeclareLaunchArgument("use_pursuit", default_value="false"),
        DeclareLaunchArgument("use_target_mock", default_value="false"),
        DeclareLaunchArgument("use_mission", default_value="false"),
        DeclareLaunchArgument("use_mission_safety_mock", default_value="false"),
        DeclareLaunchArgument("use_chassis_mode_interface", default_value="false"),
        DeclareLaunchArgument("mission_startup_enabled", default_value="false"),
        DeclareLaunchArgument("mission_config", default_value=default_mission),
        DeclareLaunchArgument("use_dual_obstacle_fusion", default_value="false"),
        DeclareLaunchArgument("use_right_driver", default_value="false"),
        DeclareLaunchArgument("allow_provisional_dual_extrinsic", default_value="false"),
        OpaqueFunction(function=_validate_competition_profile),
        LogInfo(msg=(
            "[competition] Safe default starts nothing. Hardware, map, localization, "
            "Nav2, serial, referee, pursuit and mission are independent opt-ins."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(old_car_launch),
            condition=IfCondition(enable_stack),
            launch_arguments={
                "selected_side": "left",
                "use_driver": use_driver,
                "use_lio_backend": use_lio,
                "use_map_odom_stub": map_stub_enabled,
                "use_nav2": use_nav2,
                "use_real_serial": use_real_serial,
                "use_serial_dry_run": "false",
                "use_rviz": use_rviz,
                "use_sim_time": use_sim_time,
                "nav2_params": nav2_params,
                "pointcloud_filter_enabled": "true",
                "publish_transformed_registered_cloud": gicp_enabled,
                "serial_protocol_profile": "legacy_v1_no_crc",
                "serial_device": serial_device,
                "serial_baudrate": serial_baudrate,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_launch),
            condition=IfCondition(PythonExpression([
                "'", enable_stack, "'.lower() in ['1','true','yes','on'] and '",
                use_map_server, "'.lower() in ['1','true','yes','on']",
            ])),
            launch_arguments={
                "map_bundle_manifest": map_manifest,
                "map_acceptance_policy": map_policy,
                "use_sim_time": use_sim_time,
                "autostart": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            condition=IfCondition(PythonExpression([
                "'", enable_stack, "'.lower() in ['1','true','yes','on'] and ",
                external_localization,
            ])),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amcl_launch),
            condition=IfCondition(PythonExpression([
                "'", enable_stack, "'.lower() in ['1','true','yes','on'] and ", amcl_enabled,
            ])),
            launch_arguments={
                "enable_backend": "true",
                "use_pointcloud_to_scan": "true",
                "pointcloud_topic": "/livox/left/pointcloud_filtered",
                "scan_topic": "/localization/scan",
                "map_topic": "/map",
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gicp_launch),
            condition=IfCondition(PythonExpression([
                "'", enable_stack, "'.lower() in ['1','true','yes','on'] and ", gicp_enabled,
            ])),
            launch_arguments={
                "enable_backend": "true",
                "map_bundle_manifest": map_manifest,
                "map_acceptance_policy": map_policy,
                "input_cloud_topic": "/lio/cloud_registered_transformed",
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(referee_launch),
            condition=IfCondition(enable_stack),
            launch_arguments={
                "enable_referee_interface": use_referee,
                "use_mock": use_referee_mock,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pursuit_launch),
            condition=IfCondition(enable_stack),
            launch_arguments={
                "enable_pursuit_boundary": use_pursuit,
                "use_mock_target": use_target_mock,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            condition=IfCondition(enable_stack),
            launch_arguments={
                "enable_mission_node": use_mission,
                "startup_enabled": mission_startup_enabled,
                "use_safety_mock": use_safety_mock,
                "use_sim_time": use_sim_time,
                "mission_config": mission_config,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(chassis_mode_launch),
            condition=IfCondition(enable_stack),
            launch_arguments={
                "enable_chassis_mode_gate": use_chassis_mode,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(right_driver_launch),
            condition=IfCondition(PythonExpression([
                "'", enable_stack, "'.lower() in ['1','true','yes','on'] and '",
                use_right_driver, "'.lower() in ['1','true','yes','on']",
            ])),
            launch_arguments={"use_driver": "true", "side": "right"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_fusion_launch),
            condition=IfCondition(PythonExpression([
                "'", enable_stack, "'.lower() in ['1','true','yes','on'] and '",
                use_dual_fusion, "'.lower() in ['1','true','yes','on']",
            ])),
            launch_arguments={
                "enable_fusion": "true",
                "left_filtered_topic": "/livox/left/pointcloud_filtered_fusion",
                "right_filtered_topic": "/livox/right/pointcloud_filtered_fusion",
                "output_topic": "/points/obstacles_fused",
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        Node(
            condition=IfCondition(enable_stack),
            package="rm_system_monitor",
            executable="readiness_monitor",
            name="readiness_monitor",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "require_lio": ParameterValue(use_lio, value_type=bool),
                "require_obstacle_input": ParameterValue(use_driver, value_type=bool),
                "require_localization": ParameterValue(
                    external_localization, value_type=bool
                ),
                "require_nav2": ParameterValue(use_nav2, value_type=bool),
                "require_referee": ParameterValue(use_mission, value_type=bool),
                "require_chassis_mode": ParameterValue(use_mission, value_type=bool),
                "require_serial_transport": ParameterValue(
                    use_real_serial, value_type=bool
                ),
            }],
        ),
    ])
