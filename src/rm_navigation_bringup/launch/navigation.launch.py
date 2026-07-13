from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _validate_localization_backend(context, *args, **kwargs):
    del args, kwargs
    backend = LaunchConfiguration("relocalization_backend").perform(context)
    if backend == "none":
        return []
    localization_mode = LaunchConfiguration("global_localization_mode").perform(context)
    use_map_server = (
        LaunchConfiguration("use_map_server").perform(context).strip().lower()
        in TRUE_VALUES
    )
    if localization_mode != "external_pose":
        raise RuntimeError(
            "a relocalization backend requires "
            "global_localization_mode:=external_pose"
        )
    if not use_map_server:
        raise RuntimeError(
            "a relocalization backend requires use_map_server:=true"
        )
    return []


def generate_launch_description():
    sensor_mode = LaunchConfiguration("sensor_mode")
    selected_side = LaunchConfiguration("selected_side")
    global_localization_mode = LaunchConfiguration("global_localization_mode")
    relocalization_backend = LaunchConfiguration("relocalization_backend")
    use_driver = LaunchConfiguration("use_driver")
    use_lio_backend = LaunchConfiguration("use_lio_backend")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_chassis_stub = LaunchConfiguration("use_chassis_stub")
    use_map_server = LaunchConfiguration("use_map_server")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    update_method = LaunchConfiguration("update_method")
    nav2_params = LaunchConfiguration("nav2_params")
    map_bundle_manifest = LaunchConfiguration("map_bundle_manifest")
    allow_test_map = LaunchConfiguration("allow_test_map")
    map_acceptance_policy = LaunchConfiguration("map_acceptance_policy")
    rviz_config = LaunchConfiguration("rviz_config")
    relocalization_params = LaunchConfiguration("relocalization_params")
    gicp_relocalization_params = LaunchConfiguration("gicp_relocalization_params")
    relocalization_scan_topic = LaunchConfiguration("relocalization_scan_topic")
    relocalization_pointcloud_topic = LaunchConfiguration(
        "relocalization_pointcloud_topic"
    )
    use_relocalization_pointcloud_projection = LaunchConfiguration(
        "use_relocalization_pointcloud_projection"
    )
    gicp_input_cloud_topic = LaunchConfiguration("gicp_input_cloud_topic")

    localization_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "phase2c_relocalization_bringup.launch.py",
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    ])
    map_deployment_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "map_deployment.launch.py",
    ])
    amcl_2d_launch = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "launch",
        "amcl_2d_backend.launch.py",
    ])
    gicp_3d_launch = PathJoinSubstitution([
        FindPackageShare("rm_gicp_relocalization"),
        "launch",
        "gicp_3d_backend.launch.py",
    ])
    chassis_launch = PathJoinSubstitution([
        FindPackageShare("rm_chassis_interface"),
        "launch",
        "chassis_interface_stub.launch.py",
    ])
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_phase2f_deployment.yaml",
    ])
    default_map_bundle = PathJoinSubstitution([
        FindPackageShare("rm_map_tools"),
        "maps",
        "phase2e_test",
        "phase2e_test.bundle.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "rviz",
        "phase1.rviz",
    ])
    default_relocalization_params = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "amcl_2d.yaml",
    ])
    default_gicp_relocalization_params = PathJoinSubstitution([
        FindPackageShare("rm_gicp_relocalization"),
        "config",
        "gicp_3d.yaml",
    ])
    amcl_2d_enabled = PythonExpression([
        "'", relocalization_backend, "' == 'amcl_2d'",
    ])
    gicp_3d_enabled = PythonExpression([
        "'", relocalization_backend, "' == 'gicp_3d'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "sensor_mode",
            default_value="single",
            choices=["single", "dual"],
            description="Single MID360 is the safe default until dual-sensor calibration.",
        ),
        DeclareLaunchArgument(
            "selected_side",
            default_value="left",
            choices=["left", "right"],
        ),
        DeclareLaunchArgument(
            "global_localization_mode",
            default_value="stub",
            choices=["stub", "external_pose"],
            description="Select exactly one map->odom owner.",
        ),
        DeclareLaunchArgument(
            "relocalization_backend",
            default_value="none",
            choices=["none", "amcl_2d", "gicp_3d"],
            description=(
                "Selected upstream global-pose backend. It never owns canonical TF."
            ),
        ),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument(
            "use_nav2",
            default_value="false",
            description=(
                "Disabled by default until the real obstacle input and deployment "
                "map profile are selected."
            ),
        ),
        DeclareLaunchArgument("use_chassis_stub", default_value="true"),
        DeclareLaunchArgument(
            "use_map_server",
            default_value="false",
            description=(
                "Start nav2_map_server from a validated map bundle. Disabled "
                "by default so the safe bringup does not require a deployment map."
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "update_method",
            default_value="bundle",
            choices=["bundle", "async", "adaptive"],
        ),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        DeclareLaunchArgument("map_bundle_manifest", default_value=default_map_bundle),
        DeclareLaunchArgument(
            "allow_test_map",
            default_value="false",
            description=(
                "Deprecated compatibility alias for "
                "map_acceptance_policy:=allow_test."
            ),
        ),
        DeclareLaunchArgument(
            "map_acceptance_policy",
            default_value="approved_only",
            choices=["approved_only", "allow_candidate", "allow_test"],
            description=(
                "Explicit map asset gate. Candidate maps are allowed only when "
                "this argument is set to allow_candidate."
            ),
        ),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument(
            "relocalization_params", default_value=default_relocalization_params
        ),
        DeclareLaunchArgument(
            "gicp_relocalization_params",
            default_value=default_gicp_relocalization_params,
        ),
        DeclareLaunchArgument(
            "relocalization_scan_topic", default_value="/localization/scan"
        ),
        DeclareLaunchArgument(
            "relocalization_pointcloud_topic", default_value="/points/obstacles"
        ),
        DeclareLaunchArgument(
            "use_relocalization_pointcloud_projection", default_value="false"
        ),
        DeclareLaunchArgument(
            "gicp_input_cloud_topic", default_value="/lio/cloud_registered"
        ),
        OpaqueFunction(function=_validate_localization_backend),
        LogInfo(msg=[
            "[navigation] Canonical stack entry. Hardware, FAST-LIO, and Nav2 ",
            "remain explicit opt-ins. global_localization_mode=",
            global_localization_mode,
            ", sensor_mode=",
            sensor_mode,
            ", relocalization_backend=",
            relocalization_backend,
            ".",
        ]),
        LogInfo(msg=(
            "[navigation] The default Nav2 file is the simulation-proven MPPI "
            "baseline, not a real-robot deployment acceptance. Real 3D obstacle "
            "input and an approved map bundle are still required."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "sensor_mode": sensor_mode,
                "selected_side": selected_side,
                "global_localization_mode": global_localization_mode,
                "use_driver": use_driver,
                "use_lio_backend": use_lio_backend,
                "update_method": update_method,
                "use_rviz": "false",
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(chassis_launch),
            condition=IfCondition(use_chassis_stub),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_deployment_launch),
            condition=IfCondition(use_map_server),
            launch_arguments={
                "map_bundle_manifest": map_bundle_manifest,
                "allow_test_map": allow_test_map,
                "map_acceptance_policy": map_acceptance_policy,
                "use_sim_time": use_sim_time,
                "autostart": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amcl_2d_launch),
            condition=IfCondition(amcl_2d_enabled),
            launch_arguments={
                "enable_backend": "true",
                "params_file": relocalization_params,
                "scan_topic": relocalization_scan_topic,
                "map_topic": "/map",
                "use_pointcloud_to_scan": use_relocalization_pointcloud_projection,
                "pointcloud_topic": relocalization_pointcloud_topic,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gicp_3d_launch),
            condition=IfCondition(gicp_3d_enabled),
            launch_arguments={
                "enable_backend": "true",
                "params_file": gicp_relocalization_params,
                "map_bundle_manifest": map_bundle_manifest,
                "allow_test_map": allow_test_map,
                "map_acceptance_policy": map_acceptance_policy,
                "input_cloud_topic": gicp_input_cloud_topic,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(use_nav2),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
                "autostart": "true",
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
