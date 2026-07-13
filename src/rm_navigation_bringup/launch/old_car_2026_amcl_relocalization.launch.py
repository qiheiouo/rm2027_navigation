from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _as_bool(value: str) -> bool:
    return value.strip().lower() in TRUE_VALUES


def _validate_runtime(context, *args, **kwargs):
    del args, kwargs
    enabled = _as_bool(
        LaunchConfiguration("enable_relocalization").perform(context)
    )
    if not enabled:
        return [LogInfo(msg="[old_car_amcl] disabled; no map or AMCL backend started.")]

    manifest = LaunchConfiguration("map_bundle_manifest").perform(context).strip()
    if not manifest:
        raise RuntimeError(
            "old-car AMCL relocalization requires map_bundle_manifest"
        )
    side = LaunchConfiguration("selected_side").perform(context).strip().lower()
    if side != "left":
        raise RuntimeError(
            "old-car AMCL projection currently requires selected_side:=left"
        )
    use_driver = _as_bool(LaunchConfiguration("use_driver").perform(context))
    use_lio_backend = _as_bool(
        LaunchConfiguration("use_lio_backend").perform(context)
    )
    if use_driver and not use_lio_backend:
        raise RuntimeError(
            "old-car AMCL with the real driver requires use_lio_backend:=true"
        )
    policy = LaunchConfiguration("map_acceptance_policy").perform(context)
    if use_driver and policy == "allow_test":
        raise RuntimeError(
            "test_only map assets are forbidden when the real old-car driver is enabled"
        )
    return [
        LogInfo(msg=[
            "[old_car_amcl] no-motion profile enabled; map policy=",
            policy,
            ". Nav2 and both serial modes remain disabled.",
        ])
    ]


def generate_launch_description():
    enabled = LaunchConfiguration("enable_relocalization")
    use_driver = LaunchConfiguration("use_driver")
    use_lio_backend = LaunchConfiguration("use_lio_backend")
    selected_side = LaunchConfiguration("selected_side")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_bundle_manifest = LaunchConfiguration("map_bundle_manifest")
    map_acceptance_policy = LaunchConfiguration("map_acceptance_policy")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")

    old_car_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "old_car_2026_validation.launch.py",
    ])
    map_deployment_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "map_deployment.launch.py",
    ])
    global_pose_bridge_launch = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "launch",
        "global_pose_bridge.launch.py",
    ])
    amcl_launch = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "launch",
        "amcl_2d_backend.launch.py",
    ])

    stub_enabled = PythonExpression([
        "'", enabled, "'.lower() not in ['1', 'true', 'yes', 'on']",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("enable_relocalization", default_value="false"),
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("use_lio_backend", default_value="false"),
        DeclareLaunchArgument(
            "selected_side", default_value="left", choices=["left", "right"]
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_bundle_manifest", default_value=""),
        DeclareLaunchArgument(
            "map_acceptance_policy",
            default_value="approved_only",
            choices=["approved_only", "allow_candidate", "allow_test"],
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/livox/left/pointcloud_filtered",
        ),
        DeclareLaunchArgument("set_initial_pose", default_value="false"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        OpaqueFunction(function=_validate_runtime),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(old_car_launch),
            launch_arguments={
                "selected_side": selected_side,
                "use_driver": use_driver,
                "use_lio_backend": use_lio_backend,
                "use_map_odom_stub": stub_enabled,
                "use_nav2": "false",
                "use_mapping": "false",
                "use_serial_dry_run": "false",
                "use_real_serial": "false",
                "use_rviz": use_rviz,
                "use_sim_time": use_sim_time,
                "pointcloud_filter_enabled": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_deployment_launch),
            condition=IfCondition(enabled),
            launch_arguments={
                "map_bundle_manifest": map_bundle_manifest,
                "map_acceptance_policy": map_acceptance_policy,
                "use_sim_time": use_sim_time,
                "autostart": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(global_pose_bridge_launch),
            condition=IfCondition(enabled),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amcl_launch),
            condition=IfCondition(enabled),
            launch_arguments={
                "enable_backend": "true",
                "use_pointcloud_to_scan": "true",
                "pointcloud_topic": pointcloud_topic,
                "scan_topic": "/localization/scan",
                "map_topic": "/map",
                "set_initial_pose": LaunchConfiguration("set_initial_pose"),
                "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                "use_sim_time": use_sim_time,
            }.items(),
        ),
    ])
