from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _as_bool(value: str) -> bool:
    return value.strip().lower() in TRUE_VALUES


def _launch_backend(context, *args, **kwargs):
    del args, kwargs
    if not _as_bool(LaunchConfiguration("enable_backend").perform(context)):
        return [LogInfo(msg="[gicp_3d] backend disabled; no node started.")]

    from rm_map_tools import MapBundleError, resolve_map_bundle_for_runtime

    manifest = LaunchConfiguration("map_bundle_manifest").perform(context).strip()
    allow_test_map = _as_bool(
        LaunchConfiguration("allow_test_map").perform(context)
    )
    if not manifest:
        raise RuntimeError("gicp_3d requires map_bundle_manifest")
    try:
        bundle = resolve_map_bundle_for_runtime(
            manifest, allow_test_map=allow_test_map
        )
    except MapBundleError as exc:
        raise RuntimeError(f"GICP map bundle rejected: {exc}") from exc
    if not bundle["pcd_path"]:
        raise RuntimeError(
            "gicp_3d requires map_type:=occupancy_with_pcd; "
            "occupancy_only bundles are valid only for 2D localization"
        )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    raw_pose_topic = LaunchConfiguration("raw_pose_topic")
    global_pose_topic = LaunchConfiguration("global_pose_topic")
    return [
        LogInfo(msg=[
            "[gicp_3d] map_id=", bundle["map_id"],
            " revision=", bundle["revision"],
            " pcd=", bundle["pcd_path"],
            ". Backend publishes no TF.",
        ]),
        Node(
            package="rm_gicp_relocalization",
            executable="gicp_relocalization_node",
            name="gicp_relocalization",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "prior_pcd_file": bundle["pcd_path"],
                    "map_id": bundle["map_id"],
                    "input_cloud_topic": LaunchConfiguration("input_cloud_topic"),
                    "raw_pose_topic": raw_pose_topic,
                },
            ],
        ),
        Node(
            package="rm_relocalization_bridge",
            executable="global_pose_gate_node",
            name="gicp_pose_gate",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "input_topic": raw_pose_topic,
                    "output_topic": global_pose_topic,
                },
            ],
        ),
    ]


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("rm_gicp_relocalization"),
        "config",
        "gicp_3d.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("enable_backend", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("map_bundle_manifest", default_value=""),
        DeclareLaunchArgument("allow_test_map", default_value="false"),
        DeclareLaunchArgument(
            "input_cloud_topic", default_value="/lio/cloud_registered"
        ),
        DeclareLaunchArgument(
            "raw_pose_topic", default_value="/localization/gicp_pose_raw"
        ),
        DeclareLaunchArgument(
            "global_pose_topic", default_value="/localization/global_pose"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_launch_backend),
    ])
