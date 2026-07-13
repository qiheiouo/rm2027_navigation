from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _as_bool(value: str) -> bool:
    return value.strip().lower() in TRUE_VALUES


def _start_map_server(context, *args, **kwargs):
    from rm_map_tools import MapBundleError, resolve_map_bundle_for_runtime

    manifest = LaunchConfiguration("map_bundle_manifest").perform(context).strip()
    allow_test_map = _as_bool(LaunchConfiguration("allow_test_map").perform(context))
    acceptance_policy = LaunchConfiguration("map_acceptance_policy").perform(context)
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    autostart = _as_bool(LaunchConfiguration("autostart").perform(context))

    if not manifest:
        raise RuntimeError("map_bundle_manifest must be set when map deployment is enabled")

    try:
        bundle = resolve_map_bundle_for_runtime(
            manifest,
            allow_test_map=allow_test_map,
            acceptance_policy=acceptance_policy,
        )
    except MapBundleError as exc:
        raise RuntimeError(f"map bundle rejected for runtime: {exc}") from exc

    return [
        LogInfo(msg=[
            "[map_deployment] map_id=",
            bundle["map_id"],
            " revision=",
            bundle["revision"],
            " status=",
            bundle["deployment_status"],
            " acceptance_policy=",
            bundle["acceptance_policy"],
            " map_type=",
            bundle["map_type"],
            " occupancy=",
            bundle["occupancy_yaml_path"],
            " pcd=",
            bundle["pcd_path"] or "<none>",
        ]),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "yaml_filename": bundle["occupancy_yaml_path"],
                "frame_id": "map",
                "topic_name": "map",
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
            }],
        ),
    ]


def generate_launch_description():
    default_manifest = PathJoinSubstitution([
        FindPackageShare("rm_map_tools"),
        "maps",
        "phase2e_test",
        "phase2e_test.bundle.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_bundle_manifest",
            default_value=default_manifest,
            description="Path to an rm_map_tools *.bundle.yaml manifest.",
        ),
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
                "approved_only is the deployment default; allow_candidate is "
                "for explicit field experiments; allow_test also accepts fixtures."
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        OpaqueFunction(function=_start_map_server),
    ])
