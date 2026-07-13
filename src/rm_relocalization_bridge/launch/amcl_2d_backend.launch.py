from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _launch_backend(context, *args, **kwargs):
    del args, kwargs
    enabled = (
        LaunchConfiguration("enable_backend").perform(context).strip().lower()
        in TRUE_VALUES
    )
    if not enabled:
        return [LogInfo(msg="[amcl_2d] backend disabled; no localization node started.")]

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    if not params_file or not Path(params_file).is_file():
        raise RuntimeError(
            f"amcl_2d params_file must be a regular file, got: {params_file!r}"
        )
    scan_topic = LaunchConfiguration("scan_topic")
    map_topic = LaunchConfiguration("map_topic")
    raw_pose_topic = LaunchConfiguration("raw_pose_topic")
    global_pose_topic = LaunchConfiguration("global_pose_topic")
    actions = [
        LogInfo(msg=[
            "[amcl_2d] AMCL publishes no TF. Raw poses pass through ",
            raw_pose_topic,
            " before reaching ",
            global_pose_topic,
            ".",
        ]),
    ]

    use_pointcloud_to_scan = (
        LaunchConfiguration("use_pointcloud_to_scan").perform(context).strip().lower()
        in TRUE_VALUES
    )
    if use_pointcloud_to_scan:
        projection_params = (
            LaunchConfiguration("scan_projection_params").perform(context).strip()
        )
        if not projection_params or not Path(projection_params).is_file():
            raise RuntimeError(
                "amcl_2d scan_projection_params must be a regular file, "
                f"got: {projection_params!r}"
            )
        actions.append(Node(
            package="rm_mid360_driver_bridge",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan_node",
            output="screen",
            parameters=[
                projection_params,
                {
                    "input_topic": LaunchConfiguration("pointcloud_topic"),
                    "output_topic": scan_topic,
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ))

    actions.extend([
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "scan_topic": scan_topic,
                    "map_topic": map_topic,
                    "set_initial_pose": ParameterValue(
                        LaunchConfiguration("set_initial_pose"), value_type=bool
                    ),
                    "initial_pose.x": ParameterValue(
                        LaunchConfiguration("initial_pose_x"), value_type=float
                    ),
                    "initial_pose.y": ParameterValue(
                        LaunchConfiguration("initial_pose_y"), value_type=float
                    ),
                    "initial_pose.yaw": ParameterValue(
                        LaunchConfiguration("initial_pose_yaw"), value_type=float
                    ),
                    "update_min_d": ParameterValue(
                        LaunchConfiguration("update_min_d"), value_type=float
                    ),
                    "update_min_a": ParameterValue(
                        LaunchConfiguration("update_min_a"), value_type=float
                    ),
                    "tf_broadcast": False,
                },
            ],
            remappings=[("amcl_pose", raw_pose_topic)],
        ),
        Node(
            package="rm_relocalization_bridge",
            executable="global_pose_gate_node",
            name="amcl_pose_gate",
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
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_amcl_2d",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart"), value_type=bool
                ),
                "node_names": ["amcl"],
            }],
        ),
    ])
    return actions


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "amcl_2d.yaml",
    ])
    default_projection = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "pointcloud_to_scan_2d.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("enable_backend", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("scan_projection_params", default_value=default_projection),
        DeclareLaunchArgument("use_pointcloud_to_scan", default_value="false"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/points/obstacles"),
        DeclareLaunchArgument("scan_topic", default_value="/localization/scan"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument(
            "raw_pose_topic", default_value="/localization/amcl_pose_raw"
        ),
        DeclareLaunchArgument(
            "global_pose_topic", default_value="/localization/global_pose"
        ),
        DeclareLaunchArgument("set_initial_pose", default_value="false"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument("update_min_d", default_value="0.05"),
        DeclareLaunchArgument("update_min_a", default_value="0.05"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_launch_backend),
    ])
