import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


TRUE_VALUES = {"1", "true", "yes", "on"}
EXECUTABLES = {
    "bundle": "laserMapping_bundle",
    "async": "laserMapping_async",
    "adaptive": "laserMapping_adaptive",
}


def _launch_setup(context, *args, **kwargs):
    use_backend = (
        LaunchConfiguration("use_backend").perform(context).strip().lower()
        in TRUE_VALUES
    )
    sensor_mode = LaunchConfiguration("sensor_mode").perform(context).strip().lower()
    selected_side = LaunchConfiguration("selected_side").perform(context).strip().lower()
    update_method = LaunchConfiguration("update_method").perform(context).strip().lower()

    actions = [
        LogInfo(
            msg=[
                "FAST-LIO boundary: /Odometry -> /odometry/fast_lio_raw; ",
                "backend TF is quarantined; lio_adapter alone owns canonical ",
                "odom -> base_link."
            ]
        )
    ]

    if not use_backend:
        actions.append(
            LogInfo(
                msg=(
                    "use_backend:=false, so fast_lio_multi is not launched. "
                    "This is the safe default for build-only validation."
                )
            )
        )
        return actions

    if sensor_mode not in {"single", "dual"}:
        raise RuntimeError("sensor_mode must be 'single' or 'dual'")
    if selected_side not in {"left", "right"}:
        raise RuntimeError("selected_side must be 'left' or 'right'")
    if update_method not in EXECUTABLES:
        raise RuntimeError("update_method must be bundle, async, or adaptive")

    config_override = LaunchConfiguration("config_file").perform(context).strip()
    if config_override:
        config_path = config_override
    else:
        config_name = f"fast_lio_multi_{sensor_mode}_mid360.yaml"
        config_path = os.path.join(
            get_package_share_directory("rm_lio_bringup"),
            "config",
            config_name,
        )

    parameter_overrides = {
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        ),
        "common.publish_tf_results": ParameterValue(
            LaunchConfiguration("publish_tf_results"), value_type=bool
        ),
        "publish.scan_publish_en": ParameterValue(
            LaunchConfiguration("scan_publish_en"), value_type=bool
        ),
    }
    if sensor_mode == "single":
        parameter_overrides["common.lid_topic"] = f"/livox/{selected_side}/lidar"

    actions.append(
        Node(
            package="fast_lio_multi",
            executable=EXECUTABLES[update_method],
            name="fast_lio_multi_backend",
            output="screen",
            parameters=[config_path, parameter_overrides],
            remappings=[
                ("/Odometry", LaunchConfiguration("raw_odom_topic")),
                ("/tf", LaunchConfiguration("quarantine_tf_topic")),
                ("/tf_static", LaunchConfiguration("quarantine_tf_static_topic")),
                ("/cloud_registered", "/lio/cloud_registered"),
                ("/cloud_registered_tf", "/lio/cloud_registered_transformed"),
                ("/cloud_registered_body", "/lio/cloud_registered_body"),
                ("/Laser_map", "/lio/map"),
                ("/path", "/lio/path"),
                ("/mavros/vision_pose/pose", "/lio/backend_pose"),
                ("/calc_time", "/lio/diagnostics/calc_time"),
                ("/point_number", "/lio/diagnostics/point_number"),
                ("/localizability_x", "/lio/diagnostics/localizability_x"),
                ("/localizability_y", "/lio/diagnostics/localizability_y"),
                ("/localizability_z", "/lio/diagnostics/localizability_z"),
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_backend", default_value="false"),
        DeclareLaunchArgument("sensor_mode", default_value="single"),
        DeclareLaunchArgument("selected_side", default_value="left"),
        DeclareLaunchArgument("update_method", default_value="bundle"),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("raw_odom_topic", default_value="/odometry/fast_lio_raw"),
        DeclareLaunchArgument(
            "quarantine_tf_topic", default_value="/fast_lio/_quarantine/tf"
        ),
        DeclareLaunchArgument(
            "quarantine_tf_static_topic",
            default_value="/fast_lio/_quarantine/tf_static",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_tf_results", default_value="false"),
        DeclareLaunchArgument(
            "scan_publish_en",
            default_value="false",
            description=(
                "Publish world-registered scans. Keep disabled for normal "
                "navigation and enable explicitly for managed mapping."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
