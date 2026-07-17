from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_bridge(context, *args, **kwargs):
    del args, kwargs
    config_file = LaunchConfiguration("config_file").perform(context).strip()
    if not config_file or not Path(config_file).is_file():
        raise RuntimeError(
            "global pose bridge config_file must be a regular file, "
            f"got: {config_file!r}"
        )
    return [Node(
        package="rm_relocalization_bridge",
        executable="map_odom_from_global_pose",
        name="map_odom_from_global_pose",
        output="screen",
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "upstream_valid_topic": LaunchConfiguration(
                    "upstream_valid_topic"
                ),
            },
        ],
    )]


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "map_odom_from_global_pose.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("upstream_valid_topic", default_value=""),
        DeclareLaunchArgument("config_file", default_value=default_config),
        OpaqueFunction(function=_launch_bridge),
    ])
