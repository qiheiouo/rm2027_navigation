from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _launch_mission(context, *args, **kwargs):
    del args, kwargs
    if (
        LaunchConfiguration("enable_mission_node").perform(context).strip().lower()
        not in TRUE_VALUES
    ):
        return []

    config_path = Path(
        LaunchConfiguration("mission_config").perform(context).strip()
    )
    if not config_path.is_file():
        raise RuntimeError(
            f"mission_config must be a regular file, got: {str(config_path)!r}"
        )
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    try:
        mission_parameters = dict(
            document["competition_mission_node"]["ros__parameters"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError(
            "mission_config must contain "
            "competition_mission_node.ros__parameters"
        ) from error

    # ROS 2 Humble loses the element type of an empty YAML sequence. These
    # arrays are optional, so omit only empty values and retain typed defaults.
    for optional_array in ("home_pose", "patrol_waypoints"):
        if mission_parameters.get(optional_array) == []:
            mission_parameters.pop(optional_array)

    return [Node(
        package="rm_competition_mission",
        executable="competition_mission_node",
        name="competition_mission_node",
        output="screen",
        parameters=[
            mission_parameters,
            {
                "tree_xml": LaunchConfiguration("tree_xml"),
                "startup_enabled": ParameterValue(
                    LaunchConfiguration("startup_enabled"), value_type=bool
                ),
                "require_chassis_mode": ParameterValue(
                    LaunchConfiguration("require_chassis_mode"), value_type=bool
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            },
        ],
    )]


def generate_launch_description():
    enabled = LaunchConfiguration("enable_mission_node")
    use_safety_mock = LaunchConfiguration("use_safety_mock")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_config = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"),
        "config",
        "mission_safe.yaml",
    ])
    default_tree = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"),
        "trees",
        "competition_default.xml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("enable_mission_node", default_value="false"),
        DeclareLaunchArgument("startup_enabled", default_value="false"),
        DeclareLaunchArgument("use_safety_mock", default_value="false"),
        DeclareLaunchArgument("require_chassis_mode", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("mission_config", default_value=default_config),
        DeclareLaunchArgument("tree_xml", default_value=default_tree),
        OpaqueFunction(function=_launch_mission),
        Node(
            condition=IfCondition(PythonExpression([
                "'", enabled, "'.lower() in ['1','true','yes','on'] and '",
                use_safety_mock, "'.lower() in ['1','true','yes','on']",
            ])),
            package="rm_competition_mission",
            executable="mission_safety_mock_node",
            name="mission_safety_mock_node",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
