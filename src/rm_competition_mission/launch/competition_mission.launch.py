from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enabled = LaunchConfiguration("enable_mission_node")
    startup_enabled = LaunchConfiguration("startup_enabled")
    use_safety_mock = LaunchConfiguration("use_safety_mock")
    use_sim_time = LaunchConfiguration("use_sim_time")
    mission_config = LaunchConfiguration("mission_config")
    tree_xml = LaunchConfiguration("tree_xml")

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
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("mission_config", default_value=default_config),
        DeclareLaunchArgument("tree_xml", default_value=default_tree),
        Node(
            condition=IfCondition(enabled),
            package="rm_competition_mission",
            executable="competition_mission_node",
            name="competition_mission_node",
            output="screen",
            parameters=[
                mission_config,
                {
                    "tree_xml": tree_xml,
                    "startup_enabled": ParameterValue(
                        startup_enabled, value_type=bool
                    ),
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ),
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
