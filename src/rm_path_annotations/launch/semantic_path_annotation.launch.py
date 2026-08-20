from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("rm_path_annotations"),
        "config",
        "semantic_path_annotation.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("regions_file", default_value=""),
        DeclareLaunchArgument("expected_map_id", default_value=""),
        DeclareLaunchArgument("expected_map_revision", default_value=""),
        DeclareLaunchArgument("expected_manifest_sha256", default_value=""),
        DeclareLaunchArgument("input_path_topic", default_value="/plan"),
        DeclareLaunchArgument(
            "output_topic", default_value="/navigation/annotated_path"
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("enabled")),
            package="rm_path_annotations",
            executable="semantic_path_annotator_node",
            name="semantic_path_annotator",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "regions_file": ParameterValue(
                        LaunchConfiguration("regions_file"), value_type=str
                    ),
                    "expected_map_id": ParameterValue(
                        LaunchConfiguration("expected_map_id"), value_type=str
                    ),
                    "expected_map_revision": ParameterValue(
                        LaunchConfiguration("expected_map_revision"), value_type=str
                    ),
                    "expected_manifest_sha256": ParameterValue(
                        LaunchConfiguration("expected_manifest_sha256"), value_type=str
                    ),
                    "input_path_topic": ParameterValue(
                        LaunchConfiguration("input_path_topic"), value_type=str
                    ),
                    "output_topic": ParameterValue(
                        LaunchConfiguration("output_topic"), value_type=str
                    ),
                },
            ],
        ),
    ])
