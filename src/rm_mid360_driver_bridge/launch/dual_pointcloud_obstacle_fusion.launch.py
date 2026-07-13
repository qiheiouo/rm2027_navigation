from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enabled = LaunchConfiguration("enable_fusion")
    use_sim_time = LaunchConfiguration("use_sim_time")
    filter_config = LaunchConfiguration("filter_config")
    fusion_config = LaunchConfiguration("fusion_config")
    left_input = LaunchConfiguration("left_input_topic")
    right_input = LaunchConfiguration("right_input_topic")
    left_filtered = LaunchConfiguration("left_filtered_topic")
    right_filtered = LaunchConfiguration("right_filtered_topic")
    output_topic = LaunchConfiguration("output_topic")

    default_filter_config = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "config",
        "old_car_pointcloud_filter.yaml",
    ])
    default_fusion_config = PathJoinSubstitution([
        FindPackageShare("rm_mid360_driver_bridge"),
        "config",
        "dual_pointcloud_fusion.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("enable_fusion", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("filter_config", default_value=default_filter_config),
        DeclareLaunchArgument("fusion_config", default_value=default_fusion_config),
        DeclareLaunchArgument(
            "left_input_topic", default_value="/livox/left/pointcloud"
        ),
        DeclareLaunchArgument(
            "right_input_topic", default_value="/livox/right/pointcloud"
        ),
        DeclareLaunchArgument(
            "left_filtered_topic", default_value="/livox/left/pointcloud_filtered"
        ),
        DeclareLaunchArgument(
            "right_filtered_topic", default_value="/livox/right/pointcloud_filtered"
        ),
        DeclareLaunchArgument(
            "output_topic", default_value="/points/obstacles_fused"
        ),
        Node(
            condition=IfCondition(enabled),
            package="rm_mid360_driver_bridge",
            executable="pointcloud_self_filter_node",
            name="left_pointcloud_self_filter",
            output="screen",
            parameters=[
                filter_config,
                {
                    "input_topic": left_input,
                    "output_topic": left_filtered,
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ),
        Node(
            condition=IfCondition(enabled),
            package="rm_mid360_driver_bridge",
            executable="pointcloud_self_filter_node",
            name="right_pointcloud_self_filter",
            output="screen",
            parameters=[
                filter_config,
                {
                    "input_topic": right_input,
                    "output_topic": right_filtered,
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ),
        Node(
            condition=IfCondition(enabled),
            package="rm_mid360_driver_bridge",
            executable="pointcloud_fusion_node",
            name="pointcloud_fusion_node",
            output="screen",
            parameters=[
                fusion_config,
                {
                    "input_topics": [left_filtered, right_filtered],
                    "output_topic": output_topic,
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ),
    ])
