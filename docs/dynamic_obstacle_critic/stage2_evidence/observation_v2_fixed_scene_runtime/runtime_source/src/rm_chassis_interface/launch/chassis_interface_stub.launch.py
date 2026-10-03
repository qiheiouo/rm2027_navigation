from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Phase 1 stub only. Real serial hardware is intentionally not connected.
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="rm_chassis_interface",
            executable="chassis_interface_stub",
            name="chassis_interface_stub",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
