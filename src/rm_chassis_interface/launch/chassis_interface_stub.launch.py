from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Phase 1 stub only. Real serial hardware is intentionally not connected.
    return LaunchDescription([
        Node(
            package="rm_chassis_interface",
            executable="chassis_interface_stub",
            name="chassis_interface_stub",
            output="screen",
        ),
    ])
