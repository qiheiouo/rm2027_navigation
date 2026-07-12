from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    phase2c_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "phase2c_relocalization_bringup.launch.py",
    ])
    gicp_launch = PathJoinSubstitution([
        FindPackageShare("rm_gicp_relocalization"),
        "launch",
        "gicp_3d_backend.launch.py",
    ])
    gicp_params = PathJoinSubstitution([
        FindPackageShare("rm_gicp_relocalization"),
        "config",
        "gicp_3d.yaml",
    ])
    test_manifest = PathJoinSubstitution([
        FindPackageShare("rm_map_tools"),
        "maps",
        "phase2e_test",
        "phase2j_3d_test.bundle.yaml",
    ])
    test_pcd = PathJoinSubstitution([
        FindPackageShare("rm_map_tools"),
        "maps",
        "phase2e_test",
        "phase2j_3d_test.pcd",
    ])

    return LaunchDescription([
        LogInfo(msg=(
            "Phase 2J 3D no-hardware test: synthetic PCD/current cloud -> PCL "
            "GICP -> pose gate -> Phase 2C map->odom bridge. No driver, Nav2, "
            "serial, referee, mission, or competition BT."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(phase2c_launch),
            launch_arguments={
                "global_localization_mode": "external_pose",
                "sensor_mode": "single",
                "selected_side": "left",
                "use_driver": "false",
                "use_lio_backend": "false",
                "use_rviz": "false",
                "use_sim_time": "false",
            }.items(),
        ),
        Node(
            package="rm_localization_adapters",
            executable="fake_lio_odom_publisher",
            name="phase2j_3d_fake_lio_odom_publisher",
            output="screen",
            parameters=[{
                "output_topic": "/odometry/fast_lio_raw",
                "cmd_vel_topic": "/phase2j/unused_cmd_vel",
                "odom_frame": "odom",
                "child_frame": "body",
                "motion_mode": "cmd_vel",
                "publish_rate_hz": 50.0,
                "cmd_vel_timeout_sec": 0.5,
                "sensor_offset.x": 0.0,
                "sensor_offset.y": 0.12,
                "sensor_offset.z": 0.35,
            }],
        ),
        Node(
            package="rm_gicp_relocalization",
            executable="fake_gicp_input",
            name="phase2j_fake_gicp_input",
            output="screen",
            parameters=[{
                "pcd_file": test_pcd,
                "cloud_topic": "/lio/cloud_registered",
                "cloud_frame": "odom",
                "map_frame": "map",
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gicp_launch),
            launch_arguments={
                "enable_backend": "true",
                "params_file": gicp_params,
                "map_bundle_manifest": test_manifest,
                "allow_test_map": "true",
                "input_cloud_topic": "/lio/cloud_registered",
                "use_sim_time": "false",
            }.items(),
        ),
    ])
