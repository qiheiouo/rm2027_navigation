from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _bool_arg(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in TRUE_VALUES


def _driver_node(context, side, config_name, imu_target):
    package_share = FindPackageShare("rm_mid360_driver_bridge")
    config_path = PathJoinSubstitution([package_share, "config", config_name]).perform(context)
    driver_package = LaunchConfiguration("driver_package").perform(context)
    xfer_format = int(LaunchConfiguration("xfer_format").perform(context))
    publish_freq = float(LaunchConfiguration("publish_freq").perform(context))
    output_type = int(LaunchConfiguration("output_type").perform(context))
    frame_id = LaunchConfiguration(f"{side}_frame_id").perform(context)

    return Node(
        package=driver_package,
        executable="livox_ros_driver2_node",
        name=f"livox_{side}_driver",
        output="screen",
        parameters=[{
            "xfer_format": xfer_format,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": publish_freq,
            "output_data_type": output_type,
            "frame_id": frame_id,
            "lvx_file_path": "",
            "user_config_path": config_path,
            "cmdline_input_bd_code": "livox0000000001",
        }],
        remappings=[
            ("/livox/lidar", f"/livox/{side}/lidar"),
            ("/livox/lidar/pointcloud", f"/livox/{side}/pointcloud"),
            ("/livox/imu", imu_target),
        ],
    )


def _launch_setup(context, *args, **kwargs):
    use_driver = _bool_arg(context, "use_driver")
    enable_left = _bool_arg(context, "enable_left")
    enable_right = _bool_arg(context, "enable_right")
    lio_imu_source = LaunchConfiguration("lio_imu_source").perform(context).strip().lower()

    actions = [
        LogInfo(msg=[
            "rm_mid360_driver_bridge dual contract: left=/livox/left/lidar, ",
            "right=/livox/right/lidar, selected IMU=/livox/lio_imu. ",
            "This bridge must not publish localization TF or odometry."
        ])
    ]

    if not use_driver:
        actions.append(LogInfo(msg=[
            "use_driver:=false, so livox_ros_driver2 is not launched. ",
            "This is the expected default for build-only and no-hardware validation."
        ]))
        return actions

    if lio_imu_source not in {"left", "right"}:
        actions.append(LogInfo(msg="Invalid lio_imu_source. Use 'left' or 'right'. No driver nodes launched."))
        return actions

    if enable_left:
        left_imu_topic = "/livox/lio_imu" if lio_imu_source == "left" else "/livox/left/imu_raw"
        actions.append(_driver_node(context, "left", "left_mid360_config.json", left_imu_topic))

    if enable_right:
        right_imu_topic = "/livox/lio_imu" if lio_imu_source == "right" else "/livox/right/imu_raw"
        actions.append(_driver_node(context, "right", "right_mid360_config.json", right_imu_topic))

    if not enable_left and not enable_right:
        actions.append(LogInfo(msg="Both enable_left and enable_right are false. No driver nodes launched."))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("driver_package", default_value="livox_ros_driver2"),
        DeclareLaunchArgument("enable_left", default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
        DeclareLaunchArgument("lio_imu_source", default_value="left"),
        DeclareLaunchArgument("left_frame_id", default_value="mid360_left_frame"),
        DeclareLaunchArgument("right_frame_id", default_value="mid360_right_frame"),
        DeclareLaunchArgument("xfer_format", default_value="4"),
        DeclareLaunchArgument("publish_freq", default_value="50.0"),
        DeclareLaunchArgument("output_type", default_value="0"),
        OpaqueFunction(function=_launch_setup),
    ])
