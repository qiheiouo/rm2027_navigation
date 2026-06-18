from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _bool_arg(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in TRUE_VALUES


def _launch_setup(context, *args, **kwargs):
    use_driver = _bool_arg(context, "use_driver")
    side = LaunchConfiguration("side").perform(context).strip().lower()

    if side not in {"left", "right"}:
        return [LogInfo(msg="Invalid side. Use 'left' or 'right'. No driver node launched.")]

    config_name = f"{side}_mid360_config.json"
    frame_id = LaunchConfiguration("frame_id").perform(context)
    if not frame_id:
        frame_id = f"mid360_{side}_frame"

    actions = [
        LogInfo(msg=[
            "rm_mid360_driver_bridge single contract: ",
            f"{side} lidar=/livox/{side}/lidar, IMU=/livox/lio_imu. ",
            "This bridge must not publish localization TF or odometry."
        ])
    ]

    if not use_driver:
        actions.append(LogInfo(msg=[
            "use_driver:=false, so livox_ros_driver2 is not launched. ",
            "This is the expected default for build-only and no-hardware validation."
        ]))
        return actions

    package_share = FindPackageShare("rm_mid360_driver_bridge")
    config_path = PathJoinSubstitution([package_share, "config", config_name]).perform(context)
    driver_package = LaunchConfiguration("driver_package").perform(context)

    actions.append(Node(
        package=driver_package,
        executable="livox_ros_driver2_node",
        name=f"livox_{side}_driver",
        output="screen",
        parameters=[{
            "xfer_format": int(LaunchConfiguration("xfer_format").perform(context)),
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": float(LaunchConfiguration("publish_freq").perform(context)),
            "output_data_type": int(LaunchConfiguration("output_type").perform(context)),
            "frame_id": frame_id,
            "lvx_file_path": "",
            "user_config_path": config_path,
            "cmdline_input_bd_code": "livox0000000001",
        }],
        remappings=[
            ("/livox/lidar", f"/livox/{side}/lidar"),
            ("/livox/lidar/pointcloud", f"/livox/{side}/pointcloud"),
            ("/livox/imu", "/livox/lio_imu"),
        ],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("driver_package", default_value="livox_ros_driver2"),
        DeclareLaunchArgument("side", default_value="left"),
        DeclareLaunchArgument("frame_id", default_value=""),
        DeclareLaunchArgument("xfer_format", default_value="4"),
        DeclareLaunchArgument("publish_freq", default_value="50.0"),
        DeclareLaunchArgument("output_type", default_value="0"),
        OpaqueFunction(function=_launch_setup),
    ])
