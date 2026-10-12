import ipaddress
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _bool_arg(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in TRUE_VALUES


def _adapter(side, frame_id, lidar_id):
    return Node(
        package="rm_mid360_driver_bridge",
        executable="livox_pointcloud_adapter_node",
        name=f"livox_{side}_pointcloud_adapter",
        output="screen",
        parameters=[{
            "input_topic": f"/livox/{side}/pointcloud_native",
            "pointcloud_output_topic": f"/livox/{side}/pointcloud",
            "custom_output_topic": f"/livox/{side}/lidar",
            "output_frame_id": frame_id,
            "publish_custom": True,
            "lidar_id": lidar_id,
        }],
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    if not _bool_arg(context, "use_driver"):
        return [LogInfo(msg=(
            "use_driver:=false, so livox_ros_driver2 is not launched. "
            "This is the expected default for build-only and no-hardware validation."
        ))]

    enable_left = _bool_arg(context, "enable_left")
    enable_right = _bool_arg(context, "enable_right")
    if not (enable_left and enable_right):
        raise RuntimeError(
            "dual_mid360_driver requires enable_left:=true and enable_right:=true; "
            "use single_mid360_driver.launch.py for one sensor"
        )

    lio_imu_source = LaunchConfiguration("lio_imu_source").perform(context).strip().lower()
    if lio_imu_source not in {"left", "right"}:
        raise RuntimeError("lio_imu_source must be 'left' or 'right'")

    package_share = FindPackageShare("rm_mid360_driver_bridge")
    config_path = LaunchConfiguration("config_file").perform(context).strip()
    if not config_path:
        config_path = PathJoinSubstitution([
            package_share, "config", "dual_mid360_config.json"
        ]).perform(context)
    left_ip = LaunchConfiguration("left_device_ip").perform(context).strip()
    right_ip = LaunchConfiguration("right_device_ip").perform(context).strip()
    for address in (left_ip, right_ip):
        if str(ipaddress.IPv4Address(address)) != address:
            raise RuntimeError("MID360 identities must be canonical IPv4 addresses")
    if left_ip == right_ip:
        raise RuntimeError("left and right MID360 identities must differ")
    configured = json.loads(Path(config_path).read_text())
    addresses = [entry["ip"] for entry in configured["lidar_configs"]]
    if len(addresses) != 2 or set(addresses) != {left_ip, right_ip}:
        raise RuntimeError("driver config lidar identities do not match left/right_device_ip")
    left_suffix = left_ip.replace(".", "_")
    right_suffix = right_ip.replace(".", "_")
    driver_package = LaunchConfiguration("driver_package").perform(context)
    left_frame = LaunchConfiguration("left_frame_id").perform(context)
    right_frame = LaunchConfiguration("right_frame_id").perform(context)
    left_imu = "/livox/lio_imu_raw" if lio_imu_source == "left" else "/livox/left/imu_raw"
    right_imu = "/livox/lio_imu_raw" if lio_imu_source == "right" else "/livox/right/imu_raw"

    driver = Node(
        package=driver_package,
        executable="livox_ros_driver2_node",
        name="livox_dual_driver",
        output="screen",
        parameters=[{
            # One SDK instance must configure both MID360s. Two instances race
            # and the last one redirects both sensors to its UDP host ports.
            # Multi-topic PointCloud2 is adapted below into canonical per-side
            # PointCloud2 and timing-preserving Livox CustomMsg topics.
            "xfer_format": 0,
            "multi_topic": 1,
            "data_src": 0,
            "publish_freq": float(LaunchConfiguration("publish_freq").perform(context)),
            "output_data_type": int(LaunchConfiguration("output_type").perform(context)),
            "frame_id": "livox_frame",
            "lvx_file_path": "",
            "user_config_path": config_path,
            "cmdline_input_bd_code": "livox0000000001",
        }],
        remappings=[
            (f"/livox/lidar_{left_suffix}", "/livox/left/pointcloud_native"),
            (f"/livox/lidar_{right_suffix}", "/livox/right/pointcloud_native"),
            (f"/livox/imu_{left_suffix}", left_imu),
            (f"/livox/imu_{right_suffix}", right_imu),
        ],
    )

    return [
        LogInfo(msg=(
            f"dual MID360 contract: one SDK instance, left={left_ip}, "
            f"right={right_ip}, main IMU={lio_imu_source}; canonical per-side topics"
        )),
        driver,
        _adapter("left", left_frame, 1),
        _adapter("right", right_frame, 2),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("driver_package", default_value="livox_ros_driver2"),
        DeclareLaunchArgument("enable_left", default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
        # Legacy values retained; new-car callers must supply their measured
        # network config and both real device identities together.
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("left_device_ip", default_value="192.168.1.3"),
        DeclareLaunchArgument("right_device_ip", default_value="192.168.1.166"),
        DeclareLaunchArgument("lio_imu_source", default_value="left"),
        DeclareLaunchArgument("left_frame_id", default_value="mid360_left_frame"),
        DeclareLaunchArgument("right_frame_id", default_value="mid360_right_frame"),
        DeclareLaunchArgument("publish_freq", default_value="50.0"),
        DeclareLaunchArgument("output_type", default_value="0"),
        OpaqueFunction(function=_launch_setup),
    ])
