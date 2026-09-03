from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _bool_arg(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in TRUE_VALUES


def _reconstructed_custom_adapter(side, frame_id, lidar_id):
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


def _native_custom_adapter(side, frame_id):
    return Node(
        package="rm_mid360_driver_bridge",
        executable="livox_custom_adapter_node",
        name=f"livox_{side}_native_custom_adapter",
        output="screen",
        parameters=[{
            "input_topic": f"/livox/{side}/lidar_native",
            "custom_output_topic": f"/livox/{side}/lidar",
            "pointcloud_output_topic": f"/livox/{side}/pointcloud",
            "output_frame_id": frame_id,
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
    lio_input_mode = (
        LaunchConfiguration("lio_input_mode").perform(context).strip().lower()
    )
    if lio_input_mode not in {"native_custom", "reconstructed_custom"}:
        raise RuntimeError(
            "lio_input_mode must be 'native_custom' or 'reconstructed_custom'"
        )

    package_share = FindPackageShare("rm_mid360_driver_bridge")
    config_path = PathJoinSubstitution([
        package_share, "config", "dual_mid360_config.json"
    ]).perform(context)
    driver_package = LaunchConfiguration("driver_package").perform(context)
    left_frame = LaunchConfiguration("left_frame_id").perform(context)
    right_frame = LaunchConfiguration("right_frame_id").perform(context)
    left_imu = "/livox/lio_imu_raw" if lio_imu_source == "left" else "/livox/left/imu_raw"
    right_imu = "/livox/lio_imu_raw" if lio_imu_source == "right" else "/livox/right/imu_raw"

    native_custom = lio_input_mode == "native_custom"
    driver = Node(
        package=driver_package,
        executable="livox_ros_driver2_node",
        name="livox_dual_driver",
        output="screen",
        parameters=[{
            # One SDK instance must configure both MID360s. Two instances race
            # and the last one redirects both sensors to its UDP host ports.
            # Prefer native per-device CustomMsg for FAST-LIO. Reconstructing
            # CustomMsg from PointCloud2 remains available only as an explicit
            # A/B fallback for evidence captured before this correction.
            "xfer_format": 1 if native_custom else 0,
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
            (
                "/livox/lidar_192_168_1_3",
                "/livox/left/lidar_native"
                if native_custom
                else "/livox/left/pointcloud_native",
            ),
            (
                "/livox/lidar_192_168_1_166",
                "/livox/right/lidar_native"
                if native_custom
                else "/livox/right/pointcloud_native",
            ),
            ("/livox/imu_192_168_1_3", left_imu),
            ("/livox/imu_192_168_1_166", right_imu),
        ],
    )

    return [
        LogInfo(msg=(
            "dual MID360 contract: one SDK instance, left=192.168.1.3, "
            "right=192.168.1.166, canonical PointCloud2 and CustomMsg per side; "
            f"LIO timing source={lio_input_mode}"
        )),
        driver,
        (
            _native_custom_adapter("left", left_frame)
            if native_custom
            else _reconstructed_custom_adapter("left", left_frame, 1)
        ),
        (
            _native_custom_adapter("right", right_frame)
            if native_custom
            else _reconstructed_custom_adapter("right", right_frame, 2)
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_driver", default_value="false"),
        DeclareLaunchArgument("driver_package", default_value="livox_ros_driver2"),
        DeclareLaunchArgument("enable_left", default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
        DeclareLaunchArgument("lio_imu_source", default_value="left"),
        DeclareLaunchArgument(
            "lio_input_mode",
            default_value="native_custom",
            description=(
                "native_custom preserves driver timebase/offset_time for LIO; "
                "reconstructed_custom retains the previous PointCloud2 bridge "
                "for controlled A/B only."
            ),
        ),
        DeclareLaunchArgument("left_frame_id", default_value="mid360_left_frame"),
        DeclareLaunchArgument("right_frame_id", default_value="mid360_right_frame"),
        DeclareLaunchArgument("publish_freq", default_value="50.0"),
        DeclareLaunchArgument("output_type", default_value="0"),
        OpaqueFunction(function=_launch_setup),
    ])
