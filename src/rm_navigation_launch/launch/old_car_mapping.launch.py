"""旧车建图入口。

日常只需要修改下面的“用户配置区”。FEATURES 中每行代表一个模块，注释该行即可
关闭对应模块。建图本身不会启动 Nav2、串口或 mission，也不会让车辆自动运动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# ============================ 用户配置区 ============================
FEATURES = set()
FEATURES.add("driver")   # 注释此行：不启动左 MID360
FEATURES.add("lio")      # 注释此行：不启动 FAST-LIO
FEATURES.add("mapping")  # 注释此行：不启动 OctoMap 和地图保存节点
FEATURES.add("rviz")     # 注释此行：不启动 RViz

MAP_OUTPUT_ROOT = "/data/rm27_maps"
MAP_ID = "old_car_field"
MAP_REVISION = "auto"
# ==================================================================


def _enabled(name):
    return name in FEATURES


def _bool_text(value):
    return "true" if value else "false"


def generate_launch_description():
    driver_enabled = _enabled("driver")
    lio_enabled = _enabled("lio") and driver_enabled
    mapping_enabled = _enabled("mapping") and driver_enabled and lio_enabled

    old_car_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "old_car_2026_validation.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("output_root", default_value=MAP_OUTPUT_ROOT),
        DeclareLaunchArgument("map_id", default_value=MAP_ID),
        DeclareLaunchArgument("revision", default_value=MAP_REVISION),
        LogInfo(msg=(
            "[rm_navigation_launch] 旧车建图入口：只启动显式保留的模块；"
            "Nav2、串口和 mission 始终关闭。"
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(old_car_launch),
            launch_arguments={
                "selected_side": "left",
                "use_driver": _bool_text(driver_enabled),
                "use_lio_backend": _bool_text(lio_enabled),
                "use_map_odom_stub": "true",
                "use_nav2": "false",
                "use_mapping": _bool_text(mapping_enabled),
                "use_serial_dry_run": "false",
                "use_real_serial": "false",
                "use_rviz": _bool_text(_enabled("rviz")),
                "mapping_output_root": LaunchConfiguration("output_root"),
                "mapping_map_id": LaunchConfiguration("map_id"),
                "mapping_revision": LaunchConfiguration("revision"),
            }.items(),
        ),
    ])
