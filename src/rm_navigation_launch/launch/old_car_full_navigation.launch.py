"""旧车全功能导航入口。

在“用户配置区”填写地图 bundle YAML、行为树 XML 和参数 YAML。FEATURES 中每行
代表一个模块，注释该行即可关闭；依赖于它的下游模块会一并关闭，不会绕过底层安全门。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# ============================ 用户配置区 ============================
# 地图必须填写 rm_map_tools 的 *.bundle.yaml；其中会引用真正的 map YAML/PGM/PCD。
MAP_BUNDLE_YAML = (
    "/data/rm27_maps/old_car_fresh03_field_validation/20260720T012237Z/"
    "old_car_fresh03_field_validation.bundle.yaml"
)

# 留空时使用本仓库内已经验证过的三点巡逻自转候选文件；也可填写绝对路径。
MISSION_TREE_XML = ""
MISSION_CONFIG_YAML = ""
NAV2_CONFIG_YAML = ""
RELOCALIZATION_CONFIG_YAML = ""
SCAN_PROJECTION_CONFIG_YAML = ""

RELOCALIZATION_BACKEND = "amcl_2d"  # 可选：amcl_2d / gicp_3d
MAP_ACCEPTANCE_POLICY = "allow_candidate"

SERIAL_DEVICE = "/dev/ttyACM0"
SERIAL_BAUDRATE = "115200"
SERIAL_MAX_VX = "3.0"
SERIAL_MAX_VY = "3.0"
SERIAL_MAX_WZ = "10.0"

FEATURES = set()
FEATURES.add("driver")          # 左 MID360
FEATURES.add("lio")             # FAST-LIO
FEATURES.add("map_server")      # 地图服务
FEATURES.add("relocalization")  # AMCL/GICP；注释后 Nav2/串口/mission 也会关闭
FEATURES.add("nav2")            # 规划、控制与 costmap
FEATURES.add("rviz")            # RViz
FEATURES.add("serial")          # 真实底盘串口
FEATURES.add("referee")         # /referee/state_raw 的校验门
FEATURES.add("mission")         # 指定 XML 的比赛行为树；启动后仍保持 disabled
# FEATURES.add("pursuit")       # 等视觉目标协议完成后再取消注释
# FEATURES.add("dual_fusion")   # 等右雷达外参和实车验收后再取消注释
# FEATURES.add("right_lidar")   # 必须与 dual_fusion 同时启用

# 右雷达外参当前仍是 provisional。即使取消上面两行，也必须再次显式改为 True。
ALLOW_PROVISIONAL_DUAL_EXTRINSIC = False
# ==================================================================


def _enabled(name):
    return name in FEATURES


def _bool_text(value):
    return "true" if value else "false"


def _configured_path(value, default_path):
    return value if value.strip() else default_path


def generate_launch_description():
    driver_enabled = _enabled("driver")
    lio_enabled = _enabled("lio") and driver_enabled
    map_enabled = _enabled("map_server")
    relocalization_enabled = (
        _enabled("relocalization")
        and map_enabled
        and driver_enabled
        and lio_enabled
        and RELOCALIZATION_BACKEND != "none"
    )
    nav2_enabled = _enabled("nav2") and relocalization_enabled
    serial_enabled = _enabled("serial") and nav2_enabled and driver_enabled and lio_enabled
    referee_enabled = _enabled("referee")
    mission_enabled = (
        _enabled("mission") and nav2_enabled and serial_enabled and referee_enabled
    )
    dual_fusion_enabled = _enabled("dual_fusion") and driver_enabled
    right_driver_enabled = (
        _enabled("right_lidar") and driver_enabled and dual_fusion_enabled
    )
    chassis_mode_enabled = False
    operator_authority_enabled = mission_enabled and serial_enabled

    bringup_launch = PathJoinSubstitution([
        FindPackageShare("rm_navigation_bringup"),
        "launch",
        "old_car_2026_competition.launch.py",
    ])
    default_tree = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"),
        "trees",
        "competition_three_point_spin_test.xml",
    ])
    default_mission_config = PathJoinSubstitution([
        FindPackageShare("rm_competition_mission"),
        "config",
        "mission_fresh03_three_point_spin_test.yaml",
    ])
    default_nav2_config = PathJoinSubstitution([
        FindPackageShare("rm_nav_config"),
        "config",
        "nav2_old_car_2026_left_stvl_three_point_spin_test.yaml",
    ])
    default_relocalization_config = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "amcl_2d_spin_robust_candidate.yaml",
    ])
    default_scan_projection_config = PathJoinSubstitution([
        FindPackageShare("rm_relocalization_bridge"),
        "config",
        "pointcloud_to_scan_2d_spin_robust_candidate.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("map_bundle_yaml", default_value=MAP_BUNDLE_YAML),
        DeclareLaunchArgument(
            "mission_tree_xml",
            default_value=_configured_path(MISSION_TREE_XML, default_tree),
        ),
        DeclareLaunchArgument(
            "mission_config_yaml",
            default_value=_configured_path(MISSION_CONFIG_YAML, default_mission_config),
        ),
        DeclareLaunchArgument(
            "nav2_config_yaml",
            default_value=_configured_path(NAV2_CONFIG_YAML, default_nav2_config),
        ),
        DeclareLaunchArgument(
            "relocalization_config_yaml",
            default_value=_configured_path(
                RELOCALIZATION_CONFIG_YAML, default_relocalization_config
            ),
        ),
        DeclareLaunchArgument(
            "scan_projection_config_yaml",
            default_value=_configured_path(
                SCAN_PROJECTION_CONFIG_YAML, default_scan_projection_config
            ),
        ),
        LogInfo(msg=(
            "[rm_navigation_launch] 全功能入口已加载。mission 启动状态固定为 disabled；"
            "发布初始位姿并完成安全检查后，再通过服务启用。"
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "enable_competition_stack": "true",
                "use_driver": _bool_text(driver_enabled),
                "use_lio_backend": _bool_text(lio_enabled),
                "use_map_server": _bool_text(map_enabled),
                "relocalization_backend": (
                    RELOCALIZATION_BACKEND if relocalization_enabled else "none"
                ),
                "map_bundle_manifest": LaunchConfiguration("map_bundle_yaml"),
                "map_acceptance_policy": MAP_ACCEPTANCE_POLICY,
                "relocalization_params": LaunchConfiguration(
                    "relocalization_config_yaml"
                ),
                "scan_projection_params": LaunchConfiguration(
                    "scan_projection_config_yaml"
                ),
                "nav2_params": LaunchConfiguration("nav2_config_yaml"),
                "use_nav2": _bool_text(nav2_enabled),
                "use_rviz": _bool_text(_enabled("rviz")),
                "use_real_serial": _bool_text(serial_enabled),
                "serial_protocol_profile": "hpm_crc_v1",
                "serial_referee_rx_enabled": _bool_text(
                    serial_enabled and referee_enabled
                ),
                "serial_device": SERIAL_DEVICE,
                "serial_baudrate": SERIAL_BAUDRATE,
                "serial_max_vx": SERIAL_MAX_VX,
                "serial_max_vy": SERIAL_MAX_VY,
                "serial_max_wz": SERIAL_MAX_WZ,
                "use_referee_interface": _bool_text(referee_enabled),
                "use_referee_mock": "false",
                "use_mission": _bool_text(mission_enabled),
                "mission_startup_enabled": "false",
                "mission_config": LaunchConfiguration("mission_config_yaml"),
                "mission_tree_xml": LaunchConfiguration("mission_tree_xml"),
                "allow_field_debug_inputs": _bool_text(operator_authority_enabled),
                "use_operator_chassis_authority": _bool_text(
                    operator_authority_enabled
                ),
                "use_chassis_mode_interface": _bool_text(chassis_mode_enabled),
                "use_mission_safety_mock": "false",
                "use_pursuit": _bool_text(_enabled("pursuit")),
                "use_target_mock": "false",
                "use_dual_obstacle_fusion": _bool_text(dual_fusion_enabled),
                "use_right_driver": _bool_text(right_driver_enabled),
                "allow_provisional_dual_extrinsic": _bool_text(
                    ALLOW_PROVISIONAL_DUAL_EXTRINSIC
                ),
            }.items(),
        ),
    ])
