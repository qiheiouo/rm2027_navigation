import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _box_visual(name, pose, size, color):
    return f"""
      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>"""


def _marker_visual(name, longitudinal, color):
    return f"""
      <visual name="{name}">
        <pose>{longitudinal} 0 0.006 0 0 0</pose>
        <geometry><cylinder><radius>0.09</radius><length>0.012</length></cylinder></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>"""


def _make_scene(params):
    center_x = float(params["dog_hole.center_x"])
    center_y = float(params["dog_hole.center_y"])
    yaw = float(params["dog_hole.yaw"])
    width = float(params["dog_hole.width"])
    length = float(params["dog_hole.length"])
    wall_height = float(params["dog_hole.wall_height"])
    wall_thickness = float(params["dog_hole.wall_thickness"])
    roof_clearance = float(params["dog_hole.roof_clearance"])
    entry_clearance = float(params["dog_hole.entry_clearance"])
    exit_clearance = float(params["dog_hole.exit_clearance"])
    deck_height = float(params["dog_hole.deck_height"])
    entry_slope_deg = float(params["dog_hole.entry_slope_deg"])
    exit_slope_deg = float(params["dog_hole.exit_slope_deg"])

    side_offset = 0.5 * (width + wall_thickness)
    wall_z = deck_height + 0.5 * wall_height
    roof_thickness = 0.04
    roof_z = deck_height + roof_clearance + 0.5 * roof_thickness
    approach_longitudinal = -0.5 * length - entry_clearance
    entry_longitudinal = -0.5 * length
    exit_longitudinal = 0.5 * length
    recovery_longitudinal = 0.5 * length + exit_clearance
    centerline_length = length + entry_clearance + exit_clearance
    centerline_x = 0.5 * (exit_clearance - entry_clearance)

    deck = ""
    ramps = ""
    if deck_height > 1e-4:
        deck = f"""
      <collision name="deck_collision">
        <pose>0 0 {0.5 * deck_height} 0 0 0</pose>
        <geometry><box><size>{length} {width} {deck_height}</size></box></geometry>
      </collision>
      {_box_visual(
            "deck_visual",
            f"0 0 {0.5 * deck_height} 0 0 0",
            f"{length} {width} {deck_height}",
            "0.32 0.32 0.32 1",
        )}"""

        def ramp(name, angle_deg, entry):
            if angle_deg <= 1e-4:
                return ""
            angle = math.radians(angle_deg)
            horizontal = deck_height / math.tan(angle)
            slope_length = math.hypot(horizontal, deck_height)
            local_x = (
                -0.5 * length - 0.5 * horizontal
                if entry
                else 0.5 * length + 0.5 * horizontal
            )
            pitch = -angle if entry else angle
            return f"""
      <collision name="{name}_collision">
        <pose>{local_x} 0 {0.5 * deck_height} 0 {pitch} 0</pose>
        <geometry><box><size>{slope_length} {width} 0.04</size></box></geometry>
      </collision>
      {_box_visual(
                f"{name}_visual",
                f"{local_x} 0 {0.5 * deck_height} 0 {pitch} 0",
                f"{slope_length} {width} 0.04",
                "0.38 0.38 0.38 1",
            )}"""

        ramps = ramp("entry_ramp", entry_slope_deg, True)
        ramps += ramp("exit_ramp", exit_slope_deg, False)

    link_contents = f"""
      <collision name="left_wall_collision">
        <pose>0 {side_offset} {wall_z} 0 0 0</pose>
        <geometry><box><size>{length} {wall_thickness} {wall_height}</size></box></geometry>
      </collision>
      <collision name="right_wall_collision">
        <pose>0 {-side_offset} {wall_z} 0 0 0</pose>
        <geometry><box><size>{length} {wall_thickness} {wall_height}</size></box></geometry>
      </collision>
      {_box_visual(
        "left_wall_visual",
        f"0 {side_offset} {wall_z} 0 0 0",
        f"{length} {wall_thickness} {wall_height}",
        "0.46 0.39 0.26 1",
    )}
      {_box_visual(
        "right_wall_visual",
        f"0 {-side_offset} {wall_z} 0 0 0",
        f"{length} {wall_thickness} {wall_height}",
        "0.46 0.39 0.26 1",
    )}
      {_box_visual(
        "roof_visual_only",
        f"0 0 {roof_z} 0 0 0",
        f"{length} {width + 2.0 * wall_thickness} {roof_thickness}",
        "0.40 0.34 0.23 0.28",
    )}
      {_box_visual(
        "centerline_visual",
        f"{centerline_x} 0 {deck_height + 0.003} 0 0 0",
        f"{centerline_length} 0.025 0.006",
        "0.10 0.82 0.20 1",
    )}
      {_marker_visual("approach_marker", approach_longitudinal, "0.10 0.30 0.95 1")}
      {_marker_visual("entry_marker", entry_longitudinal, "0.95 0.78 0.10 1")}
      {_marker_visual("exit_marker", exit_longitudinal, "0.95 0.45 0.10 1")}
      {_marker_visual("recovery_marker", recovery_longitudinal, "0.65 0.12 0.90 1")}
      {deck}
      {ramps}
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="dog_hole_scene">
    <static>true</static>
    <pose>{center_x} {center_y} 0 0 0 {yaw}</pose>
    <link name="dog_hole_link">
      {link_contents}
    </link>
  </model>
</sdf>
"""


def _make_nav2_profile(params):
    source = (
        Path(get_package_share_directory("rm_nav_config"))
        / "config"
        / "nav2_phase1_5_mppi.yaml"
    )
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    robot_length = float(params["robot.length"])
    robot_width = float(params["robot.width"])
    half_length = 0.5 * robot_length
    half_width = 0.5 * robot_width
    footprint = (
        f"[[{-half_length:.3f}, {-half_width:.3f}], "
        f"[{-half_length:.3f}, {half_width:.3f}], "
        f"[{half_length:.3f}, {half_width:.3f}], "
        f"[{half_length:.3f}, {-half_width:.3f}]]"
    )

    follow_path = data["controller_server"]["ros__parameters"]["FollowPath"]
    follow_path.update(
        {
            # The narrow-entry profile needs enough samples to avoid the
            # repeated soft-reset/abort cycle seen with the 300-sample base
            # profile before the dedicated alignment controller takes over.
            "batch_size": 1000,
            "retry_attempt_limit": 3,
            "vx_max": 0.65,
            "vx_min": -0.30,
            "vy_max": 0.35,
            "wz_max": 1.0,
        }
    )
    follow_path["PathAlignCritic"]["cost_weight"] = 14.0
    smoother = data["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [0.65, 0.35, 1.0]
    smoother["min_velocity"] = [-0.30, -0.35, -1.0]
    smoother["max_accel"] = [1.2, 1.0, 2.0]
    smoother["max_decel"] = [-1.2, -1.0, -2.0]

    for costmap_name in ("local_costmap", "global_costmap"):
        costmap = data[costmap_name][costmap_name]["ros__parameters"]
        costmap["footprint"] = footprint
        costmap["footprint_padding"] = 0.01
        costmap["inflation_layer"]["inflation_radius"] = 0.32
        costmap["inflation_layer"]["cost_scaling_factor"] = 10.0

    output = Path("/tmp/rm2027_dog_hole_sim/nav2_dog_hole_sim.yaml")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(data, sort_keys=False),
        encoding="utf-8",
    )
    return output


def _launch_setup(context):
    config_path = Path(LaunchConfiguration("dog_hole_config").perform(context))
    auto_start = LaunchConfiguration("auto_start")
    gimbal_yaw = LaunchConfiguration("gimbal_yaw")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    config_data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = config_data["dog_hole_manager"]["ros__parameters"]

    output_dir = Path("/tmp/rm2027_dog_hole_sim")
    output_dir.mkdir(parents=True, exist_ok=True)
    scene_path = output_dir / "dog_hole_scene.sdf"
    scene_path.write_text(_make_scene(params), encoding="utf-8")
    nav2_path = _make_nav2_profile(params)

    gazebo_launch = (
        Path(get_package_share_directory("rm_simulation"))
        / "launch"
        / "phase1_5_gazebo.launch.py"
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={
                "headless": headless,
                "use_nav2": "true",
                "use_rviz": use_rviz,
                "nav2_params": str(nav2_path),
                "gimbal_use_input": "true",
                "gimbal_input_topic": "/gimbal/state",
                "gimbal_yaw": gimbal_yaw,
                "use_localization_disturbance": "true",
                "localization_reference_yaw": str(
                    params["dog_hole.yaw"]
                ),
                "localization_lateral_noise_std_m": str(
                    params["simulation.localization.lateral_noise_std_m"]
                ),
                "localization_yaw_noise_std_rad": str(
                    params["simulation.localization.yaw_noise_std_rad"]
                ),
                "localization_delay_sec": str(
                    params["simulation.localization.delay_sec"]
                ),
                "localization_lateral_drift_amplitude_m": str(
                    params[
                        "simulation.localization.lateral_drift_amplitude_m"
                    ]
                ),
                "localization_yaw_drift_amplitude_rad": str(
                    params[
                        "simulation.localization.yaw_drift_amplitude_rad"
                    ]
                ),
                "localization_drift_frequency_hz": str(
                    params["simulation.localization.drift_frequency_hz"]
                ),
                "localization_random_seed": str(
                    params["simulation.localization.random_seed"]
                ),
                "use_chassis_disturbance": "true",
                "chassis_forward_scale": str(
                    params["simulation.chassis.forward_scale"]
                ),
                "chassis_lateral_positive_scale": str(
                    params["simulation.chassis.lateral_positive_scale"]
                ),
                "chassis_lateral_negative_scale": str(
                    params["simulation.chassis.lateral_negative_scale"]
                ),
                "chassis_angular_scale": str(
                    params["simulation.chassis.angular_scale"]
                ),
                "chassis_lateral_time_constant_sec": str(
                    params["simulation.chassis.lateral_time_constant_sec"]
                ),
                "chassis_angular_time_constant_sec": str(
                    params["simulation.chassis.angular_time_constant_sec"]
                ),
            }.items(),
        ),
        Node(
            package="rm_dog_hole",
            executable="dog_hole_manager",
            name="dog_hole_manager",
            output="screen",
            parameters=[
                str(config_path),
                {"auto_start": ParameterValue(auto_start, value_type=bool)},
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_dog_hole_scene",
                    output="screen",
                    arguments=[
                        "-world",
                        "phase1_omni",
                        "-file",
                        str(scene_path),
                        "-name",
                        "dog_hole_scene",
                    ],
                )
            ],
        ),
    ]


def generate_launch_description():
    default_config = (
        Path(get_package_share_directory("rm_dog_hole"))
        / "config"
        / "dog_hole_sim.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("gimbal_yaw", default_value="0.65"),
            DeclareLaunchArgument(
                "dog_hole_config",
                default_value=str(default_config),
            ),
            LogInfo(
                msg=[
                    "[dog_hole_sim] New-car simulation only. ",
                    "No real serial, MID360 or old-car launch is started.",
                ]
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
