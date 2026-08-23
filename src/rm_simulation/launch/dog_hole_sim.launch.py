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
from launch.conditions import IfCondition
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


def _footprint_points(params):
    values = params.get("robot.footprint", [])
    if not values:
        half_length = 0.5 * float(params["robot.length"])
        half_width = 0.5 * float(params["robot.width"])
        return [
            [-half_length, -half_width],
            [-half_length, half_width],
            [half_length, half_width],
            [half_length, -half_width],
        ]
    if len(values) < 6 or len(values) % 2:
        raise RuntimeError(
            "robot.footprint must contain at least three x/y pairs"
        )
    return [
        [float(values[index]), float(values[index + 1])]
        for index in range(0, len(values), 2)
    ]


def _robot_geometry(params):
    profile = str(params["robot.geometry_profile"])
    if profile not in ("deformed", "undeformed"):
        raise RuntimeError(
            "robot.geometry_profile must be 'deformed' or 'undeformed'"
        )
    height = float(params[f"robot.{profile}_height"])
    lidar_z = float(params[f"robot.{profile}_lidar_offset_z"])
    geometry = {
        "profile": profile,
        "height": height,
        "gimbal_joint_height": float(params["robot.gimbal_joint_height"]),
        "gimbal_radius": float(params["robot.gimbal_radius"]),
        "gimbal_thickness": float(params["robot.gimbal_thickness"]),
        "lidar_x": float(params["robot.sim_lidar_offset_x"]),
        "lidar_y": float(params["robot.sim_lidar_offset_y"]),
        "lidar_z": lidar_z,
        "lidar_size_x": float(params["robot.sim_lidar_size_x"]),
        "lidar_size_y": float(params["robot.sim_lidar_size_y"]),
        "lidar_size_z": float(params["robot.sim_lidar_size_z"]),
    }
    physical_top = (
        geometry["gimbal_joint_height"]
        + geometry["lidar_z"]
        + 0.5 * geometry["lidar_size_z"]
    )
    positive_dimensions = (
        "height",
        "gimbal_joint_height",
        "gimbal_radius",
        "gimbal_thickness",
        "lidar_z",
        "lidar_size_x",
        "lidar_size_y",
        "lidar_size_z",
    )
    if any(geometry[key] <= 0.0 for key in positive_dimensions):
        raise RuntimeError("robot geometry dimensions must be positive")
    if physical_top > height + 1e-9:
        raise RuntimeError(
            f"{profile} lidar top {physical_top:.3f} m exceeds "
            f"robot.{profile}_height {height:.3f} m"
        )
    geometry["physical_top"] = physical_top
    return geometry


def _make_robot_world(
    params,
    spawn_x,
    spawn_y,
    spawn_yaw,
    physics_max_step_size,
    lidar_update_rate,
    lidar_samples,
):
    geometry = _robot_geometry(params)
    source = (
        Path(get_package_share_directory("rm_simulation"))
        / "worlds"
        / "phase1_omni.sdf"
    )
    text = source.read_text(encoding="utf-8")
    remove_begin = "    <!-- FIELD_GEOMETRY_REMOVE_BEGIN -->"
    remove_end = "    <!-- FIELD_GEOMETRY_REMOVE_END -->"
    if remove_begin not in text or remove_end not in text:
        raise RuntimeError("field-geometry removal markers are missing")
    prefix, remainder = text.split(remove_begin, maxsplit=1)
    _, suffix = remainder.split(remove_end, maxsplit=1)
    text = prefix + suffix
    replacements = {
        "<max_step_size>0.001</max_step_size>": (
            f"<max_step_size>{physics_max_step_size}</max_step_size>"
        ),
        "<update_rate>15</update_rate>": (
            f"<update_rate>{lidar_update_rate}</update_rate>"
        ),
        "<samples>720</samples>": f"<samples>{lidar_samples}</samples>",
        (
            '<model name="rm_sentry_2027" '
            'xmlns:ignition="http://ignitionrobotics.org/schema">\n'
            "      <pose>0 0 0 0 0 0</pose>"
        ): (
            '<model name="rm_sentry_2027" '
            'xmlns:ignition="http://ignitionrobotics.org/schema">\n'
            f"      <pose>{spawn_x} {spawn_y} 0 0 0 {spawn_yaw}</pose>"
        ),
        '<pose relative_to="base_link">0 0 0.115 0 0 0</pose>': (
            '<pose relative_to="base_link">0 0 '
            f'{geometry["gimbal_joint_height"]} 0 0 0</pose>'
        ),
        "<radius>0.20</radius><length>0.04</length>": (
            f'<radius>{geometry["gimbal_radius"]}</radius>'
            f'<length>{geometry["gimbal_thickness"]}</length>'
        ),
        '<pose relative_to="gimbal_yaw_link">0.12 0 0.065 0 0 0</pose>': (
            '<pose relative_to="gimbal_yaw_link">'
            f'{geometry["lidar_x"]} {geometry["lidar_y"]} '
            f'{geometry["lidar_z"]} 0 0 0</pose>'
        ),
        "<size>0.08 0.08 0.06</size>": (
            f'<size>{geometry["lidar_size_x"]} '
            f'{geometry["lidar_size_y"]} '
            f'{geometry["lidar_size_z"]}</size>'
        ),
        # The generic planar regression world deliberately models ideal
        # rollers with zero secondary friction. On an incline that makes the
        # placeholder chassis slide sideways even when commanded vy is zero.
        # Keep this finite coefficient local to the generated field-geometry
        # world until the real wheel/contact parameters are measured.
        "<mu>1.0</mu><mu2>0.0</mu2>": (
            "<mu>1.0</mu><mu2>0.35</mu2>"
        ),
    }
    for old, new in replacements.items():
        if old not in text:
            raise RuntimeError(
                f"new-car world template token is missing: {old}"
            )
        text = text.replace(old, new)
    return text, geometry


def _make_scene(params):
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
        "roof_visual",
        f"0 0 {roof_z} 0 0 0",
        f"{length} {width + 2.0 * wall_thickness} {roof_thickness}",
        "0.40 0.34 0.23 0.28",
    )}
      <collision name="roof_collision">
        <pose>0 0 {roof_z} 0 0 0</pose>
        <geometry>
          <box>
            <size>{length} {width + 2.0 * wall_thickness} {roof_thickness}</size>
          </box>
        </geometry>
      </collision>
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
    <!-- ros_gz_sim create supplies the configured world pose. Keeping this
         pose local avoids its default zero pose silently overriding the
         configured corridor center and yaw. -->
    <pose>0 0 0 0 0 0</pose>
    <link name="dog_hole_link">
      {link_contents}
    </link>
  </model>
</sdf>
"""


def _make_nav2_profile(
    params,
    heading_policy,
    global_costmap_width,
    global_costmap_height,
    global_costmap_update_frequency,
    global_costmap_resolution,
    global_costmap_rolling_window,
    global_costmap_origin_x,
    global_costmap_origin_y,
    mppi_batch_size,
    bt_loop_duration,
):
    source = (
        Path(get_package_share_directory("rm_nav_config"))
        / "config"
        / "nav2_phase1_5_mppi.yaml"
    )
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    footprint = str(_footprint_points(params))

    follow_path = data["controller_server"]["ros__parameters"]["FollowPath"]
    follow_path.update(
        {
            # The narrow-entry profile needs enough samples to avoid the
            # repeated soft-reset/abort cycle seen with the 300-sample base
            # profile before the dedicated alignment controller takes over.
            "batch_size": mppi_batch_size,
            "retry_attempt_limit": 3,
            "vx_max": 0.65,
            "vx_min": -0.30,
            "vy_max": 0.35,
            "wz_max": 1.0,
        }
    )
    if heading_policy == "path_aligned":
        # The Omni model can still satisfy the critics while strafing into a
        # ramp edge. DiffDrive makes base-forward alignment a hard kinematic
        # constraint for this simulation profile; it does not change any
        # real-robot Nav2 configuration.
        follow_path["motion_model"] = "DiffDrive"
        follow_path["wz_max"] = 0.45
        follow_path["PathAngleCritic"]["forward_preference"] = True
        follow_path["PathAngleCritic"]["cost_weight"] = 6.0
        follow_path["PathAngleCritic"]["max_angle_to_furthest"] = 0.20
        follow_path["TwirlingCritic"]["enabled"] = True
        follow_path["PreferForwardCritic"]["enabled"] = True
        goal_checker = data["controller_server"]["ros__parameters"][
            "general_goal_checker"
        ]
        goal_checker["yaw_goal_tolerance"] = 0.05
    elif heading_policy != "baseline":
        raise RuntimeError(
            "heading_policy must be 'baseline' or 'path_aligned'"
        )
    follow_path["PathAlignCritic"]["cost_weight"] = 14.0
    smoother = data["velocity_smoother"]["ros__parameters"]
    angular_velocity_limit = 0.45 if heading_policy == "path_aligned" else 1.0
    smoother["max_velocity"] = [0.65, 0.35, angular_velocity_limit]
    smoother["min_velocity"] = [-0.30, -0.35, -angular_velocity_limit]
    smoother["max_accel"] = [1.2, 1.0, 2.0]
    smoother["max_decel"] = [-1.2, -1.0, -2.0]

    for costmap_name in ("local_costmap", "global_costmap"):
        costmap = data[costmap_name][costmap_name]["ros__parameters"]
        costmap["footprint"] = footprint
        costmap["footprint_padding"] = 0.01
        costmap["inflation_layer"]["inflation_radius"] = 0.32
        costmap["inflation_layer"]["cost_scaling_factor"] = 10.0

    global_costmap = data["global_costmap"]["global_costmap"][
        "ros__parameters"
    ]
    global_costmap["width"] = global_costmap_width
    global_costmap["height"] = global_costmap_height
    global_costmap["update_frequency"] = global_costmap_update_frequency
    global_costmap["resolution"] = global_costmap_resolution
    global_costmap["rolling_window"] = global_costmap_rolling_window
    global_costmap["origin_x"] = global_costmap_origin_x
    global_costmap["origin_y"] = global_costmap_origin_y
    data["bt_navigator"]["ros__parameters"][
        "bt_loop_duration"
    ] = bt_loop_duration

    output = Path("/tmp/rm2027_dog_hole_sim/nav2_dog_hole_sim.yaml")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(data, sort_keys=False),
        encoding="utf-8",
    )
    return output


def _make_heading_fusion_profile(geometry, initial_gimbal_yaw):
    cosine = math.cos(initial_gimbal_yaw)
    sine = math.sin(initial_gimbal_yaw)
    sensor_x = geometry["lidar_x"]
    sensor_y = geometry["lidar_y"]
    initial_sensor_x = cosine * sensor_x - sine * sensor_y
    initial_sensor_y = sine * sensor_x + cosine * sensor_y
    initial_sensor_z = (
        geometry["gimbal_joint_height"] + geometry["lidar_z"]
    )
    data = {
        "lio_adapter": {
            "ros__parameters": {
                "raw_odom_topic": "/odometry/fast_lio_raw",
                "output_odom_topic": "/odometry/lio",
                "expected_input_odom_frame": "odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "input_sensor_frame": "sim_lidar_link",
                "gimbal_frame": "gimbal_yaw_link",
                "publish_tf": True,
                "pose_conversion_mode": "chassis_heading_fusion",
                "raw_odom_parent_frame_mode": "sensor_initial",
                "use_tf_sensor_to_base": False,
                "use_latest_transform": False,
                "allow_placeholder_fallback": False,
                "tf_lookup_timeout_sec": 0.0,
                "tf_queue": {
                    "max_size": 100,
                    "max_wait_sec": 0.2,
                    "retry_rate_hz": 200.0,
                },
                "backend_child_frame_alias_enabled": False,
                "backend_child_frame_alias_source": "body",
                "backend_child_frame_alias_target": "sim_lidar_link",
                "twist_mode": "finite_difference",
                "twist_estimator": {
                    "min_dt_sec": 0.001,
                    "max_dt_sec": 0.5,
                    "smoothing_alpha": 1.0,
                    "max_linear_speed": 5.0,
                    "max_angular_speed": 20.0,
                },
                "twist_variance_diagonal": [
                    1.0,
                    1.0,
                    1.0,
                    4.0,
                    4.0,
                    4.0,
                ],
                "heading_fusion": {
                    "heading_topic": "/chassis/heading",
                    "derived_gimbal_topic": "/gimbal/state_derived",
                    "cache_size": 500,
                    "max_heading_match_dt_sec": 0.03,
                    "yaw_variance": 0.01,
                    "initial_gimbal_yaw_rad": initial_gimbal_yaw,
                    "gimbal_center_in_base": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": geometry["gimbal_joint_height"],
                    },
                    "initial_alignment_confirmed": True,
                    "initial_base_to_sensor": {
                        "x": initial_sensor_x,
                        "y": initial_sensor_y,
                        "z": initial_sensor_z,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": initial_gimbal_yaw,
                    },
                },
                "input_to_base_placeholder": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                },
            }
        }
    }
    output = Path(
        "/tmp/rm2027_dog_hole_sim/"
        "lio_adapter_chassis_heading_fusion_sim.yaml"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(data, sort_keys=False),
        encoding="utf-8",
    )
    return output


def _launch_setup(context):
    config_path = Path(LaunchConfiguration("dog_hole_config").perform(context))
    robot_geometry_profile = LaunchConfiguration(
        "robot_geometry_profile"
    ).perform(context)
    auto_start = LaunchConfiguration("auto_start")
    gimbal_yaw = LaunchConfiguration("gimbal_yaw")
    gimbal_motion_mode = LaunchConfiguration("gimbal_motion_mode")
    gimbal_amplitude = LaunchConfiguration("gimbal_amplitude")
    gimbal_frequency = LaunchConfiguration("gimbal_frequency")
    gimbal_angular_velocity = LaunchConfiguration("gimbal_angular_velocity")
    heading_policy = LaunchConfiguration("heading_policy").perform(context)
    initial_gimbal_yaw = float(gimbal_yaw.perform(context))
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    spawn_dog_hole_scene = LaunchConfiguration("spawn_dog_hole_scene")
    spawn_ramp_scene = LaunchConfiguration("spawn_ramp_scene")
    active_ramp_filter = LaunchConfiguration("active_ramp_filter")
    scan_output_frame = LaunchConfiguration("scan_output_frame")
    publish_sensor_truth_tf = LaunchConfiguration(
        "publish_sensor_truth_tf"
    )
    ramp_filter_config = Path(
        LaunchConfiguration("ramp_filter_config").perform(context)
    )
    robot_spawn_x = float(
        LaunchConfiguration("robot_spawn_x").perform(context)
    )
    robot_spawn_y = float(
        LaunchConfiguration("robot_spawn_y").perform(context)
    )
    robot_spawn_yaw = float(
        LaunchConfiguration("robot_spawn_yaw").perform(context)
    )
    global_costmap_width = int(
        float(LaunchConfiguration("global_costmap_width").perform(context))
    )
    global_costmap_height = int(
        float(LaunchConfiguration("global_costmap_height").perform(context))
    )
    global_costmap_update_frequency = float(
        LaunchConfiguration(
            "global_costmap_update_frequency"
        ).perform(context)
    )
    global_costmap_resolution = float(
        LaunchConfiguration("global_costmap_resolution").perform(context)
    )
    global_costmap_rolling_window = (
        LaunchConfiguration("global_costmap_rolling_window")
        .perform(context)
        .strip()
        .lower()
        == "true"
    )
    global_costmap_origin_x = float(
        LaunchConfiguration("global_costmap_origin_x").perform(context)
    )
    global_costmap_origin_y = float(
        LaunchConfiguration("global_costmap_origin_y").perform(context)
    )
    mppi_batch_size = int(
        float(LaunchConfiguration("mppi_batch_size").perform(context))
    )
    bt_loop_duration = int(
        float(LaunchConfiguration("bt_loop_duration").perform(context))
    )
    physics_max_step_size = float(
        LaunchConfiguration("physics_max_step_size").perform(context)
    )
    lidar_update_rate = float(
        LaunchConfiguration("lidar_update_rate").perform(context)
    )
    lidar_samples = int(
        float(LaunchConfiguration("lidar_samples").perform(context))
    )

    config_data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = config_data["dog_hole_manager"]["ros__parameters"]
    params["robot.geometry_profile"] = robot_geometry_profile

    manager_overrides = {}
    for launch_name, parameter_name in (
        ("dog_hole_center_x", "dog_hole.center_x"),
        ("dog_hole_center_y", "dog_hole.center_y"),
        ("dog_hole_yaw", "dog_hole.yaw"),
        ("dog_hole_width", "dog_hole.width"),
        ("dog_hole_length", "dog_hole.length"),
    ):
        value = LaunchConfiguration(launch_name).perform(context).strip()
        if value:
            params[parameter_name] = float(value)
            manager_overrides[parameter_name] = float(value)

    final_goal_values = [
        LaunchConfiguration(name).perform(context).strip()
        for name in (
            "dog_hole_final_goal_x",
            "dog_hole_final_goal_y",
            "dog_hole_final_goal_yaw",
        )
    ]
    if any(final_goal_values):
        if not all(final_goal_values):
            raise RuntimeError(
                "all dog_hole_final_goal_* values must be supplied together"
            )
        final_goal = [float(value) for value in final_goal_values]
        params["final_goal"] = final_goal
        manager_overrides["final_goal"] = final_goal

    output_dir = Path("/tmp/rm2027_dog_hole_sim")
    output_dir.mkdir(parents=True, exist_ok=True)
    scene_path = output_dir / "dog_hole_scene.sdf"
    scene_path.write_text(_make_scene(params), encoding="utf-8")
    world_text, geometry = _make_robot_world(
        params,
        robot_spawn_x,
        robot_spawn_y,
        robot_spawn_yaw,
        physics_max_step_size,
        lidar_update_rate,
        lidar_samples,
    )
    world_path = output_dir / f'new_car_{geometry["profile"]}.sdf'
    world_path.write_text(world_text, encoding="utf-8")
    nav2_path = _make_nav2_profile(
        params,
        heading_policy,
        global_costmap_width,
        global_costmap_height,
        global_costmap_update_frequency,
        global_costmap_resolution,
        global_costmap_rolling_window,
        global_costmap_origin_x,
        global_costmap_origin_y,
        mppi_batch_size,
        bt_loop_duration,
    )
    lio_adapter_path = _make_heading_fusion_profile(
        geometry, initial_gimbal_yaw
    )
    derived_gimbal_adapter_path = (
        Path(get_package_share_directory("rm_localization_adapters"))
        / "config"
        / "gimbal_state_adapter_derived.yaml"
    )
    roof_clearance = float(params["dog_hole.roof_clearance"])
    clearance = roof_clearance - geometry["height"]

    gazebo_launch = (
        Path(get_package_share_directory("rm_simulation"))
        / "launch"
        / "phase1_5_gazebo.launch.py"
    )
    ramp_scene_path = (
        Path(get_package_share_directory("rm_simulation"))
        / "models"
        / "ramp_perception_scene.sdf"
    )
    return [
        LogInfo(
            msg=(
                f'[dog_hole_sim] geometry_profile={geometry["profile"]}, '
                f'configured_height={geometry["height"]:.3f} m, '
                f'roof_clearance={roof_clearance:.3f} m, '
                f'vertical_margin={clearance:.3f} m, '
                f'heading_policy={heading_policy}'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={
                "headless": headless,
                "use_nav2": "true",
                "use_rviz": use_rviz,
                "scan_output_topic": "/simulation/scan_ramp_unfiltered",
                "scan_output_frame": scan_output_frame,
                "nav2_params": str(nav2_path),
                "world": str(world_path),
                "gimbal_use_input": "true",
                "gimbal_input_topic": "/gimbal/state",
                "gimbal_yaw": gimbal_yaw,
                "gimbal_motion_mode": gimbal_motion_mode,
                "gimbal_amplitude": gimbal_amplitude,
                "gimbal_frequency": gimbal_frequency,
                "gimbal_angular_velocity": gimbal_angular_velocity,
                "use_chassis_heading_fusion": "true",
                "lio_adapter_config": str(lio_adapter_path),
                "gimbal_state_adapter_config": str(
                    derived_gimbal_adapter_path
                ),
                "gimbal_joint_height": str(geometry["gimbal_joint_height"]),
                "sim_lidar_x": str(geometry["lidar_x"]),
                "sim_lidar_y": str(geometry["lidar_y"]),
                "sim_lidar_z": str(geometry["lidar_z"]),
                "heading_world_offset_rad": str(
                    params[
                        "simulation.localization.heading_world_offset_rad"
                    ]
                ),
                "heading_timestamp_offset_sec": str(
                    params[
                        "simulation.localization.heading_timestamp_offset_sec"
                    ]
                ),
                "publish_sensor_truth_tf": publish_sensor_truth_tf,
                "sensor_truth_parent_frame": "map",
                "sensor_truth_frame": "sim_lidar_physics_frame",
                # Synthetic FAST-LIO is intentionally zeroed at its initial
                # pose. Offset map->odom by the Gazebo spawn pose so map-bound
                # field geometry and localization share one coordinate system.
                "use_identity_map_odom_stub": "false",
                "map_to_odom_x": str(robot_spawn_x),
                "map_to_odom_y": str(robot_spawn_y),
                "map_to_odom_yaw": str(robot_spawn_yaw),
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
            package="rm_mid360_driver_bridge",
            executable="ramp_laserscan_filter_node",
            name="ramp_laserscan_filter_node",
            output="screen",
            parameters=[
                str(ramp_filter_config),
                {
                    "filter_enabled": ParameterValue(
                        active_ramp_filter, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="rm_dog_hole",
            executable="dog_hole_manager",
            name="dog_hole_manager",
            output="screen",
            parameters=[
                str(config_path),
                manager_overrides,
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
                    condition=IfCondition(spawn_dog_hole_scene),
                    arguments=[
                        "-world",
                        "phase1_omni",
                        "-file",
                        str(scene_path),
                        "-name",
                        "dog_hole_scene",
                        "-x",
                        str(params["dog_hole.center_x"]),
                        "-y",
                        str(params["dog_hole.center_y"]),
                        "-Y",
                        str(params["dog_hole.yaw"]),
                    ],
                )
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_ramp_perception_scene",
                    output="screen",
                    condition=IfCondition(spawn_ramp_scene),
                    arguments=[
                        "-world",
                        "phase1_omni",
                        "-file",
                        str(ramp_scene_path),
                        "-name",
                        "ramp_perception_scene",
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
    default_ramp_filter_config = (
        Path(get_package_share_directory("rm_mid360_driver_bridge"))
        / "config"
        / "ramp_laserscan_filter_sim.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "spawn_dog_hole_scene", default_value="true"
            ),
            DeclareLaunchArgument("spawn_ramp_scene", default_value="true"),
            DeclareLaunchArgument("active_ramp_filter", default_value="true"),
            DeclareLaunchArgument(
                "scan_output_frame", default_value="sim_lidar_link"
            ),
            DeclareLaunchArgument(
                "publish_sensor_truth_tf", default_value="false"
            ),
            DeclareLaunchArgument(
                "ramp_filter_config",
                default_value=str(default_ramp_filter_config),
            ),
            DeclareLaunchArgument("robot_spawn_x", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("global_costmap_width", default_value="12.0"),
            DeclareLaunchArgument("global_costmap_height", default_value="12.0"),
            DeclareLaunchArgument(
                "global_costmap_update_frequency", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "global_costmap_resolution", default_value="0.05"
            ),
            DeclareLaunchArgument(
                "global_costmap_rolling_window", default_value="true"
            ),
            DeclareLaunchArgument(
                "global_costmap_origin_x", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "global_costmap_origin_y", default_value="0.0"
            ),
            DeclareLaunchArgument("mppi_batch_size", default_value="1000"),
            DeclareLaunchArgument("bt_loop_duration", default_value="10"),
            DeclareLaunchArgument(
                "physics_max_step_size", default_value="0.001"
            ),
            DeclareLaunchArgument("lidar_update_rate", default_value="15.0"),
            DeclareLaunchArgument("lidar_samples", default_value="720"),
            DeclareLaunchArgument("dog_hole_center_x", default_value=""),
            DeclareLaunchArgument("dog_hole_center_y", default_value=""),
            DeclareLaunchArgument("dog_hole_yaw", default_value=""),
            DeclareLaunchArgument("dog_hole_width", default_value=""),
            DeclareLaunchArgument("dog_hole_length", default_value=""),
            DeclareLaunchArgument(
                "dog_hole_final_goal_x", default_value=""
            ),
            DeclareLaunchArgument(
                "dog_hole_final_goal_y", default_value=""
            ),
            DeclareLaunchArgument(
                "dog_hole_final_goal_yaw", default_value=""
            ),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument(
                "robot_geometry_profile", default_value="deformed"
            ),
            DeclareLaunchArgument("gimbal_yaw", default_value="0.65"),
            DeclareLaunchArgument(
                "gimbal_motion_mode", default_value="continuous"
            ),
            DeclareLaunchArgument("gimbal_amplitude", default_value="0.8"),
            DeclareLaunchArgument("gimbal_frequency", default_value="0.10"),
            DeclareLaunchArgument(
                "gimbal_angular_velocity", default_value="0.60"
            ),
            DeclareLaunchArgument(
                "heading_policy",
                default_value="baseline",
                choices=["baseline", "path_aligned"],
            ),
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
