#!/usr/bin/env python3
"""Generate isolated, hashable fixtures from unchanged main SDF and ROS profiles."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from fixture_profile import fixture_profile


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(output, scenario="crossing", profile="legacy"):
    fixture=fixture_profile(profile)
    repo = Path(__file__).resolve().parents[3]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    original = repo / "src/rm_simulation/worlds/phase1_omni.sdf"
    # Gazebo reads this extension by literal ignition prefix; ElementTree ns0
    # renaming silently removes the wheel friction reference-frame contract.
    ET.register_namespace("ignition", "http://ignitionrobotics.org/schema")
    tree = ET.parse(original)
    world = tree.getroot().find("world")
    world.remove(world.find("model[@name='center_block']"))
    actor_source = repo / "src/rm_simulation/models/moving_obstacle.sdf"
    actor = ET.parse(actor_source).getroot().find("model")
    ET.SubElement(actor, "pose").text = "4.9 0 0 0 0 0"
    if scenario == "head_on":
        actor.find("joint[@name='slider_joint']/axis/xyz").text = "1 0 0"
        limits = actor.find("joint[@name='slider_joint']/axis/limit")
        limits.find("lower").text = "-3.5"
        limits.find("upper").text = "0.95"
    if scenario == "course":
        wall_source = repo / "src/rm_simulation/models/course_wall.sdf"
        for name, y in (("course_wall_left", .525), ("course_wall_right", -.525)):
            wall = ET.parse(wall_source).getroot().find("model")
            wall.set("name", name)
            ET.SubElement(wall, "pose").text = f"3 {y} 0 0 0 0"
            world.append(wall)
    if scenario != "actuator":
        world.append(actor)
        sensor=ET.SubElement(actor.find("link[@name='obstacle_link']"),"sensor",{ "name":"actor_contact", "type":"contact"})
        ET.SubElement(sensor,"update_rate").text="100"
        contact=ET.SubElement(sensor,"contact")
        ET.SubElement(contact,"collision").text="obstacle_collision"
        ET.SubElement(contact,"topic").text="/simulation/oracle/contacts"
        ET.SubElement(sensor,"always_on").text="1"
        ET.SubElement(world,"plugin",{"filename":"ignition-gazebo-contact-system", "name":"gz::sim::systems::Contact"})
    for model in (world.find("model[@name='rm_sentry_2027']"), actor):
        plugin = ET.SubElement(model, "plugin", {
            "filename": "ignition-gazebo-pose-publisher-system",
            "name": "gz::sim::systems::PosePublisher"})
        for key, value in {"publish_link_pose":"true", "publish_model_pose":"true",
                           "publish_collision_pose":"false", "publish_visual_pose":"false",
                           "publish_nested_model_pose":"true", "use_pose_vector_msg":"true",
                           "update_frequency":"50"}.items():
            ET.SubElement(plugin, key).text = value
    tree.write(output / "world.sdf", encoding="unicode", xml_declaration=True)
    config = yaml.safe_load((repo / "experiments/temporal_mpc/integration/dual_controller_humble.yaml").read_text())
    def sim_time(value):
        if isinstance(value, dict):
            for key in value:
                if key == "use_sim_time": value[key] = True
                else: sim_time(value[key])
    sim_time(config)
    # Python action acknowledgement allowance; control/solver deadlines stay unchanged.
    config["bt_navigator"]["ros__parameters"]["default_server_timeout"] = 1000
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    local.update(rolling_window=True, width=8, height=8, track_unknown_space=False,
                 plugins=["static_layer", "stvl_layer", "inflation_layer"])
    old = yaml.safe_load((repo / "src/rm_nav_config/config/nav2_old_car_2026_left_stvl.yaml").read_text())
    stvl = old["local_costmap"]["local_costmap"]["ros__parameters"]["stvl_layer"]
    stvl["observation_sources"] = "scan_mark scan_clear"
    mark = stvl.pop("left_mid360_mark")
    clear = stvl.pop("left_mid360_clear")
    mark.update(topic="/simulation/scan_points", obstacle_min_range=.12, obstacle_max_range=6.)
    clear.update(topic="/simulation/scan_points", horizontal_fov_angle=6.283185,
                 vertical_fov_angle=.10, vertical_fov_offset=0.)
    stvl["scan_mark"], stvl["scan_clear"] = mark, clear
    local["stvl_layer"] = stvl
    smoother = config["velocity_smoother"]["ros__parameters"]
    smoother.update(max_velocity=[.8,.5,0.], min_velocity=[-.5,-.5,0.], velocity_timeout=.15)
    (output / "nav2.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    tracker = yaml.safe_load((repo / "experiments/temporal_mpc/ros2/rm_dynamic_obstacle_tracking/config/dynamic_obstacle_tracking_shadow.yaml").read_text())
    parameters = tracker["dynamic_obstacle_tracker_shadow"]["ros__parameters"]
    parameters.update(use_sim_time=True, scan_topic="/scan", predictions_topic="/dynamic_obstacle_predictions")
    parameters["prediction"].update(anchor_mode="last_observation_cv", velocity_decay_tau=0., max_speed=0.)
    (output / "tracker.yaml").write_text(yaml.safe_dump(tracker, sort_keys=False))
    bridge = yaml.safe_load((repo / "src/rm_simulation/config/ros_gz_bridge.yaml").read_text())
    bridge += yaml.safe_load((repo / "src/rm_simulation/config/course_dynamic_bridge.yaml").read_text())
    for model in ("rm_sentry_2027", "moving_obstacle"):
        bridge.append(dict(ros_topic_name=f"/simulation/oracle/{model}",
                           gz_topic_name=f"/model/{model}/pose", ros_type_name="tf2_msgs/msg/TFMessage",
                           gz_type_name="ignition.msgs.Pose_V", direction="GZ_TO_ROS"))
    bridge.append(dict(ros_topic_name="/simulation/oracle/world",
                       gz_topic_name="/world/phase1_omni/dynamic_pose/info", ros_type_name="tf2_msgs/msg/TFMessage",
                       gz_type_name="ignition.msgs.Pose_V", direction="GZ_TO_ROS"))
    bridge.append(dict(ros_topic_name="/simulation/oracle/contacts",
                       gz_topic_name="/simulation/oracle/contacts", ros_type_name="ros_gz_interfaces/msg/Contacts",
                       gz_type_name="ignition.msgs.Contacts", direction="GZ_TO_ROS"))
    # These oracle messages are deliberately NOT /tf and never fed into TF buffers.
    (output / "bridge.yaml").write_text(yaml.safe_dump(bridge, sort_keys=False))
    manifest = dict(scenario=scenario, goal=fixture['goal'], fixture_profile=profile, map=fixture,
                    goal_phase_s=10., period_s=8.,
                    robot_half_extent=[.325,.300], actor_half_extent=[.225,.275],
                    geometry_mode="nominal_diameter", clearance_gate_m=.05,
                    execution_guard=scenario != "actuator",execution_model="fixed_yaw_velocity_zoh/v1",
                    source_inputs={str(p.relative_to(repo)):digest(p) for p in (original,actor_source)},
                    generated={p.name:digest(p) for p in output.iterdir() if p.is_file()},
                    oracle="model-relative PosePublisher links composed with model pose; no TF authority")
    (output / "scene.json").write_text(json.dumps(manifest,indent=2)+"\n")

if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--scenario", choices=["crossing","head_on","course","actuator"], default="crossing")
    parser.add_argument("--profile", choices=["legacy","open_long"], default="legacy")
    args=parser.parse_args()
    prepare(args.output,args.scenario,args.profile)
