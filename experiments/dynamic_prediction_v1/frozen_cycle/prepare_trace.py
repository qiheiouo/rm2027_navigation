#!/usr/bin/env python3
"""Create isolated, observer-only Nav2 1.1.20 and V1 critic source copies.

Input is the SHA-pinned upstream source from the existing MPPI diagnostic.
The normal V1 critic and installed Nav2 libraries are never edited.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[3]
PRIOR = ROOT / "docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922"
PLUGIN = ROOT / "experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def insert(path, old, new):
    content = path.read_text()
    count = content.count(old)
    if count != 1:
        raise ValueError(f"expected one anchor in {path}: {old!r}; found {count}")
    path.write_text(content.replace(old, new))


def prepare(upstream_root, output):
    if output.exists():
        raise FileExistsError("diagnostic source output exists; refusing to overwrite")
    pinned = json.loads((PRIOR / "upstream.json").read_text())
    for relative, digest in pinned["files"].items():
        path = upstream_root / relative
        if sha(path) != digest:
            raise ValueError(f"Nav2 1.1.20 upstream hash mismatch: {relative}")
    output.mkdir(parents=True)
    nav = output / "nav2_mppi_controller"
    plugin = output / "rm_dynamic_prediction_critic"
    shutil.copytree(upstream_root / "nav2_mppi_controller", nav)
    shutil.copytree(PLUGIN, plugin, ignore=shutil.ignore_patterns("__pycache__"))
    for name in ("controller", "optimizer", "critic_manager", "critics/cost_critic"):
        path = nav / "src" / f"{name}.cpp"
        path.write_text('#include "nav2_mppi_controller/cycle_trace.hpp"\n' + path.read_text())
    # Reuse the already reviewed asynchronous writer and raw-map snapshot.
    shutil.copy2(PRIOR / "cycle_trace.cpp", nav / "src/cycle_trace.cpp")
    shutil.copy2(PRIOR / "cycle_trace.hpp", nav / "include/nav2_mppi_controller/cycle_trace.hpp")
    c = nav / "src/controller.cpp"
    insert(c, "  optimizer_.shutdown();", "  optimizer_.shutdown();\n  tdt_trace::shutdown();")
    insert(c, "  nav2_costmap_2d::Costmap2D * costmap =",
           "  tdt_trace::Cycle trace(robot_pose, robot_speed, clock_);\n  nav2_costmap_2d::Costmap2D * costmap =")
    insert(c, "  geometry_msgs::msg::TwistStamped cmd =",
           "  trace.locked(costmap_ros_, transformed_plan);\n  geometry_msgs::msg::TwistStamped cmd =")
    insert(c, "  return cmd;", "  trace.output(cmd);\n  return cmd;")
    o = nav / "src/optimizer.cpp"
    insert(o, "  prepare(robot_pose, robot_speed, plan, goal_checker);",
           "  prepare(robot_pose, robot_speed, plan, goal_checker);\n"
           "  tdt_trace::controls(\"initial\", control_sequence_);\n"
           "  tdt_trace::value(\"settings\", {{\"dt\", settings_.model_dt}, "
           "{\"batch\", settings_.batch_size}, {\"steps\", settings_.time_steps}, "
           "{\"iterations\", settings_.iteration_count}, {\"temperature\", settings_.temperature}, "
           "{\"gamma\", settings_.gamma}, {\"vx_std\", settings_.sampling_std.vx}, "
           "{\"vy_std\", settings_.sampling_std.vy}, {\"wz_std\", settings_.sampling_std.wz}, "
           "{\"vx_min\", settings_.constraints.vx_min}, {\"vx_max\", settings_.constraints.vx_max}, "
           "{\"vy_max\", settings_.constraints.vy}, {\"wz_max\", settings_.constraints.wz}, "
           "{\"offset\", settings_.shift_control_sequence ? 1 : 0}});")
    insert(o, "  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);",
           "  tdt_trace::controls(\"before_filter\", control_sequence_);\n"
           "  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);\n"
           "  tdt_trace::controls(\"after_filter\", control_sequence_);")
    insert(o, "    generateNoisedTrajectories();",
           "    generateNoisedTrajectories();\n"
           "    tdt_trace::tensor(\"sampled.cvx\", state_.cvx);\n"
           "    tdt_trace::tensor(\"sampled.cvy\", state_.cvy);\n"
           "    tdt_trace::tensor(\"sampled.cwz\", state_.cwz);\n"
           "    tdt_trace::tensor(\"sampled.vx\", state_.vx);\n"
           "    tdt_trace::tensor(\"sampled.vy\", state_.vy);\n"
           "    tdt_trace::tensor(\"sampled.wz\", state_.wz);")
    insert(o, "    updateControlSequence();",
           "    tdt_trace::scored(critics_data_);\n"
           "    updateControlSequence();\n"
           "    tdt_trace::tensor(\"weighted.costs\", costs_);\n"
           "    tdt_trace::controls(\"after_update\", control_sequence_);")
    insert(o, "  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));",
           "  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));\n"
           "  tdt_trace::tensor(\"weighted.probability\", softmaxes);")
    cm = nav / "src/critic_manager.cpp"
    insert(cm, "    critics_[q]->score(data);",
           "    critics_[q]->score(data);\n"
           "    tdt_trace::tensor(\"critic.\" + critics_[q]->getName(), data.costs);\n"
           "    tdt_trace::value(\"critic.fail\", data.fail_flag);")
    cost = nav / "src/critics/cost_critic.cpp"
    insert(cost, "  bool all_trajectories_collide = true;",
           "  std::vector<uint8_t> trace_collisions(tdt_trace::active() ? data.trajectories.x.shape(0) : 0);\n"
           "  bool all_trajectories_collide = true;")
    insert(cost, "    if (!trajectory_collide) {",
           "    if (!trace_collisions.empty()) trace_collisions[i] = trajectory_collide;\n"
           "    if (!trajectory_collide) {")
    insert(cost, "  data.costs += xt::pow",
           "  if (tdt_trace::active()) tdt_trace::collision_mask(trace_collisions, near_goal);\n"
           "  data.costs += xt::pow")
    cmake = nav / "CMakeLists.txt"
    insert(cmake, "  src/controller.cpp", "  src/cycle_trace.cpp\n  src/controller.cpp")
    insert(cmake, "set(libraries mppi_controller mppi_critics)",
           "target_link_libraries(mppi_critics mppi_controller)\nset(libraries mppi_controller mppi_critics)")
    source = plugin / "src/prediction_critic.cpp"
    source.write_text('#include "nav2_mppi_controller/cycle_trace.hpp"\n' + source.read_text())
    insert(source, "    if (tracks.empty()) {log_skip(\"no confirmed track\");return;}",
           "    if (tracks.empty()) {log_skip(\"no confirmed track\");return;}\n"
           "    if (tdt_trace::active()) {\n"
           "      tdt_trace::Json tracks_json=tdt_trace::Json::array();\n"
           "      for (const auto & tr:message->tracks) {\n"
           "        tdt_trace::Json future=tdt_trace::Json::array();\n"
           "        for (const auto & p:tr.prediction) future.push_back({p.x,p.y,p.z});\n"
           "        tracks_json.push_back({{\"id\",tr.track_id},{\"state\",tr.state},\n"
           "          {\"xy\",{tr.position.x,tr.position.y}},\n"
           "          {\"position_z\",tr.position.z},\n"
           "          {\"vxy\",{tr.velocity.x,tr.velocity.y}},\n"
           "          {\"velocity_z\",tr.velocity.z},\n"
           "          {\"size_xy\",{tr.size.x,tr.size.y}},\n"
           "          {\"size_z\",tr.size.z},\n"
           "          {\"last_observation_stamp_ns\",rclcpp::Time(tr.last_observation_stamp,RCL_ROS_TIME).nanoseconds()},\n"
           "          {\"observation_count\",tr.observation_count},{\"miss_count\",tr.miss_count},\n"
           "          {\"prediction_xyz\",future}});\n"
           "      }\n"
           "      const double source_t=rclcpp::Time(message->header.stamp,RCL_ROS_TIME).seconds();\n"
           "      tdt_trace::value(\"prediction.input\",{{\"status\",\"accepted\"},\n"
           "        {\"schema\",message->schema},{\"authority\",message->authority},\n"
           "        {\"complete\",message->complete},{\"frame\",message->header.frame_id},\n"
           "        {\"source_stamp_ns\",rclcpp::Time(message->header.stamp,RCL_ROS_TIME).nanoseconds()},\n"
           "        {\"processing_stamp_ns\",rclcpp::Time(message->processing_stamp,RCL_ROS_TIME).nanoseconds()},\n"
           "        {\"source_age_s\",age},{\"consumer_sim_s\",source_t+age},\n"
           "        {\"prediction_dt\",message->prediction_dt},\n"
           "        {\"prediction_steps\",message->prediction_steps},\n"
           "        {\"total_track_count\",message->total_track_count},\n"
           "        {\"tracks\",tracks_json}});\n"
           "    }")
    insert(source, "    if (node) RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 2000,",
           "    if (tdt_trace::active()) tdt_trace::value(\"prediction.skip\",reason);\n"
           "    if (node) RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 2000,")
    # ament_target_dependencies already resolves nav2_mppi_controller's
    # exported absolute library path. A bare -lmppi_controller would fail.
    manifest = {
        "schema": "rm_dynamic_prediction_trace_sources/v1",
        "upstream_archive_sha256": pinned["archive_sha256"],
        "original_plugin_source_sha256": sha(PLUGIN / "src/prediction_critic.cpp"),
        "patch_script_sha256": sha(Path(__file__)),
        "observer_source_sha256": {name: sha(PRIOR / name) for name in ("cycle_trace.cpp", "cycle_trace.hpp")},
        "instrumented_source_sha256": {
            str(path.relative_to(output)): sha(path) for tree in (nav, plugin)
            for path in tree.rglob("*") if path.is_file()
        },
    }
    (output / "trace_sources.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream_root", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    result = prepare(args.upstream_root, args.output_dir)
    print(json.dumps({"files": len(result["instrumented_source_sha256"]),
                      "output": str(args.output_dir)}, indent=2))
