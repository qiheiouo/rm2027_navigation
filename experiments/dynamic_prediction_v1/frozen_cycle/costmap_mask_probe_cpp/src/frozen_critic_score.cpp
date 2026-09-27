// Score frozen sampled rollouts with installed Nav2 critics except V1.
#include <cmath>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using Json = nlohmann::json;

namespace
{
template<class T>
T read(std::ifstream & file)
{
  T value{};
  file.read(reinterpret_cast<char *>(&value), sizeof(value));
  if (!file) {throw std::runtime_error("truncated input");}
  return value;
}

std::vector<rclcpp::Parameter> parameters(const Json & values)
{
  std::vector<rclcpp::Parameter> result;
  for (auto it = values.begin(); it != values.end(); ++it) {
    const auto & value = it.value();
    if (value.is_boolean()) {
      result.emplace_back(it.key(), value.get<bool>());
    } else if (value.is_number_integer()) {
      result.emplace_back(it.key(), value.get<int>());
    } else if (value.is_number()) {
      result.emplace_back(it.key(), value.get<double>());
    } else if (value.is_string()) {
      result.emplace_back(it.key(), value.get<std::string>());
    } else if (value.is_array()) {
      result.emplace_back(it.key(), value.get<std::vector<std::string>>());
    } else {
      throw std::runtime_error("unsupported parameter: " + it.key());
    }
  }
  return result;
}

geometry_msgs::msg::PoseStamped pose(const Json & xyz, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame;
  result.pose.position.x = xyz.at(0).get<double>();
  result.pose.position.y = xyz.at(1).get<double>();
  const double yaw = xyz.at(2).get<double>();
  result.pose.orientation.z = std::sin(yaw / 2);
  result.pose.orientation.w = std::cos(yaw / 2);
  return result;
}

class FrozenOptimizer : public mppi::Optimizer
{
public:
  std::vector<float> score(
    const geometry_msgs::msg::PoseStamped & robot,
    const geometry_msgs::msg::Twist & speed,
    const nav_msgs::msg::Path & path, std::ifstream & map_file,
    std::ifstream & controls_file, uint32_t batch)
  {
    prepare(robot, speed, path, nullptr);
    const auto magic = read<uint32_t>(controls_file);
    const auto controls_count = read<uint32_t>(controls_file);
    const auto controls_steps = read<uint32_t>(controls_file);
    if (magic != 0x43545231 || controls_count != batch ||
      controls_steps != settings_.time_steps)
    {throw std::runtime_error("sampled control header mismatch");}
    for (uint32_t i = 0; i < batch; ++i) {
      for (uint32_t j = 0; j < settings_.time_steps; ++j) {
        state_.cvx(i, j) = read<float>(controls_file);
        state_.cvy(i, j) = read<float>(controls_file);
        state_.cwz(i, j) = read<float>(controls_file);
      }
    }
    for (uint32_t i = 0; i < batch; ++i) {
      state_.vx(i, 0) = static_cast<float>(speed.linear.x);
      state_.vy(i, 0) = static_cast<float>(speed.linear.y);
      state_.wz(i, 0) = static_cast<float>(speed.angular.z);
      for (uint32_t j = 1; j < settings_.time_steps; ++j) {
        state_.vx(i, j) = state_.cvx(i, j - 1);
        state_.vy(i, j) = state_.cvy(i, j - 1);
        state_.wz(i, j) = state_.cwz(i, j - 1);
      }
    }
    for (uint32_t i = 0; i < batch; ++i) {
      for (uint32_t j = 0; j < settings_.time_steps; ++j) {
        generated_trajectories_.x(i, j) = read<float>(map_file);
        generated_trajectories_.y(i, j) = read<float>(map_file);
        generated_trajectories_.yaws(i, j) = read<float>(map_file);
      }
    }
    critic_manager_.evalTrajectoriesScores(critics_data_);
    if (critics_data_.fail_flag) {throw std::runtime_error("critic failure flag");}
    return std::vector<float>(costs_.begin(), costs_.end());
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr << "usage: frozen_critic_score meta.json map_and_poses.bin controls.bin output.bin\n";
    return 2;
  }
  try {
    rclcpp::init(argc, argv);
    const auto meta = Json::parse(std::ifstream(argv[1]));
    const auto frame = meta.at("map_frame").get<std::string>();
    const auto batch = meta.at("batch").get<uint32_t>();
    std::ifstream map_file(argv[2], std::ios::binary);
    std::ifstream controls_file(argv[3], std::ios::binary);
    if (!map_file || !controls_file) {throw std::runtime_error("cannot open fixture");}
    if (read<uint32_t>(map_file) != 0x43504D31) {
      throw std::runtime_error("map fixture magic mismatch");
    }
    const auto width = read<uint32_t>(map_file);
    const auto height = read<uint32_t>(map_file);
    const auto pose_count = read<uint32_t>(map_file);
    const auto steps = read<uint32_t>(map_file);
    const auto vertices = read<uint32_t>(map_file);
    (void)read<uint32_t>(map_file);  // validated inflation shortcut threshold
    const auto resolution = read<double>(map_file);
    const auto origin_x = read<double>(map_file);
    const auto origin_y = read<double>(map_file);
    if (batch > pose_count || width == 0 || height == 0 || vertices < 3 ||
      steps != 30)
    {throw std::runtime_error("fixture dimensions differ from frozen profile");}
    std::vector<geometry_msgs::msg::Point> footprint;
    for (uint32_t i = 0; i < vertices; ++i) {
      geometry_msgs::msg::Point point;
      point.x = read<double>(map_file);
      point.y = read<double>(map_file);
      footprint.push_back(point);
    }
    nav2_costmap_2d::Costmap2D raw(
      width, height, resolution, origin_x, origin_y, 255);
    map_file.read(reinterpret_cast<char *>(raw.getCharMap()), width * height);
    if (!map_file) {throw std::runtime_error("raw map truncated");}

    rclcpp::NodeOptions costmap_options;
    costmap_options.parameter_overrides(parameters(meta.at("costmap_parameters")));
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      costmap_options);
    costmap->on_configure(rclcpp_lifecycle::State{});
    *costmap->getCostmap() = raw;
    const auto actual_footprint = costmap->getRobotFootprint();
    if (actual_footprint.size() != footprint.size()) {
      throw std::runtime_error("configured footprint vertex count differs");
    }
    for (size_t i = 0; i < footprint.size(); ++i) {
      if (std::hypot(actual_footprint[i].x - footprint[i].x,
        actual_footprint[i].y - footprint[i].y) > 1e-5)
      {throw std::runtime_error("configured padded footprint differs");}
    }

    {
      rclcpp::NodeOptions options;
      options.parameter_overrides(parameters(meta.at("parameters")));
      auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "frozen_replay", options);
      mppi::ParametersHandler handler(node);
      FrozenOptimizer optimizer;
      optimizer.initialize(node, "FollowPath", costmap, &handler);
      auto robot = pose(meta.at("pose"), frame);
      geometry_msgs::msg::Twist speed;
      speed.linear.x = meta.at("speed").at(0).get<double>();
      speed.linear.y = meta.at("speed").at(1).get<double>();
      speed.angular.z = meta.at("speed").at(2).get<double>();
      nav_msgs::msg::Path path;
      path.header.frame_id = frame;
      for (const auto & item : meta.at("path")) {
        path.poses.push_back(pose(item, frame));
      }
      const auto score_start = std::chrono::steady_clock::now();
      const auto scores = optimizer.score(
        robot, speed, path, map_file, controls_file, batch);
      const auto score_end = std::chrono::steady_clock::now();
      std::cout << "score_eval_ms=" <<
        std::chrono::duration<double, std::milli>(score_end - score_start).count() << '\n';
      std::ofstream output(argv[4], std::ios::binary);
      if (!output) {throw std::runtime_error("cannot open score output");}
      for (const auto score : scores) {
        output.write(reinterpret_cast<const char *>(&score), sizeof(score));
      }
    }
    costmap->on_cleanup(rclcpp_lifecycle::State{});
    rclcpp::shutdown();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
  return 0;
}
