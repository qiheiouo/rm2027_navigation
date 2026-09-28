// Replay a frozen batch through native MPPI integration, critics and aggregation.
// Frozen accepted prediction input drives a mirror of the V1 geometry score.
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
#include "rm_dynamic_prediction_critic/geometry.hpp"

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

struct ReplayResult
{
  std::vector<float> scores;
  std::vector<float> prediction_scores;
  std::vector<float> full_scores;
  std::vector<float> poses;
  std::vector<float> before_filter;
  std::vector<float> after_filter;
  double integrate_ms{};
  double score_ms{};
  double prediction_ms{};
  double aggregate_ms{};
  double filter_ms{};
};

std::vector<float> score_prediction(
  const Json & meta, const mppi::models::Trajectories & trajectories,
  const std::vector<geometry_msgs::msg::Point> & footprint_msg, float dt)
{
  namespace geometry = rm_dynamic_prediction_critic;
  const auto & input = meta.at("prediction_input");
  const auto & params = meta.at("prediction_parameters");
  if (input.at("status") != "accepted" || !input.at("complete").get<bool>() ||
    input.at("schema") != "rm_dynamic_obstacle_predictions/v1" ||
    input.at("authority") != "shadow_only" ||
    input.at("frame") != meta.at("map_frame"))
  {throw std::runtime_error("prediction input differs from consumed V1 frame");}
  const double age = input.at("source_age_s").get<double>();
  const double horizon = params.at("horizon").get<double>();
  const double max_age = params.at("max_age").get<double>();
  const double width = params.at("object_width").get<double>();
  const double height = params.at("object_height").get<double>();
  const double acceleration = params.at("reference_acceleration").get<double>();
  if (age < 0 || age > max_age || !(dt > 0) ||
    params.value("collision_rank_mode", std::string("legacy")) != "legacy")
  {throw std::runtime_error("frozen V1 profile differs from legacy accepted mode");}
  const size_t steps = std::min(trajectories.x.shape(1),
    static_cast<size_t>(std::floor(horizon / dt + 1e-9)));
  if (steps == 0) {throw std::runtime_error("prediction horizon is empty");}
  struct Track {geometry::Point center, velocity, visible;};
  std::vector<Track> tracks;
  for (const auto & item : input.at("tracks")) {
    if (item.at("state").get<int>() != 2) {continue;}
    tracks.push_back({
      {item.at("xy").at(0).get<double>(), item.at("xy").at(1).get<double>()},
      {item.at("vxy").at(0).get<double>(), item.at("vxy").at(1).get<double>()},
      {item.at("size_xy").at(0).get<double>(), item.at("size_xy").at(1).get<double>()}
    });
  }
  if (tracks.empty()) {throw std::runtime_error("no confirmed frozen V1 track");}
  std::vector<geometry::Point> footprint;
  for (const auto & point : footprint_msg) {
    footprint.push_back({point.x, point.y});
  }
  std::vector<std::vector<geometry::Box>> boxes(steps);
  for (size_t step = 0; step < steps; ++step) {
    for (const auto & track : tracks) {
      boxes[step].push_back(geometry::predicted_box(
        track.center, track.velocity, track.visible, {width, height},
        age, (step + 1) * static_cast<double>(dt), acceleration));
    }
  }
  std::vector<float> scores;
  scores.reserve(trajectories.x.shape(0));
  for (size_t row = 0; row < trajectories.x.shape(0); ++row) {
    bool collision = false;
    double repulsive = 0;
    for (size_t step = 0; step < steps && !collision; ++step) {
      const auto polygon = geometry::transform(
        footprint, trajectories.x(row, step), trajectories.y(row, step),
        trajectories.yaws(row, step));
      for (const auto & box : boxes[step]) {
        const double clearance = geometry::polygon_box_distance(polygon, box);
        if (clearance <= 1e-9) {collision = true; break;}
        if (clearance < 0.02) {repulsive += 300.0;}
      }
    }
    if (collision) {repulsive = 1000000.0;}
    scores.push_back(static_cast<float>((3.81 / 254.0) * repulsive / steps));
  }
  return scores;
}

void write_floats(const std::string & path, const std::vector<float> & values)
{
  std::ofstream output(path, std::ios::binary);
  if (!output) {throw std::runtime_error("cannot open output: " + path);}
  output.write(reinterpret_cast<const char *>(values.data()),
    values.size() * sizeof(float));
  if (!output) {throw std::runtime_error("cannot write output: " + path);}
}

class FrozenOptimizer : public mppi::Optimizer
{
public:
  ReplayResult replay(
    const geometry_msgs::msg::PoseStamped & robot,
    const geometry_msgs::msg::Twist & speed,
    const nav_msgs::msg::Path & path, const Json & meta,
    std::ifstream & controls_file,
    const std::vector<geometry_msgs::msg::Point> & footprint, uint32_t batch)
  {
    prepare(robot, speed, path, nullptr);
    const auto & initial = meta.at("initial_controls");
    const auto & history = meta.at("history_controls");
    if (initial.size() != settings_.time_steps || history.size() != 4) {
      throw std::runtime_error("initial controls or filter history length differs");
    }
    for (size_t j = 0; j < settings_.time_steps; ++j) {
      if (initial.at(j).size() != 3) {throw std::runtime_error("initial control width differs");}
      control_sequence_.vx(j) = initial.at(j).at(0).get<float>();
      control_sequence_.vy(j) = initial.at(j).at(1).get<float>();
      control_sequence_.wz(j) = initial.at(j).at(2).get<float>();
    }
    for (size_t j = 0; j < 4; ++j) {
      if (history.at(j).size() != 3) {throw std::runtime_error("history width differs");}
      control_history_[j] = {history.at(j).at(0).get<float>(),
        history.at(j).at(1).get<float>(), history.at(j).at(2).get<float>()};
    }
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
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    updateStateVelocities(state_);
    integrateStateVelocities(generated_trajectories_, state_);
    const auto integrated = Clock::now();
    ReplayResult result;
    critic_manager_.evalTrajectoriesScores(critics_data_);
    if (critics_data_.fail_flag) {throw std::runtime_error("critic failure flag");}
    const auto scored = Clock::now();
    result.scores.assign(costs_.begin(), costs_.end());
    result.prediction_scores = score_prediction(
      meta, generated_trajectories_, footprint, settings_.model_dt);
    for (uint32_t i = 0; i < batch; ++i) {
      costs_(i) += result.prediction_scores[i];
    }
    const auto predicted = Clock::now();
    result.full_scores.assign(costs_.begin(), costs_.end());
    const auto aggregate_start = Clock::now();
    updateControlSequence();
    const auto aggregated = Clock::now();
    for (size_t j = 0; j < settings_.time_steps; ++j) {
      result.before_filter.push_back(control_sequence_.vx(j));
      result.before_filter.push_back(control_sequence_.vy(j));
      result.before_filter.push_back(control_sequence_.wz(j));
    }
    const auto filter_start = Clock::now();
    mppi::utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
    const auto filtered = Clock::now();
    for (size_t j = 0; j < settings_.time_steps; ++j) {
      result.after_filter.push_back(control_sequence_.vx(j));
      result.after_filter.push_back(control_sequence_.vy(j));
      result.after_filter.push_back(control_sequence_.wz(j));
    }
    result.poses.reserve(batch * settings_.time_steps * 3);
    for (uint32_t i = 0; i < batch; ++i) {
      for (uint32_t j = 0; j < settings_.time_steps; ++j) {
        result.poses.push_back(generated_trajectories_.x(i, j));
        result.poses.push_back(generated_trajectories_.y(i, j));
        result.poses.push_back(generated_trajectories_.yaws(i, j));
      }
    }
    result.integrate_ms = std::chrono::duration<double, std::milli>(integrated - start).count();
    result.score_ms = std::chrono::duration<double, std::milli>(scored - integrated).count();
    result.prediction_ms = std::chrono::duration<double, std::milli>(predicted - scored).count();
    result.aggregate_ms = std::chrono::duration<double, std::milli>(aggregated - aggregate_start).count();
    result.filter_ms = std::chrono::duration<double, std::milli>(filtered - filter_start).count();
    return result;
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr << "usage: frozen_cycle_replay meta.json map_and_poses.bin controls.bin output_prefix\n";
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
      const auto result = optimizer.replay(
        robot, speed, path, meta, controls_file, actual_footprint, batch);
      const std::string prefix = argv[4];
      write_floats(prefix + "_scores.bin", result.scores);
      write_floats(prefix + "_prediction_scores.bin", result.prediction_scores);
      write_floats(prefix + "_full_scores.bin", result.full_scores);
      write_floats(prefix + "_poses.bin", result.poses);
      write_floats(prefix + "_before_filter.bin", result.before_filter);
      write_floats(prefix + "_after_filter.bin", result.after_filter);
      Json timing = {{"integrate_ms", result.integrate_ms},
        {"score_ms", result.score_ms},
        {"prediction_ms", result.prediction_ms},
        {"aggregate_ms", result.aggregate_ms},
        {"filter_ms", result.filter_ms},
        {"batch", batch}, {"time_steps", steps}};
      std::ofstream(prefix + "_timing.json") << timing.dump(2) << '\n';
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
