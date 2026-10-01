#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rm_dynamic_obstacle_critic/guard.hpp"
#include "rm_dynamic_obstacle_critic/static_map.hpp"
#include <chrono>

namespace mppi::critics {
namespace dyn = rm_dynamic_obstacle_critic;
// Experimental first-command proxy. Native SG/weighting/smoothing happens
// later; this critic conveys an objective, never a final-command safety
// certificate.
class StaticStoppingCritic final : public CriticFunction {
public:
  void initialize() override {
    auto get = parameters_handler_->getParamGetter(name_);
    auto parent_get = parameters_handler_->getParamGetter(parent_name_);
    auto global_get = parameters_handler_->getParamGetter("");
    std::string model;
    double frequency;
    parent_get(model, "motion_model", std::string("Omni"),
               ParameterType::Static);
    parent_get(dt_, "model_dt", .1, ParameterType::Static);
    parent_get(steps_, "time_steps", 30, ParameterType::Static);
    global_get(frequency, "controller_frequency", 10., ParameterType::Static);
    parent_get(vx_min_, "vx_min", -.5, ParameterType::Static);
    parent_get(vx_max_, "vx_max", .8, ParameterType::Static);
    parent_get(vy_max_, "vy_max", .5, ParameterType::Static);
    parent_get(wz_max_, "wz_max", 1.2, ParameterType::Static);
    get(rejection_cost_, "rejection_cost", 10000., ParameterType::Static);
    get(cfg_.horizon, "horizon", .5, ParameterType::Static);
    get(cfg_.dt, "simulation_dt", .02, ParameterType::Static);
    get(cfg_.response_delay, "response_delay", .1, ParameterType::Static);
    get(cfg_.linear_deceleration, "linear_deceleration", 1.,
        ParameterType::Static);
    get(cfg_.angular_deceleration, "angular_deceleration", 2.,
        ParameterType::Static);
    int budget;
    get(budget, "max_simulation_steps", 512, ParameterType::Static);
    get(threshold_, "collision_threshold", 203, ParameterType::Static);
    if (model != "Omni" || !std::isfinite(frequency) || frequency <= 0 ||
        !std::isfinite(dt_) || dt_ <= 0 ||
        std::abs(1. / frequency - dt_) > 1e-6 || steps_ < 2 || threshold_ < 1 ||
        threshold_ > 254 || budget < 1 || budget > 4096 ||
        !std::isfinite(cfg_.response_delay) || cfg_.response_delay < 0 ||
        !std::isfinite(vx_min_) || vx_min_ > 0)
      throw std::invalid_argument(
          "unsupported stopping proxy grid/model or parameters");
    for (double p : {cfg_.horizon, cfg_.dt, cfg_.linear_deceleration,
                     cfg_.angular_deceleration, rejection_cost_, vx_max_,
                     vy_max_, wz_max_})
      if (!std::isfinite(p) || p <= 0)
        throw std::invalid_argument(
            "stopping proxy parameters must be finite and positive");
    if (cfg_.dt > cfg_.horizon)
      throw std::invalid_argument("stopping simulation dt exceeds horizon");
    cfg_.max_steps = static_cast<size_t>(budget);
    auto node = parent_.lock();
    if (!node)
      throw std::runtime_error("missing stopping critic parent");
    diagnostics_ =
        node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/static_stopping/diagnostics", 10);
    diagnostics_->on_activate();
  }
  void score(CriticData &data) override {
    if (!enabled_)
      return;
    const auto begin = std::chrono::steady_clock::now();
    const auto &s = data.state;
    if (s.cvx.shape() != s.cvy.shape() || s.cvx.shape() != s.cwz.shape() ||
        s.cvx.shape(0) != data.costs.size() ||
        s.cvx.shape(1) != static_cast<size_t>(steps_) ||
        std::abs(data.model_dt - dt_) > 1e-6) {
      data.fail_flag = true;
      return;
    }
    const auto &q = s.pose.pose.orientation;
    const auto &p = s.pose.pose.position;
    dyn::Pose start{p.x, p.y, std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)};
    dyn::Velocity measured{s.speed.linear.x, s.speed.linear.y,
                           s.speed.angular.z};
    if (s.pose.header.frame_id != costmap_ros_->getGlobalFrameID() ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(q.x) ||
        !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
        std::abs(q.x) > 1e-3 || std::abs(q.y) > 1e-3 ||
        std::abs(q.z * q.z + q.w * q.w - 1) > 1e-3 ||
        !std::isfinite(measured.x) || !std::isfinite(measured.y) ||
        !std::isfinite(measured.yaw)) {
      data.fail_flag = true;
      return;
    }
    // Validate the entire scored prefix before modifying any costs.
    for (size_t i = 0; i < data.costs.size(); ++i)
      if (!std::isfinite(s.cvx(i, 1)) || !std::isfinite(s.cvy(i, 1)) ||
          !std::isfinite(s.cwz(i, 1)) || !std::isfinite(data.costs(i)) ||
          data.costs(i) + rejection_cost_ > std::numeric_limits<float>::max()) {
        data.fail_flag = true;
        return;
      }
    nav2_msgs::msg::Costmap snapshot;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
          *costmap_->getMutex());
      snapshot.metadata.size_x = costmap_->getSizeInCellsX();
      snapshot.metadata.size_y = costmap_->getSizeInCellsY();
      snapshot.metadata.resolution = costmap_->getResolution();
      snapshot.metadata.origin.position.x = costmap_->getOriginX();
      snapshot.metadata.origin.position.y = costmap_->getOriginY();
      snapshot.metadata.origin.orientation.w = 1;
      const size_t size = static_cast<size_t>(snapshot.metadata.size_x) *
                          snapshot.metadata.size_y;
      if (size)
        snapshot.data.assign(costmap_->getCharMap(),
                             costmap_->getCharMap() + size);
    }
    std::vector<dyn::Point> footprint;
    for (auto v : costmap_ros_->getRobotFootprint())
      footprint.push_back({v.x, v.y});
    try {
      dyn::validate_footprint(footprint);
    } catch (const std::exception &) {
      data.fail_flag = true;
      return;
    }
    dyn::Array no_dynamic_obstacles;
    size_t passing = 0, measured_rejected = 0, proposed_rejected = 0;
    auto static_clear = [&](const auto &poly, double reserve) {
      return dyn::static_map_clear(snapshot, poly, reserve, threshold_);
    };
    // For a fixed map and no dynamic obstacles, the measured stopping path is
    // identical for all candidates. Additional time after it stops repeats the
    // same static pose/reserve. Cache this branch only in this static critic.
    // The runtime guard continues to check both paths against future CV states.
    const auto measured_check = dyn::check_command(
        start, measured, {}, footprint, no_dynamic_obstacles, {}, 0, {}, cfg_,
        static_clear, dyn::GuardPaths::MeasuredOnly);
    for (size_t i = 0; i < data.costs.size(); ++i) {
      const dyn::Velocity proxy{
          std::clamp(static_cast<double>(s.cvx(i, 1)), vx_min_, vx_max_),
          std::clamp(static_cast<double>(s.cvy(i, 1)), -vy_max_, vy_max_),
          std::clamp(static_cast<double>(s.cwz(i, 1)), -wz_max_, wz_max_)};
      auto result =
          measured_check.pass
              ? dyn::check_command(start, {}, proxy, footprint,
                                   no_dynamic_obstacles, {}, 0, {}, cfg_,
                                   static_clear, dyn::GuardPaths::ProposedOnly)
              : measured_check;
      if (result.pass)
        ++passing;
      else {
        data.costs(i) += static_cast<float>(rejection_cost_);
        measured_rejected += result.collision_branch == 0;
        proposed_rejected += result.collision_branch == 1;
      }
    }
    auto node = parent_.lock();
    if (!node) {
      data.fail_flag = true;
      return;
    }
    const auto now = node->now();
    if (last_report_ > 0 && now.seconds() >= last_report_ &&
        now.seconds() - last_report_ < 1.)
      return;
    last_report_ = now.seconds();
    diagnostic_msgs::msg::DiagnosticArray out;
    out.header.stamp = now;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name_;
    status.hardware_id = "experimental_unsmoothed_static_command_proxy";
    status.message = "ok";
    status.level = status.OK;
    auto add = [&](const std::string &key, double value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = std::to_string(value);
      status.values.push_back(kv);
    };
    add("candidate_command_offset", 1);
    add("passing_proxies", passing);
    add("batch", data.costs.size());
    add("measured_path_rejections", measured_rejected);
    add("proposed_path_rejections", proposed_rejected);
    add("score_ms", std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - begin)
                        .count());
    out.status.push_back(status);
    diagnostics_->publish(out);
  }

private:
  dyn::GuardParameters cfg_;
  double dt_, vx_min_, vx_max_, vy_max_, wz_max_, rejection_cost_,
      last_report_{0};
  int steps_, threshold_;
  rclcpp_lifecycle::LifecyclePublisher<
      diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
};
} // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::StaticStoppingCritic,
                       mppi::critics::CriticFunction)
