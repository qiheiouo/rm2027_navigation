#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rm_dynamic_obstacle_critic/model.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mppi::critics {
namespace dyn = rm_dynamic_obstacle_critic;
class DynamicObstacleCritic final : public CriticFunction {
public:
  void initialize() override {
    auto node = parent_.lock();
    if (!node)
      throw std::runtime_error("missing critic parent");
    auto get = parameters_handler_->getParamGetter(name_);
    auto parent_get = parameters_handler_->getParamGetter(parent_name_);
    parent_get(model_dt_, "model_dt", 0.1, ParameterType::Static);
    parent_get(time_steps_, "time_steps", 30, ParameterType::Static);
    get(topic_, "topic",
        std::string("/perception/dynamic_obstacles_shadow/predictions"),
        ParameterType::Static);
    get(horizon_, "prediction_horizon", 3.0, ParameterType::Static);
    get(limits_.input_frame, "input_frame", std::string("map"),
        ParameterType::Static);
    get(limits_.max_tf_age, "max_tf_age", 0.1, ParameterType::Static);
    int max_tracks;
    get(max_tracks, "max_tracks", 64, ParameterType::Static);
    if (max_tracks < 1 || max_tracks > 256)
      throw std::invalid_argument("max_tracks must be 1..256");
    limits_.max_tracks = static_cast<size_t>(max_tracks);
    get(limits_.max_age, "max_age", 0.4, ParameterType::Static);
    get(limits_.max_observation_age, "max_observation_age", 0.4,
        ParameterType::Static);
    get(limits_.max_speed, "max_obstacle_speed", 3.0, ParameterType::Static);
    get(limits_.max_extent, "max_obstacle_extent", 3.0, ParameterType::Static);
    get(limits_.minimum_radius, "minimum_obstacle_radius", 0.36,
        ParameterType::Static);
    get(limits_.jump_tolerance, "jump_tolerance", 0.5, ParameterType::Static);
    get(cost_.weight, "cost_weight", 1.0, ParameterType::Static);
    get(cost_.influence_distance, "influence_distance", 0.4,
        ParameterType::Static);
    get(cost_.safety_margin, "safety_margin", 0.02, ParameterType::Static);
    get(cost_.collision_cost, "collision_cost", 10000.0, ParameterType::Static);
    get(diagnostics_topic_, "diagnostics_topic",
        std::string("/dynamic_critic/diagnostics"), ParameterType::Static);
    get(markers_topic_, "markers_topic",
        std::string("/dynamic_critic/predictions"), ParameterType::Static);
    get(diagnostics_period_, "diagnostics_period", 1.0, ParameterType::Static);
    const double finite_params[] = {model_dt_,
                                    horizon_,
                                    limits_.max_tf_age,
                                    limits_.max_age,
                                    limits_.max_observation_age,
                                    limits_.max_speed,
                                    limits_.max_extent,
                                    limits_.minimum_radius,
                                    limits_.jump_tolerance,
                                    cost_.weight,
                                    cost_.influence_distance,
                                    cost_.safety_margin,
                                    cost_.collision_cost,
                                    diagnostics_period_};
    for (double p : finite_params)
      if (!std::isfinite(p))
        throw std::invalid_argument("nonfinite CV critic parameter");
    if (time_steps_ <= 0 || model_dt_ <= 0 || !std::isfinite(horizon_) ||
        std::abs(time_steps_ * model_dt_ - horizon_) > 1e-6 ||
        limits_.input_frame.empty() || limits_.max_tf_age <= 0 ||
        limits_.max_age <= 0 || limits_.max_observation_age <= 0 ||
        limits_.max_speed <= 0 || limits_.max_extent <= 0 ||
        limits_.minimum_radius <= 0 || limits_.jump_tolerance < 0 ||
        cost_.weight < 0 || cost_.safety_margin <= 0 ||
        cost_.influence_distance <= cost_.safety_margin ||
        cost_.collision_cost <= 0 || diagnostics_period_ <= 0)
      throw std::invalid_argument(
          "invalid CV critic parameters or MPPI/prediction horizon mismatch");
    tf_ = costmap_ros_->getTfBuffer();
    sub_ = node->create_subscription<dyn::Array>(
        topic_, rclcpp::QoS(1), [this](dyn::Array::ConstSharedPtr msg) {
          cache_.receive(std::move(msg), limits_);
        });
    diagnostics_ =
        node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic_, 10);
    markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        markers_topic_, 10);
    diagnostics_->on_activate();
    markers_
        ->on_activate(); // Diagnostic publishers carry no control authority.
  }
  void score(CriticData &data) override {
    if (!enabled_)
      return;
    auto node = parent_.lock();
    if (!node) {
      data.fail_flag = true;
      return;
    }
    auto now = node->now();
    std::string error;
    auto msg = cache_.get(error);
    if (!msg)
      error = "missing_input";
    else if (error.empty())
      error = dyn::validate(*msg, now.seconds(), limits_);
    const auto &traj = data.trajectories;
    if (error.empty() &&
        (std::abs(data.model_dt - model_dt_) > 1e-6 ||
         traj.x.shape(1) != static_cast<size_t>(time_steps_) ||
         std::abs(msg->prediction_dt - model_dt_) > 1e-6 ||
         msg->prediction_steps != static_cast<uint32_t>(time_steps_)))
      error = "prediction_grid_or_rollout_horizon_mismatch";
    if (error.empty() && (traj.x.shape() != traj.y.shape() ||
                          traj.x.shape() != traj.yaws.shape() ||
                          data.costs.size() != traj.x.shape(0)))
      error = "rollout_shape";
    std::vector<dyn::Point> footprint;
    dyn::Rigid2D tr;
    if (error.empty())
      try {
        for (auto p : costmap_ros_->getRobotFootprint())
          footprint.push_back({p.x, p.y});
        dyn::validate_footprint(footprint);
        tr =
            dyn::frame_transform(*tf_, costmap_ros_->getGlobalFrameID(),
                                 msg->header.frame_id, now, limits_.max_tf_age);
      } catch (const std::exception &e) {
        error = std::string("footprint_or_tf: ") + e.what();
      }
    if (!error.empty()) {
      data.fail_flag = true;
      report(now, error, {}, 0, 0, nullptr, {});
      RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 2000,
                           "DynamicObstacleCritic fails closed: %s",
                           error.c_str());
      return;
    }
    // Validate the complete batch before writing any costs.
    for (size_t i = 0; i < traj.x.shape(0); ++i)
      for (size_t k = 0; k < traj.x.shape(1); ++k)
        if (!std::isfinite(traj.x(i, k)) || !std::isfinite(traj.y(i, k)) ||
            !std::isfinite(traj.yaws(i, k))) {
          data.fail_flag = true;
          report(now, "nonfinite_rollout", {}, 0, 0, nullptr, {});
          return;
        }
    dyn::Risk best;
    best.cost = std::numeric_limits<double>::infinity();
    double min_cost = best.cost, max_cost = 0;
    const double age = now.seconds() - dyn::seconds(msg->header.stamp);
    for (size_t i = 0; i < traj.x.shape(0); ++i) {
      std::vector<dyn::Pose> poses;
      poses.reserve(time_steps_);
      for (size_t k = 0; k < traj.x.shape(1); ++k)
        poses.push_back({traj.x(i, k), traj.y(i, k), traj.yaws(i, k)});
      const auto risk = dyn::score(poses, footprint, *msg, limits_, cost_, age,
                                   data.model_dt, tr);
      const double added = data.costs(i) + risk.cost;
      if (!std::isfinite(added) || added > std::numeric_limits<float>::max()) {
        data.fail_flag = true;
        return;
      }
      data.costs(i) = static_cast<float>(added);
      min_cost = std::min(min_cost, risk.cost);
      max_cost = std::max(max_cost, risk.cost);
      if (risk.cost < best.cost)
        best = risk;
    }
    report(now, "ok", best, min_cost, max_cost, msg.get(), tr);
  }

private:
  void report(const rclcpp::Time &now, const std::string &status,
              const dyn::Risk &best, double min_cost, double max_cost,
              const dyn::Array *msg, const dyn::Rigid2D &tr) {
    if (last_report_ > 0 && now.seconds() >= last_report_ &&
        now.seconds() - last_report_ < diagnostics_period_)
      return;
    last_report_ = now.seconds();
    diagnostic_msgs::msg::DiagnosticArray out;
    out.header.stamp = now;
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = name_;
    s.hardware_id = "experimental_cv_critic";
    s.level = status == "ok" ? s.OK : s.ERROR;
    s.message = status;
    auto add = [&](std::string key, double value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = std::to_string(value);
      s.values.push_back(kv);
    };
    add("horizon", horizon_);
    add("dynamic_cost_min", min_cost);
    add("dynamic_cost_max", max_cost);
    add("lowest_dynamic_cost_rollout_clearance", best.minimum_clearance);
    add("lowest_dynamic_cost_rollout_TTC", best.collision_time);
    if (msg) {
      add("source_age", now.seconds() - dyn::seconds(msg->header.stamp));
      add("tracks", msg->tracks.size());
    }
    out.status.push_back(s);
    diagnostics_->publish(out);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker reset;
    reset.action = reset.DELETEALL;
    markers.markers.push_back(reset);
    if (msg)
      for (size_t i = 0; i < msg->tracks.size(); ++i) {
        const auto &t = msg->tracks[i];
        if (t.state == dyn::Obstacle::STATE_TENTATIVE)
          continue;
        visualization_msgs::msg::Marker m;
        m.header.stamp = now;
        m.header.frame_id = costmap_ros_->getGlobalFrameID();
        m.ns = "cv_future";
        m.id = static_cast<int>(i);
        m.type = m.LINE_STRIP;
        m.action = m.ADD;
        m.pose.orientation.w = 1;
        m.scale.x = 0.03;
        m.color.r = 1;
        m.color.g = 0.4;
        m.color.a = 0.9;
        m.lifetime = rclcpp::Duration::from_seconds(2 * diagnostics_period_);
        const double age = now.seconds() - dyn::seconds(msg->header.stamp);
        for (int k = 0; k <= time_steps_; ++k) {
          auto p = dyn::predict(t, age, k * model_dt_, tr);
          geometry_msgs::msg::Point q;
          q.x = p.x;
          q.y = p.y;
          q.z = 0.3;
          m.points.push_back(q);
        }
        markers.markers.push_back(m);
        RCLCPP_INFO(logger_,
                    "CV obstacle id=%llu source=(%.3f,%.3f) "
                    "velocity=(%.3f,%.3f) radius=%.3f age=%.3f",
                    static_cast<unsigned long long>(t.track_id), t.position.x,
                    t.position.y, t.velocity.x, t.velocity.y,
                    dyn::obstacle_radius(t, limits_), age);
      }
    markers_->publish(markers);
  }
  dyn::InputCache cache_;
  dyn::Limits limits_;
  dyn::CostParameters cost_;
  double model_dt_{}, horizon_{}, diagnostics_period_{}, last_report_{-1};
  int time_steps_{};
  std::string topic_, diagnostics_topic_, markers_topic_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<dyn::Array>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<
      diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
};
} // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::DynamicObstacleCritic,
                       mppi::critics::CriticFunction)
