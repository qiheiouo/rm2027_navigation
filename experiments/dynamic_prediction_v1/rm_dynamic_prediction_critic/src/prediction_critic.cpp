#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rm_competition_interfaces/msg/dynamic_obstacle_prediction_array.hpp"
#include "rm_dynamic_prediction_critic/geometry.hpp"

namespace mppi::critics {
class PredictionV1Critic final : public CriticFunction {
public:
  void initialize() override {
    auto node = parent_.lock();
    if (!node) throw std::runtime_error("PredictionV1Critic has no parent node");
    auto get = parameters_handler_->getParamGetter(name_);
    get(topic_, "topic", std::string("/perception/dynamic_obstacles_shadow/predictions"), ParameterType::Static);
    get(object_width_, "object_width", 0.45, ParameterType::Static);
    get(object_height_, "object_height", 0.55, ParameterType::Static);
    get(reference_acceleration_, "reference_acceleration", 0.5551652475612765, ParameterType::Static);
    get(max_age_, "max_age", 0.4, ParameterType::Static);
    get(horizon_, "horizon", 1.0, ParameterType::Static);
    if (!(object_width_ > 0 && object_height_ > 0 && reference_acceleration_ >= 0 &&
          max_age_ > 0 && horizon_ > 0)) {
      throw std::invalid_argument("invalid PredictionV1Critic physical input");
    }
    subscription_ = node->create_subscription<rm_competition_interfaces::msg::DynamicObstaclePredictionArray>(
      topic_, rclcpp::QoS(10), [this](rm_competition_interfaces::msg::DynamicObstaclePredictionArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(message_mutex_);
        latest_ = std::move(msg);
      });
    RCLCPP_INFO(logger_, "PredictionV1Critic subscribes %s; fixture object %.3f x %.3f m",
      topic_.c_str(), object_width_, object_height_);
  }

  void score(CriticData & data) override {
    if (!enabled_) return;
    auto node = parent_.lock();
    if (!node) return;
    rm_competition_interfaces::msg::DynamicObstaclePredictionArray::ConstSharedPtr message;
    {
      std::lock_guard<std::mutex> lock(message_mutex_);
      message = latest_;
    }
    if (!message) {log_skip("missing");return;}
    if (message->schema != "rm_dynamic_obstacle_predictions/v1" ||
        message->authority != "shadow_only" || !message->complete ||
        message->header.frame_id != costmap_ros_->getGlobalFrameID()) {
      log_skip("schema/frame/incomplete"); return;
    }
    const double age = (node->now() - rclcpp::Time(message->header.stamp, RCL_ROS_TIME)).seconds();
    if (!std::isfinite(age) || age < 0 || age > max_age_) {log_skip("age");return;}
    const double dt = static_cast<double>(data.model_dt);
    if (!std::isfinite(dt) || dt <= 0) {log_skip("model_dt");return;}
    const auto footprint_msg = costmap_ros_->getRobotFootprint();
    if (footprint_msg.size() < 3) {log_skip("footprint");return;}
    std::vector<rm_dynamic_prediction_critic::Point> footprint;
    footprint.reserve(footprint_msg.size());
    for (const auto & p : footprint_msg) footprint.push_back({p.x,p.y});
    struct Track {rm_dynamic_prediction_critic::Point center, velocity, visible;};
    std::vector<Track> tracks;
    for (const auto & track : message->tracks) {
      if (track.state != track.STATE_CONFIRMED) continue;
      const Track t{{track.position.x,track.position.y},
                    {track.velocity.x,track.velocity.y}, {track.size.x,track.size.y}};
      if (!std::isfinite(t.center.x)||!std::isfinite(t.center.y)||
          !std::isfinite(t.velocity.x)||!std::isfinite(t.velocity.y)||
          !std::isfinite(t.visible.x)||!std::isfinite(t.visible.y)||
          t.visible.x<0||t.visible.y<0) continue;
      tracks.push_back(t);
    }
    if (tracks.empty()) {log_skip("no confirmed track");return;}
    const auto & traj = data.trajectories;
    const size_t count = traj.x.shape(0);
    const size_t steps = std::min(traj.x.shape(1), static_cast<size_t>(std::floor(horizon_/dt+1e-9)));
    if (steps == 0) return;
    std::vector<std::vector<rm_dynamic_prediction_critic::Box>> boxes(steps);
    for (size_t j=0;j<steps;++j) {
      for (const auto & track:tracks) {
        boxes[j].push_back(rm_dynamic_prediction_critic::predicted_box(
          track.center,track.velocity,track.visible,{object_width_,object_height_},
          age,(j+1)*dt,reference_acceleration_));
      }
    }
    size_t collisions=0, nears=0;
    for (size_t i=0;i<count;++i) {
      bool collision=false;
      double repulsive=0;
      for (size_t j=0;j<steps && !collision;++j) {
        auto polygon=rm_dynamic_prediction_critic::transform(
          footprint,traj.x(i,j),traj.y(i,j),traj.yaws(i,j));
        for (const auto & box:boxes[j]) {
          const double clearance=rm_dynamic_prediction_critic::polygon_box_distance(polygon,box);
          if (clearance <= 1e-9) {collision=true;break;}
          // The footprint is padded by 0.03 m; another 0.02 m corresponds to
          // the existing 0.05 m body-clearance check in this fixture.
          if (clearance < 0.02) {repulsive += 300.0; ++nears;}
        }
      }
      if (collision) {repulsive=1000000.0;++collisions;}
      // Match the installed CostCritic's cost scale; retain its ordinary
      // STVL collision scoring and do not change MPPI's failure flag.
      data.costs(i) += static_cast<float>((3.81 / 254.0) * repulsive / steps);
    }
    ++accepted_cycles_;
    RCLCPP_INFO_THROTTLE(logger_, *node->get_clock(), 1000,
      "PredictionV1Critic consumed source age %.3f s, tracks %zu, predicted-collision trajectories %zu/%zu, near samples %zu, cycles %llu",
      age, tracks.size(), collisions, count, nears,
      static_cast<unsigned long long>(accepted_cycles_));
  }
private:
  void log_skip(const char * reason) {
    auto node=parent_.lock();
    if (node) RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 2000,
      "PredictionV1Critic skipped prediction (%s); baseline CostCritic remains active", reason);
  }
  std::string topic_;
  double object_width_{},object_height_{},reference_acceleration_{},max_age_{},horizon_{};
  std::mutex message_mutex_;
  rm_competition_interfaces::msg::DynamicObstaclePredictionArray::ConstSharedPtr latest_;
  rclcpp::Subscription<rm_competition_interfaces::msg::DynamicObstaclePredictionArray>::SharedPtr subscription_;
  uint64_t accepted_cycles_{0};
};
} // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::PredictionV1Critic,mppi::critics::CriticFunction)
