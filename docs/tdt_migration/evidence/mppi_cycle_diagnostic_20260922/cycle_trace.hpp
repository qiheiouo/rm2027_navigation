// Isolated diagnostic observer. Does not modify controller inputs or outputs.
#pragma once
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
namespace tdt_trace {
using Json = nlohmann::json;
int64_t steady_ns();
void value(const std::string &, const Json &) noexcept;
void blob(const std::string &, const void *, size_t, const std::string &, const std::vector<size_t> &) noexcept;
void stage(const std::string &) noexcept;
void controls(const std::string &, const mppi::models::ControlSequence &) noexcept;
void scored(const mppi::CriticData &) noexcept;
void collision_mask(const std::vector<uint8_t> &, bool) noexcept;
bool active() noexcept;
void shutdown() noexcept;
template<class T> void tensor(const std::string & name, const T & t) noexcept {
  if (!active()) return;
  blob(name, t.data(), t.size()*sizeof(typename T::value_type), "f4", {t.shape().begin(),t.shape().end()});
}
struct Cycle {
  Cycle(const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &, const rclcpp::Clock::SharedPtr &) noexcept;
  void locked(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &, const nav_msgs::msg::Path &) noexcept;
  void output(const geometry_msgs::msg::TwistStamped &) noexcept;
  ~Cycle() noexcept;
};
}
