#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_dynamic_obstacle_critic/guard.hpp"
#include "rm_dynamic_obstacle_critic/static_map.hpp"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <iomanip>
#include <sstream>
namespace dyn = rm_dynamic_obstacle_critic;
using Steady = std::chrono::steady_clock;
class SafetyGuard final : public rclcpp::Node {
public:
  SafetyGuard() : Node("dynamic_safety_guard") {
    input_ = declare_parameter("input_topic",
                               std::string("/dynamic_test/cmd_vel_smoothed"));
    output_ = declare_parameter("output_topic",
                                std::string("/dynamic_test/cmd_vel_guarded"));
    frame_ = declare_parameter("world_frame", std::string("odom"));
    base_ = declare_parameter("base_frame", std::string("base_link"));
    auto topic = declare_parameter(
        "obstacles_topic",
        std::string("/perception/dynamic_obstacles_shadow/predictions"));
    auto odom_topic =
        declare_parameter("odom_topic", std::string("/odometry/lio"));
    auto map_topic = declare_parameter(
        "costmap_topic", std::string("/local_costmap/costmap_raw"));
    command_timeout_ = declare_parameter("command_timeout", 0.15);
    odom_timeout_ = declare_parameter("odom_timeout", 0.15);
    map_timeout_ = declare_parameter("costmap_timeout", 0.5);
    limits_.input_frame = declare_parameter("input_frame", std::string("map"));
    limits_.max_tf_age = declare_parameter("max_tf_age", 0.1);
    int max_tracks = declare_parameter("max_tracks", 64);
    if (max_tracks < 1 || max_tracks > 256)
      throw std::invalid_argument("max_tracks must be 1..256");
    limits_.max_tracks = static_cast<size_t>(max_tracks);
    const double frequency = declare_parameter("frequency", 50.0);
    limits_.max_age = declare_parameter("max_age", 0.4);
    limits_.max_observation_age = declare_parameter("max_observation_age", 0.4);
    limits_.max_speed = declare_parameter("max_obstacle_speed", 3.0);
    limits_.max_extent = declare_parameter("max_obstacle_extent", 3.0);
    limits_.minimum_radius = declare_parameter("minimum_obstacle_radius", 0.36);
    limits_.jump_tolerance = declare_parameter("jump_tolerance", 0.5);
    cfg_.horizon = declare_parameter("horizon", 0.5);
    cfg_.dt = declare_parameter("simulation_dt", 0.02);
    cfg_.response_delay = declare_parameter("response_delay", 0.1);
    cfg_.linear_deceleration = declare_parameter("linear_deceleration", 1.0);
    cfg_.angular_deceleration = declare_parameter("angular_deceleration", 2.0);
    int max_steps = declare_parameter("max_simulation_steps", 512);
    if (max_steps < 1 || max_steps > 4096)
      throw std::invalid_argument("max_simulation_steps must be 1..4096");
    cfg_.max_steps = static_cast<size_t>(max_steps);
    cfg_.safety_margin = declare_parameter("safety_margin", 0.02);
    vx_min_ = declare_parameter("vx_min", -0.5);
    vx_max_ = declare_parameter("vx_max", 0.8);
    vy_max_ = declare_parameter("vy_max", 0.5);
    wz_max_ = declare_parameter("wz_max", 1.2);
    threshold_ = declare_parameter("collision_threshold", 203);
    auto vertices = declare_parameter<std::vector<double>>(
        "footprint", {-0.30, -0.24, 0.30, -0.24, 0.30, 0.24, -0.30, 0.24});
    if (vertices.size() % 2)
      throw std::invalid_argument("footprint must be [x,y,...]");
    for (size_t i = 0; i < vertices.size(); i += 2)
      footprint_.push_back({vertices[i], vertices[i + 1]});
    dyn::validate_footprint(footprint_);
    const double params[] = {command_timeout_,
                             odom_timeout_,
                             map_timeout_,
                             limits_.max_age,
                             limits_.max_observation_age,
                             frequency,
                             limits_.max_tf_age,
                             limits_.max_speed,
                             limits_.max_extent,
                             limits_.minimum_radius,
                             cfg_.horizon,
                             cfg_.dt,
                             cfg_.linear_deceleration,
                             cfg_.angular_deceleration,
                             cfg_.safety_margin,
                             vx_max_,
                             vy_max_,
                             wz_max_};
    for (double p : params)
      if (!std::isfinite(p) || p <= 0)
        throw std::invalid_argument(
            "guard parameters must be finite and positive");
    if (!std::isfinite(limits_.jump_tolerance) || limits_.jump_tolerance < 0 ||
        limits_.input_frame.empty() || input_ == output_ || frame_.empty() ||
        base_.empty() || cfg_.dt > cfg_.horizon || cfg_.response_delay < 0 ||
        !std::isfinite(cfg_.response_delay) || !std::isfinite(vx_min_) ||
        vx_min_ > 0 || threshold_ < 1 || threshold_ > 254)
      throw std::invalid_argument("invalid guard route or physical parameters");
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    out_ = create_publisher<geometry_msgs::msg::Twist>(output_, 1);
    diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/dynamic_guard/diagnostics", 10);
    obstacles_ = create_subscription<dyn::Array>(
        topic, 1, [this](dyn::Array::ConstSharedPtr m) {
          cache_.receive(m, limits_);
          last_obstacles_ = Steady::now();
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 1, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          odom_ = m;
          last_odom_ = Steady::now();
        });
    map_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
        map_topic, rclcpp::QoS(1).transient_local().reliable(),
        [this](nav2_msgs::msg::Costmap::ConstSharedPtr m) {
          map_ = m;
          last_map_ = Steady::now();
        });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        input_, 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          cmd_ = m;
          last_cmd_ = Steady::now();
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency),
                               [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Experimental guard route %s -> %s",
                input_.c_str(), output_.c_str());
  }

private:
  static double elapsed(Steady::time_point t) {
    return std::chrono::duration<double>(Steady::now() - t).count();
  }
  static bool finite(dyn::Velocity v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.yaw);
  }
  void tick() {
    const auto now = get_clock()->now();
    std::string error;
    auto obstacles = cache_.get(error);
    dyn::GuardResult result;
    dyn::StaticMapCheck static_check;
    dyn::Pose evaluated_pose{};
    dyn::Velocity measured_velocity{}, proposed_velocity{};
    dyn::Rigid2D evaluated_transform;
    bool evaluated = false;
    result.pass = false;
    if (!cmd_ || elapsed(last_cmd_) > command_timeout_)
      error = "command_watchdog";
    else if (!odom_ || elapsed(last_odom_) > odom_timeout_)
      error = "odom_watchdog";
    else if (!map_ || elapsed(last_map_) > map_timeout_)
      error = "costmap_watchdog";
    else if (!obstacles || elapsed(last_obstacles_) > limits_.max_age)
      error = "obstacle_watchdog";
    if (error.empty())
      error = dyn::validate(*obstacles, now.seconds(), limits_);
    if (error.empty()) {
      const double odom_age = now.seconds() - dyn::seconds(odom_->header.stamp);
      const double map_age = now.seconds() - dyn::seconds(map_->header.stamp);
      if (odom_age < 0 || odom_age > odom_timeout_ || map_age < 0 ||
          map_age > map_timeout_ || odom_->header.frame_id != frame_ ||
          odom_->child_frame_id != base_ || map_->header.frame_id != frame_)
        error = "stale_or_wrong_frame_pose_map";
      else
        try {
          auto q = odom_->pose.pose.orientation;
          auto p = odom_->pose.pose.position;
          dyn::Pose pose{p.x, p.y,
                         std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)};
          dyn::Velocity measured{odom_->twist.twist.linear.x,
                                 odom_->twist.twist.linear.y,
                                 odom_->twist.twist.angular.z};
          dyn::Velocity command{cmd_->linear.x, cmd_->linear.y,
                                cmd_->angular.z};
          if (!std::isfinite(pose.yaw) || !std::isfinite(q.x) ||
              !std::isfinite(q.y) || !std::isfinite(q.z * q.z + q.w * q.w) ||
              cmd_->linear.z != 0 || cmd_->angular.x != 0 ||
              cmd_->angular.y != 0 || !finite(measured) || !finite(command) ||
              !std::isfinite(p.x) || !std::isfinite(p.y) ||
              std::abs(q.x) > 1e-3 || std::abs(q.y) > 1e-3 ||
              std::abs(q.z * q.z + q.w * q.w - 1) > 1e-3 ||
              command.x < vx_min_ || command.x > vx_max_ ||
              std::abs(command.y) > vy_max_ ||
              std::abs(command.yaw) > wz_max_ ||
              std::hypot(measured.x, measured.y) >
                  std::hypot(std::max(vx_max_, -vx_min_), vy_max_) * 1.1 ||
              std::abs(measured.yaw) > wz_max_ * 1.1)
            error = "invalid_pose_velocity_or_output_bounds";
          else {
            // Advance the fresh odometry measurement to evaluation time under
            // its measured twist.
            pose = dyn::advance(pose, measured, odom_age);
            const auto tr =
                dyn::frame_transform(*tf_, frame_, obstacles->header.frame_id,
                                     now, limits_.max_tf_age);
            evaluated_pose = pose;
            measured_velocity = measured;
            proposed_velocity = command;
            evaluated_transform = tr;
            evaluated = true;
            result = dyn::check_command(
                pose, measured, command, footprint_, *obstacles, limits_,
                now.seconds() - dyn::seconds(obstacles->header.stamp), tr, cfg_,
                [this, &static_check](const std::vector<dyn::Point> &poly,
                                      double reserve) {
                  static_check =
                      dyn::check_static_map(*map_, poly, reserve, threshold_);
                  return static_check.clear;
                });
            error = result.reason;
          }
        } catch (const std::exception &e) {
          error = std::string("tf_or_geometry: ") + e.what();
        }
    }
    geometry_msgs::msg::Twist command;
    if (result.pass)
      command = *cmd_;
    out_->publish(command);
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now;
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "dynamic_safety_guard";
    s.hardware_id = "experimental_software_guard";
    s.message = error;
    s.level = result.pass ? s.OK : s.WARN;
    auto add = [&](std::string key, double value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      std::ostringstream out;
      out << std::setprecision(std::numeric_limits<double>::max_digits10)
          << value;
      kv.value = out.str();
      s.values.push_back(kv);
    };
    add("passed", result.pass);
    add("minimum_predicted_clearance", result.minimum_clearance);
    add("TTC", result.collision_time);
    add("emitted_vx", command.linear.x);
    add("emitted_vy", command.linear.y);
    add("emitted_wz", command.angular.z);
    add("evaluated", evaluated);
    if (evaluated) {
      add("source_stamp", dyn::seconds(obstacles->header.stamp));
      add("odom_stamp", dyn::seconds(odom_->header.stamp));
      add("costmap_stamp", dyn::seconds(map_->header.stamp));
      add("pose_x", evaluated_pose.x);
      add("pose_y", evaluated_pose.y);
      add("pose_yaw", evaluated_pose.yaw);
      add("measured_vx", measured_velocity.x);
      add("measured_vy", measured_velocity.y);
      add("measured_wz", measured_velocity.yaw);
      add("proposed_vx", proposed_velocity.x);
      add("proposed_vy", proposed_velocity.y);
      add("proposed_wz", proposed_velocity.yaw);
      add("world_transform_x", evaluated_transform.x);
      add("world_transform_y", evaluated_transform.y);
      add("world_transform_yaw", evaluated_transform.yaw);
    }
    if (result.collision_branch >= 0) {
      add("collision_branch", result.collision_branch);
      add("collision_pose_x", result.collision_pose.x);
      add("collision_pose_y", result.collision_pose.y);
      add("collision_pose_yaw", result.collision_pose.yaw);
      add("collision_vx", result.collision_velocity.x);
      add("collision_vy", result.collision_velocity.y);
      add("collision_wz", result.collision_velocity.yaw);
      add("static_reserve", result.static_reserve);
      add("dynamic_reserve", result.dynamic_reserve);
    }
    if (result.reason == "static_collision_or_unknown") {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "static_rejection";
      kv.value = static_check.reason;
      s.values.push_back(kv);
      add("static_cell_x", static_check.cell_x);
      add("static_cell_y", static_check.cell_y);
      add("static_cell_cost", static_check.cost);
      add("static_cell_world_x", static_check.cell_center.x);
      add("static_cell_world_y", static_check.cell_center.y);
      add("static_cell_distance", static_check.distance);
    }
    arr.status.push_back(s);
    diag_->publish(arr);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Guard %s clearance=%.3f TTC=%.3f emitted=(%.3f,%.3f,%.3f)",
        error.c_str(), result.minimum_clearance, result.collision_time,
        command.linear.x, command.linear.y, command.angular.z);
  }
  dyn::InputCache cache_;
  dyn::Limits limits_;
  dyn::GuardParameters cfg_;
  std::vector<dyn::Point> footprint_;
  std::string input_, output_, frame_, base_;
  double command_timeout_, odom_timeout_, map_timeout_, vx_min_, vx_max_,
      vy_max_, wz_max_;
  int threshold_;
  Steady::time_point last_cmd_, last_odom_, last_map_, last_obstacles_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr out_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr map_sub_;
  nav2_msgs::msg::Costmap::ConstSharedPtr map_;
  rclcpp::Subscription<dyn::Array>::SharedPtr obstacles_;
  geometry_msgs::msg::Twist::ConstSharedPtr cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyGuard>());
  rclcpp::shutdown();
  return 0;
}
