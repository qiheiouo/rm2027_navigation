#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rm_relocalization_bridge/relocalization_math.hpp"

namespace
{

tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  rotation.normalize();

  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(rotation);
  return transform;
}

geometry_msgs::msg::Transform transformToMessage(const tf2::Transform & transform)
{
  geometry_msgs::msg::Transform message;
  message.translation.x = transform.getOrigin().x();
  message.translation.y = transform.getOrigin().y();
  message.translation.z = transform.getOrigin().z();
  message.rotation.x = transform.getRotation().x();
  message.rotation.y = transform.getRotation().y();
  message.rotation.z = transform.getRotation().z();
  message.rotation.w = transform.getRotation().w();
  return message;
}

}  // namespace

class MapOdomFromGlobalPose : public rclcpp::Node
{
public:
  MapOdomFromGlobalPose()
  : Node("map_odom_from_global_pose"),
    odom_cache_(static_cast<std::size_t>(
        std::max<std::int64_t>(
          1, declare_parameter<std::int64_t>("odom_cache_size", 500))))
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    global_pose_topic_ = declare_parameter<std::string>(
      "global_pose_topic", "/localization/global_pose");
    upstream_valid_topic_ = declare_parameter<std::string>("upstream_valid_topic", "");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/lio");
    output_topic_ = declare_parameter<std::string>(
      "map_to_odom_topic", "/localization/map_to_odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    max_pose_odom_dt_sec_ = declare_parameter<double>("max_pose_odom_dt_sec", 0.05);
    max_global_pose_age_sec_ = declare_parameter<double>("max_global_pose_age_sec", 0.5);
    pending_pose_max_wait_sec_ = declare_parameter<double>("pending_pose_max_wait_sec", 0.2);
    correction_innovation_gate_enabled_ =
      declare_parameter<bool>("correction_innovation_gate_enabled", false);
    max_correction_translation_step_m_ =
      declare_parameter<double>("max_correction_translation_step_m", 0.35);
    max_correction_yaw_step_rad_ =
      declare_parameter<double>("max_correction_yaw_step_rad", 0.35);
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    pending_pose_max_size_ = static_cast<std::size_t>(
      std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("pending_pose_max_size", 20)));

    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    if (
      max_pose_odom_dt_sec_ < 0.0 || max_global_pose_age_sec_ < 0.0 ||
      pending_pose_max_wait_sec_ < 0.0 ||
      !std::isfinite(max_correction_translation_step_m_) ||
      max_correction_translation_step_m_ <= 0.0 ||
      !std::isfinite(max_correction_yaw_step_rad_) ||
      max_correction_yaw_step_rad_ <= 0.0)
    {
      throw std::invalid_argument("time tolerances and correction limits must be valid");
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(output_topic_, 10);

    auto valid_qos = rclcpp::QoS(1).reliable().transient_local();
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/global_localization_valid", valid_qos);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(100).reliable(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {handleOdometry(*msg);});
    global_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      global_pose_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        handleGlobalPose(*msg);
      });
    if (correction_innovation_gate_enabled_) {
      initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_, rclcpp::QoS(10).reliable(),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
          resetCorrectionBaseline("explicit initial pose request");
        });
    }
    if (!upstream_valid_topic_.empty()) {
      upstream_valid_required_ = true;
      upstream_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
        upstream_valid_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {handleUpstreamValid(msg->data);});
    }

    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/reset_map_to_odom",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        valid_ = false;
        has_accepted_correction_ = false;
        pending_global_poses_.clear();
        publishValid(false);
        response->success = true;
        response->message = "map->odom correction invalidated";
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publishCurrentTransform();});

    publishValid(false);
    RCLCPP_INFO(
      get_logger(),
      "Global-pose bridge ready: %s + %s -> dynamic %s -> %s. "
      "It publishes no identity fallback.",
      global_pose_topic_.c_str(), odom_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str());
    if (upstream_valid_required_) {
      RCLCPP_INFO(
        get_logger(), "map->odom validity follows upstream gate %s.",
        upstream_valid_topic_.c_str());
    }
    if (correction_innovation_gate_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "Correction innovation gate enabled: max translation step %.3f m, "
        "max yaw step %.3f rad; baseline resets on %s.",
        max_correction_translation_step_m_, max_correction_yaw_step_rad_,
        initial_pose_topic_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Correction innovation gate disabled.");
    }
  }

private:
  void resetCorrectionBaseline(const char * reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    valid_ = false;
    has_accepted_correction_ = false;
    pending_global_poses_.clear();
    publishValid(false);
    RCLCPP_WARN(
      get_logger(), "Invalidated map->odom and reset correction baseline: %s.", reason);
  }

  bool validatePose(const geometry_msgs::msg::Pose & pose) const
  {
    const double quaternion_norm_squared =
      pose.orientation.x * pose.orientation.x +
      pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z +
      pose.orientation.w * pose.orientation.w;
    if (
      !std::isfinite(pose.position.x) ||
      !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) ||
      !std::isfinite(quaternion_norm_squared) ||
      quaternion_norm_squared <= 1.0e-12)
    {
      return false;
    }
    const tf2::Transform transform = poseToTransform(pose);
    return rm_relocalization_bridge::isFiniteTransform(transform);
  }

  void handleUpstreamValid(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    upstream_valid_ = value;
    if (!value) {
      valid_ = false;
      pending_global_poses_.clear();
    }
    publishValid(valid_ && upstream_valid_);
  }

  void handleOdometry(const nav_msgs::msg::Odometry & msg)
  {
    if (msg.header.frame_id != odom_frame_ || msg.child_frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting odometry with frames '%s' -> '%s'; expected '%s' -> '%s'.",
        msg.header.frame_id.c_str(), msg.child_frame_id.c_str(),
        odom_frame_.c_str(), base_frame_.c_str());
      return;
    }
    const rclcpp::Time stamp(msg.header.stamp);
    if (stamp.nanoseconds() <= 0 || !validatePose(msg.pose.pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting odometry with zero timestamp or invalid pose.");
      return;
    }

    const auto transform = poseToTransform(msg.pose.pose);
    std::lock_guard<std::mutex> lock(mutex_);
    const bool monotonic = odom_cache_.add({stamp.nanoseconds(), transform});
    if (!monotonic) {
      valid_ = false;
      has_accepted_correction_ = false;
      pending_global_poses_.clear();
      publishValid(false);
      RCLCPP_WARN(
        get_logger(),
        "Odometry timestamp reset detected. Cleared cache and invalidated map->odom.");
      return;
    }
    processPendingGlobalPoses();
  }

  void handleGlobalPose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    if (msg.header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "Rejecting global pose in frame '%s'; expected '%s'.",
        msg.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    if (stamp.nanoseconds() <= 0 || !validatePose(msg.pose.pose)) {
      RCLCPP_WARN(get_logger(), "Rejecting global pose with zero timestamp or invalid pose.");
      return;
    }

    const auto current_time = now();
    const double age = (current_time - stamp).seconds();
    if (current_time.nanoseconds() > 0 && age > max_global_pose_age_sec_) {
      RCLCPP_WARN(
        get_logger(), "Rejecting stale global pose: age %.3f s exceeds %.3f s.",
        age, max_global_pose_age_sec_);
      return;
    }

    const auto map_to_base = poseToTransform(msg.pose.pose);
    const auto tolerance_ns = static_cast<std::int64_t>(max_pose_odom_dt_sec_ * 1.0e9);

    std::lock_guard<std::mutex> lock(mutex_);
    const auto odom_sample = odom_cache_.nearest(stamp.nanoseconds(), tolerance_ns);
    if (!odom_sample.has_value()) {
      if (pending_global_poses_.size() >= pending_pose_max_size_) {
        pending_global_poses_.pop_front();
        RCLCPP_WARN(get_logger(), "Global-pose pending queue full; dropped oldest pose.");
      }
      pending_global_poses_.push_back({
        stamp.nanoseconds(), map_to_base, std::chrono::steady_clock::now()});
      return;
    }

    acceptCorrection(map_to_base, *odom_sample, stamp.nanoseconds());
  }

  struct PendingGlobalPose
  {
    std::int64_t stamp_nanoseconds;
    tf2::Transform map_to_base;
    std::chrono::steady_clock::time_point received_at;
  };

  void processPendingGlobalPoses()
  {
    const auto tolerance_ns = static_cast<std::int64_t>(max_pose_odom_dt_sec_ * 1.0e9);
    const auto now_steady = std::chrono::steady_clock::now();
    auto pending = pending_global_poses_.begin();
    while (pending != pending_global_poses_.end()) {
      const auto odom_sample = odom_cache_.nearest(pending->stamp_nanoseconds, tolerance_ns);
      if (odom_sample.has_value()) {
        acceptCorrection(pending->map_to_base, *odom_sample, pending->stamp_nanoseconds);
        pending = pending_global_poses_.erase(pending);
        continue;
      }

      const double waiting_seconds =
        std::chrono::duration<double>(now_steady - pending->received_at).count();
      if (waiting_seconds > pending_pose_max_wait_sec_) {
        RCLCPP_WARN(
          get_logger(),
          "Global pose timed out after %.3f s without timestamp-matched odometry.",
          waiting_seconds);
        pending = pending_global_poses_.erase(pending);
        continue;
      }
      ++pending;
    }
  }

  void acceptCorrection(
    const tf2::Transform & map_to_base,
    const rm_relocalization_bridge::TimedTransform & odom_sample,
    std::int64_t source_stamp_nanoseconds)
  {

    const auto candidate_map_to_odom = rm_relocalization_bridge::computeMapToOdom(
      map_to_base, odom_sample.transform);
    if (!rm_relocalization_bridge::isFiniteTransform(candidate_map_to_odom)) {
      RCLCPP_ERROR(get_logger(), "Computed non-finite map->odom; correction rejected.");
      valid_ = false;
      publishValid(false);
      return;
    }

    if (correction_innovation_gate_enabled_ && has_accepted_correction_) {
      const auto innovation = rm_relocalization_bridge::measureCorrectionInnovation(
        map_to_odom_, candidate_map_to_odom);
      if (
        innovation.translation_xy_m > max_correction_translation_step_m_ ||
        innovation.yaw_rad > max_correction_yaw_step_rad_)
      {
        valid_ = false;
        publishValid(false);
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Rejected map->odom correction innovation: translation %.3f m (limit %.3f), "
          "yaw %.3f rad (limit %.3f). Keeping the last accepted correction but "
          "withholding TF until a consistent pose returns or %s resets the baseline.",
          innovation.translation_xy_m, max_correction_translation_step_m_,
          innovation.yaw_rad, max_correction_yaw_step_rad_, initial_pose_topic_.c_str());
        return;
      }
    }

    map_to_odom_ = candidate_map_to_odom;
    has_accepted_correction_ = true;
    valid_ = true;
    publishValid(valid_ && (!upstream_valid_required_ || upstream_valid_));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Accepted global pose at %.6f; map->odom correction updated.",
      static_cast<double>(source_stamp_nanoseconds) / 1.0e9);
  }

  void publishValid(bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    valid_pub_->publish(message);
  }

  void publishCurrentTransform()
  {
    tf2::Transform transform;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!valid_ || (upstream_valid_required_ && !upstream_valid_)) {
        return;
      }
      transform = map_to_odom_;
    }

    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    message.transform = transformToMessage(transform);
    transform_pub_->publish(message);
    if (publish_tf_) {
      tf_broadcaster_->sendTransform(message);
    }
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string global_pose_topic_;
  std::string upstream_valid_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string initial_pose_topic_;
  bool publish_tf_;
  bool correction_innovation_gate_enabled_;
  double publish_rate_hz_;
  double max_pose_odom_dt_sec_;
  double max_global_pose_age_sec_;
  double pending_pose_max_wait_sec_;
  double max_correction_translation_step_m_;
  double max_correction_yaw_step_rad_;
  std::size_t pending_pose_max_size_;

  std::mutex mutex_;
  rm_relocalization_bridge::TimedTransformCache odom_cache_;
  std::deque<PendingGlobalPose> pending_global_poses_;
  tf2::Transform map_to_odom_;
  bool has_accepted_correction_ = false;
  bool valid_ = false;
  bool upstream_valid_required_ = false;
  bool upstream_valid_ = false;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
  global_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
  initial_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr upstream_valid_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomFromGlobalPose>());
  rclcpp::shutdown();
  return 0;
}
