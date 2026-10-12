#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rm_competition_interfaces/msg/localization_status.hpp"
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
    started_at_ = now();
    recovery_after_ns_ = started_at_.nanoseconds();
    last_clock_ns_ = recovery_after_ns_;
    producer_id_ = std::to_string(std::random_device{}()) + ":" +
      std::to_string(std::random_device{}());
    status_timeout_sec_ = declare_parameter<double>("status_timeout_sec", 1.0);
    max_future_sec_ = declare_parameter<double>("max_future_sec", 0.1);
    if (!std::isfinite(status_timeout_sec_) || status_timeout_sec_ <= 0.0 ||
      !std::isfinite(max_future_sec_) || max_future_sec_ < 0.0)
    {
      throw std::invalid_argument("localization status time limits are invalid");
    }
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
    pending_pose_max_size_ = static_cast<std::size_t>(
      std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("pending_pose_max_size", 20)));

    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    if (
      max_pose_odom_dt_sec_ < 0.0 || max_global_pose_age_sec_ < 0.0 ||
      pending_pose_max_wait_sec_ < 0.0)
    {
      throw std::invalid_argument("time tolerances must not be negative");
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(output_topic_, 10);

    auto valid_qos = rclcpp::QoS(1).reliable().transient_local();
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/global_localization_valid", valid_qos);

    status_pub_ = create_publisher<rm_competition_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(1).reliable());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(100).reliable(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {handleOdometry(*msg);});
    upstream_valid_required_ = !upstream_valid_topic_.empty();
    if (upstream_valid_required_) {
      upstream_status_sub_ = create_subscription<rm_competition_interfaces::msg::LocalizationStatus>(
        upstream_valid_topic_ + "/status", rclcpp::QoS(1).reliable(),
        [this](const rm_competition_interfaces::msg::LocalizationStatus::SharedPtr msg) {
          handleUpstreamStatus(*msg);
        });
    } else {
      // Direct external-pose fixtures have a finite correction lease, no cached Bool authority.
      global_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        global_pose_topic_, rclcpp::QoS(10).reliable(),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          checkValidityLocked();
          handleGlobalPoseLocked(*msg);
        });
    }

    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/reset_map_to_odom",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        recovery_after_ns_ = std::max(recovery_after_ns_, now().nanoseconds());
        invalidateLocked();
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
  }

private:
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

  void invalidateLocked()
  {
    valid_ = false;
    upstream_valid_ = false;
    recovery_after_ns_ = std::max(recovery_after_ns_, std::max(last_pose_ns_, upstream_pose_ns_));
    pending_global_poses_.clear();
    publishValid(false);
  }

  void checkValidityLocked()
  {
    const auto clock_ns = now().nanoseconds();
    if (started_at_.nanoseconds() <= 0 && clock_ns > 0) {
      started_at_ = now();
      recovery_after_ns_ = clock_ns;
    }
    const bool clock_reversed = clock_ns < last_clock_ns_;
    last_clock_ns_ = std::max(last_clock_ns_, clock_ns);
    const double silence = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_evidence_received_).count();
    if (clock_reversed || ((valid_ || upstream_valid_) &&
      (silence > status_timeout_sec_ || clock_ns > evidence_until_.nanoseconds() ||
      (upstream_valid_required_ &&
      (clock_ns - upstream_emitted_ns_) * 1.0e-9 > status_timeout_sec_))))
    {
      if (clock_reversed) {recovery_after_ns_ = last_clock_ns_;}
      invalidateLocked();
    }
  }

  void handleUpstreamStatus(const rm_competition_interfaces::msg::LocalizationStatus & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    checkValidityLocked();
    const auto well_formed = [](const builtin_interfaces::msg::Time & stamp) {
        return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
      };
    if (!well_formed(msg.header.stamp) || !well_formed(msg.started_at) ||
      !well_formed(msg.valid_until))
    {
      invalidateLocked();
      return;
    }
    const auto current = now();
    const auto emitted = rclcpp::Time(msg.header.stamp);
    const auto epoch = rclcpp::Time(msg.started_at).nanoseconds();
    const auto deadline = rclcpp::Time(msg.valid_until);
    const double age = (current - emitted).seconds();
    if (msg.producer_id.empty() || msg.sequence == 0 || epoch <= 0 ||
      epoch > emitted.nanoseconds() || emitted.nanoseconds() <= started_at_.nanoseconds() ||
      age < -max_future_sec_ || age > status_timeout_sec_ || epoch < upstream_epoch_ns_ ||
      (epoch == upstream_epoch_ns_ && !upstream_id_.empty() && msg.producer_id != upstream_id_) ||
      (msg.producer_id == upstream_id_ &&
      (msg.sequence <= upstream_sequence_ || emitted.nanoseconds() < upstream_emitted_ns_)))
    {
      invalidateLocked();
      return;
    }
    if (msg.producer_id != upstream_id_) {
      if (!upstream_id_.empty()) {invalidateLocked();}
      upstream_id_ = msg.producer_id;
      upstream_epoch_ns_ = epoch;
      upstream_pose_ns_ = 0;
    }
    upstream_sequence_ = msg.sequence;
    upstream_emitted_ns_ = emitted.nanoseconds();
    if (!msg.valid || deadline <= current ||
      (deadline - emitted).seconds() > 1.6)
    {
      invalidateLocked();
      return;
    }
    upstream_valid_ = true;
    last_evidence_received_ = std::chrono::steady_clock::now();
    evidence_until_ = deadline;
    const auto source_ns = rclcpp::Time(msg.pose.header.stamp).nanoseconds();
    if (source_ns > upstream_pose_ns_) {
      upstream_pose_ns_ = source_ns;
      handleGlobalPoseLocked(msg.pose);
    } else if (source_ns < upstream_pose_ns_) {
      invalidateLocked();
    }
    // A heartbeat for the held pose cannot restore validity after invalidation.
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
    checkValidityLocked();
    const bool monotonic = odom_cache_.add({stamp.nanoseconds(), transform});
    if (!monotonic) {
      invalidateLocked();
      RCLCPP_WARN(
        get_logger(),
        "Odometry timestamp reset detected. Cleared cache and invalidated map->odom.");
      return;
    }
    processPendingGlobalPoses();
  }

  void handleGlobalPoseLocked(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    if (msg.header.frame_id != map_frame_) {
      invalidateLocked();
      RCLCPP_WARN(
        get_logger(), "Rejecting global pose in frame '%s'; expected '%s'.",
        msg.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    if (stamp.nanoseconds() <= 0 || !validatePose(msg.pose.pose)) {
      invalidateLocked();
      RCLCPP_WARN(get_logger(), "Rejecting global pose with zero timestamp or invalid pose.");
      return;
    }

    const auto current_time = now();
    const double age = (current_time - stamp).seconds();
    if (stamp.nanoseconds() <= recovery_after_ns_ || stamp.nanoseconds() <= last_pose_ns_ ||
      age < -max_future_sec_ || age > max_global_pose_age_sec_) {
      invalidateLocked();
      RCLCPP_WARN(
        get_logger(), "Rejecting stale, future, replayed or pre-session global pose: age %.3f s, age limit %.3f s.",
        age, max_global_pose_age_sec_);
      return;
    }

    const auto map_to_base = poseToTransform(msg.pose.pose);
    const auto tolerance_ns = static_cast<std::int64_t>(max_pose_odom_dt_sec_ * 1.0e9);

    last_pose_ns_ = stamp.nanoseconds();
    if (!upstream_valid_required_) {
      last_evidence_received_ = std::chrono::steady_clock::now();
      evidence_until_ = now() + rclcpp::Duration::from_seconds(status_timeout_sec_);
    }
    const auto odom_sample = odom_cache_.nearest(stamp.nanoseconds(), tolerance_ns);
    if (!odom_sample.has_value()) {
      if (pending_global_poses_.size() >= pending_pose_max_size_) {
        pending_global_poses_.pop_front();
        RCLCPP_WARN(get_logger(), "Global-pose pending queue full; dropped oldest pose.");
      }
      pending_global_poses_.push_back({
        stamp.nanoseconds(), map_to_base, std::chrono::steady_clock::now(), msg});
      return;
    }

    acceptCorrection(map_to_base, *odom_sample, msg);
  }

  struct PendingGlobalPose
  {
    std::int64_t stamp_nanoseconds;
    tf2::Transform map_to_base;
    std::chrono::steady_clock::time_point received_at;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
  };

  void processPendingGlobalPoses()
  {
    const auto tolerance_ns = static_cast<std::int64_t>(max_pose_odom_dt_sec_ * 1.0e9);
    const auto now_steady = std::chrono::steady_clock::now();
    auto pending = pending_global_poses_.begin();
    while (pending != pending_global_poses_.end()) {
      const double waiting_seconds =
        std::chrono::duration<double>(now_steady - pending->received_at).count();
      const double source_age = (now().nanoseconds() - pending->stamp_nanoseconds) * 1.0e-9;
      if (waiting_seconds > pending_pose_max_wait_sec_ || source_age > max_global_pose_age_sec_ ||
        pending->stamp_nanoseconds <= recovery_after_ns_ ||
        (upstream_valid_required_ && !upstream_valid_))
      {
        pending = pending_global_poses_.erase(pending);
        continue;
      }
      const auto odom_sample = odom_cache_.nearest(pending->stamp_nanoseconds, tolerance_ns);
      if (odom_sample.has_value()) {
        const auto matched = *pending;
        pending_global_poses_.erase(pending);
        acceptCorrection(matched.map_to_base, *odom_sample, matched.pose);
        pending = pending_global_poses_.begin();
        continue;
      }
      ++pending;
    }
  }

  void acceptCorrection(
    const tf2::Transform & map_to_base,
    const rm_relocalization_bridge::TimedTransform & odom_sample,
    const geometry_msgs::msg::PoseWithCovarianceStamped & source_pose)
  {

    map_to_odom_ = rm_relocalization_bridge::computeMapToOdom(
      map_to_base, odom_sample.transform);
    if (!rm_relocalization_bridge::isFiniteTransform(map_to_odom_)) {
      RCLCPP_ERROR(get_logger(), "Computed non-finite map->odom; correction rejected.");
      invalidateLocked();
      return;
    }

    accepted_pose_ = source_pose;
    valid_ = true;
    publishValid(valid_ && (!upstream_valid_required_ || upstream_valid_));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Accepted global pose at %.6f; map->odom correction updated.",
      rclcpp::Time(source_pose.header.stamp).seconds());
  }

  void publishValid(bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    valid_pub_->publish(message);
    rm_competition_interfaces::msg::LocalizationStatus status;
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.producer_id = producer_id_;
    status.started_at = started_at_;
    status.sequence = ++sequence_;
    status.valid = value;
    status.valid_until = evidence_until_;
    status.pose = accepted_pose_;
    status_pub_->publish(status);
  }

  void publishCurrentTransform()
  {
    tf2::Transform transform;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      checkValidityLocked();
      publishValid(valid_ && (!upstream_valid_required_ || upstream_valid_));
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

  std::string producer_id_, upstream_id_;
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME}, evidence_until_{0, 0, RCL_ROS_TIME};
  int64_t recovery_after_ns_{0}, last_clock_ns_{0}, last_pose_ns_{0};
  int64_t upstream_epoch_ns_{0}, upstream_emitted_ns_{0}, upstream_pose_ns_{0};
  uint64_t sequence_{0}, upstream_sequence_{0};
  double status_timeout_sec_{1.0}, max_future_sec_{0.1};
  std::chrono::steady_clock::time_point last_evidence_received_;
  geometry_msgs::msg::PoseWithCovarianceStamped accepted_pose_;
  rclcpp::Publisher<rm_competition_interfaces::msg::LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::LocalizationStatus>::SharedPtr upstream_status_sub_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string global_pose_topic_;
  std::string upstream_valid_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  bool publish_tf_;
  double publish_rate_hz_;
  double max_pose_odom_dt_sec_;
  double max_global_pose_age_sec_;
  double pending_pose_max_wait_sec_;
  std::size_t pending_pose_max_size_;

  std::mutex mutex_;
  rm_relocalization_bridge::TimedTransformCache odom_cache_;
  std::deque<PendingGlobalPose> pending_global_poses_;
  tf2::Transform map_to_odom_;
  bool valid_ = false;
  bool upstream_valid_required_ = false;
  bool upstream_valid_ = false;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
  global_pose_sub_;
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
