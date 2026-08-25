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

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
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
    autonomous_recovery_enabled_ =
      declare_parameter<bool>("autonomous_recovery_enabled", false);
    recovery_linear_speed_threshold_mps_ =
      declare_parameter<double>("recovery_linear_speed_threshold_mps", 0.08);
    recovery_angular_speed_threshold_radps_ =
      declare_parameter<double>("recovery_angular_speed_threshold_radps", 0.15);
    recovery_motion_confirmation_sec_ =
      declare_parameter<double>("recovery_motion_confirmation_sec", 0.12);
    recovery_stationary_hold_sec_ =
      declare_parameter<double>("recovery_stationary_hold_sec", 0.6);
    recovery_reseed_cooldown_sec_ =
      declare_parameter<double>("recovery_reseed_cooldown_sec", 2.0);
    recovery_settle_sec_ = declare_parameter<double>("recovery_settle_sec", 0.5);
    recovery_required_consistent_poses_ = static_cast<std::size_t>(
      std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("recovery_required_consistent_poses", 5)));
    recovery_initial_pose_xy_variance_ =
      declare_parameter<double>("recovery_initial_pose_xy_variance", 0.09);
    recovery_initial_pose_yaw_variance_ =
      declare_parameter<double>("recovery_initial_pose_yaw_variance", 0.0685);
    recovery_state_topic_ = declare_parameter<std::string>(
      "recovery_state_topic", "/localization/correction_recovery_state");
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
      max_correction_yaw_step_rad_ <= 0.0 ||
      !std::isfinite(recovery_linear_speed_threshold_mps_) ||
      recovery_linear_speed_threshold_mps_ < 0.0 ||
      !std::isfinite(recovery_angular_speed_threshold_radps_) ||
      recovery_angular_speed_threshold_radps_ < 0.0 ||
      !std::isfinite(recovery_motion_confirmation_sec_) ||
      recovery_motion_confirmation_sec_ < 0.0 ||
      !std::isfinite(recovery_stationary_hold_sec_) || recovery_stationary_hold_sec_ < 0.0 ||
      !std::isfinite(recovery_reseed_cooldown_sec_) || recovery_reseed_cooldown_sec_ <= 0.0 ||
      !std::isfinite(recovery_settle_sec_) || recovery_settle_sec_ < 0.0 ||
      !std::isfinite(recovery_initial_pose_xy_variance_) ||
      recovery_initial_pose_xy_variance_ < 0.0 ||
      !std::isfinite(recovery_initial_pose_yaw_variance_) ||
      recovery_initial_pose_yaw_variance_ < 0.0)
    {
      throw std::invalid_argument("time tolerances and correction limits must be valid");
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(output_topic_, 10);

    auto valid_qos = rclcpp::QoS(1).reliable().transient_local();
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/global_localization_valid", valid_qos);
    recovery_state_pub_ = create_publisher<std_msgs::msg::String>(
      recovery_state_topic_, valid_qos);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(100).reliable(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {handleOdometry(*msg);});
    global_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      global_pose_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        handleGlobalPose(*msg);
      });
    if (correction_innovation_gate_enabled_) {
      if (autonomous_recovery_enabled_) {
        initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          initial_pose_topic_, rclcpp::QoS(10).reliable());
      }
      initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_, rclcpp::QoS(10).reliable(),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          handleInitialPose(*msg);
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
        resetRecoveryLocked();
        pending_global_poses_.clear();
        publishValid(false);
        publishRecoveryState("waiting_for_baseline");
        response->success = true;
        response->message = "map->odom correction invalidated";
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publishCurrentTransform();});

    publishValid(false);
    publishRecoveryState("waiting_for_baseline");
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
      if (autonomous_recovery_enabled_) {
        RCLCPP_INFO(
          get_logger(),
          "Autonomous correction recovery enabled: stationary <= %.3f m/s and %.3f rad/s "
          "for %.2f s; motion confirmation %.2f s; reseed cooldown %.2f s; "
          "settle %.2f s; require %zu poses.",
          recovery_linear_speed_threshold_mps_, recovery_angular_speed_threshold_radps_,
          recovery_stationary_hold_sec_, recovery_motion_confirmation_sec_,
          recovery_reseed_cooldown_sec_, recovery_settle_sec_,
          recovery_required_consistent_poses_);
      }
    } else {
      RCLCPP_INFO(get_logger(), "Correction innovation gate disabled.");
    }
  }

private:
  void resetRecoveryLocked()
  {
    correction_fault_latched_ = false;
    recovery_stationary_since_valid_ = false;
    recovery_motion_candidate_valid_ = false;
    recovery_has_reseeded_ = false;
    recovery_consistent_pose_count_ = 0;
    recovery_reseed_attempts_ = 0;
    pending_automatic_initial_pose_stamp_nanoseconds_ = 0;
  }

  void publishRecoveryState(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    recovery_state_pub_->publish(message);
  }

  void handleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::int64_t stamp_nanoseconds = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (
      stamp_nanoseconds > 0 &&
      stamp_nanoseconds == pending_automatic_initial_pose_stamp_nanoseconds_)
    {
      pending_automatic_initial_pose_stamp_nanoseconds_ = 0;
      RCLCPP_INFO(
        get_logger(), "Observed the bridge's autonomous AMCL reseed; trusted baseline kept.");
      return;
    }
    resetCorrectionBaselineLocked("explicit initial pose request");
  }

  void resetCorrectionBaselineLocked(const char * reason)
  {
    valid_ = false;
    has_accepted_correction_ = false;
    resetRecoveryLocked();
    pending_global_poses_.clear();
    publishValid(false);
    publishRecoveryState("waiting_for_baseline");
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

  bool recoveryStationaryHeld(const std::chrono::steady_clock::time_point & now_steady) const
  {
    if (!recovery_stationary_since_valid_) {
      return false;
    }
    return std::chrono::duration<double>(
      now_steady - recovery_stationary_since_).count() >= recovery_stationary_hold_sec_;
  }

  void publishAutonomousRecoveryPose(
    const tf2::Transform & odom_to_base,
    const builtin_interfaces::msg::Time & stamp,
    const std::chrono::steady_clock::time_point & now_steady)
  {
    const auto predicted_map_to_base = map_to_odom_ * odom_to_base;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(predicted_map_to_base.getRotation()).getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;

    tf2::Quaternion planar_rotation;
    planar_rotation.setRPY(0.0, 0.0, yaw);
    planar_rotation.normalize();

    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.pose.pose.position.x = predicted_map_to_base.getOrigin().x();
    message.pose.pose.position.y = predicted_map_to_base.getOrigin().y();
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation.x = planar_rotation.x();
    message.pose.pose.orientation.y = planar_rotation.y();
    message.pose.pose.orientation.z = planar_rotation.z();
    message.pose.pose.orientation.w = planar_rotation.w();
    message.pose.covariance[0] = recovery_initial_pose_xy_variance_;
    message.pose.covariance[7] = recovery_initial_pose_xy_variance_;
    message.pose.covariance[35] = recovery_initial_pose_yaw_variance_;

    pending_automatic_initial_pose_stamp_nanoseconds_ = rclcpp::Time(stamp).nanoseconds();
    recovery_has_reseeded_ = true;
    ++recovery_reseed_attempts_;
    recovery_consistent_pose_count_ = 0;
    recovery_last_reseed_at_ = now_steady;
    recovery_settle_until_ = now_steady + std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(std::chrono::duration<double>(recovery_settle_sec_));
    publishRecoveryState("reseeded_waiting_for_consistency");
    initial_pose_pub_->publish(message);
    RCLCPP_WARN(
      get_logger(),
      "Autonomous AMCL reseed attempt %zu at trusted prediction (%.3f, %.3f, %.3f).",
      recovery_reseed_attempts_, message.pose.pose.position.x, message.pose.pose.position.y, yaw);
  }

  void updateAutonomousRecovery(
    const nav_msgs::msg::Odometry & msg,
    const tf2::Transform & odom_to_base)
  {
    if (!correction_fault_latched_ || !autonomous_recovery_enabled_) {
      return;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    const double linear_speed = std::hypot(
      msg.twist.twist.linear.x, msg.twist.twist.linear.y);
    const double angular_speed = std::abs(msg.twist.twist.angular.z);
    const bool stationary =
      std::isfinite(linear_speed) && std::isfinite(angular_speed) &&
      linear_speed <= recovery_linear_speed_threshold_mps_ &&
      angular_speed <= recovery_angular_speed_threshold_radps_;

    if (!stationary) {
      if (!recovery_motion_candidate_valid_) {
        recovery_motion_candidate_since_ = now_steady;
        recovery_motion_candidate_valid_ = true;
        return;
      }
      const bool motion_confirmed = std::chrono::duration<double>(
        now_steady - recovery_motion_candidate_since_).count() >=
        recovery_motion_confirmation_sec_;
      if (!motion_confirmed) {
        return;
      }
      if (recovery_stationary_since_valid_) {
        publishRecoveryState("latched_waiting_for_stop");
      }
      recovery_stationary_since_valid_ = false;
      recovery_consistent_pose_count_ = 0;
      return;
    }

    recovery_motion_candidate_valid_ = false;
    if (!recovery_stationary_since_valid_) {
      recovery_stationary_since_ = now_steady;
      recovery_stationary_since_valid_ = true;
      publishRecoveryState("latched_waiting_for_stationary_hold");
      return;
    }
    if (!recoveryStationaryHeld(now_steady)) {
      return;
    }

    const bool cooldown_elapsed =
      !recovery_has_reseeded_ ||
      std::chrono::duration<double>(now_steady - recovery_last_reseed_at_).count() >=
      recovery_reseed_cooldown_sec_;
    if (cooldown_elapsed) {
      publishAutonomousRecoveryPose(odom_to_base, msg.header.stamp, now_steady);
    }
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
      resetRecoveryLocked();
      pending_global_poses_.clear();
      publishValid(false);
      publishRecoveryState("waiting_for_baseline");
      RCLCPP_WARN(
        get_logger(),
        "Odometry timestamp reset detected. Cleared cache and invalidated map->odom.");
      return;
    }
    processPendingGlobalPoses();
    updateAutonomousRecovery(msg, transform);
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

  void latchCorrectionFault()
  {
    valid_ = false;
    publishValid(false);
    if (correction_fault_latched_) {
      return;
    }
    correction_fault_latched_ = true;
    recovery_stationary_since_valid_ = false;
    recovery_motion_candidate_valid_ = false;
    recovery_has_reseeded_ = false;
    recovery_consistent_pose_count_ = 0;
    recovery_reseed_attempts_ = 0;
    pending_automatic_initial_pose_stamp_nanoseconds_ = 0;
    publishRecoveryState(
      autonomous_recovery_enabled_ ? "latched_waiting_for_stop" : "latched_manual_reset_required");
  }

  void completeAutonomousRecovery(const tf2::Transform & candidate_map_to_odom)
  {
    map_to_odom_ = candidate_map_to_odom;
    valid_ = true;
    correction_fault_latched_ = false;
    recovery_stationary_since_valid_ = false;
    recovery_motion_candidate_valid_ = false;
    recovery_has_reseeded_ = false;
    recovery_consistent_pose_count_ = 0;
    pending_automatic_initial_pose_stamp_nanoseconds_ = 0;
    publishValid(valid_ && (!upstream_valid_required_ || upstream_valid_));
    publishRecoveryState("healthy");
    RCLCPP_WARN(
      get_logger(),
      "Autonomous correction recovery completed after %zu reseed attempt(s); "
      "canonical map->odom resumed.",
      recovery_reseed_attempts_);
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
      const bool innovation_within_limits =
        innovation.translation_xy_m <= max_correction_translation_step_m_ &&
        innovation.yaw_rad <= max_correction_yaw_step_rad_;

      if (!innovation_within_limits) {
        recovery_consistent_pose_count_ = 0;
        latchCorrectionFault();
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Rejected map->odom correction innovation: translation %.3f m (limit %.3f), "
          "yaw %.3f rad (limit %.3f). Trusted correction retained; recovery state is latched.",
          innovation.translation_xy_m, max_correction_translation_step_m_,
          innovation.yaw_rad, max_correction_yaw_step_rad_);
        return;
      }

      if (correction_fault_latched_) {
        valid_ = false;
        publishValid(false);
        const auto now_steady = std::chrono::steady_clock::now();
        const bool ready_to_evaluate =
          autonomous_recovery_enabled_ && recovery_has_reseeded_ &&
          recoveryStationaryHeld(now_steady) && now_steady >= recovery_settle_until_;
        if (!ready_to_evaluate) {
          recovery_consistent_pose_count_ = 0;
          return;
        }

        ++recovery_consistent_pose_count_;
        if (recovery_consistent_pose_count_ < recovery_required_consistent_poses_) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Autonomous recovery consistency %zu/%zu; canonical TF remains withheld.",
            recovery_consistent_pose_count_, recovery_required_consistent_poses_);
          return;
        }
        completeAutonomousRecovery(candidate_map_to_odom);
        return;
      }

    }

    map_to_odom_ = candidate_map_to_odom;
    has_accepted_correction_ = true;
    valid_ = true;
    publishValid(valid_ && (!upstream_valid_required_ || upstream_valid_));
    publishRecoveryState("healthy");
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
  std::string recovery_state_topic_;
  bool publish_tf_;
  bool correction_innovation_gate_enabled_;
  bool autonomous_recovery_enabled_;
  double publish_rate_hz_;
  double max_pose_odom_dt_sec_;
  double max_global_pose_age_sec_;
  double pending_pose_max_wait_sec_;
  double max_correction_translation_step_m_;
  double max_correction_yaw_step_rad_;
  double recovery_linear_speed_threshold_mps_;
  double recovery_angular_speed_threshold_radps_;
  double recovery_motion_confirmation_sec_;
  double recovery_stationary_hold_sec_;
  double recovery_reseed_cooldown_sec_;
  double recovery_settle_sec_;
  double recovery_initial_pose_xy_variance_;
  double recovery_initial_pose_yaw_variance_;
  std::size_t recovery_required_consistent_poses_;
  std::size_t pending_pose_max_size_;

  std::mutex mutex_;
  rm_relocalization_bridge::TimedTransformCache odom_cache_;
  std::deque<PendingGlobalPose> pending_global_poses_;
  tf2::Transform map_to_odom_;
  bool has_accepted_correction_ = false;
  bool correction_fault_latched_ = false;
  bool recovery_stationary_since_valid_ = false;
  bool recovery_motion_candidate_valid_ = false;
  bool recovery_has_reseeded_ = false;
  std::size_t recovery_consistent_pose_count_ = 0;
  std::size_t recovery_reseed_attempts_ = 0;
  std::int64_t pending_automatic_initial_pose_stamp_nanoseconds_ = 0;
  std::chrono::steady_clock::time_point recovery_stationary_since_;
  std::chrono::steady_clock::time_point recovery_motion_candidate_since_;
  std::chrono::steady_clock::time_point recovery_last_reseed_at_;
  std::chrono::steady_clock::time_point recovery_settle_until_;
  bool valid_ = false;
  bool upstream_valid_required_ = false;
  bool upstream_valid_ = false;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
  initial_pose_pub_;
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
