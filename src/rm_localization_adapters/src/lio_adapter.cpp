#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/chassis_heading_state.hpp"
#include "rm_competition_interfaces/msg/gimbal_state.hpp"
#include "rm_localization_adapters/canonical_odometry.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class LioAdapter : public rclcpp::Node
{
public:
  LioAdapter()
  : Node("lio_adapter")
  {
    raw_odom_topic_ = declare_parameter<std::string>(
      "raw_odom_topic", "/odometry/fast_lio_raw");
    output_odom_topic_ = declare_parameter<std::string>(
      "output_odom_topic", "/odometry/lio");
    expected_input_odom_frame_ = declare_parameter<std::string>(
      "expected_input_odom_frame", "odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    input_sensor_frame_ = declare_parameter<std::string>("input_sensor_frame", "lio_imu_link");
    gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "gimbal_yaw_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    use_tf_sensor_to_base_ = declare_parameter<bool>("use_tf_sensor_to_base", true);
    use_latest_transform_ = declare_parameter<bool>("use_latest_transform", false);
    allow_placeholder_fallback_ = declare_parameter<bool>("allow_placeholder_fallback", false);
    pose_conversion_mode_ = declare_parameter<std::string>(
      "pose_conversion_mode", "sensor_tf");
    if (pose_conversion_mode_ != "sensor_tf" &&
      pose_conversion_mode_ != "chassis_heading_fusion")
    {
      throw std::invalid_argument(
        "pose_conversion_mode must be 'sensor_tf' or 'chassis_heading_fusion'");
    }
    raw_odom_parent_frame_mode_ = declare_parameter<std::string>(
      "raw_odom_parent_frame_mode", "canonical_odom");
    if (raw_odom_parent_frame_mode_ != "canonical_odom" &&
      raw_odom_parent_frame_mode_ != "sensor_initial")
    {
      throw std::invalid_argument(
        "raw_odom_parent_frame_mode must be 'canonical_odom' or 'sensor_initial'");
    }
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
    const int tf_queue_max_size = declare_parameter<int>("tf_queue.max_size", 100);
    tf_queue_max_wait_sec_ = declare_parameter<double>("tf_queue.max_wait_sec", 0.2);
    tf_queue_retry_rate_hz_ = declare_parameter<double>("tf_queue.retry_rate_hz", 200.0);
    if (!std::isfinite(tf_lookup_timeout_sec_) || tf_lookup_timeout_sec_ < 0.0 ||
      tf_queue_max_size <= 0 || !std::isfinite(tf_queue_max_wait_sec_) ||
      tf_queue_max_wait_sec_ <= 0.0 || !std::isfinite(tf_queue_retry_rate_hz_) ||
      tf_queue_retry_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("TF queue size, wait time, and retry rate must be positive");
    }
    tf_queue_max_size_ = static_cast<std::size_t>(tf_queue_max_size);
    backend_child_frame_alias_enabled_ = declare_parameter<bool>(
      "backend_child_frame_alias_enabled", false);
    backend_child_frame_alias_source_ = declare_parameter<std::string>(
      "backend_child_frame_alias_source", "body");
    backend_child_frame_alias_target_ = declare_parameter<std::string>(
      "backend_child_frame_alias_target", "lio_imu_link");
    twist_mode_ = declare_parameter<std::string>("twist_mode", "passthrough");

    if (twist_mode_ != "passthrough" && twist_mode_ != "finite_difference") {
      throw std::invalid_argument("twist_mode must be 'passthrough' or 'finite_difference'");
    }
    if (pose_conversion_mode_ == "chassis_heading_fusion" &&
      twist_mode_ != "finite_difference")
    {
      throw std::invalid_argument(
        "chassis_heading_fusion requires twist_mode=finite_difference");
    }

    heading_topic_ = declare_parameter<std::string>(
      "heading_fusion.heading_topic", "/chassis/heading");
    derived_gimbal_topic_ = declare_parameter<std::string>(
      "heading_fusion.derived_gimbal_topic", "/gimbal/state_derived");
    const int heading_cache_size = declare_parameter<int>(
      "heading_fusion.cache_size", 500);
    max_heading_match_dt_sec_ = declare_parameter<double>(
      "heading_fusion.max_heading_match_dt_sec", 0.03);
    heading_yaw_variance_ = declare_parameter<double>(
      "heading_fusion.yaw_variance", 0.01);
    initial_gimbal_yaw_rad_ = declare_parameter<double>(
      "heading_fusion.initial_gimbal_yaw_rad", 0.0);
    max_center_offset_xy_m_ = declare_parameter<double>(
      "heading_fusion.max_center_offset_xy_m", 0.05);
    const bool initial_alignment_confirmed = declare_parameter<bool>(
      "heading_fusion.initial_alignment_confirmed", false);

    const double initial_sensor_x = declare_parameter<double>(
      "heading_fusion.initial_base_to_sensor.x", 0.0);
    const double initial_sensor_y = declare_parameter<double>(
      "heading_fusion.initial_base_to_sensor.y", 0.0);
    const double initial_sensor_z = declare_parameter<double>(
      "heading_fusion.initial_base_to_sensor.z", 0.0);
    const double initial_sensor_roll = declare_parameter<double>(
      "heading_fusion.initial_base_to_sensor.roll", 0.0);
    const double initial_sensor_pitch = declare_parameter<double>(
      "heading_fusion.initial_base_to_sensor.pitch", 0.0);
    const double initial_sensor_yaw = declare_parameter<double>(
      "heading_fusion.initial_base_to_sensor.yaw", 0.0);
    tf2::Quaternion initial_sensor_rotation;
    initial_sensor_rotation.setRPY(
      initial_sensor_roll, initial_sensor_pitch, initial_sensor_yaw);
    initial_sensor_rotation.normalize();
    initial_base_to_sensor_.setOrigin(
      tf2::Vector3(initial_sensor_x, initial_sensor_y, initial_sensor_z));
    initial_base_to_sensor_.setRotation(initial_sensor_rotation);

    if (heading_cache_size <= 0 || !std::isfinite(max_heading_match_dt_sec_) ||
      max_heading_match_dt_sec_ <= 0.0 || !std::isfinite(heading_yaw_variance_) ||
      heading_yaw_variance_ < 0.0 || !std::isfinite(initial_gimbal_yaw_rad_) ||
      !std::isfinite(max_center_offset_xy_m_) || max_center_offset_xy_m_ < 0.0)
    {
      throw std::invalid_argument("heading_fusion parameters are invalid");
    }
    heading_cache_max_size_ = static_cast<std::size_t>(heading_cache_size);
    if (pose_conversion_mode_ == "chassis_heading_fusion") {
      if (!initial_alignment_confirmed) {
        throw std::invalid_argument(
          "chassis_heading_fusion requires initial_alignment_confirmed=true after "
          "placing the gimbal at its documented home angle");
      }
      if (std::hypot(initial_sensor_x, initial_sensor_y) > max_center_offset_xy_m_) {
        throw std::invalid_argument(
          "chassis_heading_fusion requires the sensor origin to be coaxial with the "
          "chassis yaw center within max_center_offset_xy_m");
      }
    }

    const auto twist_variance = declare_parameter<std::vector<double>>(
      "twist_variance_diagonal", {1.0, 1.0, 1.0, 4.0, 4.0, 4.0});
    if (twist_variance.size() != twist_variance_diagonal_.size()) {
      throw std::invalid_argument("twist_variance_diagonal must contain six values");
    }
    for (std::size_t index = 0; index < twist_variance.size(); ++index) {
      if (twist_variance[index] < 0.0) {
        throw std::invalid_argument("twist variances must be non-negative");
      }
      twist_variance_diagonal_[index] = twist_variance[index];
    }

    if (twist_mode_ == "finite_difference") {
      rm_localization_adapters::TwistEstimatorConfig estimator_config;
      estimator_config.min_dt_sec = declare_parameter<double>(
        "twist_estimator.min_dt_sec", 0.001);
      estimator_config.max_dt_sec = declare_parameter<double>(
        "twist_estimator.max_dt_sec", 0.5);
      estimator_config.smoothing_alpha = declare_parameter<double>(
        "twist_estimator.smoothing_alpha", 0.3);
      estimator_config.max_linear_speed = declare_parameter<double>(
        "twist_estimator.max_linear_speed", 5.0);
      estimator_config.max_angular_speed = declare_parameter<double>(
        "twist_estimator.max_angular_speed", 20.0);
      twist_estimator_ =
        std::make_unique<rm_localization_adapters::PoseTwistEstimator>(estimator_config);
    }

    const double input_to_base_x =
      declare_parameter<double>("input_to_base_placeholder.x", 0.0);
    const double input_to_base_y =
      declare_parameter<double>("input_to_base_placeholder.y", 0.0);
    const double input_to_base_z =
      declare_parameter<double>("input_to_base_placeholder.z", 0.0);
    const double input_to_base_roll =
      declare_parameter<double>("input_to_base_placeholder.roll", 0.0);
    const double input_to_base_pitch =
      declare_parameter<double>("input_to_base_placeholder.pitch", 0.0);
    const double input_to_base_yaw =
      declare_parameter<double>("input_to_base_placeholder.yaw", 0.0);

    tf2::Quaternion input_to_base_q;
    input_to_base_q.setRPY(input_to_base_roll, input_to_base_pitch, input_to_base_yaw);
    input_to_base_placeholder_.setOrigin(
      tf2::Vector3(input_to_base_x, input_to_base_y, input_to_base_z));
    input_to_base_placeholder_.setRotation(input_to_base_q);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odom_topic_, 10,
      std::bind(&LioAdapter::handleRawOdometry, this, std::placeholders::_1));
    if (pose_conversion_mode_ == "chassis_heading_fusion") {
      derived_gimbal_pub_ = create_publisher<rm_competition_interfaces::msg::GimbalState>(
        derived_gimbal_topic_, rclcpp::QoS(50));
      heading_sub_ = create_subscription<
        rm_competition_interfaces::msg::ChassisHeadingState>(
        heading_topic_, rclcpp::QoS(100),
        std::bind(&LioAdapter::handleChassisHeading, this, std::placeholders::_1));
      publishInvalidDerivedGimbal("waiting for synchronized LIO and chassis heading");
    }
    const auto retry_period = std::chrono::duration<double>(1.0 / tf_queue_retry_rate_hz_);
    tf_retry_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(retry_period),
      std::bind(&LioAdapter::retryPendingOdometry, this));

    RCLCPP_WARN(
      get_logger(),
      "Canonical LIO adapter: adapting %s to %s through TF %s->sensor with gimbal frame %s. "
      "pose_conversion_mode=%s raw_odom_parent_frame_mode=%s twist_mode=%s.",
      raw_odom_topic_.c_str(), output_odom_topic_.c_str(),
      base_frame_.c_str(), gimbal_frame_.c_str(), pose_conversion_mode_.c_str(),
      raw_odom_parent_frame_mode_.c_str(),
      twist_mode_.c_str());
  }

private:
  struct PendingOdometry
  {
    nav_msgs::msg::Odometry::SharedPtr message;
    std::chrono::steady_clock::time_point queued_at;
  };

  struct HeadingSample
  {
    std::int64_t stamp_nanoseconds{0};
    double yaw_rad{0.0};
    double yaw_rate_rad_s{0.0};
    std::uint32_t sample_sequence{0};
    std::uint32_t mcu_time_ms{0};
    std::uint32_t source_boot_id{0};
    std::uint16_t reset_counter{0};
  };

  static bool isNewerSequence(std::uint32_t candidate, std::uint32_t reference)
  {
    const auto delta = static_cast<std::uint32_t>(candidate - reference);
    return delta != 0U && delta < 0x80000000U;
  }

  void handleChassisHeading(
    const rm_competition_interfaces::msg::ChassisHeadingState::SharedPtr msg)
  {
    const bool has_stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;
    if (!msg->valid || !msg->online || !has_stamp || !std::isfinite(msg->yaw_rad) ||
      !std::isfinite(msg->yaw_rate_rad_s))
    {
      heading_cache_.clear();
      heading_stream_valid_ = false;
      publishInvalidDerivedGimbal("invalid, offline, or non-finite chassis heading");
      return;
    }

    HeadingSample sample;
    sample.stamp_nanoseconds = rclcpp::Time(msg->header.stamp).nanoseconds();
    sample.yaw_rad = rm_localization_adapters::wrap_angle(msg->yaw_rad);
    sample.yaw_rate_rad_s = msg->yaw_rate_rad_s;
    sample.sample_sequence = msg->sample_sequence;
    sample.mcu_time_ms = msg->mcu_time_ms;
    sample.source_boot_id = msg->source_boot_id;
    sample.reset_counter = msg->reset_counter;
    if (sample.stamp_nanoseconds <= 0) {
      heading_stream_valid_ = false;
      publishInvalidDerivedGimbal("zero chassis-heading timestamp");
      return;
    }

    if (have_last_heading_identity_) {
      if (sample.source_boot_id != last_heading_boot_id_ ||
        sample.reset_counter != last_heading_reset_counter_)
      {
        if (heading_fusion_initialized_) {
          heading_reset_latched_ = true;
          heading_stream_valid_ = false;
          heading_cache_.clear();
          pending_odometry_.clear();
          if (twist_estimator_) {
            twist_estimator_->reset();
          }
          publishInvalidDerivedGimbal(
            "chassis heading reset; restart lio_adapter after re-homing the gimbal");
          RCLCPP_ERROR(
            get_logger(),
            "Chassis heading source reset while fusion was active. Output is latched invalid "
            "until lio_adapter restarts with the gimbal at its known home angle.");
          return;
        }
        heading_cache_.clear();
      } else if (!isNewerSequence(sample.sample_sequence, last_heading_sequence_)) {
        return;
      }
    }

    if (!heading_cache_.empty() &&
      sample.stamp_nanoseconds <= heading_cache_.back().stamp_nanoseconds)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping non-monotonic chassis-heading timestamp.");
      return;
    }

    last_heading_boot_id_ = sample.source_boot_id;
    last_heading_reset_counter_ = sample.reset_counter;
    last_heading_sequence_ = sample.sample_sequence;
    have_last_heading_identity_ = true;
    heading_stream_valid_ = true;
    heading_cache_.push_back(sample);
    while (heading_cache_.size() > heading_cache_max_size_) {
      heading_cache_.pop_front();
    }
  }

  std::optional<HeadingSample> nearestHeading(std::int64_t stamp_nanoseconds) const
  {
    if (!heading_stream_valid_ || heading_reset_latched_ || heading_cache_.empty()) {
      return std::nullopt;
    }
    const auto tolerance_nanoseconds = static_cast<std::int64_t>(
      max_heading_match_dt_sec_ * 1.0e9);
    const HeadingSample * nearest = nullptr;
    std::int64_t nearest_delta = tolerance_nanoseconds + 1;
    for (const auto & sample : heading_cache_) {
      const std::int64_t signed_delta = sample.stamp_nanoseconds - stamp_nanoseconds;
      const std::int64_t delta = signed_delta < 0 ? -signed_delta : signed_delta;
      if (delta < nearest_delta) {
        nearest = &sample;
        nearest_delta = delta;
      }
    }
    if (nearest == nullptr || nearest_delta > tolerance_nanoseconds) {
      return std::nullopt;
    }
    HeadingSample aligned = *nearest;
    const double dt = static_cast<double>(
      stamp_nanoseconds - nearest->stamp_nanoseconds) * 1.0e-9;
    aligned.yaw_rad = rm_localization_adapters::wrap_angle(
      nearest->yaw_rad + nearest->yaw_rate_rad_s * dt);
    aligned.stamp_nanoseconds = stamp_nanoseconds;
    return aligned;
  }

  void publishInvalidDerivedGimbal(const char * reason)
  {
    if (!derived_gimbal_pub_ || derived_gimbal_invalid_published_) {
      return;
    }
    rm_competition_interfaces::msg::GimbalState message;
    message.header.stamp = now();
    message.valid = false;
    message.online = false;
    derived_gimbal_pub_->publish(message);
    derived_gimbal_invalid_published_ = true;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Derived gimbal state invalidated: %s", reason);
  }

  void publishDerivedGimbal(
    double gimbal_yaw_rad,
    const builtin_interfaces::msg::Time & stamp,
    const HeadingSample & heading)
  {
    double yaw_rate_rad_s = 0.0;
    const std::int64_t stamp_nanoseconds = rclcpp::Time(stamp).nanoseconds();
    if (have_previous_derived_gimbal_) {
      const double dt = static_cast<double>(
        stamp_nanoseconds - previous_derived_gimbal_stamp_nanoseconds_) * 1.0e-9;
      if (dt > 1.0e-4 && dt < 0.5) {
        yaw_rate_rad_s = rm_localization_adapters::wrap_angle(
          gimbal_yaw_rad - previous_derived_gimbal_yaw_rad_) / dt;
      }
    }
    previous_derived_gimbal_yaw_rad_ = gimbal_yaw_rad;
    previous_derived_gimbal_stamp_nanoseconds_ = stamp_nanoseconds;
    have_previous_derived_gimbal_ = true;

    rm_competition_interfaces::msg::GimbalState message;
    message.header.stamp = stamp;
    message.relative_yaw_rad = gimbal_yaw_rad;
    message.yaw_rate_rad_s = yaw_rate_rad_s;
    message.sample_sequence = derived_gimbal_sequence_++;
    message.mcu_time_ms = heading.mcu_time_ms;
    message.online = true;
    message.valid = true;
    derived_gimbal_pub_->publish(message);
    derived_gimbal_invalid_published_ = false;
  }

  static tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
  {
    tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
    q.normalize();
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(
      pose.position.x,
      pose.position.y,
      pose.position.z));
    transform.setRotation(q);
    return transform;
  }

  static tf2::Transform transformMsgToTransform(
    const geometry_msgs::msg::Transform & transform_msg)
  {
    tf2::Quaternion q(
      transform_msg.rotation.x,
      transform_msg.rotation.y,
      transform_msg.rotation.z,
      transform_msg.rotation.w);
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(
      transform_msg.translation.x,
      transform_msg.translation.y,
      transform_msg.translation.z));
    transform.setRotation(q);
    return transform;
  }

  static void transformToPose(
    const tf2::Transform & transform,
    geometry_msgs::msg::Pose & pose)
  {
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    pose.orientation.w = transform.getRotation().w();
  }

  void handleRawOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!pending_odometry_.empty() || !tryProcessRawOdometry(msg)) {
      enqueuePendingOdometry(msg);
    }
  }

  void enqueuePendingOdometry(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    if (pending_odometry_.size() >= tf_queue_max_size_) {
      pending_odometry_.pop_front();
      if (twist_estimator_) {
        twist_estimator_->reset();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF wait queue reached %zu messages; dropped the oldest raw odometry sample.",
        tf_queue_max_size_);
    }
    pending_odometry_.push_back({msg, std::chrono::steady_clock::now()});
  }

  void retryPendingOdometry()
  {
    while (!pending_odometry_.empty()) {
      const auto & pending = pending_odometry_.front();
      const double wait_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - pending.queued_at).count();
      if (wait_sec > tf_queue_max_wait_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropped raw odometry after waiting %.3f s for timestamped TF.",
          wait_sec);
        pending_odometry_.pop_front();
        if (twist_estimator_) {
          twist_estimator_->reset();
        }
        continue;
      }

      if (!tryProcessRawOdometry(pending.message)) {
        break;
      }
      pending_odometry_.pop_front();
    }
  }

  bool tryProcessRawOdometry(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    // Important: do not fake base_link by only changing child_frame_id.
    // For gimbal-mounted MID360, a backend may publish odometry for
    // lio_imu_link or mid360_*_frame. If the raw odom parent is already the
    // canonical base-initial odom frame, compute:
    //   T_odom_base = T_odom_sensor * inverse(T_base_sensor)
    // Some FAST-LIO backends instead use the initial sensor frame as the raw
    // parent. For those, change both the parent and child basis:
    //   T_base0_base = T_base_sensor * T_sensor0_sensor * inverse(T_base_sensor)
    // where T_base_sensor comes from robot_state_publisher and the current
    // gimbal_yaw_joint state.
    const auto & pose = msg->pose.pose;
    const double quaternion_norm_squared =
      pose.orientation.x * pose.orientation.x +
      pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z +
      pose.orientation.w * pose.orientation.w;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w) || quaternion_norm_squared < 1.0e-12)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting raw odometry with a non-finite or zero-norm pose.");
      return true;
    }

    if (!expected_input_odom_frame_.empty() &&
      msg->header.frame_id != expected_input_odom_frame_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting raw odometry parent frame '%s'; expected '%s'.",
        msg->header.frame_id.c_str(), expected_input_odom_frame_.c_str());
      return true;
    }

    std::string input_child =
      msg->child_frame_id.empty() ? input_sensor_frame_ : msg->child_frame_id;

    if (backend_child_frame_alias_enabled_ &&
      input_child == backend_child_frame_alias_source_)
    {
      if (backend_child_frame_alias_target_ == base_frame_) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Refusing backend child-frame alias '%s' directly to canonical '%s'.",
          backend_child_frame_alias_source_.c_str(), base_frame_.c_str());
        return true;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Treating backend-private child frame '%s' as semantic sensor frame '%s'.",
        backend_child_frame_alias_source_.c_str(),
        backend_child_frame_alias_target_.c_str());
      input_child = backend_child_frame_alias_target_;
    }

    tf2::Transform odom_to_input = poseToTransform(msg->pose.pose);
    tf2::Transform odom_to_base = odom_to_input;
    std::optional<HeadingSample> matched_heading;
    double derived_gimbal_yaw_rad = 0.0;

    if (pose_conversion_mode_ == "chassis_heading_fusion") {
      if (heading_reset_latched_) {
        return true;
      }
      if (input_child == base_frame_) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "chassis_heading_fusion requires raw odometry for a sensor frame, not base_link.");
        return true;
      }
      if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for non-zero raw odometry timestamps in chassis_heading_fusion mode.");
        return true;
      }
      const auto stamp_nanoseconds = rclcpp::Time(msg->header.stamp).nanoseconds();
      matched_heading = nearestHeading(stamp_nanoseconds);
      if (!matched_heading.has_value()) {
        return false;
      }
      if (!heading_fusion_initialized_) {
        raw_initial_to_sensor_at_fusion_start_ = odom_to_input;
        initial_chassis_heading_rad_ = matched_heading->yaw_rad;
        fusion_source_boot_id_ = matched_heading->source_boot_id;
        fusion_reset_counter_ = matched_heading->reset_counter;
        heading_fusion_initialized_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Initialized chassis-heading fusion at heading %.6f rad. This zero is valid only "
          "because the selected profile asserts that the gimbal is at its known home angle.",
          initial_chassis_heading_rad_);
      }
      if (matched_heading->source_boot_id != fusion_source_boot_id_ ||
        matched_heading->reset_counter != fusion_reset_counter_)
      {
        heading_reset_latched_ = true;
        publishInvalidDerivedGimbal("heading identity changed during fusion");
        return true;
      }
      const auto fused =
        rm_localization_adapters::compute_base_transform_from_chassis_heading(
        raw_initial_to_sensor_at_fusion_start_, odom_to_input, initial_base_to_sensor_,
        initial_chassis_heading_rad_, matched_heading->yaw_rad, initial_gimbal_yaw_rad_);
      odom_to_base = fused.base_initial_to_base;
      derived_gimbal_yaw_rad = fused.gimbal_yaw_rad;
    } else if (input_child != base_frame_) {
      if (use_tf_sensor_to_base_) {
        try {
          geometry_msgs::msg::TransformStamped base_to_input_msg;
          const bool has_zero_stamp =
            msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0;
          if (use_latest_transform_ || has_zero_stamp) {
            base_to_input_msg = tf_buffer_->lookupTransform(
              base_frame_,
              input_child,
              tf2::TimePointZero,
              tf2::durationFromSec(tf_lookup_timeout_sec_));
          } else {
            base_to_input_msg = tf_buffer_->lookupTransform(
              base_frame_,
              input_child,
              rclcpp::Time(msg->header.stamp),
              rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
          }
          const tf2::Transform base_to_input =
            transformMsgToTransform(base_to_input_msg.transform);
          odom_to_base = computeCanonicalBaseTransform(odom_to_input, base_to_input);
        } catch (const tf2::TransformException & ex) {
          if (!allow_placeholder_fallback_) {
            RCLCPP_DEBUG_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Waiting for %s -> %s at raw odometry timestamp: %s",
              base_frame_.c_str(), input_child.c_str(), ex.what());
            return false;
          }

          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "TF lookup failed for %s -> %s. Falling back to placeholder transform.",
            base_frame_.c_str(), input_child.c_str());
          odom_to_base = computeCanonicalBaseTransform(
            odom_to_input, input_to_base_placeholder_.inverse());
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "use_tf_sensor_to_base=false. Falling back to placeholder transform for %s.",
          input_child.c_str());
        odom_to_base = computeCanonicalBaseTransform(
          odom_to_input, input_to_base_placeholder_.inverse());
      }
    }

    nav_msgs::msg::Odometry output = *msg;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    transformToPose(odom_to_base, output.pose.pose);
    if (pose_conversion_mode_ == "chassis_heading_fusion") {
      output.pose.covariance[35] = heading_yaw_variance_;
    }

    if (twist_mode_ == "finite_difference") {
      if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot estimate canonical twist from a zero odometry timestamp.");
        return true;
      }

      const auto estimate = twist_estimator_->update(
        odom_to_base, rclcpp::Time(msg->header.stamp).nanoseconds());
      if (estimate.status != rm_localization_adapters::TwistEstimateStatus::kValid) {
        const char * reason = "initializing";
        if (estimate.status == rm_localization_adapters::TwistEstimateStatus::kInvalidTime) {
          reason = "invalid timestamp interval";
        } else if (estimate.status == rm_localization_adapters::TwistEstimateStatus::kOutlier) {
          reason = "velocity outlier";
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping canonical odometry while twist estimator is %s (dt=%.6f s).",
          reason, estimate.dt_sec);
        return true;
      }

      output.twist.twist = estimate.twist;
      output.twist.covariance.fill(0.0);
      constexpr std::array<std::size_t, 6> diagonal_indices{0, 7, 14, 21, 28, 35};
      for (std::size_t index = 0; index < diagonal_indices.size(); ++index) {
        output.twist.covariance[diagonal_indices[index]] = twist_variance_diagonal_[index];
      }
    }
    odom_pub_->publish(output);
    if (matched_heading.has_value()) {
      publishDerivedGimbal(
        derived_gimbal_yaw_rad, output.header.stamp, *matched_heading);
    }

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = output.header.stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = output.pose.pose.position.x;
      tf_msg.transform.translation.y = output.pose.pose.position.y;
      tf_msg.transform.translation.z = output.pose.pose.position.z;
      tf_msg.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
    return true;
  }

  tf2::Transform computeCanonicalBaseTransform(
    const tf2::Transform & raw_parent_to_sensor,
    const tf2::Transform & base_to_sensor) const
  {
    if (raw_odom_parent_frame_mode_ == "sensor_initial") {
      return rm_localization_adapters::compute_base_transform_from_sensor_initial(
        raw_parent_to_sensor, base_to_sensor);
    }
    return rm_localization_adapters::compute_base_transform(
      raw_parent_to_sensor, base_to_sensor);
  }

  std::string raw_odom_topic_;
  std::string output_odom_topic_;
  std::string expected_input_odom_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string input_sensor_frame_;
  std::string gimbal_frame_;
  bool publish_tf_;
  bool use_tf_sensor_to_base_;
  bool use_latest_transform_;
  bool allow_placeholder_fallback_;
  std::string pose_conversion_mode_;
  std::string raw_odom_parent_frame_mode_;
  double tf_lookup_timeout_sec_;
  std::size_t tf_queue_max_size_;
  double tf_queue_max_wait_sec_;
  double tf_queue_retry_rate_hz_;
  bool backend_child_frame_alias_enabled_;
  std::string backend_child_frame_alias_source_;
  std::string backend_child_frame_alias_target_;
  std::string twist_mode_;
  std::string heading_topic_;
  std::string derived_gimbal_topic_;
  std::size_t heading_cache_max_size_;
  double max_heading_match_dt_sec_;
  double heading_yaw_variance_;
  double initial_gimbal_yaw_rad_;
  double max_center_offset_xy_m_;
  std::array<double, 6> twist_variance_diagonal_{};
  tf2::Transform input_to_base_placeholder_;
  tf2::Transform initial_base_to_sensor_;
  tf2::Transform raw_initial_to_sensor_at_fusion_start_;
  std::unique_ptr<rm_localization_adapters::PoseTwistEstimator> twist_estimator_;

  std::deque<HeadingSample> heading_cache_;
  bool heading_stream_valid_ = false;
  bool heading_fusion_initialized_ = false;
  bool heading_reset_latched_ = false;
  bool have_last_heading_identity_ = false;
  bool derived_gimbal_invalid_published_ = false;
  bool have_previous_derived_gimbal_ = false;
  std::uint32_t last_heading_boot_id_ = 0;
  std::uint32_t last_heading_sequence_ = 0;
  std::uint32_t fusion_source_boot_id_ = 0;
  std::uint32_t derived_gimbal_sequence_ = 0;
  std::uint16_t last_heading_reset_counter_ = 0;
  std::uint16_t fusion_reset_counter_ = 0;
  std::int64_t previous_derived_gimbal_stamp_nanoseconds_ = 0;
  double initial_chassis_heading_rad_ = 0.0;
  double previous_derived_gimbal_yaw_rad_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::ChassisHeadingState>::SharedPtr
    heading_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::GimbalState>::SharedPtr
    derived_gimbal_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<PendingOdometry> pending_odometry_;
  rclcpp::TimerBase::SharedPtr tf_retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioAdapter>());
  rclcpp::shutdown();
  return 0;
}
