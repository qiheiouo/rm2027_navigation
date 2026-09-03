#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_localization_adapters/canonical_odometry.hpp"
#include "rm_localization_adapters/odometry_input_health.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
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

    rm_localization_adapters::OdometryInputHealthConfig input_health_config;
    input_health_config.enabled = declare_parameter<bool>(
      "input_health.enabled", false);
    input_health_enabled_ = input_health_config.enabled;
    input_health_config.max_message_age_sec = declare_parameter<double>(
      "input_health.max_message_age_sec", 0.2);
    input_health_config.max_future_offset_sec = declare_parameter<double>(
      "input_health.max_future_offset_sec", 0.05);
    const int recovery_fresh_samples = declare_parameter<int>(
      "input_health.recovery_fresh_samples", 5);
    if (recovery_fresh_samples <= 0) {
      throw std::invalid_argument("input_health.recovery_fresh_samples must be positive");
    }
    input_health_config.recovery_fresh_samples =
      static_cast<std::uint32_t>(recovery_fresh_samples);
    input_health_config.recovery_max_translation_m = declare_parameter<double>(
      "input_health.recovery_max_translation_m", 0.75);
    input_health_max_silence_sec_ = declare_parameter<double>(
      "input_health.max_silence_sec", 0.2);
    if (!std::isfinite(input_health_max_silence_sec_) ||
      input_health_max_silence_sec_ <= 0.0)
    {
      throw std::invalid_argument("input_health.max_silence_sec must be positive");
    }
    input_health_gate_ =
      std::make_unique<rm_localization_adapters::OdometryInputHealthGate>(
      input_health_config);
    input_health_valid_topic_ = declare_parameter<std::string>(
      "input_health.valid_topic", "/localization/lio_runtime_valid");
    input_health_status_topic_ = declare_parameter<std::string>(
      "input_health.status_topic", "/localization/lio_runtime_status");
    backend_calc_time_topic_ = declare_parameter<std::string>(
      "input_health.backend_calc_time_topic", "/lio/diagnostics/calc_time");
    backend_point_count_topic_ = declare_parameter<std::string>(
      "input_health.backend_point_count_topic", "/lio/diagnostics/point_number");

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
    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    input_health_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      input_health_valid_topic_, latched_qos);
    input_health_status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      input_health_status_topic_, rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odom_topic_, 10,
      std::bind(&LioAdapter::handleRawOdometry, this, std::placeholders::_1));
    if (!backend_calc_time_topic_.empty()) {
      backend_calc_time_sub_ = create_subscription<std_msgs::msg::Float32>(
        backend_calc_time_topic_, 10,
        [this](const std_msgs::msg::Float32::SharedPtr message) {
          last_backend_calc_time_ms_ = message->data;
          has_backend_calc_time_ = true;
        });
    }
    if (!backend_point_count_topic_.empty()) {
      backend_point_count_sub_ = create_subscription<std_msgs::msg::Float32>(
        backend_point_count_topic_, 10,
        [this](const std_msgs::msg::Float32::SharedPtr message) {
          last_backend_point_count_ = message->data;
          has_backend_point_count_ = true;
        });
    }
    const auto retry_period = std::chrono::duration<double>(1.0 / tf_queue_retry_rate_hz_);
    tf_retry_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(retry_period),
      std::bind(&LioAdapter::retryPendingOdometry, this));
    input_health_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LioAdapter::publishInputHealth, this));

    RCLCPP_WARN(
      get_logger(),
      "Canonical LIO adapter: adapting %s to %s through TF %s->sensor with gimbal frame %s. "
      "raw_odom_parent_frame_mode=%s. twist_mode=%s. Real hardware still requires calibrated "
      "and timestamped gimbal yaw. input_health=%s.",
      raw_odom_topic_.c_str(), output_odom_topic_.c_str(),
      base_frame_.c_str(), gimbal_frame_.c_str(), raw_odom_parent_frame_mode_.c_str(),
      twist_mode_.c_str(), input_health_enabled_ ? "enabled" : "disabled");
  }

private:
  struct PendingOdometry
  {
    nav_msgs::msg::Odometry::SharedPtr message;
    std::chrono::steady_clock::time_point queued_at;
  };

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
    received_raw_odometry_ = true;
    last_raw_receipt_ = std::chrono::steady_clock::now();
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

    if (input_child != base_frame_) {
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

    const auto health_decision = input_health_gate_->evaluate(
      get_clock()->now().nanoseconds(),
      rclcpp::Time(msg->header.stamp).nanoseconds(),
      output.pose.pose.position.x,
      output.pose.pose.position.y,
      output.pose.pose.position.z);
    if (health_decision != rm_localization_adapters::OdometryInputDecision::kAccept) {
      canonical_output_valid_ = false;
      if (twist_estimator_) {
        twist_estimator_->reset();
      }
      if (health_decision ==
        rm_localization_adapters::OdometryInputDecision::kRejectStale)
      {
        ++stale_rejections_;
        input_health_reason_ = "stale_raw_odometry";
      } else if (health_decision ==
        rm_localization_adapters::OdometryInputDecision::kRejectFuture)
      {
        ++future_rejections_;
        input_health_reason_ = "future_raw_odometry";
      } else if (health_decision ==
        rm_localization_adapters::OdometryInputDecision::kRejectAnchor)
      {
        ++anchor_rejections_;
        input_health_reason_ = "post_backlog_translation_mismatch";
      } else {
        input_health_reason_ = "waiting_for_fresh_consensus";
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Fail-closed raw LIO input: %s (age=%.3f s, recovery=%u).",
        input_health_reason_.c_str(), input_health_gate_->last_age_sec(),
        input_health_gate_->recovery_count());
      return true;
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
    canonical_output_valid_ = true;
    input_health_reason_ = "healthy";
    ++canonical_outputs_;

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

  void publishInputHealth()
  {
    double silence_sec = 0.0;
    if (received_raw_odometry_) {
      silence_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_raw_receipt_).count();
      if (input_health_enabled_ &&
        silence_sec > input_health_max_silence_sec_ &&
        !input_health_gate_->latched())
      {
        input_health_gate_->latch_silence();
        canonical_output_valid_ = false;
        input_health_reason_ = "raw_odometry_silence";
        ++silence_events_;
        if (twist_estimator_) {
          twist_estimator_->reset();
        }
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Fail-closed raw LIO input after %.3f s without odometry.", silence_sec);
      }
    }

    std_msgs::msg::Bool valid;
    valid.data = canonical_output_valid_ && !input_health_gate_->latched();
    input_health_valid_pub_->publish(valid);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lio_adapter/input_health";
    status.hardware_id = "fast_lio_multi";
    if (input_health_gate_->latched()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (!canonical_output_valid_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }
    status.message = input_health_reason_;
    const auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
      };
    add("enabled", input_health_enabled_ ? "true" : "false");
    add("canonical_output_valid", valid.data ? "true" : "false");
    add("latched", input_health_gate_->latched() ? "true" : "false");
    add("raw_odometry_received", received_raw_odometry_ ? "true" : "false");
    add("raw_silence_sec", std::to_string(silence_sec));
    add("last_message_age_sec", std::to_string(input_health_gate_->last_age_sec()));
    add("recovery_fresh_count", std::to_string(input_health_gate_->recovery_count()));
    add("stale_rejections", std::to_string(stale_rejections_));
    add("future_rejections", std::to_string(future_rejections_));
    add("anchor_rejections", std::to_string(anchor_rejections_));
    add("silence_events", std::to_string(silence_events_));
    add("canonical_outputs", std::to_string(canonical_outputs_));
    add(
      "backend_calc_time_ms",
      has_backend_calc_time_ ? std::to_string(last_backend_calc_time_ms_) : "unavailable");
    add(
      "backend_point_count",
      has_backend_point_count_ ? std::to_string(last_backend_point_count_) : "unavailable");
    array.status.push_back(status);
    input_health_status_pub_->publish(array);
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
  std::string raw_odom_parent_frame_mode_;
  double tf_lookup_timeout_sec_;
  std::size_t tf_queue_max_size_;
  double tf_queue_max_wait_sec_;
  double tf_queue_retry_rate_hz_;
  bool backend_child_frame_alias_enabled_;
  std::string backend_child_frame_alias_source_;
  std::string backend_child_frame_alias_target_;
  std::string twist_mode_;
  std::array<double, 6> twist_variance_diagonal_{};
  tf2::Transform input_to_base_placeholder_;
  std::unique_ptr<rm_localization_adapters::PoseTwistEstimator> twist_estimator_;
  bool input_health_enabled_{false};
  double input_health_max_silence_sec_{0.2};
  std::string input_health_valid_topic_;
  std::string input_health_status_topic_;
  std::string backend_calc_time_topic_;
  std::string backend_point_count_topic_;
  std::unique_ptr<rm_localization_adapters::OdometryInputHealthGate> input_health_gate_;
  bool received_raw_odometry_{false};
  bool canonical_output_valid_{false};
  std::chrono::steady_clock::time_point last_raw_receipt_{};
  std::string input_health_reason_{"initializing"};
  std::uint64_t stale_rejections_{0U};
  std::uint64_t future_rejections_{0U};
  std::uint64_t anchor_rejections_{0U};
  std::uint64_t silence_events_{0U};
  std::uint64_t canonical_outputs_{0U};
  float last_backend_calc_time_ms_{0.0F};
  float last_backend_point_count_{0.0F};
  bool has_backend_calc_time_{false};
  bool has_backend_point_count_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr input_health_valid_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    input_health_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr backend_calc_time_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr backend_point_count_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<PendingOdometry> pending_odometry_;
  rclcpp::TimerBase::SharedPtr tf_retry_timer_;
  rclcpp::TimerBase::SharedPtr input_health_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioAdapter>());
  rclcpp::shutdown();
  return 0;
}
