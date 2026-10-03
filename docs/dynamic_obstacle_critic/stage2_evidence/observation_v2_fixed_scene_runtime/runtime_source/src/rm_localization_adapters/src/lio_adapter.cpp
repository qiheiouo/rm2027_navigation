#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
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
    const auto retry_period = std::chrono::duration<double>(1.0 / tf_queue_retry_rate_hz_);
    tf_retry_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(retry_period),
      std::bind(&LioAdapter::retryPendingOdometry, this));

    RCLCPP_WARN(
      get_logger(),
      "Canonical LIO adapter: adapting %s to %s through TF %s->sensor with gimbal frame %s. "
      "raw_odom_parent_frame_mode=%s. twist_mode=%s. Real hardware still requires calibrated "
      "and timestamped gimbal yaw.",
      raw_odom_topic_.c_str(), output_odom_topic_.c_str(),
      base_frame_.c_str(), gimbal_frame_.c_str(), raw_odom_parent_frame_mode_.c_str(),
      twist_mode_.c_str());
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
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
