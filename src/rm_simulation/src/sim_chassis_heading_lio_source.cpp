// Copyright 2026 RM Navigation

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/chassis_heading_state.hpp"
#include "rm_competition_interfaces/msg/gimbal_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  if (rotation.length2() < 1.0e-12) {
    throw std::invalid_argument("ground-truth odometry has a zero-norm orientation");
  }
  rotation.normalize();
  return tf2::Transform(
    rotation,
    tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

void transformToPose(const tf2::Transform & transform, geometry_msgs::msg::Pose & pose)
{
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  pose.orientation.x = transform.getRotation().x();
  pose.orientation.y = transform.getRotation().y();
  pose.orientation.z = transform.getRotation().z();
  pose.orientation.w = transform.getRotation().w();
}

double yawOf(const tf2::Quaternion & rotation)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
  return yaw;
}

std::string formatDouble(double value, int precision = 9)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

class SimChassisHeadingLioSource : public rclcpp::Node
{
public:
  SimChassisHeadingLioSource()
  : Node("sim_chassis_heading_lio_source")
  {
    ground_truth_topic_ = declare_parameter<std::string>(
      "ground_truth_topic", "/simulation/ground_truth/odom");
    raw_lio_topic_ = declare_parameter<std::string>(
      "raw_lio_topic", "/odometry/fast_lio_raw");
    chassis_heading_topic_ = declare_parameter<std::string>(
      "chassis_heading_topic", "/chassis/heading");
    reference_topic_ = declare_parameter<std::string>(
      "reference_topic", "/simulation/localization/reference_base_odom");
    gimbal_truth_topic_ = declare_parameter<std::string>(
      "gimbal_truth_topic", "/simulation/gimbal/state_truth");
    gimbal_command_topic_ = declare_parameter<std::string>(
      "gimbal_command_topic", "/simulation/gimbal_yaw/command");
    fused_odom_topic_ = declare_parameter<std::string>(
      "fused_odom_topic", "/odometry/lio");
    derived_gimbal_topic_ = declare_parameter<std::string>(
      "derived_gimbal_topic", "/gimbal/state_derived");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/diagnostics");
    raw_parent_frame_ = declare_parameter<std::string>("raw_parent_frame", "odom");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "sim_lidar_link");
    publish_sensor_truth_tf_ = declare_parameter<bool>(
      "publish_sensor_truth_tf", false);
    sensor_truth_parent_frame_ = declare_parameter<std::string>(
      "sensor_truth_parent_frame", "map");
    sensor_truth_frame_ = declare_parameter<std::string>(
      "sensor_truth_frame", "sim_lidar_physics_frame");

    motion_mode_ = declare_parameter<std::string>("motion_mode", "fixed");
    gimbal_offset_rad_ = declare_parameter<double>("gimbal_offset_rad", 0.0);
    gimbal_amplitude_rad_ = declare_parameter<double>("gimbal_amplitude_rad", 0.0);
    gimbal_frequency_hz_ = declare_parameter<double>("gimbal_frequency_hz", 0.0);
    gimbal_angular_velocity_rad_s_ =
      declare_parameter<double>("gimbal_angular_velocity_rad_s", 0.0);

    gimbal_center_ = tf2::Vector3(
      declare_parameter<double>("gimbal_center_in_base.x", 0.0),
      declare_parameter<double>("gimbal_center_in_base.y", 0.0),
      declare_parameter<double>("gimbal_center_in_base.z", 0.115));
    sensor_offset_ = tf2::Vector3(
      declare_parameter<double>("sensor_offset_from_gimbal.x", 0.12),
      declare_parameter<double>("sensor_offset_from_gimbal.y", 0.0),
      declare_parameter<double>("sensor_offset_from_gimbal.z", 0.065));

    heading_world_offset_rad_ =
      declare_parameter<double>("heading_world_offset_rad", 0.0);
    heading_yaw_sign_ = declare_parameter<double>("heading_yaw_sign", 1.0);
    heading_timestamp_offset_sec_ =
      declare_parameter<double>("heading_timestamp_offset_sec", 0.0);
    heading_publish_divider_ = static_cast<std::size_t>(
      declare_parameter<int64_t>("heading_publish_divider", 1));
    source_boot_id_ = static_cast<std::uint32_t>(
      declare_parameter<int64_t>("source_boot_id", 2027));
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.002);
    yaw_tolerance_rad_ = declare_parameter<double>("yaw_tolerance_rad", 0.002);
    gimbal_tolerance_rad_ = declare_parameter<double>("gimbal_tolerance_rad", 0.002);
    truth_cache_size_ = static_cast<std::size_t>(
      declare_parameter<int64_t>("truth_cache_size", 1000));

    validateParameters();

    if (publish_sensor_truth_tf_) {
      sensor_truth_tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    raw_lio_pub_ = create_publisher<nav_msgs::msg::Odometry>(raw_lio_topic_, 20);
    heading_pub_ = create_publisher<rm_competition_interfaces::msg::ChassisHeadingState>(
      chassis_heading_topic_, 20);
    reference_pub_ = create_publisher<nav_msgs::msg::Odometry>(reference_topic_, 20);
    gimbal_truth_pub_ = create_publisher<rm_competition_interfaces::msg::GimbalState>(
      gimbal_truth_topic_, 20);
    gimbal_command_pub_ = create_publisher<std_msgs::msg::Float64>(gimbal_command_topic_, 20);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);

    ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_topic_, 20,
      std::bind(&SimChassisHeadingLioSource::handleGroundTruth, this, std::placeholders::_1));
    fused_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      fused_odom_topic_, 20,
      std::bind(&SimChassisHeadingLioSource::handleFusedOdometry, this, std::placeholders::_1));
    derived_gimbal_sub_ =
      create_subscription<rm_competition_interfaces::msg::GimbalState>(
      derived_gimbal_topic_, 20,
      std::bind(&SimChassisHeadingLioSource::handleDerivedGimbal, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SimChassisHeadingLioSource::publishDiagnostics, this));

    RCLCPP_WARN(
      get_logger(),
      "Simulation-only chassis-heading LIO source: ground truth -> %s[%s] + %s. "
      "heading world offset=%.3f rad, timestamp offset=%.3f s, yaw sign=%.1f. "
      "No canonical localization TF is published by this node.",
      raw_lio_topic_.c_str(), sensor_frame_.c_str(), chassis_heading_topic_.c_str(),
      heading_world_offset_rad_, heading_timestamp_offset_sec_, heading_yaw_sign_);
    if (publish_sensor_truth_tf_) {
      RCLCPP_WARN(
        get_logger(),
        "Simulation-only scan attitude feed is ACTIVE: %s -> %s uses Gazebo sensor truth. "
        "It must not be enabled by real-hardware launches.",
        sensor_truth_parent_frame_.c_str(), sensor_truth_frame_.c_str());
    }
  }

  ~SimChassisHeadingLioSource() override
  {
    RCLCPP_INFO(
      get_logger(),
      "Fusion truth summary: odom_samples=%zu gimbal_samples=%zu max_position_error=%.9f m "
      "max_yaw_error=%.9f rad max_gimbal_error=%.9f rad.",
      fused_sample_count_, derived_gimbal_sample_count_, max_position_error_m_,
      max_yaw_error_rad_, max_gimbal_error_rad_);
  }

private:
  struct TruthSample
  {
    std::int64_t stamp_nanoseconds{0};
    tf2::Transform base_initial_to_base;
    double gimbal_yaw_rad{0.0};
  };

  void validateParameters() const
  {
    const bool valid_mode =
      motion_mode_ == "fixed" || motion_mode_ == "sine" || motion_mode_ == "continuous";
    const bool finite =
      std::isfinite(gimbal_offset_rad_) && std::isfinite(gimbal_amplitude_rad_) &&
      std::isfinite(gimbal_frequency_hz_) &&
      std::isfinite(gimbal_angular_velocity_rad_s_) &&
      std::isfinite(heading_world_offset_rad_) && std::isfinite(heading_yaw_sign_) &&
      std::isfinite(heading_timestamp_offset_sec_) &&
      std::isfinite(position_tolerance_m_) && std::isfinite(yaw_tolerance_rad_) &&
      std::isfinite(gimbal_tolerance_rad_);
    const bool invalid_sensor_truth_frames =
      publish_sensor_truth_tf_ &&
      (sensor_truth_parent_frame_.empty() || sensor_truth_frame_.empty() ||
      sensor_truth_parent_frame_ == sensor_truth_frame_);
    if (!valid_mode || !finite || gimbal_frequency_hz_ < 0.0 ||
      std::abs(std::abs(heading_yaw_sign_) - 1.0) > 1.0e-9 ||
      std::abs(heading_timestamp_offset_sec_) > 1.0 ||
      heading_publish_divider_ < 1 || heading_publish_divider_ > 100 ||
      position_tolerance_m_ <= 0.0 || yaw_tolerance_rad_ <= 0.0 ||
      gimbal_tolerance_rad_ <= 0.0 || truth_cache_size_ < 10 ||
      raw_parent_frame_.empty() || sensor_frame_.empty() ||
      invalid_sensor_truth_frames)
    {
      throw std::invalid_argument("invalid simulated chassis-heading LIO parameters");
    }
  }

  std::pair<double, double> gimbalState(double elapsed_sec) const
  {
    if (motion_mode_ == "sine") {
      const double phase = kTwoPi * gimbal_frequency_hz_ * elapsed_sec;
      return {
        gimbal_offset_rad_ + gimbal_amplitude_rad_ * std::sin(phase),
        kTwoPi * gimbal_frequency_hz_ * gimbal_amplitude_rad_ * std::cos(phase)};
    }
    if (motion_mode_ == "continuous") {
      return {
        gimbal_offset_rad_ + gimbal_angular_velocity_rad_s_ * elapsed_sec,
        gimbal_angular_velocity_rad_s_};
    }
    return {gimbal_offset_rad_, 0.0};
  }

  tf2::Transform baseToSensor(double gimbal_yaw_rad) const
  {
    tf2::Quaternion yaw_rotation;
    yaw_rotation.setRPY(0.0, 0.0, gimbal_yaw_rad);
    yaw_rotation.normalize();
    const tf2::Vector3 base_to_sensor =
      gimbal_center_ + tf2::quatRotate(yaw_rotation, sensor_offset_);
    return tf2::Transform(yaw_rotation, base_to_sensor);
  }

  void handleGroundTruth(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() <= 0) {
      return;
    }

    tf2::Transform world_to_base;
    try {
      world_to_base = poseToTransform(message->pose.pose);
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
      return;
    }
    const double base_yaw = yawOf(world_to_base.getRotation());

    if (!initialized_ || stamp < initial_stamp_) {
      initialized_ = true;
      initial_stamp_ = stamp;
      initial_world_to_base_ = world_to_base;
      initial_base_yaw_rad_ = base_yaw;
      previous_base_yaw_rad_ = base_yaw;
      previous_base_stamp_ = stamp;
      truth_cache_.clear();
      resetMetrics();
    }

    const double elapsed_sec = std::max(0.0, (stamp - initial_stamp_).seconds());
    const auto [gimbal_yaw, gimbal_rate] = gimbalState(elapsed_sec);
    const tf2::Transform world_to_sensor = world_to_base * baseToSensor(gimbal_yaw);

    double base_yaw_rate = 0.0;
    const double base_dt = (stamp - previous_base_stamp_).seconds();
    if (base_dt > 1.0e-4 && base_dt < 0.5) {
      base_yaw_rate = wrapAngle(base_yaw - previous_base_yaw_rad_) / base_dt;
    }
    previous_base_yaw_rad_ = base_yaw;
    previous_base_stamp_ = stamp;

    const tf2::Transform base_initial_to_base =
      initial_world_to_base_.inverse() * world_to_base;
    tf2::Quaternion planar_base_rotation;
    planar_base_rotation.setRPY(0.0, 0.0, wrapAngle(base_yaw - initial_base_yaw_rad_));
    planar_base_rotation.normalize();
    const tf2::Transform planar_reference(
      planar_base_rotation, base_initial_to_base.getOrigin());

    TruthSample truth;
    truth.stamp_nanoseconds = stamp.nanoseconds();
    truth.base_initial_to_base = planar_reference;
    truth.gimbal_yaw_rad = wrapAngle(gimbal_yaw);
    truth_cache_.push_back(truth);
    while (truth_cache_.size() > truth_cache_size_) {
      truth_cache_.pop_front();
    }

    publishGimbalTruth(message->header.stamp, elapsed_sec, gimbal_yaw, gimbal_rate);
    if (ground_truth_sample_count_ % heading_publish_divider_ == 0) {
      publishChassisHeading(
        stamp, elapsed_sec, base_yaw, base_yaw_rate);
    }
    publishReference(*message, planar_reference);
    publishRawLio(*message, world_to_sensor);
    ++ground_truth_sample_count_;
  }

  void publishGimbalTruth(
    const builtin_interfaces::msg::Time & stamp, double elapsed_sec,
    double yaw_rad, double yaw_rate_rad_s)
  {
    rm_competition_interfaces::msg::GimbalState state;
    state.header.stamp = stamp;
    state.header.frame_id = "gimbal_yaw_link";
    state.relative_yaw_rad = yaw_rad;
    state.yaw_rate_rad_s = yaw_rate_rad_s;
    state.sample_sequence = gimbal_sequence_++;
    state.mcu_time_ms = static_cast<std::uint32_t>(
      std::fmod(std::max(0.0, elapsed_sec) * 1000.0, 4294967296.0));
    state.online = true;
    state.valid = true;
    gimbal_truth_pub_->publish(state);

    std_msgs::msg::Float64 command;
    command.data = yaw_rad;
    gimbal_command_pub_->publish(command);
  }

  void publishChassisHeading(
    const rclcpp::Time & source_stamp,
    double elapsed_sec, double base_yaw_rad, double base_yaw_rate_rad_s)
  {
    const rclcpp::Time heading_stamp =
      source_stamp + rclcpp::Duration::from_seconds(heading_timestamp_offset_sec_);
    if (heading_stamp.nanoseconds() <= 0) {
      return;
    }
    rm_competition_interfaces::msg::ChassisHeadingState heading;
    heading.header.stamp = heading_stamp;
    heading.header.frame_id = "sim_lower_controller_world";
    heading.yaw_rad = wrapAngle(
      heading_world_offset_rad_ + heading_yaw_sign_ *
      (base_yaw_rad + base_yaw_rate_rad_s * heading_timestamp_offset_sec_));
    heading.yaw_rate_rad_s = heading_yaw_sign_ * base_yaw_rate_rad_s;
    heading.sample_sequence = heading_sequence_++;
    heading.mcu_time_ms = static_cast<std::uint32_t>(
      std::fmod(
        std::max(0.0, elapsed_sec + heading_timestamp_offset_sec_) * 1000.0,
        4294967296.0));
    heading.reset_counter = 0;
    heading.source_boot_id = source_boot_id_;
    heading.online = true;
    heading.valid = true;
    heading_pub_->publish(heading);
  }

  void publishReference(
    const nav_msgs::msg::Odometry & input, const tf2::Transform & reference)
  {
    nav_msgs::msg::Odometry output = input;
    output.header.frame_id = "odom";
    output.child_frame_id = "base_link";
    transformToPose(reference, output.pose.pose);
    reference_pub_->publish(output);
  }

  void publishRawLio(
    const nav_msgs::msg::Odometry & input, const tf2::Transform & world_to_sensor)
  {
    nav_msgs::msg::Odometry output;
    output.header.stamp = input.header.stamp;
    output.header.frame_id = raw_parent_frame_;
    output.child_frame_id = sensor_frame_;
    output.pose.covariance = input.pose.covariance;
    transformToPose(world_to_sensor, output.pose.pose);
    raw_lio_pub_->publish(output);

    if (sensor_truth_tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = input.header.stamp;
      transform.header.frame_id = sensor_truth_parent_frame_;
      transform.child_frame_id = sensor_truth_frame_;
      transform.transform.translation.x = world_to_sensor.getOrigin().x();
      transform.transform.translation.y = world_to_sensor.getOrigin().y();
      transform.transform.translation.z = world_to_sensor.getOrigin().z();
      transform.transform.rotation.x = world_to_sensor.getRotation().x();
      transform.transform.rotation.y = world_to_sensor.getRotation().y();
      transform.transform.rotation.z = world_to_sensor.getRotation().z();
      transform.transform.rotation.w = world_to_sensor.getRotation().w();
      sensor_truth_tf_broadcaster_->sendTransform(transform);
    }
  }

  const TruthSample * findTruth(std::int64_t stamp_nanoseconds) const
  {
    for (auto iterator = truth_cache_.rbegin(); iterator != truth_cache_.rend(); ++iterator) {
      if (iterator->stamp_nanoseconds == stamp_nanoseconds) {
        return &*iterator;
      }
      if (iterator->stamp_nanoseconds < stamp_nanoseconds) {
        break;
      }
    }
    return nullptr;
  }

  void handleFusedOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const std::int64_t stamp_nanoseconds = rclcpp::Time(message->header.stamp).nanoseconds();
    const TruthSample * truth = findTruth(stamp_nanoseconds);
    if (truth == nullptr) {
      ++unmatched_fused_sample_count_;
      return;
    }
    const tf2::Transform fused = poseToTransform(message->pose.pose);
    const tf2::Vector3 position_delta =
      fused.getOrigin() - truth->base_initial_to_base.getOrigin();
    latest_position_error_m_ = position_delta.length();
    latest_yaw_error_rad_ = std::abs(wrapAngle(
      yawOf(fused.getRotation()) - yawOf(truth->base_initial_to_base.getRotation())));
    max_position_error_m_ = std::max(max_position_error_m_, latest_position_error_m_);
    max_yaw_error_rad_ = std::max(max_yaw_error_rad_, latest_yaw_error_rad_);
    ++fused_sample_count_;
  }

  void handleDerivedGimbal(
    const rm_competition_interfaces::msg::GimbalState::SharedPtr message)
  {
    if (!message->valid || !message->online) {
      return;
    }
    const std::int64_t stamp_nanoseconds = rclcpp::Time(message->header.stamp).nanoseconds();
    const TruthSample * truth = findTruth(stamp_nanoseconds);
    if (truth == nullptr) {
      ++unmatched_gimbal_sample_count_;
      return;
    }
    latest_gimbal_error_rad_ = std::abs(wrapAngle(
      message->relative_yaw_rad - truth->gimbal_yaw_rad));
    max_gimbal_error_rad_ = std::max(max_gimbal_error_rad_, latest_gimbal_error_rad_);
    ++derived_gimbal_sample_count_;
  }

  void resetMetrics()
  {
    fused_sample_count_ = 0;
    derived_gimbal_sample_count_ = 0;
    unmatched_fused_sample_count_ = 0;
    unmatched_gimbal_sample_count_ = 0;
    latest_position_error_m_ = std::numeric_limits<double>::quiet_NaN();
    latest_yaw_error_rad_ = std::numeric_limits<double>::quiet_NaN();
    latest_gimbal_error_rad_ = std::numeric_limits<double>::quiet_NaN();
    max_position_error_m_ = 0.0;
    max_yaw_error_rad_ = 0.0;
    max_gimbal_error_rad_ = 0.0;
    ground_truth_sample_count_ = 0;
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "simulation/chassis_heading_lio_fusion";
    status.hardware_id = "gazebo_truth_ab";

    const bool have_samples = fused_sample_count_ > 0 && derived_gimbal_sample_count_ > 0;
    const double expected_fused_samples = static_cast<double>(
      ground_truth_sample_count_ > 0 ? ground_truth_sample_count_ - 1 : 0);
    const double fusion_coverage = expected_fused_samples > 0.0 ?
      static_cast<double>(fused_sample_count_) / expected_fused_samples : 0.0;
    const bool within_tolerance =
      max_position_error_m_ <= position_tolerance_m_ &&
      max_yaw_error_rad_ <= yaw_tolerance_rad_ &&
      max_gimbal_error_rad_ <= gimbal_tolerance_rad_ &&
      fusion_coverage >= 0.95;
    if (!have_samples) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for matched fused odometry and derived gimbal samples";
    } else if (!within_tolerance) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "fusion error or insufficient synchronized-sample coverage";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "fusion matches simulation truth";
    }

    status.values.push_back(keyValue("fused_sample_count", std::to_string(fused_sample_count_)));
    status.values.push_back(keyValue(
      "ground_truth_sample_count", std::to_string(ground_truth_sample_count_)));
    status.values.push_back(keyValue(
      "fusion_coverage", formatDouble(fusion_coverage, 6)));
    status.values.push_back(keyValue(
      "derived_gimbal_sample_count", std::to_string(derived_gimbal_sample_count_)));
    status.values.push_back(keyValue(
      "unmatched_fused_sample_count", std::to_string(unmatched_fused_sample_count_)));
    status.values.push_back(keyValue(
      "unmatched_gimbal_sample_count", std::to_string(unmatched_gimbal_sample_count_)));
    status.values.push_back(keyValue(
      "latest_position_error_m", formatDouble(latest_position_error_m_)));
    status.values.push_back(keyValue(
      "max_position_error_m", formatDouble(max_position_error_m_)));
    status.values.push_back(keyValue(
      "latest_yaw_error_rad", formatDouble(latest_yaw_error_rad_)));
    status.values.push_back(keyValue("max_yaw_error_rad", formatDouble(max_yaw_error_rad_)));
    status.values.push_back(keyValue(
      "latest_gimbal_error_rad", formatDouble(latest_gimbal_error_rad_)));
    status.values.push_back(keyValue(
      "max_gimbal_error_rad", formatDouble(max_gimbal_error_rad_)));
    status.values.push_back(keyValue(
      "heading_world_offset_rad", formatDouble(heading_world_offset_rad_, 6)));
    status.values.push_back(keyValue(
      "heading_timestamp_offset_sec", formatDouble(heading_timestamp_offset_sec_, 6)));
    status.values.push_back(keyValue(
      "heading_publish_divider", std::to_string(heading_publish_divider_)));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);

    if (have_samples && !within_tolerance) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Fusion validation failed: coverage=%.3f max position=%.6f m yaw=%.6f rad "
        "gimbal=%.6f rad.",
        fusion_coverage, max_position_error_m_, max_yaw_error_rad_, max_gimbal_error_rad_);
    }
  }

  std::string ground_truth_topic_;
  std::string raw_lio_topic_;
  std::string chassis_heading_topic_;
  std::string reference_topic_;
  std::string gimbal_truth_topic_;
  std::string gimbal_command_topic_;
  std::string fused_odom_topic_;
  std::string derived_gimbal_topic_;
  std::string diagnostics_topic_;
  std::string raw_parent_frame_;
  std::string sensor_frame_;
  bool publish_sensor_truth_tf_{false};
  std::string sensor_truth_parent_frame_;
  std::string sensor_truth_frame_;
  std::string motion_mode_;
  double gimbal_offset_rad_{0.0};
  double gimbal_amplitude_rad_{0.0};
  double gimbal_frequency_hz_{0.0};
  double gimbal_angular_velocity_rad_s_{0.0};
  tf2::Vector3 gimbal_center_;
  tf2::Vector3 sensor_offset_;
  double heading_world_offset_rad_{0.0};
  double heading_yaw_sign_{1.0};
  double heading_timestamp_offset_sec_{0.0};
  std::size_t heading_publish_divider_{1};
  std::uint32_t source_boot_id_{2027};
  double position_tolerance_m_{0.002};
  double yaw_tolerance_rad_{0.002};
  double gimbal_tolerance_rad_{0.002};
  std::size_t truth_cache_size_{1000};

  bool initialized_{false};
  rclcpp::Time initial_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_base_stamp_{0, 0, RCL_ROS_TIME};
  tf2::Transform initial_world_to_base_;
  double initial_base_yaw_rad_{0.0};
  double previous_base_yaw_rad_{0.0};
  std::uint32_t heading_sequence_{0};
  std::uint32_t gimbal_sequence_{0};
  std::size_t ground_truth_sample_count_{0};
  std::deque<TruthSample> truth_cache_;

  std::size_t fused_sample_count_{0};
  std::size_t derived_gimbal_sample_count_{0};
  std::size_t unmatched_fused_sample_count_{0};
  std::size_t unmatched_gimbal_sample_count_{0};
  double latest_position_error_m_{std::numeric_limits<double>::quiet_NaN()};
  double latest_yaw_error_rad_{std::numeric_limits<double>::quiet_NaN()};
  double latest_gimbal_error_rad_{std::numeric_limits<double>::quiet_NaN()};
  double max_position_error_m_{0.0};
  double max_yaw_error_rad_{0.0};
  double max_gimbal_error_rad_{0.0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_lio_pub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::ChassisHeadingState>::SharedPtr heading_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reference_pub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::GimbalState>::SharedPtr gimbal_truth_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fused_odom_sub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::GimbalState>::SharedPtr
    derived_gimbal_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> sensor_truth_tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimChassisHeadingLioSource>());
  rclcpp::shutdown();
  return 0;
}
