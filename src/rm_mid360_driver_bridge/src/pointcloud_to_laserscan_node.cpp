#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rm_mid360_driver_bridge/pointcloud_deskew.hpp"

namespace
{

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const double norm_squared =
    pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && norm_squared > 1.0e-12;
}

std::string bool_text(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

class PointcloudToLaserscanNode : public rclcpp::Node
{
public:
  PointcloudToLaserscanNode()
  : Node("pointcloud_to_laserscan_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/livox/left/pointcloud_filtered");
    output_topic_ = declare_parameter<std::string>("output_topic", "/local_scan");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    constexpr double kPi = 3.14159265358979323846;
    projection_.angle_min = declare_parameter<double>("angle_min", -kPi);
    projection_.angle_max = declare_parameter<double>("angle_max", kPi);
    projection_.angle_increment = declare_parameter<double>("angle_increment", 0.01);
    projection_.scan_time = declare_parameter<double>("scan_time", 0.066);
    projection_.range_min = declare_parameter<double>("range_min", 0.45);
    projection_.range_max = declare_parameter<double>("range_max", 6.0);
    min_height_ = declare_parameter<double>("min_height", 0.18);
    max_height_ = declare_parameter<double>("max_height", 2.0);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);
    max_publish_rate_hz_ = declare_parameter<double>("max_publish_rate_hz", 0.0);

    deskew_enabled_ = declare_parameter<bool>("deskew.enabled", false);
    timestamp_field_ = declare_parameter<std::string>("deskew.timestamp_field", "timestamp");
    timestamp_interpretation_ = declare_parameter<std::string>(
      "deskew.timestamp_interpretation", "absolute_nanoseconds");
    reference_time_policy_ = declare_parameter<std::string>(
      "deskew.reference_time_policy", "header_stamp");
    failure_policy_ = declare_parameter<std::string>(
      "deskew.failure_policy", "drop_frame");
    odom_topic_ = declare_parameter<std::string>("deskew.odom_topic", "/odometry/lio");
    odom_frame_ = declare_parameter<std::string>("deskew.odom_frame", "odom");
    odom_child_frame_ = declare_parameter<std::string>(
      "deskew.odom_child_frame", "base_link");
    odom_buffer_duration_sec_ = declare_parameter<double>(
      "deskew.odom_buffer_duration_sec", 3.0);
    max_interpolation_gap_sec_ = declare_parameter<double>(
      "deskew.max_interpolation_gap_sec", 0.10);
    max_cloud_span_sec_ = declare_parameter<double>("deskew.max_cloud_span_sec", 0.10);
    point_time_before_header_tolerance_sec_ = declare_parameter<double>(
      "deskew.point_time_before_header_tolerance_sec", 0.001);
    max_point_time_after_header_sec_ = declare_parameter<double>(
      "deskew.max_point_time_after_header_sec", 0.10);
    pending_timeout_sec_ = declare_parameter<double>("deskew.pending_timeout_sec", 0.20);
    const int pending_queue_size = declare_parameter<int>("deskew.pending_queue_size", 5);
    status_topic_ = declare_parameter<std::string>(
      "deskew.status_topic", "/localization/scan_deskew/status");
    active_topic_ = declare_parameter<std::string>(
      "deskew.active_topic", "/localization/scan_deskew/active");

    validate_parameters(pending_queue_size);
    pending_queue_size_ = static_cast<std::size_t>(pending_queue_size);
    bin_count_ = static_cast<std::size_t>(
      std::floor(
        (projection_.angle_max - projection_.angle_min) /
        projection_.angle_increment) + 1.0);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudToLaserscanNode::cloud_callback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      status_topic_, rclcpp::QoS(10).reliable());
    active_pub_ = create_publisher<std_msgs::msg::Bool>(
      active_topic_, rclcpp::QoS(1).reliable().transient_local());

    if (deskew_enabled_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS().keep_last(512),
        std::bind(&PointcloudToLaserscanNode::odom_callback, this, std::placeholders::_1));
      pending_timer_ = create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&PointcloudToLaserscanNode::process_pending, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "pointcloud to laserscan: %s -> %s, target_frame=%s, range=[%.2f, %.2f], "
      "height=[%.2f, %.2f], max_rate=%.2f Hz, deskew=%s",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
      projection_.range_min, projection_.range_max, min_height_, max_height_,
      max_publish_rate_hz_, deskew_enabled_ ? "strict_se3" : "disabled");
    if (deskew_enabled_) {
      RCLCPP_WARN(
        get_logger(),
        "EXPERIMENTAL strict SE(3) deskew enabled: field=%s, interpretation=%s, "
        "reference=%s, odom=%s (%s->%s). Invalid frames are dropped.",
        timestamp_field_.c_str(), timestamp_interpretation_.c_str(),
        reference_time_policy_.c_str(), odom_topic_.c_str(), odom_frame_.c_str(),
        odom_child_frame_.c_str());
    }
    publish_status(
      deskew_enabled_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      deskew_enabled_ ? "waiting for first valid deskewed frame" :
      "deskew disabled; baseline projection active",
      false);
  }

private:
  struct PendingCloud
  {
    sensor_msgs::msg::PointCloud2::SharedPtr message;
    std::vector<double> point_times_sec;
    double min_point_time_sec{0.0};
    double max_point_time_sec{0.0};
    std::chrono::steady_clock::time_point queued_at;
  };

  void validate_parameters(int pending_queue_size) const
  {
    if (!std::isfinite(projection_.angle_min) || !std::isfinite(projection_.angle_max) ||
      !std::isfinite(projection_.angle_increment) || projection_.angle_increment <= 0.0 ||
      projection_.angle_max <= projection_.angle_min ||
      !std::isfinite(projection_.range_min) || !std::isfinite(projection_.range_max) ||
      projection_.range_max <= projection_.range_min || !std::isfinite(min_height_) ||
      !std::isfinite(max_height_) || max_height_ < min_height_ ||
      !std::isfinite(max_publish_rate_hz_) || max_publish_rate_hz_ < 0.0)
    {
      throw std::invalid_argument("invalid pointcloud_to_laserscan projection parameters");
    }
    if (!deskew_enabled_) {
      return;
    }
    if (target_frame_.empty() || timestamp_field_.empty() || odom_topic_.empty() ||
      odom_frame_.empty() || odom_child_frame_.empty())
    {
      throw std::invalid_argument("deskew frame/topic/field parameters must not be empty");
    }
    if (target_frame_ != odom_child_frame_) {
      throw std::invalid_argument(
              "strict deskew requires target_frame to equal deskew.odom_child_frame");
    }
    if (timestamp_interpretation_ != "absolute_nanoseconds") {
      throw std::invalid_argument(
              "only verified Livox absolute_nanoseconds timestamps are supported");
    }
    if (reference_time_policy_ != "header_stamp" || failure_policy_ != "drop_frame") {
      throw std::invalid_argument(
              "experimental deskew requires header_stamp reference and drop_frame policy");
    }
    if (!std::isfinite(odom_buffer_duration_sec_) || odom_buffer_duration_sec_ <= 0.0 ||
      !std::isfinite(max_interpolation_gap_sec_) || max_interpolation_gap_sec_ <= 0.0 ||
      !std::isfinite(max_cloud_span_sec_) || max_cloud_span_sec_ <= 0.0 ||
      !std::isfinite(point_time_before_header_tolerance_sec_) ||
      point_time_before_header_tolerance_sec_ < 0.0 ||
      !std::isfinite(max_point_time_after_header_sec_) ||
      max_point_time_after_header_sec_ <= 0.0 ||
      !std::isfinite(pending_timeout_sec_) || pending_timeout_sec_ <= 0.0 ||
      pending_queue_size <= 0)
    {
      throw std::invalid_argument("invalid strict deskew timing/buffer parameters");
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++clouds_received_;
    const auto callback_time = std::chrono::steady_clock::now();
    if (max_publish_rate_hz_ > 0.0 && has_selected_cloud_) {
      const double elapsed = std::chrono::duration<double>(
        callback_time - last_selected_cloud_time_).count();
      if (elapsed < 1.0 / max_publish_rate_hz_) {
        ++clouds_throttled_;
        return;
      }
    }
    last_selected_cloud_time_ = callback_time;
    has_selected_cloud_ = true;

    if (!deskew_enabled_) {
      project_without_deskew(*msg, callback_time);
      return;
    }

    latest_cloud_point_count_ = static_cast<std::size_t>(msg->width) * msg->height;
    latest_deskewed_point_count_ = 0U;
    latest_dropped_point_count_ = 0U;

    const double header_sec = stamp_seconds(msg->header.stamp);
    if (!std::isfinite(header_sec) || header_sec <= 0.0) {
      drop_frame("cloud header timestamp is zero or invalid", false);
      return;
    }
    if (has_last_cloud_stamp_ && header_sec + 1.0e-9 < last_cloud_stamp_sec_) {
      const std::size_t abandoned = pending_clouds_.size();
      clouds_dropped_ += abandoned;
      pending_clouds_.clear();
      odom_buffer_.clear();
      ++time_rollback_events_;
      RCLCPP_WARN(
        get_logger(),
        "pointcloud time moved backwards; cleared %zu pending clouds and odometry buffer",
        abandoned);
    }
    last_cloud_stamp_sec_ = header_sec;
    has_last_cloud_stamp_ = true;

    std::vector<double> point_times;
    std::string error;
    if (!rm_mid360_driver_bridge::extract_absolute_nanosecond_timestamps(
        *msg, timestamp_field_, point_times, error))
    {
      ++timestamp_failures_;
      drop_frame(error, false);
      return;
    }

    const auto range = std::minmax_element(point_times.begin(), point_times.end());
    const double min_point_time = *range.first;
    const double max_point_time = *range.second;
    if (max_point_time - min_point_time > max_cloud_span_sec_ ||
      min_point_time < header_sec - point_time_before_header_tolerance_sec_ ||
      max_point_time > header_sec + max_point_time_after_header_sec_)
    {
      ++timestamp_failures_;
      ++timestamp_bounds_failures_;
      std::ostringstream reason;
      reason << "point timestamps violate verified Livox/header bounds: header=" << header_sec <<
        " min=" << min_point_time << " max=" << max_point_time;
      drop_frame(reason.str(), false);
      return;
    }

    if (pending_clouds_.size() >= pending_queue_size_) {
      pending_clouds_.pop_front();
      ++clouds_dropped_;
      ++queue_overflow_drops_;
      publish_status(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "deskew pending queue overflow; oldest frame dropped", false);
    }
    pending_clouds_.push_back(
    {
      msg, std::move(point_times), min_point_time, max_point_time, callback_time});
    process_pending();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    ++odom_received_;
    const double stamp_sec = stamp_seconds(msg->header.stamp);
    if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != odom_child_frame_) {
      ++odom_rejected_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "reject deskew odometry frames %s->%s; expected %s->%s",
        msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
        odom_frame_.c_str(), odom_child_frame_.c_str());
      publish_status(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "odometry frame contract mismatch", false);
      return;
    }
    if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0 || !finite_pose(msg->pose.pose)) {
      ++odom_rejected_;
      publish_status(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "odometry timestamp or pose is invalid", false);
      return;
    }

    tf2::Quaternion rotation(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    rotation.normalize();
    tf2::Transform pose;
    pose.setOrigin(
      tf2::Vector3(
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    pose.setRotation(rotation);

    if (!odom_buffer_.empty() && stamp_sec + 1.0e-9 < odom_buffer_.back().stamp_sec) {
      const std::size_t abandoned = pending_clouds_.size();
      clouds_dropped_ += abandoned;
      pending_clouds_.clear();
      odom_buffer_.clear();
      ++time_rollback_events_;
      publish_status(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "odometry time moved backwards; deskew buffers reset", false);
    }
    if (!odom_buffer_.empty() &&
      std::abs(stamp_sec - odom_buffer_.back().stamp_sec) <= 1.0e-9)
    {
      odom_buffer_.back() = {stamp_sec, pose};
    } else {
      odom_buffer_.push_back({stamp_sec, pose});
    }

    const double oldest_allowed = stamp_sec - odom_buffer_duration_sec_;
    const auto first_kept = std::lower_bound(
      odom_buffer_.begin(), odom_buffer_.end(), oldest_allowed,
      [](const rm_mid360_driver_bridge::TimedPose & sample, double time) {
        return sample.stamp_sec < time;
      });
    if (first_kept != odom_buffer_.begin()) {
      odom_buffer_.erase(odom_buffer_.begin(), first_kept);
    }
    process_pending();
  }

  void process_pending()
  {
    if (!deskew_enabled_) {
      return;
    }
    while (!pending_clouds_.empty()) {
      const auto now = std::chrono::steady_clock::now();
      const PendingCloud & pending = pending_clouds_.front();
      const double wait_sec = std::chrono::duration<double>(now - pending.queued_at).count();
      const double reference_sec = stamp_seconds(pending.message->header.stamp);
      const double required_min = std::min(reference_sec, pending.min_point_time_sec);
      const double required_max = std::max(reference_sec, pending.max_point_time_sec);
      const bool odom_covers = !odom_buffer_.empty() &&
        odom_buffer_.front().stamp_sec <= required_min &&
        odom_buffer_.back().stamp_sec >= required_max;

      if (!odom_covers) {
        if (wait_sec < pending_timeout_sec_) {
          break;
        }
        ++odom_coverage_drops_;
        drop_pending_front("timed out waiting for complete odometry coverage");
        continue;
      }

      tf2::Transform sensor_to_base;
      if (!lookup_input_to_target(*pending.message, sensor_to_base, true)) {
        if (wait_sec < pending_timeout_sec_) {
          break;
        }
        ++tf_failures_;
        drop_pending_front("timed out waiting for timestamped sensor-to-base TF");
        continue;
      }

      std::string error;
      std::size_t used_points = 0;
      std::vector<float> ranges;
      const auto processing_start = std::chrono::steady_clock::now();
      if (!build_deskewed_ranges(pending, sensor_to_base, ranges, used_points, error)) {
        ++interpolation_failures_;
        drop_pending_front(error);
        continue;
      }

      const auto scan = rm_mid360_driver_bridge::make_laser_scan(
        pending.message->header, target_frame_, projection_, std::move(ranges));
      pub_->publish(scan);
      const auto completed_at = std::chrono::steady_clock::now();
      ++clouds_output_;
      points_output_ += used_points;
      last_latency_ms_ = std::chrono::duration<double, std::milli>(
        completed_at - pending.queued_at).count();
      last_processing_ms_ = std::chrono::duration<double, std::milli>(
        completed_at - processing_start).count();
      latest_cloud_point_count_ = pending.point_times_sec.size();
      latest_deskewed_point_count_ = used_points;
      latest_dropped_point_count_ = pending.point_times_sec.size() - used_points;
      last_success_stamp_sec_ = reference_sec;
      last_failure_.clear();
      pending_clouds_.pop_front();
      publish_status(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "strict SE(3) deskew active", true);
    }
  }

  bool build_deskewed_ranges(
    const PendingCloud & pending,
    const tf2::Transform & sensor_to_base,
    std::vector<float> & ranges,
    std::size_t & used_points,
    std::string & error)
  {
    tf2::Transform reference_pose;
    const double reference_sec = stamp_seconds(pending.message->header.stamp);
    if (!rm_mid360_driver_bridge::interpolate_pose(
        odom_buffer_, reference_sec, max_interpolation_gap_sec_, reference_pose, error))
    {
      return false;
    }

    ranges.assign(bin_count_, std::numeric_limits<float>::infinity());
    used_points = 0;
    std::size_t linear_index = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> input_x(*pending.message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> input_y(*pending.message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> input_z(*pending.message, "z");
      for (; input_x != input_x.end();
        ++input_x, ++input_y, ++input_z, ++linear_index)
      {
        const float x = *input_x;
        const float y = *input_y;
        const float z = *input_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          ++points_invalid_;
          continue;
        }
        if (linear_index >= pending.point_times_sec.size()) {
          error = "point/timestamp record count mismatch";
          return false;
        }
        tf2::Transform point_pose;
        if (!rm_mid360_driver_bridge::interpolate_pose(
            odom_buffer_, pending.point_times_sec[linear_index],
            max_interpolation_gap_sec_, point_pose, error))
        {
          return false;
        }
        const tf2::Vector3 point = rm_mid360_driver_bridge::deskew_point_to_reference(
          tf2::Vector3(x, y, z), sensor_to_base, point_pose, reference_pose);
        accumulate_point(point, ranges, used_points);
      }
    } catch (const std::runtime_error & exception) {
      error = std::string("cannot read xyz fields: ") + exception.what();
      return false;
    }
    points_input_ += linear_index;
    if (linear_index != pending.point_times_sec.size()) {
      error = "point/timestamp record count mismatch";
      return false;
    }
    return true;
  }

  void project_without_deskew(
    const sensor_msgs::msg::PointCloud2 & message,
    const std::chrono::steady_clock::time_point & callback_time)
  {
    tf2::Transform input_to_target;
    if (!lookup_input_to_target(message, input_to_target, false)) {
      ++tf_failures_;
      drop_frame("cannot transform pointcloud to target frame", false);
      return;
    }

    std::vector<float> ranges(bin_count_, std::numeric_limits<float>::infinity());
    std::size_t used_points = 0;
    std::size_t input_points = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> input_x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> input_y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> input_z(message, "z");
      for (; input_x != input_x.end();
        ++input_x, ++input_y, ++input_z, ++input_points)
      {
        const float x = *input_x;
        const float y = *input_y;
        const float z = *input_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          ++points_invalid_;
          continue;
        }
        accumulate_point(input_to_target * tf2::Vector3(x, y, z), ranges, used_points);
      }
    } catch (const std::runtime_error & exception) {
      drop_frame(std::string("cannot read xyz fields: ") + exception.what(), false);
      return;
    }
    points_input_ += input_points;
    points_output_ += used_points;
    pub_->publish(
      rm_mid360_driver_bridge::make_laser_scan(
        message.header, target_frame_, projection_, std::move(ranges)));
    ++clouds_output_;
    latest_cloud_point_count_ = input_points;
    latest_deskewed_point_count_ = used_points;
    latest_dropped_point_count_ = input_points - used_points;
    last_latency_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_time).count();
    last_success_stamp_sec_ = stamp_seconds(message.header.stamp);
    last_failure_.clear();
    publish_status(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "deskew disabled; baseline projection active", false);
  }

  void accumulate_point(
    const tf2::Vector3 & point,
    std::vector<float> & ranges,
    std::size_t & used_points) const
  {
    if (point.z() < min_height_ || point.z() > max_height_) {
      return;
    }
    const double range = std::hypot(point.x(), point.y());
    if (range < projection_.range_min || range > projection_.range_max) {
      return;
    }
    const double angle = std::atan2(point.y(), point.x());
    if (angle < projection_.angle_min || angle > projection_.angle_max) {
      return;
    }
    const auto index = static_cast<std::size_t>(
      std::floor((angle - projection_.angle_min) / projection_.angle_increment));
    if (index >= ranges.size()) {
      return;
    }
    ranges[index] = std::min(ranges[index], static_cast<float>(range));
    ++used_points;
  }

  bool lookup_input_to_target(
    const sensor_msgs::msg::PointCloud2 & msg,
    tf2::Transform & input_to_target,
    bool nonblocking)
  {
    if (msg.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "drop pointcloud with empty frame_id");
      return false;
    }
    if (target_frame_.empty() || msg.header.frame_id == target_frame_) {
      input_to_target.setIdentity();
      return true;
    }
    try {
      const double timeout_sec = nonblocking ? 0.0 : std::max(0.0, transform_timeout_sec_);
      const auto transform_msg = tf_buffer_.lookupTransform(
        target_frame_, msg.header.frame_id, msg.header.stamp,
        tf2::durationFromSec(timeout_sec));
      tf2::fromMsg(transform_msg.transform, input_to_target);
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot transform pointcloud %s -> %s: %s",
        msg.header.frame_id.c_str(), target_frame_.c_str(), exception.what());
      return false;
    }
  }

  void drop_pending_front(const std::string & reason)
  {
    pending_clouds_.pop_front();
    drop_frame(reason, true);
  }

  void drop_frame(const std::string & reason, bool interpolation_or_coverage)
  {
    (void)interpolation_or_coverage;
    ++clouds_dropped_;
    latest_deskewed_point_count_ = 0U;
    latest_dropped_point_count_ = latest_cloud_point_count_;
    last_failure_ = reason;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "strict deskew dropped pointcloud frame: %s", reason.c_str());
    publish_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, reason, false);
  }

  void publish_status(std::uint8_t level, const std::string & message, bool active)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = get_fully_qualified_name() + std::string("/se3_deskew");
    status.hardware_id = "livox_point_time_plus_lio_odometry";
    status.message = message;
    const auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
      };
    add("enabled", bool_text(deskew_enabled_));
    add("active", bool_text(active));
    add("timestamp_field", timestamp_field_);
    add("timestamp_unit", "absolute nanoseconds stored as FLOAT64");
    add("timestamp_interpretation", timestamp_interpretation_);
    add("reference_time", "cloud header stamp (Livox packet base/first-point time)");
    add("odom_source", odom_topic_ + " (" + odom_frame_ + "->" + odom_child_frame_ + ")");
    add("failure_policy", failure_policy_);
    add("clouds_received", std::to_string(clouds_received_));
    add("clouds_output", std::to_string(clouds_output_));
    add("clouds_dropped", std::to_string(clouds_dropped_));
    add("clouds_throttled", std::to_string(clouds_throttled_));
    add("timestamp_failures", std::to_string(timestamp_failures_));
    add("timestamp_bounds_failures", std::to_string(timestamp_bounds_failures_));
    add("interpolation_failures", std::to_string(interpolation_failures_));
    add("odom_coverage_drops", std::to_string(odom_coverage_drops_));
    add("tf_failures", std::to_string(tf_failures_));
    add("queue_overflow_drops", std::to_string(queue_overflow_drops_));
    add("time_rollback_events", std::to_string(time_rollback_events_));
    add("odom_received", std::to_string(odom_received_));
    add("odom_rejected", std::to_string(odom_rejected_));
    add("odom_buffer_length", std::to_string(odom_buffer_.size()));
    add("pending_clouds", std::to_string(pending_clouds_.size()));
    add("points_input", std::to_string(points_input_));
    add("points_output", std::to_string(points_output_));
    add("points_invalid", std::to_string(points_invalid_));
    add("latest_cloud_point_count", std::to_string(latest_cloud_point_count_));
    add("latest_deskewed_point_count", std::to_string(latest_deskewed_point_count_));
    add("latest_dropped_point_count", std::to_string(latest_dropped_point_count_));
    add("last_processing_ms", std::to_string(last_processing_ms_));
    add("last_latency_ms", std::to_string(last_latency_ms_));
    add("last_success_stamp_sec", std::to_string(last_success_stamp_sec_));
    add("last_failure", last_failure_);
    array.status.push_back(std::move(status));
    status_pub_->publish(array);
    std_msgs::msg::Bool active_message;
    active_message.data = active;
    active_pub_->publish(active_message);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  rm_mid360_driver_bridge::LaserProjection projection_;
  double min_height_{0.0};
  double max_height_{0.0};
  double transform_timeout_sec_{0.0};
  double max_publish_rate_hz_{0.0};
  std::size_t bin_count_{0};

  bool deskew_enabled_{false};
  std::string timestamp_field_;
  std::string timestamp_interpretation_;
  std::string reference_time_policy_;
  std::string failure_policy_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string odom_child_frame_;
  double odom_buffer_duration_sec_{0.0};
  double max_interpolation_gap_sec_{0.0};
  double max_cloud_span_sec_{0.0};
  double point_time_before_header_tolerance_sec_{0.0};
  double max_point_time_after_header_sec_{0.0};
  double pending_timeout_sec_{0.0};
  std::size_t pending_queue_size_{0};
  std::string status_topic_;
  std::string active_topic_;

  bool has_selected_cloud_{false};
  std::chrono::steady_clock::time_point last_selected_cloud_time_;
  bool has_last_cloud_stamp_{false};
  double last_cloud_stamp_sec_{0.0};
  std::vector<rm_mid360_driver_bridge::TimedPose> odom_buffer_;
  std::deque<PendingCloud> pending_clouds_;

  std::uint64_t clouds_received_{0};
  std::uint64_t clouds_output_{0};
  std::uint64_t clouds_dropped_{0};
  std::uint64_t clouds_throttled_{0};
  std::uint64_t timestamp_failures_{0};
  std::uint64_t timestamp_bounds_failures_{0};
  std::uint64_t interpolation_failures_{0};
  std::uint64_t odom_coverage_drops_{0};
  std::uint64_t tf_failures_{0};
  std::uint64_t queue_overflow_drops_{0};
  std::uint64_t time_rollback_events_{0};
  std::uint64_t odom_received_{0};
  std::uint64_t odom_rejected_{0};
  std::uint64_t points_input_{0};
  std::uint64_t points_output_{0};
  std::uint64_t points_invalid_{0};
  std::size_t latest_cloud_point_count_{0};
  std::size_t latest_deskewed_point_count_{0};
  std::size_t latest_dropped_point_count_{0};
  double last_processing_ms_{0.0};
  double last_latency_ms_{0.0};
  double last_success_stamp_sec_{0.0};
  std::string last_failure_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudToLaserscanNode>());
  rclcpp::shutdown();
  return 0;
}
