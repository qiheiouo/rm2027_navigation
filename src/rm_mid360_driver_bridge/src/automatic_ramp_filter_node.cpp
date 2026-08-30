#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rm_mid360_driver_bridge/pointcloud_deskew.hpp"
#include "rm_mid360_driver_bridge/ramp_plane_filter.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

diagnostic_msgs::msg::KeyValue key_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

double angle_distance(double first, double second)
{
  return std::abs(std::atan2(std::sin(first - second), std::cos(first - second)));
}

rm_mid360_driver_bridge::Point2d region_center(
  const rm_mid360_driver_bridge::RampRegion & region)
{
  rm_mid360_driver_bridge::Point2d center;
  for (const auto & point : region.polygon) {
    center.x += point.x;
    center.y += point.y;
  }
  const double divisor = static_cast<double>(region.polygon.size());
  center.x /= divisor;
  center.y /= divisor;
  return center;
}

}  // namespace

class AutomaticRampFilterNode : public rclcpp::Node
{
public:
  AutomaticRampFilterNode()
  : Node("automatic_ramp_filter_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/points/obstacles_fused");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/perception/ramp/automatic_filtered_shadow");
    secondary_input_topic_ = declare_parameter<std::string>("secondary_input_topic", "");
    secondary_output_topic_ = declare_parameter<std::string>("secondary_output_topic", "");
    diagnostic_topic_ = declare_parameter<std::string>(
      "diagnostic_topic", "/diagnostics");
    detection_frame_ = declare_parameter<std::string>("detection_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    filter_enabled_ = declare_parameter<bool>("filter_enabled", true);
    shadow_only_ = declare_parameter<bool>("shadow_only", true);
    transform_timeout_sec_ = declare_parameter<double>(
      "transform_timeout_sec", 0.05);

    min_forward_ = declare_parameter<double>("roi.min_forward", -0.40);
    max_forward_ = declare_parameter<double>("roi.max_forward", 3.00);
    half_width_ = declare_parameter<double>("roi.half_width", 1.20);
    min_z_from_base_ = declare_parameter<double>("roi.min_z_from_base", -0.80);
    max_z_from_base_ = declare_parameter<double>("roi.max_z_from_base", 0.80);
    voxel_size_ = declare_parameter<double>("voxel_size", 0.08);
    detection_update_period_sec_ = declare_parameter<double>(
      "detection.update_period_sec", 0.10);
    accumulation_window_sec_ = declare_parameter<double>(
      "detection.accumulation_window_sec", 0.30);
    diagnostic_period_sec_ = declare_parameter<double>(
      "diagnostics.publish_period_sec", 0.20);

    detection_config_.min_slope_rad =
      declare_parameter<double>("detection.min_slope_deg", 5.0) * kPi / 180.0;
    detection_config_.max_slope_rad =
      declare_parameter<double>("detection.max_slope_deg", 25.0) * kPi / 180.0;
    detection_config_.inlier_tolerance = declare_parameter<double>(
      "detection.inlier_tolerance", 0.025);
    detection_config_.surface_tolerance = declare_parameter<double>(
      "detection.surface_tolerance", 0.05);
    detection_config_.min_inliers = static_cast<std::size_t>(std::max<std::int64_t>(
        3, declare_parameter<std::int64_t>("detection.min_inliers", 30)));
    detection_config_.min_inlier_ratio = declare_parameter<double>(
      "detection.min_inlier_ratio", 0.12);
    detection_config_.min_length = declare_parameter<double>(
      "detection.min_length", 0.60);
    detection_config_.min_width = declare_parameter<double>(
      "detection.min_width", 0.55);
    detection_config_.min_height_span = declare_parameter<double>(
      "detection.min_height_span", 0.08);
    detection_config_.ransac_iterations =
      static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("detection.ransac_iterations", 256)));

    confirmation_frames_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("tracking.confirmation_frames", 5)));
    max_missed_frames_ = static_cast<std::size_t>(std::max<std::int64_t>(
        0, declare_parameter<std::int64_t>("tracking.max_missed_frames", 8)));
    pending_max_missed_frames_ = static_cast<std::size_t>(std::max<std::int64_t>(
        0, declare_parameter<std::int64_t>("tracking.pending_max_missed_frames", 2)));
    max_center_shift_ = declare_parameter<double>("tracking.max_center_shift", 0.60);
    max_slope_change_rad_ =
      declare_parameter<double>("tracking.max_slope_change_deg", 3.0) * kPi / 180.0;
    max_yaw_change_rad_ =
      declare_parameter<double>("tracking.max_yaw_change_deg", 15.0) * kPi / 180.0;

    validate_parameters();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostic_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AutomaticRampFilterNode::primary_cloud_callback, this, std::placeholders::_1));
    if (!secondary_input_topic_.empty()) {
      secondary_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        secondary_output_topic_, rclcpp::SensorDataQoS());
      secondary_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        secondary_input_topic_, rclcpp::SensorDataQoS(),
        std::bind(
          &AutomaticRampFilterNode::secondary_cloud_callback, this,
          std::placeholders::_1));
    }

    RCLCPP_WARN(
      get_logger(),
      "automatic ramp filter is %s: primary %s -> %s, secondary %s -> %s, "
      "detection_frame=%s update=%.2f s accumulate=%.2f s confirm=%zu",
      shadow_only_ ? "SHADOW-ONLY" : "ACTIVE", input_topic_.c_str(),
      output_topic_.c_str(),
      secondary_input_topic_.empty() ? "disabled" : secondary_input_topic_.c_str(),
      secondary_output_topic_.empty() ? "disabled" : secondary_output_topic_.c_str(),
      detection_frame_.c_str(), detection_update_period_sec_, accumulation_window_sec_,
      confirmation_frames_);
  }

private:
  struct VoxelCell
  {
    double local_z{std::numeric_limits<double>::infinity()};
    rm_mid360_driver_bridge::Point3d point;
  };

  struct DetectionFrame
  {
    std::int64_t stamp_nanoseconds{0};
    std::vector<rm_mid360_driver_bridge::Point3d> points;
  };

  struct StreamStatistics
  {
    std::size_t input_points{0U};
    std::size_t removed_points{0U};
  };

  void validate_parameters()
  {
    const bool secondary_partially_configured =
      secondary_input_topic_.empty() != secondary_output_topic_.empty();
    if (input_topic_.empty() || output_topic_.empty() || detection_frame_.empty() ||
      base_frame_.empty() || input_topic_ == output_topic_ ||
      secondary_partially_configured ||
      (!secondary_input_topic_.empty() &&
      (secondary_input_topic_ == input_topic_ ||
      secondary_input_topic_ == output_topic_ ||
      secondary_input_topic_ == secondary_output_topic_ ||
      secondary_output_topic_ == input_topic_ ||
      secondary_output_topic_ == output_topic_)))
    {
      throw std::invalid_argument(
              "automatic ramp topics/frames must be non-empty and topics must differ");
    }
    if (transform_timeout_sec_ < 0.0 || min_forward_ >= max_forward_ ||
      half_width_ <= 0.0 || min_z_from_base_ >= max_z_from_base_ ||
      voxel_size_ <= 0.0 || max_center_shift_ <= 0.0 ||
      detection_update_period_sec_ < 0.0 || accumulation_window_sec_ < 0.0 ||
      diagnostic_period_sec_ <= 0.0 || max_slope_change_rad_ <= 0.0 ||
      max_yaw_change_rad_ <= 0.0)
    {
      throw std::invalid_argument("automatic ramp ROI, TF, voxel or tracking parameters invalid");
    }
    if (detection_config_.min_slope_rad <= 0.0 ||
      detection_config_.max_slope_rad <= detection_config_.min_slope_rad ||
      detection_config_.max_slope_rad >= 0.25 * kPi ||
      detection_config_.inlier_tolerance <= 0.0 ||
      detection_config_.surface_tolerance <= 0.0 ||
      detection_config_.surface_tolerance > 0.20 ||
      detection_config_.min_inlier_ratio <= 0.0 ||
      detection_config_.min_inlier_ratio > 1.0 ||
      detection_config_.min_length <= 0.0 || detection_config_.min_width <= 0.0 ||
      detection_config_.min_height_span <= 0.0)
    {
      throw std::invalid_argument("automatic ramp detection parameters invalid");
    }
  }

  void primary_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    process_cloud(message, publisher_, true, primary_statistics_);
  }

  void secondary_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    process_cloud(message, secondary_publisher_, false, secondary_statistics_);
  }

  void process_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & message,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    bool detection_source,
    StreamStatistics & statistics)
  {
    const auto report_diagnostic =
      [this, detection_source](std::uint8_t level, const std::string & message) {
        if (detection_source) {
          maybe_publish_diagnostic(level, message);
        }
      };
    const std::size_t input_count =
      static_cast<std::size_t>(message->width) * message->height;
    statistics.input_points = input_count;
    statistics.removed_points = 0U;
    if (!filter_enabled_) {
      publisher->publish(*message);
      report_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "fail-closed passthrough: disabled");
      return;
    }
    if (message->header.frame_id.empty() ||
      (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0U))
    {
      publisher->publish(*message);
      if (detection_source) {
        reset_tracking();
      }
      report_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "fail-closed passthrough: empty frame or zero timestamp");
      return;
    }

    tf2::Transform input_to_detection;
    std::string transform_error;
    if (!lookup_transform(
        detection_frame_, message->header.frame_id, message->header.stamp,
        input_to_detection, transform_error))
    {
      publisher->publish(*message);
      if (detection_source) {
        reset_tracking();
      }
      report_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "fail-closed passthrough: " + transform_error);
      return;
    }

    if (detection_source) {
      tf2::Transform input_to_base;
      if (!lookup_transform(
          base_frame_, message->header.frame_id, message->header.stamp,
          input_to_base, transform_error))
      {
        publisher->publish(*message);
        reset_tracking();
        report_diagnostic(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "fail-closed passthrough: " + transform_error);
        return;
      }
      try {
        update_detection(*message, input_to_detection, input_to_base);
      } catch (const std::exception & error) {
        publisher->publish(*message);
        reset_tracking();
        report_diagnostic(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          std::string("fail-closed malformed cloud passthrough: ") + error.what());
        return;
      }
    }

    if (!confirmed_detection_) {
      publisher->publish(*message);
      report_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        pending_detection_ ? "candidate accumulating; fail-closed passthrough" :
        "no ramp candidate; fail-closed passthrough");
      return;
    }

    std::vector<std::size_t> kept_indices;
    kept_indices.reserve(input_count);
    std::size_t removed_count = 0U;
    const std::vector<rm_mid360_driver_bridge::RampRegion> regions{
      confirmed_detection_->region};
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*message, "z");
      std::size_t linear_index = 0U;
      for (; x_iterator != x_iterator.end();
        ++x_iterator, ++y_iterator, ++z_iterator, ++linear_index)
      {
        if (!std::isfinite(*x_iterator) || !std::isfinite(*y_iterator) ||
          !std::isfinite(*z_iterator))
        {
          kept_indices.push_back(linear_index);
          continue;
        }
        const tf2::Vector3 target = input_to_detection *
          tf2::Vector3(*x_iterator, *y_iterator, *z_iterator);
        if (rm_mid360_driver_bridge::match_expected_ramp_surface(
            regions, target.x(), target.y(), target.z()))
        {
          ++removed_count;
        } else {
          kept_indices.push_back(linear_index);
        }
      }
      publisher->publish(
        rm_mid360_driver_bridge::select_points_preserving_fields(
          *message, kept_indices));
      statistics.removed_points = removed_count;
    } catch (const std::exception & error) {
      publisher->publish(*message);
      report_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        std::string("fail-closed filtering passthrough: ") + error.what());
      return;
    }

    report_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      shadow_only_ ? "automatic ramp confirmed; shadow output generated" :
      "automatic ramp confirmed; active output generated");
  }

  std::vector<rm_mid360_driver_bridge::Point3d> extract_detection_points(
    const sensor_msgs::msg::PointCloud2 & message,
    const tf2::Transform & input_to_detection,
    const tf2::Transform & input_to_base) const
  {
    std::map<std::pair<int, int>, VoxelCell> voxels;
    sensor_msgs::PointCloud2ConstIterator<float> x_iterator(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iterator(message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iterator(message, "z");
    for (; x_iterator != x_iterator.end(); ++x_iterator, ++y_iterator, ++z_iterator) {
      if (!std::isfinite(*x_iterator) || !std::isfinite(*y_iterator) ||
        !std::isfinite(*z_iterator))
      {
        continue;
      }
      const tf2::Vector3 input_point(*x_iterator, *y_iterator, *z_iterator);
      const tf2::Vector3 local = input_to_base * input_point;
      if (local.x() < min_forward_ || local.x() > max_forward_ ||
        std::abs(local.y()) > half_width_ || local.z() < min_z_from_base_ ||
        local.z() > max_z_from_base_)
      {
        continue;
      }
      const auto key = std::make_pair(
        static_cast<int>(std::floor(local.x() / voxel_size_)),
        static_cast<int>(std::floor(local.y() / voxel_size_)));
      const tf2::Vector3 target = input_to_detection * input_point;
      auto & cell = voxels[key];
      if (local.z() < cell.local_z) {
        cell.local_z = local.z();
        cell.point = rm_mid360_driver_bridge::Point3d{
          target.x(), target.y(), target.z()};
      }
    }
    std::vector<rm_mid360_driver_bridge::Point3d> points;
    points.reserve(voxels.size());
    for (const auto & item : voxels) {
      points.push_back(item.second.point);
    }
    return points;
  }

  void update_detection(
    const sensor_msgs::msg::PointCloud2 & message,
    const tf2::Transform & input_to_detection,
    const tf2::Transform & input_to_base)
  {
    const std::int64_t stamp_nanoseconds = rclcpp::Time(message.header.stamp).nanoseconds();
    const std::int64_t reset_threshold = 1000000000LL;
    if (latest_detection_input_stamp_nanoseconds_ > 0 &&
      stamp_nanoseconds + reset_threshold < latest_detection_input_stamp_nanoseconds_)
    {
      reset_tracking();
    }
    latest_detection_input_stamp_nanoseconds_ = std::max(
      latest_detection_input_stamp_nanoseconds_, stamp_nanoseconds);
    detection_frames_.push_back(
      DetectionFrame{
      stamp_nanoseconds,
      extract_detection_points(message, input_to_detection, input_to_base)});

    const auto window_nanoseconds = static_cast<std::int64_t>(
      accumulation_window_sec_ * 1.0e9);
    const auto cutoff = latest_detection_input_stamp_nanoseconds_ - window_nanoseconds;
    detection_frames_.erase(
      std::remove_if(
        detection_frames_.begin(), detection_frames_.end(),
        [cutoff](const DetectionFrame & frame) {
          return frame.stamp_nanoseconds < cutoff;
        }),
      detection_frames_.end());

    const auto update_nanoseconds = static_cast<std::int64_t>(
      detection_update_period_sec_ * 1.0e9);
    if (detection_update_period_sec_ > 0.0 && last_detection_stamp_nanoseconds_ > 0 &&
      latest_detection_input_stamp_nanoseconds_ - last_detection_stamp_nanoseconds_ <
      update_nanoseconds)
    {
      return;
    }
    last_detection_stamp_nanoseconds_ = latest_detection_input_stamp_nanoseconds_;

    std::map<std::pair<int, int>, VoxelCell> accumulated_voxels;
    for (const auto & frame : detection_frames_) {
      for (const auto & point : frame.points) {
        const auto key = std::make_pair(
          static_cast<int>(std::floor(point.x / voxel_size_)),
          static_cast<int>(std::floor(point.y / voxel_size_)));
        auto & cell = accumulated_voxels[key];
        if (point.z < cell.local_z) {
          cell.local_z = point.z;
          cell.point = point;
        }
      }
    }
    std::vector<rm_mid360_driver_bridge::Point3d> accumulated_points;
    accumulated_points.reserve(accumulated_voxels.size());
    for (const auto & item : accumulated_voxels) {
      accumulated_points.push_back(item.second.point);
    }
    last_detection_point_count_ = accumulated_points.size();
    last_detection_ = rm_mid360_driver_bridge::detect_automatic_ramp(
      accumulated_points, detection_config_);
    update_track(last_detection_);
  }

  bool lookup_transform(
    const std::string & target,
    const std::string & source,
    const builtin_interfaces::msg::Time & stamp,
    tf2::Transform & result,
    std::string & error)
  {
    if (target == source) {
      result.setIdentity();
      return true;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target, source, stamp,
        tf2::durationFromSec(std::max(0.0, transform_timeout_sec_)));
      tf2::fromMsg(transform.transform, result);
      return true;
    } catch (const tf2::TransformException & exception) {
      error = "TF " + source + " -> " + target + " unavailable: " + exception.what();
      return false;
    }
  }

  bool same_track(
    const rm_mid360_driver_bridge::AutomaticRampDetection & first,
    const rm_mid360_driver_bridge::AutomaticRampDetection & second) const
  {
    const auto first_center = region_center(first.region);
    const auto second_center = region_center(second.region);
    const double center_shift = std::hypot(
      first_center.x - second_center.x, first_center.y - second_center.y);
    return center_shift <= max_center_shift_ &&
           std::abs(first.region.slope_rad - second.region.slope_rad) <=
           max_slope_change_rad_ &&
           angle_distance(first.region.ascent_yaw_rad, second.region.ascent_yaw_rad) <=
           max_yaw_change_rad_;
  }

  void update_track(
    const std::optional<rm_mid360_driver_bridge::AutomaticRampDetection> & detection)
  {
    if (!detection) {
      if (pending_detection_ && ++pending_missed_count_ > pending_max_missed_frames_) {
        pending_detection_.reset();
        pending_missed_count_ = 0U;
        confirmation_count_ = 0U;
      }
      if (confirmed_detection_ && ++missed_count_ > max_missed_frames_) {
        confirmed_detection_.reset();
        missed_count_ = 0U;
      }
      return;
    }

    if (confirmed_detection_) {
      if (same_track(*confirmed_detection_, *detection)) {
        confirmed_detection_ = detection;
        missed_count_ = 0U;
      } else if (++missed_count_ > max_missed_frames_) {
        confirmed_detection_.reset();
        missed_count_ = 0U;
      }
    }

    if (pending_detection_ && same_track(*pending_detection_, *detection)) {
      pending_detection_ = detection;
      pending_missed_count_ = 0U;
      confirmation_count_ = std::min(confirmation_frames_, confirmation_count_ + 1U);
    } else {
      pending_detection_ = detection;
      pending_missed_count_ = 0U;
      confirmation_count_ = 1U;
    }
    if (confirmation_count_ >= confirmation_frames_) {
      confirmed_detection_ = detection;
      missed_count_ = 0U;
    }
  }

  void reset_tracking()
  {
    pending_detection_.reset();
    confirmed_detection_.reset();
    last_detection_.reset();
    confirmation_count_ = 0U;
    pending_missed_count_ = 0U;
    missed_count_ = 0U;
    latest_detection_input_stamp_nanoseconds_ = 0;
    last_detection_stamp_nanoseconds_ = 0;
    last_detection_point_count_ = 0U;
    detection_frames_.clear();
  }

  void maybe_publish_diagnostic(
    std::uint8_t level,
    const std::string & message)
  {
    const auto now_steady = std::chrono::steady_clock::now();
    if (last_diagnostic_valid_ && std::chrono::duration<double>(
        now_steady - last_diagnostic_steady_).count() < diagnostic_period_sec_)
    {
      return;
    }
    last_diagnostic_valid_ = true;
    last_diagnostic_steady_ = now_steady;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = get_fully_qualified_name() + std::string("/automatic_ramp_filter");
    status.hardware_id = "unlabelled_pointcloud_ramp_detector";
    status.message = message;
    status.values = {
      key_value("shadow_only", shadow_only_ ? "true" : "false"),
      key_value("detection_frame", detection_frame_),
      key_value("candidate_present", last_detection_ ? "true" : "false"),
      key_value("confirmed", confirmed_detection_ ? "true" : "false"),
      key_value("confirmation_count", std::to_string(confirmation_count_)),
      key_value("pending_missed_count", std::to_string(pending_missed_count_)),
      key_value("missed_count", std::to_string(missed_count_)),
      key_value("detection_frame_count", std::to_string(detection_frames_.size())),
      key_value("detection_points", std::to_string(last_detection_point_count_)),
      key_value("detection_update_period_sec", std::to_string(detection_update_period_sec_)),
      key_value("accumulation_window_sec", std::to_string(accumulation_window_sec_)),
      key_value("input_points", std::to_string(primary_statistics_.input_points)),
      key_value("removed_surface_points", std::to_string(primary_statistics_.removed_points)),
      key_value("secondary_enabled", secondary_input_topic_.empty() ? "false" : "true"),
      key_value(
        "secondary_input_points", std::to_string(secondary_statistics_.input_points)),
      key_value(
        "secondary_removed_surface_points",
        std::to_string(secondary_statistics_.removed_points)),
    };
    const auto & diagnostic_detection = last_detection_ ? last_detection_ : confirmed_detection_;
    if (diagnostic_detection) {
      status.values.push_back(
        key_value(
          "slope_deg",
          std::to_string(diagnostic_detection->region.slope_rad * 180.0 / kPi)));
      status.values.push_back(
        key_value("length", std::to_string(diagnostic_detection->length)));
      status.values.push_back(
        key_value("width", std::to_string(diagnostic_detection->width)));
      status.values.push_back(
        key_value(
          "inlier_ratio", std::to_string(diagnostic_detection->inlier_ratio)));
      status.values.push_back(
        key_value(
          "rms_residual", std::to_string(diagnostic_detection->rms_residual)));
    }
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string secondary_input_topic_;
  std::string secondary_output_topic_;
  std::string diagnostic_topic_;
  std::string detection_frame_;
  std::string base_frame_;
  bool filter_enabled_{true};
  bool shadow_only_{true};
  double transform_timeout_sec_{0.05};
  double min_forward_{-0.40};
  double max_forward_{3.00};
  double half_width_{1.20};
  double min_z_from_base_{-0.80};
  double max_z_from_base_{0.80};
  double voxel_size_{0.08};
  double detection_update_period_sec_{0.10};
  double accumulation_window_sec_{0.30};
  double diagnostic_period_sec_{0.20};
  rm_mid360_driver_bridge::AutomaticRampDetectionConfig detection_config_;
  std::size_t confirmation_frames_{5U};
  std::size_t max_missed_frames_{8U};
  std::size_t pending_max_missed_frames_{2U};
  double max_center_shift_{0.60};
  double max_slope_change_rad_{0.05235987755982989};
  double max_yaw_change_rad_{0.2617993877991494};
  std::size_t confirmation_count_{0U};
  std::size_t pending_missed_count_{0U};
  std::size_t missed_count_{0U};
  std::int64_t latest_detection_input_stamp_nanoseconds_{0};
  std::int64_t last_detection_stamp_nanoseconds_{0};
  std::size_t last_detection_point_count_{0U};
  std::deque<DetectionFrame> detection_frames_;
  std::optional<rm_mid360_driver_bridge::AutomaticRampDetection> last_detection_;
  std::optional<rm_mid360_driver_bridge::AutomaticRampDetection> pending_detection_;
  std::optional<rm_mid360_driver_bridge::AutomaticRampDetection> confirmed_detection_;
  StreamStatistics primary_statistics_;
  StreamStatistics secondary_statistics_;
  bool last_diagnostic_valid_{false};
  std::chrono::steady_clock::time_point last_diagnostic_steady_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr secondary_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr secondary_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutomaticRampFilterNode>());
  rclcpp::shutdown();
  return 0;
}
