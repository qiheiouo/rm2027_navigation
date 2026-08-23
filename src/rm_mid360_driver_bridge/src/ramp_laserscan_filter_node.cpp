#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

}  // namespace

class RampLaserScanFilterNode : public rclcpp::Node
{
public:
  RampLaserScanFilterNode()
  : Node("ramp_laserscan_filter_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/simulation/scan_ramp_unfiltered");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");
    diagnostic_topic_ = declare_parameter<std::string>(
      "diagnostic_topic", "/diagnostics");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    filter_enabled_ = declare_parameter<bool>("filter_enabled", true);
    shadow_only_ = declare_parameter<bool>("shadow_only", true);
    transform_timeout_sec_ = declare_parameter<double>(
      "transform_timeout_sec", 0.05);

    expected_map_id_ = declare_parameter<std::string>("expected_map_id", "");
    expected_map_revision_ = declare_parameter<std::string>(
      "expected_map_revision", "");
    active_map_id_ = declare_parameter<std::string>("active_map_id", "");
    active_map_revision_ = declare_parameter<std::string>(
      "active_map_revision", "");
    map_contract_valid_ =
      !expected_map_id_.empty() && !expected_map_revision_.empty() &&
      expected_map_id_ == active_map_id_ &&
      expected_map_revision_ == active_map_revision_;

    if (input_topic_ == output_topic_) {
      throw std::runtime_error(
              "ramp LaserScan filter input and output topics must differ");
    }
    load_regions();

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    diagnostic_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostic_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(
        &RampLaserScanFilterNode::scan_callback, this,
        std::placeholders::_1));

    RCLCPP_WARN(
      get_logger(),
      "ramp LaserScan filter is %s: %s -> %s, regions=%zu, map_contract=%s",
      shadow_only_ ? "SHADOW-ONLY" : "ACTIVE",
      input_topic_.c_str(), output_topic_.c_str(), regions_.size(),
      map_contract_valid_ ? "valid" : "INVALID_FAIL_CLOSED");
  }

private:
  void load_regions()
  {
    const auto names = declare_parameter<std::vector<std::string>>(
      "region_names", std::vector<std::string>{});
    regions_.reserve(names.size());
    for (const auto & name : names) {
      const std::string prefix = "regions." + name + ".";
      const auto polygon_values = declare_parameter<std::vector<double>>(
        prefix + "polygon_xy", std::vector<double>{});
      const auto origin = declare_parameter<std::vector<double>>(
        prefix + "origin_xyz", std::vector<double>{});
      const double ascent_yaw_deg = declare_parameter<double>(
        prefix + "ascent_yaw_deg", 0.0);
      const double slope_deg = declare_parameter<double>(
        prefix + "slope_deg", 0.0);
      const double surface_tolerance = declare_parameter<double>(
        prefix + "surface_tolerance", 0.04);

      if (polygon_values.size() < 6U || polygon_values.size() % 2U != 0U) {
        throw std::runtime_error(
                prefix + "polygon_xy must contain at least three x/y pairs");
      }
      if (origin.size() != 3U) {
        throw std::runtime_error(prefix + "origin_xyz must contain x/y/z");
      }

      rm_mid360_driver_bridge::RampRegion region;
      region.name = name;
      for (std::size_t index = 0U; index < polygon_values.size(); index += 2U) {
        region.polygon.push_back(
          rm_mid360_driver_bridge::Point2d{
          polygon_values[index], polygon_values[index + 1U]});
      }
      region.origin_x = origin[0];
      region.origin_y = origin[1];
      region.origin_height = origin[2];
      region.ascent_yaw_rad = ascent_yaw_deg * kPi / 180.0;
      region.slope_rad = slope_deg * kPi / 180.0;
      region.surface_tolerance = surface_tolerance;

      std::string error;
      if (!rm_mid360_driver_bridge::validate_ramp_region(region, error)) {
        throw std::runtime_error(error);
      }
      regions_.push_back(std::move(region));
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    if (!filter_enabled_ || !map_contract_valid_ || regions_.empty()) {
      publisher_->publish(*message);
      const std::string reason = !filter_enabled_ ? "disabled" :
        (!map_contract_valid_ ? "map contract mismatch" : "no ramp regions");
      publish_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "fail-closed passthrough: " + reason, message->ranges.size(), 0U);
      return;
    }

    tf2::Transform input_to_map;
    std::string transform_error;
    if (!lookup_input_to_map(*message, input_to_map, transform_error)) {
      publisher_->publish(*message);
      publish_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "fail-closed passthrough: " + transform_error,
        message->ranges.size(), 0U);
      return;
    }

    auto output = *message;
    std::size_t removed_count = 0U;
    double angle = static_cast<double>(message->angle_min);
    for (auto & range : output.ranges) {
      const double distance = static_cast<double>(range);
      if (std::isfinite(distance) &&
        distance >= static_cast<double>(message->range_min) &&
        distance <= static_cast<double>(message->range_max))
      {
        const tf2::Vector3 point_in_map = input_to_map * tf2::Vector3(
          distance * std::cos(angle), distance * std::sin(angle), 0.0);
        if (rm_mid360_driver_bridge::match_expected_ramp_surface(
            regions_, point_in_map.x(), point_in_map.y(), point_in_map.z()))
        {
          range = std::numeric_limits<float>::infinity();
          ++removed_count;
        }
      }
      angle += static_cast<double>(message->angle_increment);
    }

    publisher_->publish(output);
    publish_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      shadow_only_ ? "shadow output generated" : "active output generated",
      message->ranges.size(), removed_count);
  }

  bool lookup_input_to_map(
    const sensor_msgs::msg::LaserScan & message,
    tf2::Transform & input_to_map,
    std::string & error)
  {
    if (message.header.frame_id.empty()) {
      error = "empty input frame";
      return false;
    }
    if (message.header.frame_id == map_frame_) {
      input_to_map.setIdentity();
      return true;
    }
    try {
      const auto timeout = tf2::durationFromSec(
        std::max(0.0, transform_timeout_sec_));
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, message.header.frame_id, message.header.stamp, timeout);
      tf2::fromMsg(transform.transform, input_to_map);
      return true;
    } catch (const tf2::TransformException & exception) {
      error = std::string("TF unavailable: ") + exception.what();
      return false;
    }
  }

  void publish_diagnostic(
    std::uint8_t level,
    const std::string & message,
    std::size_t input_count,
    std::size_t removed_count)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = "rm_mid360_driver_bridge/ramp_laserscan_filter";
    status.hardware_id = "simulation_or_map_bound_laserscan";
    status.message = message;
    status.values = {
      key_value("shadow_only", shadow_only_ ? "true" : "false"),
      key_value("map_contract_valid", map_contract_valid_ ? "true" : "false"),
      key_value("expected_map_id", expected_map_id_),
      key_value("expected_map_revision", expected_map_revision_),
      key_value("active_map_id", active_map_id_),
      key_value("active_map_revision", active_map_revision_),
      key_value("region_count", std::to_string(regions_.size())),
      key_value("input_beams", std::to_string(input_count)),
      key_value("removed_surface_beams", std::to_string(removed_count)),
      key_value("kept_beams", std::to_string(input_count - removed_count)),
    };
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string diagnostic_topic_;
  std::string map_frame_;
  std::string expected_map_id_;
  std::string expected_map_revision_;
  std::string active_map_id_;
  std::string active_map_revision_;
  bool filter_enabled_{true};
  bool shadow_only_{true};
  bool map_contract_valid_{false};
  double transform_timeout_sec_{0.05};
  std::vector<rm_mid360_driver_bridge::RampRegion> regions_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RampLaserScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
