#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

std::string size_text(std::size_t value)
{
  return std::to_string(value);
}

}  // namespace

class RampPlaneFilterNode : public rclcpp::Node
{
public:
  RampPlaneFilterNode()
  : Node("ramp_plane_filter_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/points/obstacles_fused");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/points/obstacles_ramp_filtered_shadow");
    diagnostic_topic_ = declare_parameter<std::string>(
      "diagnostic_topic", "/diagnostics");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    filter_enabled_ = declare_parameter<bool>("filter_enabled", true);
    shadow_only_ = declare_parameter<bool>("shadow_only", true);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);

    expected_map_id_ = declare_parameter<std::string>("expected_map_id", "");
    expected_map_revision_ = declare_parameter<std::string>("expected_map_revision", "");
    active_map_id_ = declare_parameter<std::string>("active_map_id", "");
    active_map_revision_ = declare_parameter<std::string>("active_map_revision", "");
    map_contract_valid_ =
      !expected_map_id_.empty() && !expected_map_revision_.empty() &&
      expected_map_id_ == active_map_id_ &&
      expected_map_revision_ == active_map_revision_;

    load_regions();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostic_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RampPlaneFilterNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_WARN(
      get_logger(),
      "ramp plane filter is %s: %s -> %s, regions=%zu, map_contract=%s",
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
      const double slope_deg = declare_parameter<double>(prefix + "slope_deg", 0.0);
      const double surface_tolerance = declare_parameter<double>(
        prefix + "surface_tolerance", 0.04);

      if (polygon_values.size() < 6U || polygon_values.size() % 2U != 0U) {
        throw std::runtime_error(
                prefix + "polygon_xy must contain at least three x/y pairs");
      }
      if (origin.size() != 3U) {
        throw std::runtime_error(prefix + "origin_xyz must contain exactly x/y/z");
      }

      rm_mid360_driver_bridge::RampRegion region;
      region.name = name;
      for (std::size_t index = 0; index < polygon_values.size(); index += 2U) {
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

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (!filter_enabled_ || !map_contract_valid_ || regions_.empty()) {
      publisher_->publish(*message);
      const std::string reason = !filter_enabled_ ? "disabled" :
        (!map_contract_valid_ ? "map contract mismatch" : "no ramp regions");
      publish_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "fail-closed passthrough: " + reason,
        static_cast<std::size_t>(message->width) * message->height, 0U);
      return;
    }

    tf2::Transform input_to_map;
    std::string transform_error;
    if (!lookup_input_to_map(*message, input_to_map, transform_error)) {
      publisher_->publish(*message);
      publish_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "fail-closed passthrough: " + transform_error,
        static_cast<std::size_t>(message->width) * message->height, 0U);
      return;
    }

    std::vector<std::size_t> kept_indices;
    const std::size_t input_count =
      static_cast<std::size_t>(message->width) * message->height;
    kept_indices.reserve(input_count);
    std::size_t removed_count = 0U;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*message, "z");
      std::size_t linear_index = 0U;
      for (; x_iterator != x_iterator.end();
        ++x_iterator, ++y_iterator, ++z_iterator, ++linear_index)
      {
        const double x = *x_iterator;
        const double y = *y_iterator;
        const double z = *z_iterator;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          kept_indices.push_back(linear_index);
          continue;
        }
        const tf2::Vector3 point_in_map = input_to_map * tf2::Vector3(x, y, z);
        if (rm_mid360_driver_bridge::match_expected_ramp_surface(
            regions_, point_in_map.x(), point_in_map.y(), point_in_map.z()))
        {
          ++removed_count;
        } else {
          kept_indices.push_back(linear_index);
        }
      }
    } catch (const std::exception & error) {
      publisher_->publish(*message);
      publish_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        std::string("fail-closed malformed cloud passthrough: ") + error.what(),
        input_count, 0U);
      return;
    }

    try {
      publisher_->publish(
        rm_mid360_driver_bridge::select_points_preserving_fields(
          *message, kept_indices));
    } catch (const std::exception & error) {
      publisher_->publish(*message);
      publish_diagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        std::string("fail-closed field-preserving copy error: ") + error.what(),
        input_count, 0U);
      return;
    }

    publish_diagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      shadow_only_ ? "shadow output generated" : "active output generated",
      input_count, removed_count);
  }

  bool lookup_input_to_map(
    const sensor_msgs::msg::PointCloud2 & message,
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
      const auto timeout = tf2::durationFromSec(std::max(0.0, transform_timeout_sec_));
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
    status.name = "rm_mid360_driver_bridge/ramp_plane_filter";
    status.hardware_id = "simulation_or_map_bound_pointcloud";
    status.message = message;
    status.values = {
      key_value("shadow_only", shadow_only_ ? "true" : "false"),
      key_value("map_contract_valid", map_contract_valid_ ? "true" : "false"),
      key_value("expected_map_id", expected_map_id_),
      key_value("expected_map_revision", expected_map_revision_),
      key_value("active_map_id", active_map_id_),
      key_value("active_map_revision", active_map_revision_),
      key_value("region_count", size_text(regions_.size())),
      key_value("input_points", size_text(input_count)),
      key_value("removed_surface_points", size_text(removed_count)),
      key_value("kept_points", size_text(input_count - removed_count)),
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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RampPlaneFilterNode>());
  rclcpp::shutdown();
  return 0;
}
