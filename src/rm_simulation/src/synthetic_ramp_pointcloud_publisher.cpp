#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct RampSpec
{
  std::string name;
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};
  double ascent_yaw_rad{0.0};
  double slope_rad{0.0};
  double length{1.0};
  double width{0.8};
};

struct ObstacleSpec
{
  bool enabled{true};
  std::string region_name;
  double longitudinal{0.5};
  double lateral{0.0};
  double length{0.2};
  double width{0.2};
  double height{0.12};
  int samples_xy{9};
  int samples_z{7};
};

double sample_coordinate(int index, int count, double minimum, double maximum)
{
  if (count <= 1) {
    return 0.5 * (minimum + maximum);
  }
  const double ratio = static_cast<double>(index) / static_cast<double>(count - 1);
  return minimum + ratio * (maximum - minimum);
}

tf2::Vector3 ramp_point(const RampSpec & ramp, double longitudinal, double lateral)
{
  const double direction_x = std::cos(ramp.ascent_yaw_rad);
  const double direction_y = std::sin(ramp.ascent_yaw_rad);
  const double lateral_x = -direction_y;
  const double lateral_y = direction_x;
  return tf2::Vector3(
    ramp.origin_x + direction_x * longitudinal + lateral_x * lateral,
    ramp.origin_y + direction_y * longitudinal + lateral_y * lateral,
    ramp.origin_z + std::tan(ramp.slope_rad) * longitudinal);
}

}  // namespace

class SyntheticRampPointcloudPublisher : public rclcpp::Node
{
public:
  SyntheticRampPointcloudPublisher()
  : Node("synthetic_ramp_pointcloud_publisher"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/simulation/ramp/points_raw");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "sim_lidar_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.10);
    max_range_ = declare_parameter<double>("max_range", 8.0);
    longitudinal_samples_ = declare_parameter<int>("longitudinal_samples", 41);
    lateral_samples_ = declare_parameter<int>("lateral_samples", 25);

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
      !std::isfinite(max_range_) || max_range_ <= 0.0 ||
      longitudinal_samples_ < 2 || lateral_samples_ < 2)
    {
      throw std::runtime_error("invalid synthetic ramp sampling configuration");
    }

    load_ramps();
    load_obstacle();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&SyntheticRampPointcloudPublisher::publish_cloud, this));

    RCLCPP_WARN(
      get_logger(),
      "simulation-only ramp PointCloud2 publisher: ramps=%zu, %s -> %s, obstacle=%s",
      ramps_.size(), map_frame_.c_str(), sensor_frame_.c_str(),
      obstacle_.enabled ? "enabled" : "disabled");
  }

private:
  void load_ramps()
  {
    const auto names = declare_parameter<std::vector<std::string>>(
      "region_names", std::vector<std::string>{});
    if (names.empty()) {
      throw std::runtime_error("region_names must contain at least one synthetic ramp");
    }
    ramps_.reserve(names.size());
    for (const auto & name : names) {
      const std::string prefix = "regions." + name + ".";
      const auto origin = declare_parameter<std::vector<double>>(
        prefix + "origin_xyz", std::vector<double>{});
      if (origin.size() != 3U) {
        throw std::runtime_error(prefix + "origin_xyz must contain x/y/z");
      }
      RampSpec ramp;
      ramp.name = name;
      ramp.origin_x = origin[0];
      ramp.origin_y = origin[1];
      ramp.origin_z = origin[2];
      ramp.ascent_yaw_rad =
        declare_parameter<double>(prefix + "ascent_yaw_deg", 0.0) * kPi / 180.0;
      ramp.slope_rad =
        declare_parameter<double>(prefix + "slope_deg", 0.0) * kPi / 180.0;
      ramp.length = declare_parameter<double>(prefix + "length", 1.0);
      ramp.width = declare_parameter<double>(prefix + "width", 0.8);
      if (!std::isfinite(ramp.origin_x) || !std::isfinite(ramp.origin_y) ||
        !std::isfinite(ramp.origin_z) || !std::isfinite(ramp.ascent_yaw_rad) ||
        !std::isfinite(ramp.slope_rad) || std::abs(ramp.slope_rad) >= 0.785398 ||
        !std::isfinite(ramp.length) || ramp.length <= 0.0 ||
        !std::isfinite(ramp.width) || ramp.width <= 0.0)
      {
        throw std::runtime_error("invalid synthetic ramp geometry for " + name);
      }
      ramps_.push_back(std::move(ramp));
    }
  }

  void load_obstacle()
  {
    obstacle_.enabled = declare_parameter<bool>("obstacle.enabled", true);
    obstacle_.region_name = declare_parameter<std::string>(
      "obstacle.region_name", ramps_.front().name);
    obstacle_.longitudinal = declare_parameter<double>("obstacle.longitudinal", 0.5);
    obstacle_.lateral = declare_parameter<double>("obstacle.lateral", 0.0);
    obstacle_.length = declare_parameter<double>("obstacle.length", 0.2);
    obstacle_.width = declare_parameter<double>("obstacle.width", 0.2);
    obstacle_.height = declare_parameter<double>("obstacle.height", 0.12);
    obstacle_.samples_xy = declare_parameter<int>("obstacle.samples_xy", 9);
    obstacle_.samples_z = declare_parameter<int>("obstacle.samples_z", 7);

    if (!obstacle_.enabled) {
      return;
    }
    const auto ramp = find_ramp(obstacle_.region_name);
    if (ramp == ramps_.end()) {
      throw std::runtime_error("obstacle.region_name does not match a synthetic ramp");
    }
    if (!std::isfinite(obstacle_.longitudinal) ||
      obstacle_.longitudinal < 0.0 || obstacle_.longitudinal > ramp->length ||
      !std::isfinite(obstacle_.lateral) ||
      std::abs(obstacle_.lateral) > 0.5 * ramp->width ||
      !std::isfinite(obstacle_.length) || obstacle_.length <= 0.0 ||
      !std::isfinite(obstacle_.width) || obstacle_.width <= 0.0 ||
      !std::isfinite(obstacle_.height) || obstacle_.height <= 0.0 ||
      obstacle_.samples_xy < 2 || obstacle_.samples_z < 2)
    {
      throw std::runtime_error("invalid synthetic ramp obstacle geometry");
    }
  }

  std::vector<RampSpec>::const_iterator find_ramp(const std::string & name) const
  {
    return std::find_if(
      ramps_.begin(), ramps_.end(),
      [&name](const RampSpec & ramp) {return ramp.name == name;});
  }

  void append_ramp_points(std::vector<tf2::Vector3> & points) const
  {
    for (const auto & ramp : ramps_) {
      for (int longitudinal_index = 0;
        longitudinal_index < longitudinal_samples_; ++longitudinal_index)
      {
        const double longitudinal = sample_coordinate(
          longitudinal_index, longitudinal_samples_, 0.0, ramp.length);
        for (int lateral_index = 0; lateral_index < lateral_samples_; ++lateral_index) {
          const double lateral = sample_coordinate(
            lateral_index, lateral_samples_, -0.5 * ramp.width, 0.5 * ramp.width);
          points.push_back(ramp_point(ramp, longitudinal, lateral));
        }
      }
    }
  }

  void append_obstacle_points(std::vector<tf2::Vector3> & points) const
  {
    if (!obstacle_.enabled) {
      return;
    }
    const auto ramp_iterator = find_ramp(obstacle_.region_name);
    if (ramp_iterator == ramps_.end()) {
      return;
    }
    const auto & ramp = *ramp_iterator;
    const double longitudinal_min = obstacle_.longitudinal - 0.5 * obstacle_.length;
    const double longitudinal_max = obstacle_.longitudinal + 0.5 * obstacle_.length;
    const double lateral_min = obstacle_.lateral - 0.5 * obstacle_.width;
    const double lateral_max = obstacle_.lateral + 0.5 * obstacle_.width;

    for (int x_index = 0; x_index < obstacle_.samples_xy; ++x_index) {
      const double longitudinal = sample_coordinate(
        x_index, obstacle_.samples_xy, longitudinal_min, longitudinal_max);
      for (int y_index = 0; y_index < obstacle_.samples_xy; ++y_index) {
        const double lateral = sample_coordinate(
          y_index, obstacle_.samples_xy, lateral_min, lateral_max);
        tf2::Vector3 point = ramp_point(ramp, longitudinal, lateral);
        point.setZ(point.z() + obstacle_.height);
        points.push_back(point);
      }
    }

    for (int z_index = 0; z_index < obstacle_.samples_z; ++z_index) {
      const double height = sample_coordinate(
        z_index, obstacle_.samples_z, 0.0, obstacle_.height);
      for (int sample_index = 0; sample_index < obstacle_.samples_xy; ++sample_index) {
        const double longitudinal = sample_coordinate(
          sample_index, obstacle_.samples_xy, longitudinal_min, longitudinal_max);
        for (const double lateral : {lateral_min, lateral_max}) {
          tf2::Vector3 point = ramp_point(ramp, longitudinal, lateral);
          point.setZ(point.z() + height);
          points.push_back(point);
        }
        const double lateral = sample_coordinate(
          sample_index, obstacle_.samples_xy, lateral_min, lateral_max);
        for (const double edge_longitudinal : {longitudinal_min, longitudinal_max}) {
          tf2::Vector3 point = ramp_point(ramp, edge_longitudinal, lateral);
          point.setZ(point.z() + height);
          points.push_back(point);
        }
      }
    }
  }

  void publish_cloud()
  {
    const auto stamp = now();
    tf2::Transform map_to_sensor;
    try {
      const auto timeout = tf2::durationFromSec(std::max(0.0, transform_timeout_sec_));
      const auto transform = tf_buffer_.lookupTransform(
        sensor_frame_, map_frame_, stamp, timeout);
      tf2::fromMsg(transform.transform, map_to_sensor);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for %s <- %s at cloud stamp: %s",
        sensor_frame_.c_str(), map_frame_.c_str(), exception.what());
      return;
    }

    std::vector<tf2::Vector3> map_points;
    map_points.reserve(
      ramps_.size() * static_cast<std::size_t>(longitudinal_samples_) *
      static_cast<std::size_t>(lateral_samples_) + 512U);
    append_ramp_points(map_points);
    append_obstacle_points(map_points);

    std::vector<tf2::Vector3> sensor_points;
    sensor_points.reserve(map_points.size());
    for (const auto & map_point : map_points) {
      const tf2::Vector3 sensor_point = map_to_sensor * map_point;
      if (sensor_point.length() <= max_range_) {
        sensor_points.push_back(sensor_point);
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = sensor_frame_;
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(sensor_points.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(sensor_points.size());
    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
    for (const auto & point : sensor_points) {
      *x_iterator = static_cast<float>(point.x());
      *y_iterator = static_cast<float>(point.y());
      *z_iterator = static_cast<float>(point.z());
      ++x_iterator;
      ++y_iterator;
      ++z_iterator;
    }
    publisher_->publish(cloud);
  }

  std::string output_topic_;
  std::string map_frame_;
  std::string sensor_frame_;
  double publish_rate_hz_{10.0};
  double transform_timeout_sec_{0.10};
  double max_range_{8.0};
  int longitudinal_samples_{41};
  int lateral_samples_{25};
  std::vector<RampSpec> ramps_;
  ObstacleSpec obstacle_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyntheticRampPointcloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
