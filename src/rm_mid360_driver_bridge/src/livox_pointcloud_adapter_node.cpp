#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LivoxPointcloudAdapterNode : public rclcpp::Node
{
public:
  LivoxPointcloudAdapterNode()
  : Node("livox_pointcloud_adapter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "");
    pointcloud_output_topic_ = declare_parameter<std::string>(
      "pointcloud_output_topic", "");
    custom_output_topic_ = declare_parameter<std::string>("custom_output_topic", "");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");
    publish_custom_ = declare_parameter<bool>("publish_custom", true);
    lidar_id_ = declare_parameter<int>("lidar_id", 0);

    if (input_topic_.empty() || pointcloud_output_topic_.empty() || output_frame_id_.empty()) {
      throw std::invalid_argument(
              "input_topic, pointcloud_output_topic and output_frame_id must not be empty");
    }
    if (publish_custom_ && custom_output_topic_.empty()) {
      throw std::invalid_argument("custom_output_topic must not be empty when publish_custom=true");
    }
    if (lidar_id_ < 0 || lidar_id_ > std::numeric_limits<uint8_t>::max()) {
      throw std::invalid_argument("lidar_id must be in uint8 range");
    }

    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_output_topic_, rclcpp::SensorDataQoS());
    if (publish_custom_) {
      custom_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
        custom_output_topic_, rclcpp::SensorDataQoS());
    }
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LivoxPointcloudAdapterNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Livox pointcloud adapter: %s -> %s frame=%s custom=%s",
      input_topic_.c_str(), pointcloud_output_topic_.c_str(), output_frame_id_.c_str(),
      publish_custom_ ? custom_output_topic_.c_str() : "disabled");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 canonical = *msg;
    canonical.header.frame_id = output_frame_id_;
    pointcloud_pub_->publish(canonical);

    if (!publish_custom_) {
      return;
    }

    livox_ros_driver2::msg::CustomMsg custom;
    custom.header = canonical.header;
    const int64_t base_time_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (base_time_ns < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "drop pointcloud with negative timestamp");
      return;
    }
    custom.timebase = static_cast<uint64_t>(base_time_ns);
    custom.lidar_id = static_cast<uint8_t>(lidar_id_);
    custom.points.reserve(static_cast<std::size_t>(msg->width) * msg->height);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> intensity(*msg, "intensity");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> tag(*msg, "tag");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> line(*msg, "line");
      sensor_msgs::PointCloud2ConstIterator<double> timestamp(*msg, "timestamp");

      for (; x != x.end(); ++x, ++y, ++z, ++intensity, ++tag, ++line, ++timestamp) {
        livox_ros_driver2::msg::CustomPoint point;
        point.x = *x;
        point.y = *y;
        point.z = *z;
        point.reflectivity = static_cast<uint8_t>(std::clamp(
          std::lround(static_cast<double>(*intensity)), 0L, 255L));
        point.tag = *tag;
        point.line = *line;

        // livox_ros_driver2 PointCloud2 stores an absolute nanosecond value in
        // its FLOAT64 timestamp field. CustomMsg expects a uint32 offset from
        // header.stamp/timebase. Double precision may round a few hundred ns,
        // which is far below the sensor sampling interval.
        const double offset = *timestamp - static_cast<double>(base_time_ns);
        point.offset_time = static_cast<uint32_t>(std::clamp(
          std::llround(offset), 0LL,
          static_cast<long long>(std::numeric_limits<uint32_t>::max())));
        custom.points.push_back(point);
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "drop incompatible Livox PointCloud2: %s", error.what());
      return;
    }

    custom.point_num = static_cast<uint32_t>(custom.points.size());
    custom_pub_->publish(custom);
  }

  std::string input_topic_;
  std::string pointcloud_output_topic_;
  std::string custom_output_topic_;
  std::string output_frame_id_;
  bool publish_custom_;
  int lidar_id_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxPointcloudAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
