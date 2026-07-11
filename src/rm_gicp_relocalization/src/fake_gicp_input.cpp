#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class FakeGicpInput : public rclcpp::Node
{
public:
  FakeGicpInput()
  : Node("fake_gicp_input")
  {
    const auto pcd_file = declare_parameter<std::string>("pcd_file", "");
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/lio/cloud_registered");
    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    if (pcd_file.empty()) {
      throw std::invalid_argument("pcd_file is required");
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, cloud) < 0) {
      throw std::runtime_error("cannot load synthetic PCD: " + pcd_file);
    }
    pcl::toROSMsg(cloud, cloud_message_);
    cloud_message_.header.frame_id = cloud_frame_;

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS());
    initial_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1).reliable().transient_local());
    cloud_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {publishCloud();});
    initial_pose_timer_ = create_wall_timer(
      std::chrono::seconds(2), [this]() {publishInitialPose();});
  }

private:
  void publishCloud()
  {
    cloud_message_.header.stamp = now();
    cloud_pub_->publish(cloud_message_);
  }

  void publishInitialPose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = map_frame_;
    pose.pose.pose.orientation.w = 1.0;
    pose.pose.covariance[0] = 0.05;
    pose.pose.covariance[7] = 0.05;
    pose.pose.covariance[35] = 0.05;
    initial_pose_pub_->publish(pose);
    initial_pose_timer_->cancel();
  }

  std::string cloud_topic_;
  std::string cloud_frame_;
  std::string map_frame_;
  sensor_msgs::msg::PointCloud2 cloud_message_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_pub_;
  rclcpp::TimerBase::SharedPtr cloud_timer_;
  rclcpp::TimerBase::SharedPtr initial_pose_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeGicpInput>());
  rclcpp::shutdown();
  return 0;
}
