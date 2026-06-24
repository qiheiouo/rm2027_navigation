#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace
{

tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  rotation.normalize();
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(rotation);
  return transform;
}

geometry_msgs::msg::Pose transformToPose(const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  pose.orientation.x = transform.getRotation().x();
  pose.orientation.y = transform.getRotation().y();
  pose.orientation.z = transform.getRotation().z();
  pose.orientation.w = transform.getRotation().w();
  return pose;
}

}  // namespace

class FakeGlobalPosePublisher : public rclcpp::Node
{
public:
  FakeGlobalPosePublisher()
  : Node("fake_global_pose_publisher")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/lio");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/localization/global_pose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_divider_ = declare_parameter<int>("publish_divider", 10);
    max_publications_ = declare_parameter<int>("max_publications", 0);
    startup_delay_sec_ = declare_parameter<double>("startup_delay_sec", 0.0);

    const double x = declare_parameter<double>("map_to_odom.x", 3.0);
    const double y = declare_parameter<double>("map_to_odom.y", -1.0);
    const double z = declare_parameter<double>("map_to_odom.z", 0.0);
    const double roll = declare_parameter<double>("map_to_odom.roll", 0.0);
    const double pitch = declare_parameter<double>("map_to_odom.pitch", 0.0);
    const double yaw = declare_parameter<double>("map_to_odom.yaw", 0.35);

    if (publish_divider_ <= 0 || max_publications_ < 0 || startup_delay_sec_ < 0.0) {
      throw std::invalid_argument(
              "publish_divider must be positive; publication limits must not be negative");
    }

    tf2::Quaternion rotation;
    rotation.setRPY(roll, pitch, yaw);
    map_to_odom_.setOrigin(tf2::Vector3(x, y, z));
    map_to_odom_.setRotation(rotation);

    auto global_pose_qos = rclcpp::QoS(1).reliable().transient_local();
    global_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic_, global_pose_qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(100).reliable(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {handleOdometry(*msg);});

    RCLCPP_WARN(
      get_logger(),
      "Test-only global pose source enabled. It publishes no TF and must not be used on hardware.");
  }

private:
  void handleOdometry(const nav_msgs::msg::Odometry & odom)
  {
    if (odom.header.frame_id != odom_frame_ || odom.child_frame_id != base_frame_) {
      return;
    }
    const double startup_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - startup_time_).count();
    if (startup_age < startup_delay_sec_) {
      return;
    }
    ++sample_count_;
    if (sample_count_ % static_cast<std::size_t>(publish_divider_) != 0U) {
      return;
    }
    if (max_publications_ > 0 && publication_count_ >= max_publications_) {
      return;
    }

    const tf2::Transform odom_to_base = poseToTransform(odom.pose.pose);
    const tf2::Transform map_to_base = map_to_odom_ * odom_to_base;

    geometry_msgs::msg::PoseWithCovarianceStamped global_pose;
    global_pose.header.stamp = odom.header.stamp;
    global_pose.header.frame_id = map_frame_;
    global_pose.pose.pose = transformToPose(map_to_base);
    global_pose.pose.covariance[0] = 0.05;
    global_pose.pose.covariance[7] = 0.05;
    global_pose.pose.covariance[14] = 0.10;
    global_pose.pose.covariance[21] = 0.10;
    global_pose.pose.covariance[28] = 0.10;
    global_pose.pose.covariance[35] = 0.05;
    global_pose_pub_->publish(global_pose);
    ++publication_count_;
  }

  std::string odom_topic_;
  std::string output_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  int publish_divider_;
  int max_publications_;
  double startup_delay_sec_;
  int publication_count_ = 0;
  std::size_t sample_count_ = 0U;
  std::chrono::steady_clock::time_point startup_time_ = std::chrono::steady_clock::now();
  tf2::Transform map_to_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeGlobalPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
