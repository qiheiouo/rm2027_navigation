#include <chrono>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

class FakeLioOdomPublisher : public rclcpp::Node
{
public:
  FakeLioOdomPublisher()
  : Node("fake_lio_odom_publisher")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/odometry/fast_lio_raw");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "lio_imu_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    x_ = declare_parameter<double>("pose.x", 0.0);
    y_ = declare_parameter<double>("pose.y", 0.12);
    z_ = declare_parameter<double>("pose.z", 0.35);
    roll_ = declare_parameter<double>("pose.roll", 0.0);
    pitch_ = declare_parameter<double>("pose.pitch", 0.0);
    yaw_ = declare_parameter<double>("pose.yaw", 0.0);

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz must be positive. Falling back to 20 Hz.");
      publish_rate_hz_ = 20.0;
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        publishOdometry();
      });

    RCLCPP_WARN(
      get_logger(),
      "Phase 1 test helper: publishing fake raw LIO odometry %s -> %s on %s.",
      odom_frame_.c_str(), child_frame_.c_str(), output_topic_.c_str());
  }

private:
  void publishOdometry()
  {
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom);
  }

  std::string output_topic_;
  std::string odom_frame_;
  std::string child_frame_;
  double publish_rate_hz_;
  double x_;
  double y_;
  double z_;
  double roll_;
  double pitch_;
  double yaw_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLioOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
