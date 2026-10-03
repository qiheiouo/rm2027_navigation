#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

class FakeLioOdomPublisher : public rclcpp::Node
{
public:
  FakeLioOdomPublisher()
  : Node("fake_lio_odom_publisher")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/odometry/fast_lio_raw");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "lio_imu_link");
    motion_mode_ = declare_parameter<std::string>("motion_mode", "static");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.5);

    base_x_ = declare_parameter<double>("base_pose.x", 0.0);
    base_y_ = declare_parameter<double>("base_pose.y", 0.0);
    base_z_ = declare_parameter<double>("base_pose.z", 0.0);
    base_roll_ = declare_parameter<double>("base_pose.roll", 0.0);
    base_pitch_ = declare_parameter<double>("base_pose.pitch", 0.0);
    base_yaw_ = declare_parameter<double>("base_pose.yaw", 0.0);

    sensor_x_ = declare_parameter<double>("sensor_offset.x", 0.0);
    sensor_y_ = declare_parameter<double>("sensor_offset.y", 0.12);
    sensor_z_ = declare_parameter<double>("sensor_offset.z", 0.35);
    sensor_roll_ = declare_parameter<double>("sensor_offset.roll", 0.0);
    sensor_pitch_ = declare_parameter<double>("sensor_offset.pitch", 0.0);
    sensor_yaw_ = declare_parameter<double>("sensor_offset.yaw", 0.0);

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz must be positive. Falling back to 20 Hz.");
      publish_rate_hz_ = 20.0;
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = now();
        have_cmd_ = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        publishOdometry();
      });

    RCLCPP_WARN(
      get_logger(),
      "Phase 1 test helper: publishing fake raw LIO odometry %s -> %s on %s "
      "with motion_mode=%s. This is not a real LIO backend.",
      odom_frame_.c_str(), child_frame_.c_str(), output_topic_.c_str(), motion_mode_.c_str());
  }

private:
  static tf2::Transform makeTransform(
    double x, double y, double z, double roll, double pitch, double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(x, y, z));
    transform.setRotation(q);
    return transform;
  }

  void integrateMotion(const rclcpp::Time & stamp)
  {
    if (motion_mode_ != "cmd_vel") {
      last_publish_time_ = stamp;
      have_last_publish_time_ = true;
      return;
    }

    if (!have_last_publish_time_) {
      last_publish_time_ = stamp;
      have_last_publish_time_ = true;
      return;
    }

    const double dt = (stamp - last_publish_time_).seconds();
    last_publish_time_ = stamp;
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    geometry_msgs::msg::Twist cmd;
    if (have_cmd_ && (stamp - last_cmd_time_).seconds() <= cmd_vel_timeout_sec_) {
      cmd = last_cmd_;
    }

    const double cos_yaw = std::cos(base_yaw_);
    const double sin_yaw = std::sin(base_yaw_);
    base_x_ += (cmd.linear.x * cos_yaw - cmd.linear.y * sin_yaw) * dt;
    base_y_ += (cmd.linear.x * sin_yaw + cmd.linear.y * cos_yaw) * dt;
    base_yaw_ += cmd.angular.z * dt;
  }

  void publishOdometry()
  {
    const auto stamp = now();
    integrateMotion(stamp);

    const tf2::Transform odom_to_base = makeTransform(
      base_x_, base_y_, base_z_, base_roll_, base_pitch_, base_yaw_);
    const tf2::Transform base_to_sensor = makeTransform(
      sensor_x_, sensor_y_, sensor_z_, sensor_roll_, sensor_pitch_, sensor_yaw_);
    const tf2::Transform odom_to_sensor = odom_to_base * base_to_sensor;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose.position.x = odom_to_sensor.getOrigin().x();
    odom.pose.pose.position.y = odom_to_sensor.getOrigin().y();
    odom.pose.pose.position.z = odom_to_sensor.getOrigin().z();
    odom.pose.pose.orientation.x = odom_to_sensor.getRotation().x();
    odom.pose.pose.orientation.y = odom_to_sensor.getRotation().y();
    odom.pose.pose.orientation.z = odom_to_sensor.getRotation().z();
    odom.pose.pose.orientation.w = odom_to_sensor.getRotation().w();
    odom_pub_->publish(odom);
  }

  std::string output_topic_;
  std::string cmd_vel_topic_;
  std::string odom_frame_;
  std::string child_frame_;
  std::string motion_mode_;
  double publish_rate_hz_;
  double cmd_vel_timeout_sec_;
  double base_x_;
  double base_y_;
  double base_z_;
  double base_roll_;
  double base_pitch_;
  double base_yaw_;
  double sensor_x_;
  double sensor_y_;
  double sensor_z_;
  double sensor_roll_;
  double sensor_pitch_;
  double sensor_yaw_;
  bool have_cmd_ = false;
  bool have_last_publish_time_ = false;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_publish_time_;
  geometry_msgs::msg::Twist last_cmd_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLioOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
