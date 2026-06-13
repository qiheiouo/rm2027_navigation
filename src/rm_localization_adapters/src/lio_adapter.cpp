#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"

class LioAdapter : public rclcpp::Node
{
public:
  LioAdapter()
  : Node("lio_adapter")
  {
    raw_odom_topic_ = declare_parameter<std::string>(
      "raw_odom_topic", "/odometry/fast_lio_raw");
    output_odom_topic_ = declare_parameter<std::string>(
      "output_odom_topic", "/odometry/lio");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    input_sensor_frame_ = declare_parameter<std::string>("input_sensor_frame", "lio_imu_link");
    gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "gimbal_yaw_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    use_zero_yaw_gimbal_placeholder_ =
      declare_parameter<bool>("use_zero_yaw_gimbal_placeholder", true);

    const double input_to_base_x =
      declare_parameter<double>("input_to_base_placeholder.x", 0.0);
    const double input_to_base_y =
      declare_parameter<double>("input_to_base_placeholder.y", 0.0);
    const double input_to_base_z =
      declare_parameter<double>("input_to_base_placeholder.z", 0.0);
    const double input_to_base_roll =
      declare_parameter<double>("input_to_base_placeholder.roll", 0.0);
    const double input_to_base_pitch =
      declare_parameter<double>("input_to_base_placeholder.pitch", 0.0);
    const double input_to_base_yaw =
      declare_parameter<double>("input_to_base_placeholder.yaw", 0.0);

    tf2::Quaternion input_to_base_q;
    input_to_base_q.setRPY(input_to_base_roll, input_to_base_pitch, input_to_base_yaw);
    input_to_base_placeholder_.setOrigin(
      tf2::Vector3(input_to_base_x, input_to_base_y, input_to_base_z));
    input_to_base_placeholder_.setRotation(input_to_base_q);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odom_topic_, 10,
      std::bind(&LioAdapter::handleRawOdometry, this, std::placeholders::_1));

    RCLCPP_WARN(
      get_logger(),
      "Phase 1 skeleton: adapting %s to %s. The gimbal transform is a "
      "zero-yaw placeholder unless calibrated and timestamped for the 2027 robot.",
      raw_odom_topic_.c_str(), output_odom_topic_.c_str());
  }

private:
  static tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
  {
    tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(
      pose.position.x,
      pose.position.y,
      pose.position.z));
    transform.setRotation(q);
    return transform;
  }

  static void transformToPose(
    const tf2::Transform & transform,
    geometry_msgs::msg::Pose & pose)
  {
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    pose.orientation.w = transform.getRotation().w();
  }

  void handleRawOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Important: do not fake base_link by only changing child_frame_id.
    // For gimbal-mounted MID360, a backend may publish odom->lio_imu_link or
    // odom->mid360_*_frame. Real hardware must compute odom->base_link using
    // the timestamped gimbal yaw and measured sensor extrinsics. This skeleton
    // only provides a zero-yaw placeholder path for early build/bag tests.
    const std::string input_child =
      msg->child_frame_id.empty() ? input_sensor_frame_ : msg->child_frame_id;

    tf2::Transform odom_to_input = poseToTransform(msg->pose.pose);
    tf2::Transform odom_to_base = odom_to_input;

    if (input_child == input_sensor_frame_) {
      if (!use_zero_yaw_gimbal_placeholder_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dynamic %s transform support is TODO. Using the placeholder transform.",
          gimbal_frame_.c_str());
      }
      odom_to_base = odom_to_input * input_to_base_placeholder_;
    } else if (input_child != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unexpected raw odometry child_frame_id '%s'. Treating it as '%s' is a TODO.",
        input_child.c_str(), input_sensor_frame_.c_str());
      odom_to_base = odom_to_input * input_to_base_placeholder_;
    }

    nav_msgs::msg::Odometry output = *msg;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    transformToPose(odom_to_base, output.pose.pose);
    odom_pub_->publish(output);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = output.header.stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = output.pose.pose.position.x;
      tf_msg.transform.translation.y = output.pose.pose.position.y;
      tf_msg.transform.translation.z = output.pose.pose.position.z;
      tf_msg.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  std::string raw_odom_topic_;
  std::string output_odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string input_sensor_frame_;
  std::string gimbal_frame_;
  bool publish_tf_;
  bool use_zero_yaw_gimbal_placeholder_;
  tf2::Transform input_to_base_placeholder_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioAdapter>());
  rclcpp::shutdown();
  return 0;
}
