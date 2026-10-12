#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/chassis_mode.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rm_competition_interfaces/msg/localization_status.hpp"

using namespace std::chrono_literals;

class MissionSafetyMockNode : public rclcpp::Node
{
public:
  MissionSafetyMockNode()
  : Node("mission_safety_mock_node")
  {
    started_at_ = now();
    producer_id_ = "mock:" + std::to_string(std::random_device{}());
    status_pub_ = create_publisher<rm_competition_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(1).reliable());
    localization_topic_ = declare_parameter<std::string>(
      "localization_valid_topic", "/localization/global_localization_valid");
    chassis_topic_ = declare_parameter<std::string>("chassis_mode_topic", "/chassis/mode");
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    localization_pub_ = create_publisher<std_msgs::msg::Bool>(localization_topic_, qos);
    chassis_pub_ = create_publisher<rm_competition_interfaces::msg::ChassisMode>(
      chassis_topic_, qos);
    timer_ = create_wall_timer(100ms, [this]() {publish();});
    RCLCPP_WARN(
      get_logger(), "MOCK mission safety inputs enabled; they are not hardware authority");
  }

private:
  void publish()
  {
    std_msgs::msg::Bool localization;
    localization.data = true;
    localization_pub_->publish(localization);
    if (started_at_.nanoseconds() <= 0) {started_at_ = now();}
    rm_competition_interfaces::msg::LocalizationStatus status;
    status.header.stamp = now();
    status.producer_id = producer_id_;
    status.started_at = started_at_;
    status.sequence = ++sequence_;
    status.valid = true;
    status.valid_until = now() + rclcpp::Duration::from_seconds(1.0);
    status.pose.header.stamp = now();
    status.pose.header.frame_id = "map";
    status.pose.pose.pose.orientation.w = 1.0;
    status_pub_->publish(status);

    rm_competition_interfaces::msg::ChassisMode chassis;
    chassis.header.stamp = now();
    chassis.online = true;
    chassis.autonomous_enabled = true;
    chassis.emergency_stop = false;
    chassis.mode = rm_competition_interfaces::msg::ChassisMode::MODE_AUTONOMOUS;
    chassis_pub_->publish(chassis);
  }

  std::string producer_id_;
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME};
  uint64_t sequence_{0};
  rclcpp::Publisher<rm_competition_interfaces::msg::LocalizationStatus>::SharedPtr status_pub_;
  std::string localization_topic_;
  std::string chassis_topic_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localization_pub_;
  rclcpp::Publisher<rm_competition_interfaces::msg::ChassisMode>::SharedPtr chassis_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionSafetyMockNode>());
  rclcpp::shutdown();
  return 0;
}
