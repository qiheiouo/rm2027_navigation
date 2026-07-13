#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/chassis_mode.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class MissionSafetyMockNode : public rclcpp::Node
{
public:
  MissionSafetyMockNode()
  : Node("mission_safety_mock_node")
  {
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

    rm_competition_interfaces::msg::ChassisMode chassis;
    chassis.header.stamp = now();
    chassis.online = true;
    chassis.autonomous_enabled = true;
    chassis.emergency_stop = false;
    chassis.mode = rm_competition_interfaces::msg::ChassisMode::MODE_AUTONOMOUS;
    chassis_pub_->publish(chassis);
  }

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
