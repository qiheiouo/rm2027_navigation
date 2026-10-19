#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/managed_ingress_status.hpp"

// Ordinary ROS publication mistakes are filtered by the actual sample GID.
// This is not cryptographic authentication. The selected secure runtime must
// prevent writers from bypassing this node and restrict Action request writers.
class ManagedMotionIngress : public rclcpp::Node
{
public:
  ManagedMotionIngress() : Node("managed_motion_ingress")
  {
    const auto input = declare_parameter<std::string>("input_topic", "");
    const auto output = declare_parameter<std::string>("output_topic", "");
    const auto status = declare_parameter<std::string>("status_topic", "");
    const auto ids = declare_parameter<std::vector<std::string>>("allowed_gids", std::vector<std::string>{});
    if (input.empty() || output.empty() || status.empty() || input == output ||
      ids.empty() || ids.size() > 8U)
    {throw std::invalid_argument("immutable topics and 1..8 enrolled publisher GIDs required");}
    for (const auto & id : ids) {
      if (id.size() != 2U * RMW_GID_STORAGE_SIZE ||
        id.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos)
      {throw std::invalid_argument("GID must be exact hexadecimal RMW bytes");}
      Gid gid{};
      for (size_t i = 0; i < gid.size(); ++i) {
        gid[i] = static_cast<uint8_t>(std::stoul(id.substr(2U * i, 2), nullptr, 16));
      }
      if (std::all_of(gid.begin(), gid.end(), [](uint8_t v) {return v == 0;}) ||
        std::find(allowed_.begin(), allowed_.end(), gid) != allowed_.end())
      {throw std::invalid_argument("zero or duplicate GID");}
      allowed_.push_back(gid);
    }
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    instance_ = std::to_string(::getpid()) + ":" + std::to_string(ticks);
    output_ = create_publisher<geometry_msgs::msg::Twist>(output, rclcpp::QoS(1).reliable());
    status_ = create_publisher<rm_competition_interfaces::msg::ManagedIngressStatus>(
      status, rclcpp::QoS(1).reliable());
    input_ = create_subscription<geometry_msgs::msg::Twist>(input, rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m, const rclcpp::MessageInfo & info) {
        const auto begin = std::chrono::steady_clock::now();
        Gid gid{};
        std::copy_n(info.get_rmw_message_info().publisher_gid.data, gid.size(), gid.begin());
        // No graph-discovery race: an unenrolled sample is never forwarded,
        // including the first sample, or after exit/rejoin/name spoofing.
        if (std::find(allowed_.begin(), allowed_.end(), gid) == allowed_.end()) {
          RCLCPP_WARN_ONCE(get_logger(), "Dropped unenrolled sample: vy=%.3f angular.x=%.3f",
            m->linear.y, m->angular.x);
          ++rejected_; return;
        }
        const std::array<double, 6> values{
          m->linear.x, m->linear.y, m->linear.z, m->angular.x, m->angular.y, m->angular.z};
        if (!active_ || !std::all_of(values.begin(), values.end(),
          [](double v) {return std::isfinite(v);}))
        {active_ = false; ++rejected_; return;}
        output_->publish(*m);
        accepted_at_ = begin;
        ++accepted_;
        max_callback_ns_ = std::max(max_callback_ns_, static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - begin).count()));
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      rm_competition_interfaces::msg::ManagedIngressStatus m;
      m.header.stamp = now(); m.instance_id = instance_; m.sequence = ++sequence_;
      m.active = active_; m.accepted_count = accepted_; m.rejected_count = rejected_;
      m.last_accepted_age_sec = accepted_ == 0 ? 1e6 :
        std::chrono::duration<double>(std::chrono::steady_clock::now() - accepted_at_).count();
      m.max_callback_ns = max_callback_ns_; status_->publish(m);
    });
  }
private:
  using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
  std::vector<Gid> allowed_;
  std::string instance_;
  bool active_{true};
  uint64_t sequence_{0}, accepted_{0}, rejected_{0}, max_callback_ns_{0};
  std::chrono::steady_clock::time_point accepted_at_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<rm_competition_interfaces::msg::ManagedIngressStatus>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManagedMotionIngress>());
  rclcpp::shutdown();
  return 0;
}
