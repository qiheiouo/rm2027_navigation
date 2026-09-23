#include "nav2_mppi_controller/cycle_trace.hpp"
#include <stdexcept>
int main() {
  auto clock=std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  geometry_msgs::msg::PoseStamped pose;pose.pose.orientation.w=1;
  geometry_msgs::msg::Twist speed;
  mppi::models::ControlSequence c;c.reset(3);c.vx.fill(0.25);c.vy.fill(-0.1);c.wz.fill(0.3);
  {
    tdt_trace::Cycle trace(pose,speed,clock);
    tdt_trace::controls("before_filter",c);
    c.vx.fill(0.75);tdt_trace::controls("after_filter",c);
    geometry_msgs::msg::TwistStamped out;out.twist.linear.x=c.vx(1);trace.output(out);
    c.vx.fill(9.0); // Snapshots must own the old values independently of caller mutation.
  }
  try {tdt_trace::Cycle trace(pose,speed,clock);tdt_trace::stage("before_exception");throw std::runtime_error("probe");}catch(const std::runtime_error &){}
  tdt_trace::shutdown();
}
