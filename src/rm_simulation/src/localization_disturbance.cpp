#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class LocalizationDisturbance : public rclcpp::Node
{
public:
  LocalizationDisturbance()
  : Node("localization_disturbance")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/simulation/ground_truth/odom");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/simulation/localization/odom");
    reference_yaw_ = declare_parameter<double>("reference_yaw", 0.0);
    lateral_noise_std_m_ =
      declare_parameter<double>("lateral_noise_std_m", 0.0);
    yaw_noise_std_rad_ =
      declare_parameter<double>("yaw_noise_std_rad", 0.0);
    delay_sec_ = declare_parameter<double>("delay_sec", 0.0);
    lateral_drift_amplitude_m_ =
      declare_parameter<double>("lateral_drift_amplitude_m", 0.0);
    yaw_drift_amplitude_rad_ =
      declare_parameter<double>("yaw_drift_amplitude_rad", 0.0);
    drift_frequency_hz_ =
      declare_parameter<double>("drift_frequency_hz", 0.0);
    const int64_t random_seed =
      declare_parameter<int64_t>("random_seed", 20270728);

    validateParameters();
    generator_.seed(static_cast<std::mt19937::result_type>(random_seed));
    lateral_noise_distribution_ =
      std::normal_distribution<double>(0.0, lateral_noise_std_m_);
    yaw_noise_distribution_ =
      std::normal_distribution<double>(0.0, yaw_noise_std_rad_);

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_,
      10,
      std::bind(
        &LocalizationDisturbance::handleOdometry,
        this,
        std::placeholders::_1));
    timer_ = create_wall_timer(
      5ms, std::bind(&LocalizationDisturbance::publishReady, this));

    RCLCPP_WARN(
      get_logger(),
      "Simulation-only localization disturbance: lateral_noise=%.4f m, "
      "yaw_noise=%.3f deg, delay=%.3f s, lateral_drift=%.4f m, "
      "yaw_drift=%.3f deg at %.3f Hz. No TF is published.",
      lateral_noise_std_m_,
      radiansToDegrees(yaw_noise_std_rad_),
      delay_sec_,
      lateral_drift_amplitude_m_,
      radiansToDegrees(yaw_drift_amplitude_rad_),
      drift_frequency_hz_);
  }

private:
  struct PendingOdometry
  {
    nav_msgs::msg::Odometry message;
    rclcpp::Time release_time;
  };

  static double radiansToDegrees(double radians)
  {
    return radians * 180.0 / std::acos(-1.0);
  }

  void validateParameters() const
  {
    const bool invalid =
      !std::isfinite(reference_yaw_) ||
      !std::isfinite(lateral_noise_std_m_) ||
      !std::isfinite(yaw_noise_std_rad_) ||
      !std::isfinite(delay_sec_) ||
      !std::isfinite(lateral_drift_amplitude_m_) ||
      !std::isfinite(yaw_drift_amplitude_rad_) ||
      !std::isfinite(drift_frequency_hz_) ||
      lateral_noise_std_m_ < 0.0 ||
      yaw_noise_std_rad_ < 0.0 ||
      delay_sec_ < 0.0 ||
      delay_sec_ > 2.0 ||
      lateral_drift_amplitude_m_ < 0.0 ||
      yaw_drift_amplitude_rad_ < 0.0 ||
      drift_frequency_hz_ < 0.0;
    if (invalid) {
      throw std::invalid_argument(
              "localization disturbance parameters must be finite, "
              "non-negative, and delay_sec must not exceed 2.0");
    }
  }

  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    const rclcpp::Time stamp(input->header.stamp, get_clock()->get_clock_type());
    if (!has_first_stamp_ || stamp < first_stamp_) {
      first_stamp_ = stamp;
      has_first_stamp_ = true;
      pending_.clear();
    }
    const double elapsed = std::max(0.0, (stamp - first_stamp_).seconds());
    const double drift_phase =
      2.0 * std::acos(-1.0) * drift_frequency_hz_ * elapsed;
    const double lateral_error =
      lateral_noise_distribution_(generator_) +
      lateral_drift_amplitude_m_ * std::sin(drift_phase);
    const double yaw_error =
      yaw_noise_distribution_(generator_) +
      yaw_drift_amplitude_rad_ * std::sin(drift_phase);

    nav_msgs::msg::Odometry output = *input;
    output.pose.pose.position.x +=
      -std::sin(reference_yaw_) * lateral_error;
    output.pose.pose.position.y +=
      std::cos(reference_yaw_) * lateral_error;

    tf2::Quaternion orientation(
      output.pose.pose.orientation.x,
      output.pose.pose.orientation.y,
      output.pose.pose.orientation.z,
      output.pose.pose.orientation.w);
    orientation.normalize();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    orientation.setRPY(roll, pitch, yaw + yaw_error);
    output.pose.pose.orientation.x = orientation.x();
    output.pose.pose.orientation.y = orientation.y();
    output.pose.pose.orientation.z = orientation.z();
    output.pose.pose.orientation.w = orientation.w();

    if (delay_sec_ <= 0.0) {
      publisher_->publish(output);
      return;
    }
    pending_.push_back(
      PendingOdometry{
        std::move(output),
        stamp + rclcpp::Duration::from_seconds(delay_sec_)});
  }

  void publishReady()
  {
    const auto current_time = now();
    while (!pending_.empty() && pending_.front().release_time <= current_time) {
      publisher_->publish(pending_.front().message);
      pending_.pop_front();
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double reference_yaw_;
  double lateral_noise_std_m_;
  double yaw_noise_std_rad_;
  double delay_sec_;
  double lateral_drift_amplitude_m_;
  double yaw_drift_amplitude_rad_;
  double drift_frequency_hz_;

  bool has_first_stamp_{false};
  rclcpp::Time first_stamp_{0, 0, RCL_ROS_TIME};
  std::deque<PendingOdometry> pending_;
  std::mt19937 generator_;
  std::normal_distribution<double> lateral_noise_distribution_;
  std::normal_distribution<double> yaw_noise_distribution_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationDisturbance>());
  rclcpp::shutdown();
  return 0;
}
