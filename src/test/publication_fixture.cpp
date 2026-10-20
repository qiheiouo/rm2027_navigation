#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

namespace {

using namespace std::chrono_literals;

std::shared_ptr<livox_ros::DriverNode> MakeDriver() {
  rclcpp::NodeOptions options;
  options.parameter_overrides(std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("xfer_format", static_cast<int>(livox_ros::kPointCloud2Msg)),
      rclcpp::Parameter("multi_topic", 0),
      rclcpp::Parameter("data_src", static_cast<int>(livox_ros::kSourceRawLidar)),
      rclcpp::Parameter("publish_freq", 100.0),
      rclcpp::Parameter("output_data_type", static_cast<int>(livox_ros::kOutputToRos)),
      rclcpp::Parameter("frame_id", "livox_frame"),
      rclcpp::Parameter("user_config_path", "/tmp/aud020-missing-config.json"),
  });
  return std::make_shared<livox_ros::DriverNode>(options);
}

struct SyntheticInput {
  livox_ros::LdsLidar* lds;
  uint32_t handle = 0x0100007f;
  uint8_t index = 0;
  uint64_t stamp = 1'000'000'000;

  SyntheticInput() : lds(livox_ros::LdsLidar::GetInstance(100.0)) {
    assert(lds->cache_index_.GetFreeIndex(livox_ros::kLivoxLidarType,
                                          handle, index) == 0);
    auto& lidar = lds->lidars_[index];
    lidar.lidar_type = livox_ros::kLivoxLidarType;
    lidar.handle = handle;
    lidar.connect_state = livox_ros::kConnectStateSampling;
  }

  void PublishOnce() {
    livox_ros::ImuData imu{};
    imu.lidar_type = livox_ros::kLivoxLidarType;
    imu.handle = handle;
    imu.time_stamp = stamp;
    imu.acc_z = 1.0f;
    imu.gyro_z = 0.01f;
    lds->StorageImuData(&imu);

    livox_ros::PointXyzlt point{};
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    point.intensity = 4.0f;
    point.offset_time = 1000;
    livox_ros::PointFrame frame{};
    frame.lidar_num = 1;
    frame.base_time[0] = stamp;
    frame.lidar_point[0].lidar_type = livox_ros::kLivoxLidarType;
    frame.lidar_point[0].handle = handle;
    frame.lidar_point[0].points_num = 1;
    frame.lidar_point[0].points = &point;
    lds->StoragePointData(&frame);
    stamp += 10'000'000;
  }
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto observer = std::make_shared<rclcpp::Node>("aud020_publication_observer");
  std::atomic<int> point_count{0};
  std::atomic<int> imu_count{0};
  auto point_subscription = observer->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        assert(msg->width == 1);
        assert(msg->header.frame_id == "livox_frame");
        point_count.fetch_add(1);
      });
  auto imu_subscription = observer->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", 10,
      [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        assert(std::isfinite(msg->linear_acceleration.z));
        assert(std::isfinite(msg->angular_velocity.z));
        imu_count.fetch_add(1);
      });

  SyntheticInput input;
  auto driver = MakeDriver();
  const auto publish_deadline = std::chrono::steady_clock::now() + 7s;
  while ((point_count.load() == 0 || imu_count.load() == 0) &&
         std::chrono::steady_clock::now() < publish_deadline) {
    input.PublishOnce();
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(20ms);
  }
  assert(point_count.load() > 0);
  assert(imu_count.load() > 0);

  // Let both workers block again after input stops, then require bounded exit.
  std::this_thread::sleep_for(50ms);
  auto exit_start = std::chrono::steady_clock::now();
  driver.reset();
  auto stopped_exit = std::chrono::steady_clock::now() - exit_start;
  assert(stopped_exit < 500ms);

  // Recreate during the interruptible startup wait.
  for (int i = 0; i < 3; ++i) {
    driver = MakeDriver();
    std::this_thread::sleep_for(20ms);
    exit_start = std::chrono::steady_clock::now();
    driver.reset();
    assert(std::chrono::steady_clock::now() - exit_start < 500ms);
  }

  // Keep synthetic SDK-like callbacks active while destruction requests exit.
  driver = MakeDriver();
  std::this_thread::sleep_for(3100ms);
  std::atomic<bool> produce{true};
  std::thread producer([&] {
    while (produce.load()) {
      input.PublishOnce();
      std::this_thread::sleep_for(2ms);
    }
  });
  std::this_thread::sleep_for(50ms);
  exit_start = std::chrono::steady_clock::now();
  driver.reset();
  auto active_exit = std::chrono::steady_clock::now() - exit_start;
  produce.store(false);
  producer.join();
  assert(active_exit < 500ms);

  std::cout << "PUBLICATION_PASS point=" << point_count.load()
            << " imu=" << imu_count.load()
            << " stopped_exit_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(stopped_exit).count()
            << " active_exit_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(active_exit).count()
            << std::endl;

  point_subscription.reset();
  imu_subscription.reset();
  observer.reset();
  rclcpp::shutdown();
  return 0;
}
