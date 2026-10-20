#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "lddc.h"
#include "lds.h"

namespace {

using namespace std::chrono_literals;

livox_ros::Lds& TestLds() {
  // Static storage matches the real singleton's zero-initialization contract.
  static livox_ros::Lds lds(10.0, livox_ros::kSourceRawLidar);
  return lds;
}

void WaitForReturn(const std::atomic<int>& returns, int target) {
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (returns.load() < target && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  assert(returns.load() >= target);
}

void RunCycle(const std::string& name, bool inject_activity) {
  auto& lds = TestLds();
  lds.CleanRequestExit();
  std::string frame_id = "livox_frame";
  livox_ros::Lddc lddc(livox_ros::kPointCloud2Msg, 0,
                       livox_ros::kSourceRawLidar,
                       livox_ros::kOutputToRos, 10.0, frame_id);
  assert(lddc.RegisterLds(&lds) == 0);

  std::atomic<int> point_returns{0};
  std::atomic<int> imu_returns{0};
  std::thread point_thread([&] {
    do {
      lddc.DistributePointCloudData();
      point_returns.fetch_add(1);
    } while (!lds.IsRequestExit());
  });
  std::thread imu_thread([&] {
    do {
      lddc.DistributeImuData();
      imu_returns.fetch_add(1);
    } while (!lds.IsRequestExit());
  });

  std::this_thread::sleep_for(20ms);
  if (inject_activity) {
    lds.pcd_semaphore_.Signal();
    lds.imu_semaphore_.Signal();
    WaitForReturn(point_returns, 1);
    WaitForReturn(imu_returns, 1);
    // Both consumers re-enter their waits after the synthetic activity stops.
    std::this_thread::sleep_for(20ms);
  } else {
    assert(point_returns.load() == 0);
    assert(imu_returns.load() == 0);
  }

  const auto request_time = std::chrono::steady_clock::now();
  lds.RequestExit();
  point_thread.join();
  imu_thread.join();
  const auto exit_latency = std::chrono::steady_clock::now() - request_time;
  assert(exit_latency < 500ms);
  assert(lds.IsRequestExit());
  assert(point_returns.load() >= 1);
  assert(imu_returns.load() >= 1);

  std::cout << "LIFECYCLE_PASS " << name << " exit_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(exit_latency).count()
            << std::endl;
}

}  // namespace

int main() {
  RunCycle("no_input", false);
  RunCycle("input_stopped", true);
  RunCycle("active_data_exit", true);
  for (int i = 0; i < 20; ++i) {
    RunCycle("repeat_" + std::to_string(i), i % 2 == 0);
  }
  return 0;
}
