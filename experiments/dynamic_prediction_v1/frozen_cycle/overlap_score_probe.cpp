// Standalone geometry parity and single-thread timing probe. No ROS runtime.
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "rm_dynamic_prediction_critic/geometry.hpp"

using rm_dynamic_prediction_critic::Point;

struct Pose {double x, y, yaw;};

int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  const size_t benchmark_batch = std::stoul(argv[1]);
  const size_t repetitions = std::stoul(argv[2]);
  size_t count, steps, footprint_size;
  double age, dt, object_width, object_height, acceleration;
  Point center, velocity, visible;
  if (!(std::cin >> count >> steps >> footprint_size >> age >> dt >> object_width >>
      object_height >> acceleration >> center.x >> center.y >> velocity.x >>
      velocity.y >> visible.x >> visible.y) ||
      count == 0 || steps == 0 || footprint_size < 3 ||
      benchmark_batch == 0 || repetitions == 0) {
    throw std::invalid_argument("bad probe header");
  }
  std::vector<Point> footprint(footprint_size);
  for (auto & point : footprint) {
    if (!(std::cin >> point.x >> point.y)) throw std::invalid_argument("bad footprint");
  }
  std::vector<Pose> poses(count * steps);
  for (auto & pose : poses) {
    if (!(std::cin >> pose.x >> pose.y >> pose.yaw)) throw std::invalid_argument("bad pose");
  }
  std::vector<double> scores(benchmark_batch);
  volatile double checksum = 0;
  const auto start = std::chrono::steady_clock::now();
  for (size_t repetition = 0; repetition < repetitions; ++repetition) {
    for (size_t i = 0; i < benchmark_batch; ++i) {
      double score = 0;
      for (size_t j = 0; j < steps; ++j) {
        const auto & pose = poses[(i % count) * steps + j];
        const auto robot = rm_dynamic_prediction_critic::transform(
          footprint, pose.x, pose.y, pose.yaw);
        score += rm_dynamic_prediction_critic::uniform_center_overlap_fraction(
          robot, center, velocity, visible, {object_width, object_height},
          age, (j + 1) * dt, acceleration) / steps;
      }
      scores[i] = score;
      checksum += score;
    }
  }
  const auto stop = std::chrono::steady_clock::now();
  const double per_batch_ms =
    std::chrono::duration<double, std::milli>(stop - start).count() / repetitions;
  std::cout << std::setprecision(17) << per_batch_ms << ' ' << checksum << '\n';
  for (size_t i = 0; i < count && i < benchmark_batch; ++i) {
    std::cout << scores[i] << '\n';
  }
}
