// Apply Nav2's installed footprint collision checker to a frozen raw map.
// Binary input is little-endian: 7 uint32_t, 3 double, 2 doubles per
// footprint vertex, width*height bytes, then count*steps float x/y/yaw.
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace
{
template<class T>
T read(std::ifstream & input)
{
  T value{};
  input.read(reinterpret_cast<char *>(&value), sizeof(T));
  if (!input) {throw std::runtime_error("truncated frozen input");}
  return value;
}

bool collides(
  nav2_costmap_2d::Costmap2D & map,
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> & checker,
  const nav2_costmap_2d::Footprint & footprint, unsigned char threshold,
  float x, float y, float yaw)
{
  unsigned int mx, my;
  const unsigned char center = map.worldToMap(x, y, mx, my) ?
    map.getCost(mx, my) : nav2_costmap_2d::NO_INFORMATION;
  if (center < 1) {return false;}
  auto cost = center;
  if (cost >= threshold) {
    cost = static_cast<unsigned char>(checker.footprintCostAtPose(
      x, y, yaw, footprint));
  }
  return cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
         cost == nav2_costmap_2d::NO_INFORMATION;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: costmap_mask_probe input.bin output.txt\n";
    return 2;
  }
  try {
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) {throw std::runtime_error("cannot open input");}
    constexpr uint32_t magic = 0x43504D31;
    if (read<uint32_t>(input) != magic) {throw std::runtime_error("bad magic");}
    const auto width = read<uint32_t>(input);
    const auto height = read<uint32_t>(input);
    const auto count = read<uint32_t>(input);
    const auto steps = read<uint32_t>(input);
    const auto vertices = read<uint32_t>(input);
    const auto threshold = read<uint32_t>(input);
    const auto resolution = read<double>(input);
    const auto origin_x = read<double>(input);
    const auto origin_y = read<double>(input);
    if (width == 0 || height == 0 || count == 0 || steps == 0 ||
      vertices < 3 || threshold < 1 || threshold > 253 ||
      width * height > 1000000 || count * steps > 1000000)
    {throw std::runtime_error("invalid frozen dimensions");}
    nav2_costmap_2d::Footprint footprint;
    for (uint32_t i = 0; i < vertices; ++i) {
      geometry_msgs::msg::Point point;
      point.x = read<double>(input);
      point.y = read<double>(input);
      footprint.push_back(point);
    }
    nav2_costmap_2d::Costmap2D map(
      width, height, resolution, origin_x, origin_y,
      nav2_costmap_2d::NO_INFORMATION);
    input.read(reinterpret_cast<char *>(map.getCharMap()), width * height);
    if (!input) {throw std::runtime_error("truncated raw costmap");}
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
      checker(&map);
    std::ofstream output(argv[2]);
    if (!output) {throw std::runtime_error("cannot open output");}
    for (uint32_t i = 0; i < count; ++i) {
      bool hit = false;
      for (uint32_t j = 0; j < steps; ++j) {
        const float x = read<float>(input);
        const float y = read<float>(input);
        const float yaw = read<float>(input);
        if (!hit && collides(map, checker, footprint, threshold,
          x, y, yaw)) {hit = true;}
      }
      output << (hit ? '1' : '0') << '\n';
    }
    if (input.peek() != std::ifstream::traits_type::eof()) {
      throw std::runtime_error("unexpected trailing bytes");
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
