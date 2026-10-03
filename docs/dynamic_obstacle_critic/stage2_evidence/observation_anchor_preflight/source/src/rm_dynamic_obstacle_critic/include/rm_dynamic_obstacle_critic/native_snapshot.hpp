#pragma once

#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rm_dynamic_obstacle_critic/evidence_json.hpp"
#include <cstring>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

namespace rm_dynamic_obstacle_critic {
// Evidence only. No optimizer member access, subscriptions or control writes.
inline void write_native_snapshot(
    const std::filesystem::path &directory, size_t ordinal, double capture_stamp,
    const mppi::CriticData &data, nav2_costmap_2d::Costmap2DROS &costmap_ros) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  const uint32_t endian = 1;
  if (*reinterpret_cast<const uint8_t *>(&endian) != 1)
    throw std::runtime_error("snapshot requires little-endian IEEE float32");
  const auto &tr = data.trajectories;
  const auto batch = tr.x.shape(0), steps = tr.x.shape(1);
  if (!batch || batch > 512 || !steps || steps > 64 ||
      !std::isfinite(data.model_dt) || data.model_dt <= 0 ||
      tr.x.shape() != tr.y.shape() || tr.x.shape() != tr.yaws.shape() ||
      data.costs.size() != batch || data.path.x.size() > 2000 ||
      data.path.x.shape() != data.path.y.shape() || data.path.x.shape() != data.path.yaws.shape())
    throw std::runtime_error("snapshot tensor/grid budget or shape mismatch");
  const auto &state = data.state;
  for (const auto *tensor : {&state.vx, &state.vy, &state.wz, &state.cvx, &state.cvy, &state.cwz})
    if (tensor->shape() != tr.x.shape()) throw std::runtime_error("snapshot state shape mismatch");
  const auto *grid = costmap_ros.getCostmap();
  const size_t cells = size_t(grid->getSizeInCellsX()) * grid->getSizeInCellsY();
  if (!cells || cells > 65536 || !std::isfinite(grid->getResolution()) || grid->getResolution() <= 0 ||
      !std::isfinite(grid->getOriginX()) || !std::isfinite(grid->getOriginY()))
    throw std::runtime_error("snapshot map cell budget or geometry");
  const auto &pose = state.pose.pose;
  const auto &speed = state.speed;
  for (double v : {capture_stamp, pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                  speed.linear.x, speed.linear.y, speed.linear.z,
                  speed.angular.x, speed.angular.y, speed.angular.z})
    if (!std::isfinite(v)) throw std::runtime_error("snapshot nonfinite metadata");
  std::ostringstream json;
  json.imbue(std::locale::classic());
  json << std::setprecision(17) << "{\"schema\":1,\"ordinal\":" << ordinal
       << ",\"capture_stamp\":" << capture_stamp << ",\"pose_stamp_sec\":" << state.pose.header.stamp.sec
       << ",\"pose_stamp_nanosec\":" << state.pose.header.stamp.nanosec
       << ",\"pose_frame\":" << json_string(state.pose.header.frame_id)
       << ",\"evaluation_frame\":" << json_string(costmap_ros.getGlobalFrameID())
       << ",\"base_frame\":" << json_string(costmap_ros.getBaseFrameID())
       << ",\"model_dt\":" << data.model_dt << ",\"fail_flag\":" << (data.fail_flag ? "true" : "false")
       << ",\"pose\":[" << pose.position.x << ',' << pose.position.y << ',' << pose.position.z
       << ',' << pose.orientation.x << ',' << pose.orientation.y << ',' << pose.orientation.z << ',' << pose.orientation.w
       << "],\"speed\":[" << speed.linear.x << ',' << speed.linear.y << ',' << speed.linear.z
       << ',' << speed.angular.x << ',' << speed.angular.y << ',' << speed.angular.z
       << "],\"map\":{\"size\":[" << grid->getSizeInCellsX() << ',' << grid->getSizeInCellsY()
       << "],\"origin\":[" << grid->getOriginX() << ',' << grid->getOriginY()
       << "],\"resolution\":" << grid->getResolution() << "},\"padded_footprint\":[";
  bool comma = false;
  for (const auto &point : costmap_ros.getRobotFootprint()) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) throw std::runtime_error("nonfinite footprint");
    if (comma) json << ',';
    json << '[' << point.x << ',' << point.y << ']'; comma = true;
  }
  json << "],\"blocks\":[";
  std::vector<uint8_t> payload;
  payload.reserve(9 * batch * steps * sizeof(float) + cells + 25000);
  comma = false;
  auto block = [&](const std::string &name, const void *bytes, size_t size,
                   const std::string &dtype, const auto &shape) {
    if (comma) json << ',';
    json << "{\"name\":" << json_string(name) << ",\"dtype\":" << json_string(dtype)
         << ",\"offset\":" << payload.size() << ",\"bytes\":" << size << ",\"shape\":[";
    bool dim_comma = false;
    for (size_t d : shape) { if (dim_comma) json << ','; json << d; dim_comma = true; }
    json << "]}"; comma = true;
    const auto *start = static_cast<const uint8_t *>(bytes);
    if (size) payload.insert(payload.end(), start, start + size);
  };
  auto floats = [&](const std::string &name, const auto &tensor) {
    // xtensor in the pinned native API is contiguous row-major. Refuse any
    // alternate layout rather than mislabelling its bytes.
    if (!tensor.is_contiguous() || tensor.layout() != xt::layout_type::row_major)
      throw std::runtime_error("snapshot requires contiguous row-major tensor");
    block(name, tensor.data(), tensor.size()*sizeof(float), "<f4", tensor.shape());
  };
  floats("x",tr.x); floats("y",tr.y); floats("yaw",tr.yaws);
  floats("vx",state.vx); floats("vy",state.vy); floats("wz",state.wz);
  floats("cvx",state.cvx); floats("cvy",state.cvy); floats("cwz",state.cwz);
  floats("critic_costs_before_control_regularization",data.costs);
  floats("path_x",data.path.x); floats("path_y",data.path.y); floats("path_yaw",data.path.yaws);
  block("raw_costmap",grid->getCharMap(),cells,"|u1",
        std::vector<size_t>{grid->getSizeInCellsY(),grid->getSizeInCellsX()});
  json << "],\"payload_bytes\":" << payload.size()
       << ",\"stage\":\"after listed critics; before gamma/softmax/aggregation/SG\","
       << "\"unavailable\":[\"control_sequence_mean\",\"SG_history\",\"exact_dynamic_consumer_input\",\"returned_control\"]}\n";
  const auto prefix = directory / ("cycle_" + std::to_string(ordinal));
  const auto binary = prefix.string() + ".bin", metadata = prefix.string() + ".json";
  if (std::filesystem::exists(binary) || std::filesystem::exists(metadata))
    throw std::runtime_error("refusing to overwrite native snapshot");
  std::ofstream data_file(binary + ".tmp",std::ios::binary);
  data_file.exceptions(std::ios::failbit | std::ios::badbit);
  data_file.write(reinterpret_cast<const char *>(payload.data()),payload.size()); data_file.close();
  std::ofstream meta_file(metadata + ".tmp");
  meta_file.exceptions(std::ios::failbit | std::ios::badbit);
  meta_file << json.str(); meta_file.close();
  std::filesystem::rename(binary + ".tmp",binary);
  std::filesystem::rename(metadata + ".tmp",metadata); // Commit metadata last.
}
} // namespace rm_dynamic_obstacle_critic
