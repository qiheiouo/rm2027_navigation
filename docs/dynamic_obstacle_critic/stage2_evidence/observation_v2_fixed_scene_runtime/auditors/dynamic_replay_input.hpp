#pragma once
// Private offline reader only. Uses the original public message, no ROS APIs.
#include "rm_dynamic_obstacle_critic/model.hpp"
#include <array>
#include <cstring>
#include <istream>
#include <ostream>

namespace rm_dynamic_obstacle_critic::offline {
template<typename T> void read_values(std::istream &input, T *data, size_t count) {
  input.read(reinterpret_cast<char *>(data), count * sizeof(T));
  if (!input) throw std::runtime_error("truncated dynamic replay");
}
inline void number(std::ostream &out, double value) {
  if (std::isfinite(value)) out << value; else out << "null";
}
inline uint32_t read_header(std::istream &input) {
  static_assert(sizeof(float) == 4 && sizeof(double) == 8);
  const uint32_t endian = 1;
  if (*reinterpret_cast<const uint8_t *>(&endian) != 1)
    throw std::runtime_error("little endian required");
  char magic[8]; read_values(input, magic, 8);
#ifdef RM_OBSERVATION_DIAMETER_REPLAY
  if (std::memcmp(magic, "RMDYNRP2", 8)) throw std::runtime_error("observation replay schema");
#else
  if (std::memcmp(magic, "RMDYNRP1", 8)) throw std::runtime_error("dynamic replay schema");
#endif
  uint32_t count; read_values(input, &count, 1);
  if (count < 1 || count > 1000) throw std::runtime_error("dynamic replay record budget");
  return count;
}
struct Record {
  uint32_t ordinal, batch, steps;
  std::array<double, 10> params;
  Rigid2D transform;
  Limits limits;
  CostParameters cost;
  std::vector<Point> footprint;
  Array message;
  std::array<std::vector<float>, 3> rollout;
  std::vector<float> before, after;
  std::vector<double> actual_risk;
};
inline Record read_record(std::istream &input, int64_t &previous) {
  std::array<uint32_t, 5> grid; read_values(input, grid.data(), grid.size());
  Record r;
  r.ordinal = grid[0]; r.batch = grid[1]; r.steps = grid[2];
  const auto tracks = grid[3], points = grid[4];
  if (r.ordinal >= 1000 || r.ordinal <= previous || r.batch < 1 || r.batch > 512 ||
      r.steps < 1 || r.steps > 64 || tracks > 64 || points < 3 || points > 128)
    throw std::runtime_error("dynamic replay bounded grid");
  previous = r.ordinal; read_values(input, r.params.data(), r.params.size());
  for (double value : r.params)
    if (!std::isfinite(value)) throw std::runtime_error("nonfinite dynamic replay parameter");
  r.transform = {r.params[2], r.params[3], r.params[4]}; r.limits.minimum_radius = r.params[5];
  r.cost = {r.params[6], r.params[7], r.params[8], r.params[9]};
#ifdef RM_OBSERVATION_DIAMETER_REPLAY
  uint32_t mode, prior_size;
  read_values(input, &mode, 1);
  read_values(input, &r.limits.nominal_obstacle_diameter, 1);
  read_values(input, &prior_size, 1);
  if (mode > 1 || prior_size > 4096) throw std::runtime_error("observation replay geometry budget");
  r.limits.input_schema = Array::SCHEMA_OBSERVATION_ANCHOR;
  r.message.schema = r.limits.input_schema;
  r.limits.obstacle_radius_mode = mode ? "nominal_diameter" : "visible_extent";
  r.limits.geometry_prior_source.resize(prior_size);
  read_values(input, r.limits.geometry_prior_source.data(), prior_size);
  if (!valid_radius_model(r.limits)) throw std::runtime_error("observation replay radius model");
#endif
  if (r.params[0] <= 0 || r.params[1] < 0 || r.params[5] <= 0 || r.params[6] < 0 ||
      r.params[7] <= r.params[8] || r.params[8] <= 0 || r.params[9] <= 0)
    throw std::runtime_error("dynamic replay parameter range");
  r.footprint.resize(points);
  for (auto &point : r.footprint) { read_values(input, &point.x, 1); read_values(input, &point.y, 1); }
  validate_footprint(r.footprint); r.message.tracks.resize(tracks);
  for (auto &track : r.message.tracks) {
    read_values(input, &track.track_id, 1); read_values(input, &track.state, 1);
    for (double *value : {&track.position.x, &track.position.y, &track.velocity.x,
        &track.velocity.y, &track.size.x, &track.size.y}) {
      read_values(input, value, 1);
      if (!std::isfinite(*value)) throw std::runtime_error("nonfinite dynamic replay track");
    }
    if (track.state < 1 || track.state > 3 || track.size.x <= 0 || track.size.y <= 0)
      throw std::runtime_error("dynamic replay track state/size");
  }
  for (auto &axis : r.rollout) { axis.resize(r.batch*r.steps); read_values(input, axis.data(), axis.size()); }
  r.before.resize(r.batch); r.after.resize(r.batch); r.actual_risk.resize(r.batch);
  read_values(input, r.before.data(), r.batch); read_values(input, r.actual_risk.data(), r.batch);
  read_values(input, r.after.data(), r.batch);
  return r;
}
struct Evaluation {
  size_t exact_risk{0}, exact_after{0};
  std::vector<Risk> risks;
};
inline Evaluation evaluate_actual(const Record &r) {
  Evaluation result;
  for (size_t row = 0; row < r.batch; ++row) {
    std::vector<Pose> poses; poses.reserve(r.steps);
    for (size_t k = 0; k < r.steps; ++k) {
      for (const auto &axis : r.rollout)
        if (!std::isfinite(axis[row*r.steps+k])) throw std::runtime_error("nonfinite dynamic replay rollout");
      poses.push_back({r.rollout[0][row*r.steps+k], r.rollout[1][row*r.steps+k], r.rollout[2][row*r.steps+k]});
    }
    auto risk = score(poses, r.footprint, r.message, r.limits, r.cost, r.params[1], r.params[0], r.transform);
    const float added = static_cast<float>(double(r.before[row])+risk.cost);
    if (!std::isfinite(r.before[row]) || !std::isfinite(r.after[row]) ||
        !std::isfinite(r.actual_risk[row]) || !std::isfinite(risk.cost))
      throw std::runtime_error("nonfinite dynamic replay cost");
    result.exact_risk += std::memcmp(&risk.cost, &r.actual_risk[row], 8) == 0;
    result.exact_after += std::memcmp(&added, &r.after[row], 4) == 0;
    result.risks.push_back(risk);
  }
  return result;
}
}  // namespace rm_dynamic_obstacle_critic::offline
