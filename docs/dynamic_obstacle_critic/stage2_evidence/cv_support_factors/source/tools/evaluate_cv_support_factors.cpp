// Offline factor interventions only. No oracle is emitted as a public message.
#include "dynamic_replay_input.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace dyn = rm_dynamic_obstacle_critic;
namespace offline = dyn::offline;
using Matrix = std::array<std::vector<float>, 3>;

dyn::Risk future_circle_score(const std::vector<dyn::Pose> &poses,
    const offline::Record &context, const dyn::Array &message, uint64_t track_id,
    const std::vector<dyn::Point> &centers) {
  dyn::Risk risk;
  for (size_t k = 0; k < poses.size(); ++k) {
    const auto &pose = poses[k];
    const double time = (k+1)*context.params[0];
    const auto placed = dyn::transform(context.footprint, pose.x, pose.y, pose.yaw);
    double gap = std::numeric_limits<double>::infinity();
    for (const auto &track : message.tracks) {
      if (track.state == dyn::Obstacle::STATE_TENTATIVE) continue;
      const auto center = track.track_id == track_id ? context.transform.apply(centers[k]) :
          dyn::predict(track, context.params[1], time, context.transform);
      gap = std::min(gap, dyn::circle_clearance(placed, center, dyn::obstacle_radius(track, context.limits)));
    }
    risk.minimum_clearance = std::min(risk.minimum_clearance, gap);
    if (gap <= context.cost.safety_margin) risk.collision_time = std::min(risk.collision_time, time);
    risk.cost += dyn::penalty(gap, context.cost)*context.params[0];
  }
  return risk;
}

int main(int argc, char **argv) {
  try {
    if (argc != 4) throw std::invalid_argument("ACTUAL_INPUT FACTOR_INPUT NEW_OUTPUT_JSONL");
    if (std::filesystem::exists(argv[3])) throw std::invalid_argument("refusing to overwrite factor output");
    std::ifstream actual(argv[1], std::ios::binary);
    if (offline::read_header(actual) != 1) throw std::runtime_error("one actual context required");
    int64_t previous = -1; const auto context = offline::read_record(actual, previous);
    if (context.ordinal != 298 || context.batch != 300 || context.steps != 30)
      throw std::runtime_error("registered source298 raw batch required");
    if (actual.peek() != std::char_traits<char>::eof()) throw std::runtime_error("trailing actual context");
    const auto baseline = offline::evaluate_actual(context);
    if (baseline.exact_risk != context.batch || baseline.exact_after != context.batch)
      throw std::runtime_error("actual raw risk/after bits mismatch");
    std::ifstream input(argv[2], std::ios::binary); char magic[8]; offline::read_values(input, magic, 8);
    if (std::memcmp(magic, "RMDYNHX1", 8)) throw std::runtime_error("factor hypothesis schema");
    std::array<uint32_t, 3> grid; offline::read_values(input, grid.data(), grid.size());
    float dt; offline::read_values(input, &dt, 1);
    uint64_t track_id; uint32_t factors;
    offline::read_values(input, &track_id, 1); offline::read_values(input, &factors, 1);
    const auto ordinal = grid[0], rows = grid[1], steps = grid[2];
    if (ordinal != context.ordinal || rows != 311 || steps != context.steps || steps != 30 ||
        !std::isfinite(dt) || double(dt) != context.params[0] || factors != 4)
      throw std::runtime_error("factor source/grid/dt/count mismatch");
    const auto associated = std::find_if(context.message.tracks.begin(), context.message.tracks.end(),
        [&](const auto &track) { return track.track_id == track_id; });
    if (associated == context.message.tracks.end() || associated->state == dyn::Obstacle::STATE_TENTATIVE)
      throw std::runtime_error("factor actor id/state mismatch");
    std::array<std::array<double, 6>, 4> replacements;
    for (auto &values : replacements) {
      offline::read_values(input, values.data(), values.size());
      for (double value : values) if (!std::isfinite(value)) throw std::runtime_error("nonfinite factor state");
      if (values[4] <= 0 || values[5] <= 0) throw std::runtime_error("factor physical size invalid");
    }
    const std::array<double, 6> original{associated->position.x, associated->position.y,
        associated->velocity.x, associated->velocity.y, associated->size.x, associated->size.y};
    if (std::memcmp(original.data(), replacements[0].data(), sizeof(original)))
      throw std::runtime_error("factor baseline state is not actual");
    std::vector<dyn::Point> centers(steps);
    for (auto &center : centers) {
      offline::read_values(input, &center.x, 1); offline::read_values(input, &center.y, 1);
      if (!std::isfinite(center.x) || !std::isfinite(center.y)) throw std::runtime_error("nonfinite oracle center");
    }
    Matrix processed;
    for (auto &axis : processed) {
      axis.resize(rows*steps); offline::read_values(input, axis.data(), axis.size());
      for (float value : axis) if (!std::isfinite(value)) throw std::runtime_error("nonfinite factor path");
    }
    if (input.peek() != std::char_traits<char>::eof()) throw std::runtime_error("trailing factor input");
    std::ofstream output(argv[3]); output.exceptions(std::ios::failbit|std::ios::badbit);
    output << std::setprecision(17);
    output << "{\"schema\":1,\"scope\":\"offline factor interventions, no public/actual oracle events\","
           << "\"source_ordinal\":" << ordinal << ",\"raw_rows\":" << context.batch
           << ",\"processed_rows\":" << rows << ",\"steps\":" << steps << ",\"model_dt\":" << dt
           << ",\"actual_raw_risk_bit_exact_rows\":" << baseline.exact_risk
           << ",\"actual_raw_after_bit_exact_rows\":" << baseline.exact_after << "}\n";
    const std::array<const char *, 4> names{"actual_baseline", "source_geometry_oracle",
        "source_bracket_CV_oracle", "future_circle_oracle"};
    for (size_t factor = 0; factor < factors; ++factor) {
      auto message = context.message;
      auto track = std::find_if(message.tracks.begin(), message.tracks.end(),
          [&](const auto &value) { return value.track_id == track_id; });
      const auto &values = replacements[factor];
      track->position.x = values[0]; track->position.y = values[1];
      track->velocity.x = values[2]; track->velocity.y = values[3];
      track->size.x = values[4]; track->size.y = values[5];
      for (size_t kind = 0; kind < 2; ++kind) {
        const auto &matrix = kind == 0 ? context.rollout : processed;
        const size_t count = kind == 0 ? context.batch : rows;
        for (size_t row = 0; row < count; ++row) {
          std::vector<dyn::Pose> poses; poses.reserve(steps);
          for (size_t k = 0; k < steps; ++k)
            poses.push_back({matrix[0][row*steps+k], matrix[1][row*steps+k], matrix[2][row*steps+k]});
          const auto risk = factor == 3 ? future_circle_score(poses, context, message, track_id, centers) :
              dyn::score(poses, context.footprint, message, context.limits, context.cost,
                         context.params[1], context.params[0], context.transform);
          if (!std::isfinite(risk.cost)) throw std::runtime_error("nonfinite factor risk");
          output << "{\"factor\":\"" << names[factor] << "\",\"path_kind\":\""
                 << (kind == 0 ? "raw" : "processed") << "\",\"row\":" << row
                 << ",\"risk\":" << risk.cost << ",\"minimum_clearance\":";
          offline::number(output, risk.minimum_clearance); output << ",\"collision_time\":";
          offline::number(output, risk.collision_time); output << "}\n";
        }
      }
    }
    std::cout << "source=" << ordinal << " actual_raw_bits_exact=1 offline_factors=" << factors << '\n';
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
