// Pure offline hypothesis evaluation. Never impersonates an actual score record.
#include "dynamic_replay_input.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  try {
    if (argc != 4) throw std::invalid_argument("ACTUAL_INPUT PROCESSED_PATHS NEW_OUTPUT_JSONL");
    if (std::filesystem::exists(argv[3])) throw std::invalid_argument("refusing to overwrite processed CV output");
    namespace dyn = rm_dynamic_obstacle_critic;
    namespace offline = dyn::offline;
    std::ifstream input(argv[1], std::ios::binary);
    if (offline::read_header(input) != 1) throw std::runtime_error("one exact consumed context required");
    int64_t previous = -1; const auto context = offline::read_record(input, previous);
    if (input.peek() != std::char_traits<char>::eof()) throw std::runtime_error("trailing actual context");
    const auto actual = offline::evaluate_actual(context);
    if (actual.exact_risk != context.batch || actual.exact_after != context.batch)
      throw std::runtime_error("actual raw risk/after bits mismatch");
    std::ifstream paths(argv[2], std::ios::binary); char magic[8]; offline::read_values(paths, magic, 8);
    if (std::memcmp(magic, "RMDYNPX1", 8)) throw std::runtime_error("processed path schema");
    std::array<uint32_t, 3> grid; offline::read_values(paths, grid.data(), grid.size());
    float dt; offline::read_values(paths, &dt, 1);
    const auto ordinal = grid[0], rows = grid[1], steps = grid[2];
    if (ordinal != context.ordinal || rows < 1 || rows > 512 || steps != context.steps ||
        !std::isfinite(dt) || double(dt) != context.params[0])
      throw std::runtime_error("processed source ordinal/grid/dt mismatch");
    std::array<std::vector<float>, 3> matrix;
    for (auto &axis : matrix) {
      axis.resize(rows*steps); offline::read_values(paths, axis.data(), axis.size());
      for (float value : axis) if (!std::isfinite(value)) throw std::runtime_error("nonfinite processed path");
    }
    if (paths.peek() != std::char_traits<char>::eof()) throw std::runtime_error("trailing processed path input");
    std::ofstream output(argv[3]); output.exceptions(std::ios::failbit|std::ios::badbit);
    output << std::setprecision(17);
    output << "{\"schema\":1,\"scope\":\"offline processed CV hypotheses, not actual critic consumption\","
           << "\"source_ordinal\":" << ordinal << ",\"rows\":" << rows << ",\"steps\":" << steps
           << ",\"model_dt\":" << dt << ",\"actual_raw_risk_bit_exact_rows\":" << actual.exact_risk
           << ",\"actual_raw_after_bit_exact_rows\":" << actual.exact_after << "}\n";
    for (size_t row = 0; row < rows; ++row) {
      std::vector<dyn::Pose> poses; poses.reserve(steps);
      for (size_t k = 0; k < steps; ++k)
        poses.push_back({matrix[0][row*steps+k], matrix[1][row*steps+k], matrix[2][row*steps+k]});
      const auto risk = dyn::score(poses, context.footprint, context.message, context.limits,
                                  context.cost, context.params[1], context.params[0], context.transform);
      if (!std::isfinite(risk.cost)) throw std::runtime_error("nonfinite processed risk");
      output << "{\"row\":" << row << ",\"risk\":" << risk.cost << ",\"minimum_clearance\":";
      offline::number(output, risk.minimum_clearance); output << ",\"collision_time\":";
      offline::number(output, risk.collision_time); output << "}\n";
    }
    std::cout << "source=" << ordinal << " actual_raw_bits_exact=1 processed_hypothesis_rows=" << rows << '\n';
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
