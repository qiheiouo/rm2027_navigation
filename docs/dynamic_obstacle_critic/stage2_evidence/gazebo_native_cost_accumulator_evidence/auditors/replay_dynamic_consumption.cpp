// Offline only. Recompute the existing pure CV/geometry score with the runtime
// compiler policy. No optimizer, subscriptions, TF, goals or control publisher.
#include "dynamic_replay_input.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::invalid_argument("INPUT NEW_OUTPUT_JSONL");
    if (std::filesystem::exists(argv[2])) throw std::invalid_argument("refusing to overwrite dynamic replay");
    namespace offline = rm_dynamic_obstacle_critic::offline;
    std::ifstream input(argv[1], std::ios::binary);
    const auto count = offline::read_header(input);
    std::ofstream output(argv[2]); output.exceptions(std::ios::failbit|std::ios::badbit);
    output << std::setprecision(17); bool all_exact = true; int64_t previous = -1;
    for (uint32_t record = 0; record < count; ++record) {
      const auto r = offline::read_record(input, previous);
      const auto result = offline::evaluate_actual(r);
      all_exact &= result.exact_risk == r.batch && result.exact_after == r.batch;
      output << "{\"ordinal\":" << r.ordinal << ",\"batch\":" << r.batch << ",\"steps\":" << r.steps
             << ",\"risk_double_bit_exact_rows\":" << result.exact_risk
             << ",\"after_float_bit_exact_rows\":" << result.exact_after << ",\"risk\":[";
      for (size_t row = 0; row < r.batch; ++row) { if (row) output << ','; output << result.risks[row].cost; }
      output << "],\"minimum_clearance\":[";
      for (size_t row = 0; row < r.batch; ++row) { if (row) output << ','; offline::number(output, result.risks[row].minimum_clearance); }
      output << "],\"collision_time\":[";
      for (size_t row = 0; row < r.batch; ++row) { if (row) output << ','; offline::number(output, result.risks[row].collision_time); }
      output << "]}\n";
    }
    if (input.peek() != std::char_traits<char>::eof()) throw std::runtime_error("trailing dynamic replay input");
    std::cout << "records=" << count << " all_risk_and_after_bits_exact=" << all_exact << '\n';
    return all_exact ? 0 : 2;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
