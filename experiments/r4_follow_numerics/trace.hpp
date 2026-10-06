#pragma once
#include <filesystem>
#include <string>
namespace r4_trace {
extern std::string mode, condition;
extern int cycle;
void save(const std::filesystem::path & output);
}
