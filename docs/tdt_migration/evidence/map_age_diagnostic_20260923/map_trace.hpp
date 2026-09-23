// Diagnostic-only append logger for isolated Nav2 1.1.20 costmap build.
#pragma once
#include <chrono>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>

namespace tdt_map_trace
{
inline uint64_t steady_ns() noexcept
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch()).count());
}
inline uint64_t fnv64(const unsigned char * data, size_t size) noexcept
{
  uint64_t h = UINT64_C(14695981039346656037);
  for (size_t i = 0; i < size; ++i) {
    h ^= data[i]; h *= UINT64_C(1099511628211);
  }
  return h;
}
inline bool enabled() noexcept {return std::getenv("TDT_COSTMAP_TRACE_DIR") != nullptr;}
inline void emit(const char * format, ...) noexcept
{
  if (!enabled()) {return;}
  static const int fd = [] () noexcept {
    const char * dir = std::getenv("TDT_COSTMAP_TRACE_DIR");
    if (!dir) {return -1;}
    char path[4096];
    int n =
      std::snprintf(path, sizeof(path), "%s/map_%ld.jsonl", dir, static_cast<long>(::getpid()));
    if (n < 0 || static_cast<size_t>(n) >= sizeof(path)) {return -1;}
    return ::open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
  } ();
  if (fd < 0) {return;}
  char line[1024];
  va_list args;
  va_start(args, format);
  int n = std::vsnprintf(line, sizeof(line) - 1, format, args);
  va_end(args);
  if (n < 0 || static_cast<size_t>(n) >= sizeof(line) - 1) {return;}
  line[n++] = '\n';
  if (::write(fd, line, static_cast<size_t>(n)) != n) {return;}
}
}  // namespace tdt_map_trace
