// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#include "rm_tdt_planner/snapshot_guard.hpp"
#include <algorithm>
#include <cmath>
namespace rm_tdt_planner
{
SnapshotAdmission validate_latest_candidate(const Grid & planned, const Grid & latest,
  const std::vector<Point> & before, const std::vector<Point> & after,
  const std::vector<Point> & path, const Options & options)
{
  if (before.size() < 3 || before.size() != after.size()) {return SnapshotAdmission::FootprintChanged;}
  double radius = 0.0;
  for (size_t i = 0; i < before.size(); ++i) {
    if (!std::isfinite(before[i].x) || !std::isfinite(before[i].y) ||
      before[i].x != after[i].x || before[i].y != after[i].y)
    {return SnapshotAdmission::FootprintChanged;}
    radius = std::max(radius, std::hypot(before[i].x, before[i].y));
  }
  if (radius != options.radius) {return SnapshotAdmission::FootprintChanged;}
  if (planned.width != latest.width || planned.height != latest.height ||
    planned.resolution != latest.resolution || planned.cost_interpretation != latest.cost_interpretation)
  {return SnapshotAdmission::GeometryChanged;}
  // Exactly the existing continuous checker and existing physical/numerical margins.
  if (!collision_free(latest, path, options)) {return SnapshotAdmission::Unsafe;}
  return planned.origin_x == latest.origin_x && planned.origin_y == latest.origin_y &&
         planned.costs == latest.costs ? SnapshotAdmission::Unchanged : SnapshotAdmission::Revalidated;
}
const char * snapshot_admission_name(SnapshotAdmission a)
{
  switch (a) {
    case SnapshotAdmission::Unchanged: return "unchanged";
    case SnapshotAdmission::Revalidated: return "revalidated";
    case SnapshotAdmission::FootprintChanged: return "footprint_changed";
    case SnapshotAdmission::GeometryChanged: return "geometry_changed";
    case SnapshotAdmission::Unsafe: return "unsafe_latest_path";
  }
  return "invalid_admission";
}
}
