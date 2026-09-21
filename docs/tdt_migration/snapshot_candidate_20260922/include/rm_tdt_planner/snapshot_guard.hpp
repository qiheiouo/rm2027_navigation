// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#pragma once
#include "rm_tdt_planner/planner.hpp"
namespace rm_tdt_planner
{
enum class SnapshotAdmission {Unchanged, Revalidated, FootprintChanged, GeometryChanged, Unsafe};
// OFFLINE PROPOSAL ONLY. Caller holds the latest master-map mutex through
// validation/return and fixes the global frame. Candidate must be newly planned.
SnapshotAdmission validate_latest_candidate(const Grid & planned, const Grid & latest,
  const std::vector<Point> & planned_footprint, const std::vector<Point> & latest_footprint,
  const std::vector<Point> & path, const Options & options);
const char * snapshot_admission_name(SnapshotAdmission admission);
}
