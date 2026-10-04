// Copyright 2026 RM Navigation. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
namespace rm_temporal_mpc {
// Velocity has finite bounded operational inputs; remaining >= 0. The returned
// acceleration respects the actuator bound even at floating-point endpoints.
inline double stopping_acceleration(double velocity, double candidate, double remaining) {
  const double target=std::clamp(velocity+.05*candidate,-remaining,remaining);
  return std::clamp((target-velocity)/.05,-1.,1.);
}
}
