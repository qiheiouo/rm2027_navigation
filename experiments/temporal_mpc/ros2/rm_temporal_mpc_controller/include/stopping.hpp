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
// Re-anchoring also changes the proposal's saturation endpoints. Project to the
// intersection of fixed velocity and stopping bounds, then independently check
// the complete repaired trajectory. This never relaxes a velocity constraint.
inline double bounded_stopping_acceleration(double velocity, double candidate,
                                            double remaining, double lower, double upper) {
  const double target=std::clamp(velocity+.05*candidate,
                                std::max(lower,-remaining),std::min(upper,remaining));
  return std::clamp((target-velocity)/.05,-1.,1.);
}
}
