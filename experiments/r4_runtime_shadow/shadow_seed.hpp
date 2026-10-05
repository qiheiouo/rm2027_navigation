#pragma once

#include <optional>
#include <rm_r4_prediction_consumption/follow.hpp>

namespace r4_runtime_shadow
{
// Counterfactual diagnostic history only. Context invalidation resets optimizer
// warm state; it does not erase an otherwise fresh previous virtual command.
struct SeedDecision
{
  rm_r4_prediction_consumption::Vec2 velocity;
  int64_t stamp_ns;
  bool reset_seed, reset_warm;
};
inline SeedDecision decide_seed(
  const std::optional<rm_r4_prediction_consumption::Vec2> & previous,
  int64_t previous_epoch, int64_t epoch, bool context_reset)
{
  const bool available = previous && previous_epoch > 0 && previous_epoch < epoch &&
    epoch - previous_epoch <= 100000000;
  return {available ? *previous : rm_r4_prediction_consumption::Vec2{},
    available ? previous_epoch : epoch, !available, !available || context_reset};
}
}  // namespace r4_runtime_shadow
