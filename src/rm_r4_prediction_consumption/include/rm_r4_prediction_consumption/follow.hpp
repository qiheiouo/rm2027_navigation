#pragma once

#include <array>
#include <chrono>
#include <memory>
#include "rm_r4_prediction_consumption/consumption.hpp"

namespace rm_r4_prediction_consumption
{
using FollowClock = std::chrono::steady_clock;

// Explicit actual limits. command_rate is a conservative magnitude bounded by
// both acceleration and deceleration limits; this is a target rate, not a plant.
struct FollowLimits
{
  Vec2 lower, upper, command_rate;
  double cruise{}, progress_upper{};
  std::string digest() const;
};

// These identities are supplied by the existing host; they are not grants.
struct FollowIdentity
{
  std::string host_instance, execution_id, base_frame;
  uint64_t authority_epoch{}, cycle_sequence{};
};
struct FollowState
{
  Vec2 position, measured_body_velocity, last_applied_body_velocity;
  double yaw{}, measured_yaw_rate{}, last_applied_yaw_rate{};
  int64_t pose_stamp_ns{}, velocity_stamp_ns{}, tf_stamp_ns{}, applied_stamp_ns{};
  std::string frame, body_frame;
};

// Host copies state/TF/applied evidence under its existing synchronization before
// passing this owned value. Prediction and route are already immutable values.
struct FollowInput
{
  PredictionSnapshot prediction;
  PreparedCorridor route;
  BodyPolicy body;
  FollowLimits limits;
  FollowIdentity identity;
  FollowState state;
  double progress{};
  FollowClock::time_point acquired;
  bool reset_warm{};
};
struct FollowControl {Vec2 body_velocity; double progress_rate{};};
// Shared original input validation/identity; no solver call or new acquisition.
std::string fingerprint_follow_input(const FollowInput & input);
struct FollowStage {Vec2 position, body_velocity; double yaw{}, progress{};};
struct FollowProposal
{
  FollowIdentity identity;
  std::string input_digest, receipt_digest, path_digest, map_digest, limits_digest;
  int64_t epoch_ns{};
  FollowClock::time_point acquired, source_deadline;
  Vec2 body_velocity;
  double yaw_rate{};
  std::array<FollowControl, 15> controls;
  std::array<FollowStage, 31> stages;
};
struct FollowResult
{
  std::optional<FollowProposal> proposal;
  std::string reason, solver_status{"not_run"};
  int iterations{};
  double solver_seconds{}, elapsed_seconds{}, nominal_dynamic_cost{}, solved_dynamic_cost{};
  std::optional<double> minimum_constraint_slack;
  bool used_warm{};
};

// One synchronous local QP. Caller serializes calls; no worker or ROS/output API.
// Budgets reject late results but do not preempt assembly, factorization or solve.
class FollowSolver
{
public:
  FollowSolver();
  ~FollowSolver();
  FollowSolver(const FollowSolver &) = delete;
  FollowSolver & operator=(const FollowSolver &) = delete;
  FollowResult solve(FollowInput input);
  void reset();  // Existing host invokes this on lifecycle/input acquisition failure.
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
// Research value adapter: source evidence remains raw. Future command yaw rate
// is intrinsically zero, so a nonzero last-applied yaw command is unavailable.
// Distinct input/result wrappers cannot be implicitly passed to a legacy host.
struct AlignedFollowInput {FollowInput source;};
struct AlignedFollowResult
{
  FollowState source_state;
  FollowResult value;
};
std::string fingerprint_aligned_follow_input(const AlignedFollowInput & input);
class AlignedFollowAdapter
{
public:
  AlignedFollowAdapter();
  ~AlignedFollowAdapter();
  AlignedFollowAdapter(const AlignedFollowAdapter &) = delete;
  AlignedFollowAdapter & operator=(const AlignedFollowAdapter &) = delete;
  AlignedFollowResult solve(AlignedFollowInput input);
  void reset();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
// World-frame Research values. No angular command or legacy host conversion.
struct WorldFollowInput
{
  PredictionSnapshot prediction;
  PreparedCorridor route;
  BodyPolicy body;
  FollowLimits limits;  // Explicit world-axis Research bounds, not body actuator certification.
  FollowIdentity identity;
  FollowState source_state;  // Raw body measurement/history; never relabelled as world.
  Vec2 preceding_world_velocity;  // Virtual prior XY proposal in offline replay.
  int64_t preceding_world_stamp_ns{};
  std::string velocity_frame;
  double progress{};
  FollowClock::time_point acquired;
  bool reset_warm{};
};
struct WorldFollowControl {Vec2 world_velocity; double progress_rate{};};
struct WorldFollowStage {Vec2 position, world_velocity; double progress{};};
struct WorldFollowProposal
{
  FollowIdentity identity;
  std::string input_digest, receipt_digest, path_digest, map_digest, limits_digest, velocity_frame;
  int64_t epoch_ns{};
  FollowClock::time_point acquired, source_deadline;
  Vec2 world_velocity;
  std::array<WorldFollowControl, 15> controls;
  std::array<WorldFollowStage, 31> stages;
};
struct WorldFollowResult
{
  FollowState source_state;
  std::optional<WorldFollowProposal> proposal;
  std::string reason, solver_status{"not_run"};
  int iterations{};
  double solver_seconds{}, elapsed_seconds{}, nominal_dynamic_cost{}, solved_dynamic_cost{};
  std::optional<double> minimum_constraint_slack;
  bool used_warm{};
};
class WorldFollowAdapter
{
public:
  WorldFollowAdapter();
  ~WorldFollowAdapter();
  WorldFollowAdapter(const WorldFollowAdapter &) = delete;
  WorldFollowAdapter & operator=(const WorldFollowAdapter &) = delete;
  WorldFollowResult solve(WorldFollowInput input);
  void reset();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace rm_r4_prediction_consumption
