#include "rm_r4_prediction_consumption/follow.hpp"
#include "digest.hpp"
#include <rm_navigation_execution_adapters/current_geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>
#include <type_traits>
#include <Eigen/Core>
#include <osqp.h>

#if !defined(PROFILING) || defined(DFLOAT) || defined(EMBEDDED)
#error "Follow requires the registered non-embedded, double, profiling OSQP build"
#endif
static_assert(std::string_view(OSQP_VERSION) == "0.6.3", "Fixed OSQP version required");
static_assert(std::is_same_v<c_float, double>, "OSQP scalar ABI must be double");

namespace rm_r4_prediction_consumption
{
namespace
{
constexpr int nodes = 15, stages = 31, variables = 45, constraints = 168;
constexpr double period = .05, decision_dt = .1, cycle_budget = .04, solver_budget = .015;
using Vector = Eigen::Matrix<double, variables, 1>;
using Row = Eigen::Matrix<double, 1, variables>;
using Matrix = Eigen::Matrix<double, variables, variables>;
using XYMap = Eigen::Matrix<double, 2, variables>;
struct Maps {std::array<XYMap, stages> xy; std::array<Row, stages> progress; Eigen::Matrix2d rotation;};
double seconds(FollowClock::time_point begin) {return std::chrono::duration<double>(FollowClock::now() - begin).count();}
Eigen::Vector2d vec(Vec2 p) {return {p.x, p.y};}
Vec2 point(Eigen::Vector2d p) {return {p.x(), p.y()};}
bool finite(Vec2 p) {return std::isfinite(p.x) && std::isfinite(p.y);}
bool bounded(Vec2 p, const FollowLimits & l)
{return p.x >= l.lower.x && p.x <= l.upper.x && p.y >= l.lower.y && p.y <= l.upper.y;}
void text_identity(Digest & d, const FollowIdentity & id)
{d.text(id.host_instance); d.text(id.execution_id); d.text(id.base_frame); d.integer(id.authority_epoch);}

struct WorldContext {Vec2 preceding; int64_t stamp{}; std::string frame;};
template<bool World> Vec2 measured_velocity(const FollowInput & in)
{
  if constexpr (!World) {return in.state.measured_body_velocity;}
  const auto v = in.state.measured_body_velocity; const double c = std::cos(in.state.yaw), s = std::sin(in.state.yaw);
  return {c * v.x - s * v.y, s * v.x + c * v.y};
}
struct ModelStart {Vec2 position; double yaw{};};
template<bool Aligned, bool World = false> ModelStart model_start(const FollowInput & in)
{
  ModelStart out{in.state.position, in.state.yaw};
  if constexpr (World) {
    const auto v = measured_velocity<true>(in); const double age = (in.prediction.epoch_ns() - in.state.pose_stamp_ns) * 1e-9;
    out.position = {out.position.x + v.x * age, out.position.y + v.y * age};
  } else if constexpr (Aligned) {
    namespace execution = rm_navigation_execution_adapters;
    execution::Pose pose{out.position.x, out.position.y, out.yaw};
    int64_t remaining = in.prediction.epoch_ns() - in.state.pose_stamp_ns;
    while (remaining > 0) {
      const auto step = std::min<int64_t>(remaining, 50000000);
      pose = execution::integrate_held_body_twist(pose,
        {in.state.measured_body_velocity.x, in.state.measured_body_velocity.y, in.state.measured_yaw_rate}, step * 1e-9);
      remaining -= step;
    }
    out = {{pose.x, pose.y}, pose.yaw};
  }
  return out;
}
template<bool Aligned, bool World = false> Bounds centre_bounds(const FollowInput & in, ModelStart start)
{
  if constexpr (World) {return in.route.local_bounds(start.position);}
  else if constexpr (!Aligned) {return in.route.local_bounds(in.state.position);}
  else {
    auto body = in.body; body.yaw = start.yaw;
    const auto support = body.support(); const auto free = in.route.local_free_bounds(start.position, body);
    return {free.xmin - support.xmin + body.static_clearance, free.xmax - support.xmax - body.static_clearance,
      free.ymin - support.ymin + body.static_clearance, free.ymax - support.ymax - body.static_clearance};
  }
}
template<bool Aligned, bool World = false> std::string validate(const FollowInput & in, const WorldContext * world = nullptr)
{
  const auto & s = in.state; const auto epoch = in.prediction.epoch_ns();
  in.limits.digest();
  if constexpr (Aligned && !World) {
    if (s.last_applied_yaw_rate != 0.) {throw ContractError("aligned Follow requires zero last-applied yaw command");}
  }
  for (const auto & text : {in.identity.host_instance, in.identity.execution_id, in.identity.base_frame}) {
    if (text.empty() || text.size() > 128) {throw ContractError("host identity/frame");}
  }
  if (in.identity.cycle_sequence == 0 || !finite(s.position) ||
    !finite(s.measured_body_velocity) || !finite(s.last_applied_body_velocity) ||
    std::hypot(s.measured_body_velocity.x, s.measured_body_velocity.y) > 3. ||
    (!World && !bounded(s.last_applied_body_velocity, in.limits)) || !std::isfinite(s.yaw) ||
    !std::isfinite(s.measured_yaw_rate) || (!World && std::abs(s.measured_yaw_rate) > (Aligned ? 3. : 1e-6)) ||
    !std::isfinite(s.last_applied_yaw_rate) || (!World && s.last_applied_yaw_rate != 0.) || std::abs(s.yaw - in.body.yaw) > 1e-9 ||
    in.prediction.frame() != in.route.frame() || in.prediction.body_digest() != in.body.digest() ||
    in.route.body_digest() != in.body.digest() || !std::isfinite(in.progress) ||
    in.progress < 0 || in.progress > in.route.arcs().back() || s.tf_stamp_ns != s.pose_stamp_ns ||
    s.frame != in.route.frame() || s.body_frame != in.identity.base_frame)
  {throw ContractError(World ? "world XY coherent source values" : (Aligned ? "aligned coherent control values" : "fixed-yaw coherent control values"));}
  for (auto stamp : {s.pose_stamp_ns, s.velocity_stamp_ns, s.tf_stamp_ns, s.applied_stamp_ns}) {
    if (stamp <= 0 || stamp > epoch || epoch - stamp > 100000000) {
      throw ContractError("state/TF/applied stamp age");
    }
  }
  if constexpr (World) {
    if (!world || world->frame != in.route.frame() || world->frame == s.body_frame ||
      !in.body.yaw_invariant_circle || s.velocity_stamp_ns != s.pose_stamp_ns ||
      !finite(world->preceding) || !bounded(world->preceding, in.limits) || world->stamp <= 0 ||
      world->stamp > epoch || epoch - world->stamp > 100000000) {throw ContractError("explicit world XY/circle/history frame");}
  }
  const auto start = model_start<Aligned, World>(in); centre_bounds<Aligned, World>(in, start);
  Digest d; d.text(World ? "r4_follow_world_xy/v1" : (Aligned ? "r4_follow_aligned_input/v1" : "r4_follow_input/v1")); text_identity(d, in.identity);
  d.integer(in.identity.cycle_sequence); d.integer(epoch);
  d.text(in.prediction.receipt_digest()); d.text(in.prediction.policy_digest());
  d.text(in.route.path_digest()); d.text(in.route.map_digest()); d.text(in.route.policy_digest());
  d.integer(in.route.generation()); d.text(in.body.digest()); d.text(in.limits.digest());
  d.point(s.position); d.point(s.measured_body_velocity); d.point(s.last_applied_body_velocity);
  d.text(s.frame); d.text(s.body_frame);
  d.number(s.yaw); d.number(s.measured_yaw_rate); d.number(s.last_applied_yaw_rate);
  for (auto stamp : {s.pose_stamp_ns, s.velocity_stamp_ns, s.tf_stamp_ns, s.applied_stamp_ns}) {d.integer(stamp);}
  if constexpr (World) {
    d.text("world_measured_to_epoch/world_XY_future/v1"); d.text(world->frame);
    d.point(measured_velocity<true>(in)); d.point(world->preceding); d.integer(world->stamp); d.point(start.position);
  } else if constexpr (Aligned) {
    d.text("held_measured_to_epoch/future_wz_zero/v1"); d.point(start.position); d.number(start.yaw);
  }
  d.number(in.progress); d.integer(in.reset_warm);
  d.integer(std::chrono::duration_cast<std::chrono::nanoseconds>(in.acquired.time_since_epoch()).count());
  return d.finish();
}
template<bool Aligned, bool World = false> std::string warm_key(const FollowInput & in)
{
  Digest d; text_identity(d, in.identity); d.text(in.route.path_digest()); d.text(in.route.map_digest());
  if constexpr (Aligned || World) {
    d.text(World ? "world_XY/v1" : "epoch_aligned/v1"); d.text(in.route.geometry_policy_digest());
    d.integer(in.route.generation()); d.text(in.body.geometry_digest());
    d.text(in.limits.digest()); d.text(in.prediction.producer_id()); d.integer(in.prediction.generation());
    d.text(in.prediction.policy().digest());
  } else {
    d.text(in.route.policy_digest()); d.integer(in.route.generation()); d.text(in.body.digest());
    d.text(in.limits.digest()); d.text(in.prediction.producer_id()); d.integer(in.prediction.generation());
    d.text(in.prediction.policy_digest());
  }
  return d.finish();
}
Maps maps(double yaw)
{
  Maps out; const double c = std::cos(yaw), s = std::sin(yaw); out.rotation << c, -s, s, c;
  for (int k = 0; k < stages; ++k) {
    out.xy[k].setZero(); out.progress[k].setZero();
    for (int j = 0; j < nodes; ++j) {
      const double dt = std::clamp(k * period - j * decision_dt, 0., decision_dt);
      out.xy[k].block<2, 2>(0, 3 * j) = dt * out.rotation; out.progress[k][3 * j + 2] = dt;
    }
  }
  return out;
}
template<bool World = false> std::array<FollowStage, stages> rollout(const FollowInput & in, ModelStart start, const Maps & map, const Vector & z)
{
  std::array<FollowStage, stages> out;
  for (int k = 0; k < stages; ++k) {
    const int j = std::min(k / 2, nodes - 1);
    out[k] = {point(vec(start.position) + map.xy[k] * z),
      k == 0 ? measured_velocity<World>(in) : Vec2{z[3 * j], z[3 * j + 1]},
      start.yaw, in.progress + (map.progress[k] * z).value()};
  }
  return out;
}
Vector cold_seed(const FollowInput & in, const Maps & map, Vec2 preceding)
{
  Vector z; Vec2 last = preceding; double progress = in.progress;
  for (int j = 0; j < nodes; ++j) {
    const auto ref = in.route.sample(progress);
    const double cruise = std::min(in.limits.cruise, std::sqrt(2 * std::max(0., in.route.arcs().back() - progress)));
    const Eigen::Vector2d target = map.rotation.transpose() * vec(ref.tangent) * cruise;
    const double dt = j == 0 ? period : decision_dt;
    last = {std::clamp(target.x(), std::max(in.limits.lower.x, last.x - dt * in.limits.command_rate.x),
        std::min(in.limits.upper.x, last.x + dt * in.limits.command_rate.x)),
      std::clamp(target.y(), std::max(in.limits.lower.y, last.y - dt * in.limits.command_rate.y),
        std::min(in.limits.upper.y, last.y + dt * in.limits.command_rate.y))};
    const double speed = std::min({in.limits.progress_upper,
        std::max(0., vec(ref.tangent).dot(map.rotation * vec(last))),
        std::max(0., (in.route.arcs().back() - progress) / decision_dt)});
    z.segment<3>(3 * j) << last.x, last.y, speed; progress += decision_dt * speed;
  }
  return z;
}

// Storage outlives osqp_setup/solve; OSQP copies these inputs into its workspace.
struct SparseInput
{
  std::vector<c_int> columns, rows;
  std::vector<c_float> values;
  csc matrix{};
  SparseInput(const Eigen::MatrixXd & dense, bool upper)
  {
    for (int col = 0; col < dense.cols(); ++col) {
      columns.push_back(values.size());
      for (int row = 0; row < dense.rows() && (!upper || row <= col); ++row) {
        if (dense(row, col) != 0.) {rows.push_back(row); values.push_back(dense(row, col));}
      }
    }
    columns.push_back(values.size());
    matrix = {static_cast<c_int>(values.size()), static_cast<c_int>(dense.rows()),
      static_cast<c_int>(dense.cols()), columns.data(), rows.data(), values.data(), -1};
  }
};
struct Workspace
{
  OSQPWorkspace * value{};
  ~Workspace() {if (value) {osqp_cleanup(value);}}
};
}

std::string FollowLimits::digest() const
{
  if (!finite(lower) || !finite(upper) || !finite(command_rate) || lower.x >= 0 || lower.y >= 0 ||
    upper.x <= 0 || upper.y <= 0 || lower.x < -3 || lower.y < -3 || upper.x > 3 || upper.y > 3 ||
    command_rate.x <= 0 || command_rate.y <= 0 || command_rate.x > 5 || command_rate.y > 5 ||
    !std::isfinite(cruise) || cruise <= 0 || cruise > 3 ||
    !std::isfinite(progress_upper) || progress_upper <= 0 || progress_upper > 3)
  {throw ContractError("explicit actual Follow limits");}
  Digest d; d.text("r4_follow_limits/v1"); d.point(lower); d.point(upper); d.point(command_rate);
  d.number(cruise); d.number(progress_upper); return d.finish();
}
struct SharedFollowState
{
  std::optional<Vector> previous;
  std::string key, seen_host, seen_execution;
  uint64_t seen_authority{}, seen_sequence{};
  int64_t previous_epoch{}, seen_epoch{};
  void clear_warm() {previous.reset(); key.clear(); previous_epoch = 0;}
};
struct FollowSolver::Impl : SharedFollowState {};
struct AlignedFollowAdapter::Impl : SharedFollowState {};
struct WorldFollowAdapter::Impl : SharedFollowState {};
std::string fingerprint_follow_input(const FollowInput & input) {return validate<false>(input);}
std::string fingerprint_aligned_follow_input(const AlignedFollowInput & input) {return validate<true>(input.source);}
FollowSolver::FollowSolver() : impl_(std::make_unique<Impl>()) {}
FollowSolver::~FollowSolver() = default;
void FollowSolver::reset() {impl_ = std::make_unique<Impl>();}

template<bool Aligned, bool World = false> FollowResult solve_follow(FollowInput in, SharedFollowState & state, const WorldContext * world = nullptr)
{
  auto * impl_ = &state;
  FollowResult result;
  auto fail = [&](const std::string & reason) {
      impl_->clear_warm(); result.proposal.reset(); result.reason = reason;
      result.elapsed_seconds = in.acquired.time_since_epoch().count() > 0 && in.acquired <= FollowClock::now() ?
        seconds(in.acquired) : -1.; return result;
    };
  try {
    if (in.acquired.time_since_epoch().count() <= 0 || in.acquired > FollowClock::now()) {return fail("invalid_acquisition");}
    if (seconds(in.acquired) >= cycle_budget) {return fail("cycle_deadline_before_assembly");}
    if (std::strcmp(osqp_version(), "0.6.3") != 0) {return fail("solver_version");}
    const auto digest = validate<Aligned, World>(in, world); const auto key = warm_key<Aligned, World>(in);
    const auto preceding = World ? world->preceding : in.state.last_applied_body_velocity;
    if (impl_->seen_host == in.identity.host_instance && impl_->seen_execution == in.identity.execution_id &&
      impl_->seen_authority == in.identity.authority_epoch &&
      (in.identity.cycle_sequence <= impl_->seen_sequence || in.prediction.epoch_ns() <= impl_->seen_epoch))
    {return fail("nonincreasing_cycle");}
    impl_->seen_host = in.identity.host_instance; impl_->seen_execution = in.identity.execution_id;
    impl_->seen_authority = in.identity.authority_epoch; impl_->seen_sequence = in.identity.cycle_sequence;
    impl_->seen_epoch = in.prediction.epoch_ns();
    const auto start = model_start<Aligned, World>(in); const auto map = maps(World ? 0. : start.yaw); Vector seed = cold_seed(in, map, preceding);
    const double age = (in.prediction.epoch_ns() - impl_->previous_epoch) * 1e-9;
    if (!in.reset_warm && impl_->previous && impl_->key == key && age > 0 && age <= .15) {
      seed.setZero();
      for (int k = 0; k < nodes; ++k) {
        for (int j = 0; j < nodes; ++j) {
          const double overlap = std::max(0., std::min((k + 1) * decision_dt + age, (j + 1) * decision_dt) -
              std::max(k * decision_dt + age, j * decision_dt));
          seed.segment<3>(3 * k) += overlap / decision_dt * impl_->previous->segment<3>(3 * j);
        }
        seed.segment<3>(3 * k) += std::clamp((k + 1) * decision_dt + age - 1.5, 0., decision_dt) /
          decision_dt * impl_->previous->segment<3>(42);
      }
      result.used_warm = true;
    }
    const auto nominal = rollout<World>(in, start, map, seed); TemporalSoftField field(in.prediction);
    Matrix P = Matrix::Identity() * 1e-8; Vector q = Vector::Zero();
    auto residual = [&](const Row & row, double nominal_value, double weight) {
        const double constant = nominal_value - (row * seed).value();
        P.noalias() += weight * row.transpose() * row; q.noalias() += weight * constant * row.transpose();
      };
    for (int k = 0; k < 30; ++k) {
      if (seconds(in.acquired) >= cycle_budget) {return fail("cycle_deadline_in_assembly");}
      const double s = std::clamp(nominal[k].progress, 0., in.route.arcs().back());
      const auto ref = in.route.sample(s); const Eigen::Vector2d tangent = vec(ref.tangent);
      const Eigen::Vector2d normal{-tangent.y(), tangent.x()};
      const Eigen::Vector2d delta = vec(nominal[k].position) - vec(ref.position);
      residual(normal.transpose() * map.xy[k], normal.dot(delta), period * 20.);
      residual(tangent.transpose() * map.xy[k] - map.progress[k], tangent.dot(delta), period * 8.);
      const int j = std::min(k / 2, nodes - 1); Row velocity = Row::Zero(), rate = Row::Zero();
      velocity.segment<2>(3 * j) = tangent.transpose() * map.rotation; rate[3 * j + 2] = 1.;
      residual(velocity - rate, ((velocity - rate) * seed).value(), period * 2.);
      const double cruise = std::min(in.limits.cruise, std::sqrt(2 * std::max(0., in.route.arcs().back() - s)));
      residual(rate, (rate * seed).value() - cruise, period * 2.);
      const auto soft = [&]() {
        if constexpr (Aligned && !World) {return field.sample(nominal[k].position, start.yaw, k);}
        else {return field.sample(nominal[k].position, k);}
      }();
      residual(vec(soft.gradient).transpose() * map.xy[k], soft.residual, period);
      result.nominal_dynamic_cost += .5 * period * soft.residual * soft.residual;
    }
    Eigen::Matrix<double, 30, variables> difference = Eigen::Matrix<double, 30, variables>::Zero();
    Eigen::Matrix<double, 30, 1> previous = Eigen::Matrix<double, 30, 1>::Zero();
    for (int j = 0; j < nodes; ++j) {
      q[3 * j + 2] -= .1 * decision_dt;
      for (int axis = 0; axis < 2; ++axis) {
        const int row = 2 * j + axis; difference(row, 3 * j + axis) = 1.;
        if (j) {difference(row, 3 * (j - 1) + axis) = -1.;}
        else {previous[row] = axis == 0 ? preceding.x : preceding.y;}
        const double dt = j == 0 ? period : decision_dt;
        residual(difference.row(row) / dt, ((difference.row(row) * seed).value() - previous[row]) / dt, .1 * decision_dt);
        if (j) {
          const double prior_dt = j == 1 ? period : decision_dt;
          const Row jerk = difference.row(row) / dt - difference.row(row - 2) / prior_dt;
          residual(jerk, (jerk * seed).value() + previous[row - 2] / prior_dt, .02);
        }
      }
    }
    Eigen::Matrix<double, constraints, variables> A; Eigen::Matrix<double, constraints, 1> lo, hi;
    A.setZero(); A.topRows<45>().setIdentity(); A.middleRows<30>(45) = difference;
    for (int j = 0; j < nodes; ++j) {
      lo.segment<3>(3 * j) << in.limits.lower.x, in.limits.lower.y, 0.;
      hi.segment<3>(3 * j) << in.limits.upper.x, in.limits.upper.y, in.limits.progress_upper;
      const double dt = j == 0 ? period : decision_dt;
      lo.segment<2>(45 + 2 * j) = previous.segment<2>(2 * j) - dt * vec(in.limits.command_rate);
      hi.segment<2>(45 + 2 * j) = previous.segment<2>(2 * j) + dt * vec(in.limits.command_rate);
    }
    const auto box = centre_bounds<Aligned, World>(in, start);
    const double reserve = .5 * period * std::hypot(std::max(-in.limits.lower.x, in.limits.upper.x),
        std::max(-in.limits.lower.y, in.limits.upper.y));
    for (int k = 0; k < stages; ++k) {
      A.row(75 + k) = map.xy[k].row(0); A.row(106 + k) = map.xy[k].row(1);
      A.row(137 + k) = map.progress[k];
      lo[75 + k] = box.xmin + reserve - start.position.x; hi[75 + k] = box.xmax - reserve - start.position.x;
      lo[106 + k] = box.ymin + reserve - start.position.y; hi[106 + k] = box.ymax - reserve - start.position.y;
      lo[137 + k] = -in.progress; hi[137 + k] = in.route.arcs().back() - in.progress;
    }
    if ((lo.array() > hi.array()).any()) {return fail("empty_static_bounds");}
    Matrix transform = Matrix::Zero(); Vector offset;
    for (int k = 0; k < nodes; ++k) {
      offset.segment<3>(3 * k) << preceding.x, preceding.y, 0.;
      transform(3 * k + 2, 3 * k + 2) = 1.;
      for (int j = 0; j <= k; ++j) {
        transform(3 * k, 3 * j) = transform(3 * k + 1, 3 * j + 1) = j == 0 ? period : decision_dt;
      }
    }
    const Matrix rate_P = transform.transpose() * P * transform;
    const Vector rate_q = transform.transpose() * (P * offset + q);
    const Eigen::MatrixXd rate_A = A * transform;
    Eigen::VectorXd rate_lo = lo - A * offset, rate_hi = hi - A * offset;
    Vector warm = seed;
    for (int k = 0; k < nodes; ++k) {
      const Eigen::Vector2d last = k == 0 ? vec(preceding) : seed.segment<2>(3 * (k - 1));
      warm.segment<2>(3 * k) = (seed.segment<2>(3 * k) - last) / (k == 0 ? period : decision_dt);
    }
    if (!rate_P.allFinite() || !rate_q.allFinite() || !rate_A.allFinite() || !warm.allFinite()) {return fail("nonfinite_qp");}
    SparseInput sparse_P(rate_P, true), sparse_A(rate_A, false);
    const double remaining = cycle_budget - seconds(in.acquired);
    if (remaining <= 0.) {return fail("cycle_deadline_after_assembly");}
    OSQPSettings settings; osqp_set_default_settings(&settings);
    settings.verbose = 0; settings.max_iter = 400; settings.eps_abs = settings.eps_rel = 1e-6;
    settings.polish = 1; settings.check_termination = 10; settings.time_limit = std::min(solver_budget, remaining);
    // rate_q is const; OSQPData's legacy API is mutable, but setup copies it.
    Vector gradient = rate_q;
    OSQPData data{variables, constraints, &sparse_P.matrix, &sparse_A.matrix, gradient.data(), rate_lo.data(), rate_hi.data()};
    Vector solution; bool solved = false; const auto solver_start = FollowClock::now();
    {
      Workspace workspace;
      const auto setup_status = osqp_setup(&workspace.value, &data, &settings);
      if (setup_status != 0 || !workspace.value) {result.solver_status = "setup_failed";}
      else if (osqp_warm_start_x(workspace.value, warm.data()) != 0) {result.solver_status = "warm_start_failed";}
      else {
        // Factorization/setup is part of the same wall budget, not a new lease.
        const double solve_remaining = std::min(solver_budget - seconds(solver_start), cycle_budget - seconds(in.acquired));
        if (solve_remaining <= 0.) {result.solver_status = "setup_overrun";}
        // 0.6.3 counts setup_time in the first solve's time_limit. Add that
        // already-counted internal time, retaining the external wall deadline.
        else if (osqp_update_time_limit(workspace.value,
            workspace.value->info->setup_time + solve_remaining) != 0)
        {result.solver_status = "time_limit_failed";}
        else {
          const auto exit = osqp_solve(workspace.value); const auto * info = workspace.value->info;
          result.solver_status = info->status; result.iterations = info->iter;
          solved = exit == 0 && info->status_val == OSQP_SOLVED && workspace.value->solution && workspace.value->solution->x;
          if (solved) {solution = Eigen::Map<Vector>(workspace.value->solution->x);}
        }
      }
    }
    result.solver_seconds = seconds(solver_start);
    if (result.solver_seconds > solver_budget || seconds(in.acquired) >= cycle_budget) {return fail("cycle_or_solver_deadline");}
    if (!solved || !solution.allFinite()) {return fail("solver_not_solved");}
    Vector z = offset + transform * solution; Vec2 last = preceding;
    for (int k = 0; k < nodes; ++k) {
      const double dt = k == 0 ? period : decision_dt;
      last = {std::clamp(z[3 * k], std::max(in.limits.lower.x, last.x - dt * in.limits.command_rate.x),
          std::min(in.limits.upper.x, last.x + dt * in.limits.command_rate.x)),
        std::clamp(z[3 * k + 1], std::max(in.limits.lower.y, last.y - dt * in.limits.command_rate.y),
          std::min(in.limits.upper.y, last.y + dt * in.limits.command_rate.y))};
      z.segment<3>(3 * k) << last.x, last.y, std::clamp(z[3 * k + 2], 0., in.limits.progress_upper);
    }
    const Eigen::VectorXd values = A * z;
    result.minimum_constraint_slack = std::min((values - lo).minCoeff(), (hi - values).minCoeff());
    if (!z.allFinite() || *result.minimum_constraint_slack < -1e-5) {return fail("hard_static_or_bounds");}
    FollowProposal proposal; proposal.identity = in.identity; proposal.input_digest = digest;
    proposal.receipt_digest = in.prediction.receipt_digest(); proposal.path_digest = in.route.path_digest();
    proposal.map_digest = in.route.map_digest(); proposal.limits_digest = in.limits.digest();
    proposal.epoch_ns = in.prediction.epoch_ns(); proposal.acquired = in.acquired;
    proposal.source_deadline = in.acquired + std::chrono::milliseconds(75);
    proposal.body_velocity = {z[0], z[1]}; proposal.stages = rollout<World>(in, start, map, z);
    for (int k = 0; k < nodes; ++k) {proposal.controls[k] = {{z[3 * k], z[3 * k + 1]}, z[3 * k + 2]};}
    for (int k = 0; k < 30; ++k) {
      const auto soft = [&]() {
        if constexpr (Aligned && !World) {return field.sample(proposal.stages[k].position, start.yaw, k);}
        else {return field.sample(proposal.stages[k].position, k);}
      }();
      result.solved_dynamic_cost += .5 * period * soft.residual * soft.residual;
    }
    if (seconds(in.acquired) >= cycle_budget) {return fail("cycle_deadline_after_validation");}
    impl_->previous = z; impl_->previous_epoch = in.prediction.epoch_ns(); impl_->key = key;
    result.proposal = std::move(proposal); result.reason = "solved_soft_dynamic";
    result.elapsed_seconds = seconds(in.acquired);
    if (result.elapsed_seconds >= cycle_budget) {return fail("cycle_deadline_after_validation");}
    return result;
  } catch (const std::exception & error) {return fail(std::string("invalid_or_failed_input: ") + error.what());}
}
FollowResult FollowSolver::solve(FollowInput in)
{return solve_follow<false>(std::move(in), *impl_);}
AlignedFollowAdapter::AlignedFollowAdapter() : impl_(std::make_unique<Impl>()) {}
AlignedFollowAdapter::~AlignedFollowAdapter() = default;
void AlignedFollowAdapter::reset() {impl_ = std::make_unique<Impl>();}
AlignedFollowResult AlignedFollowAdapter::solve(AlignedFollowInput input)
{
  auto source_state = input.source.state;
  return {std::move(source_state), solve_follow<true>(std::move(input.source), *impl_)};
}
WorldFollowAdapter::WorldFollowAdapter() : impl_(std::make_unique<Impl>()) {}
WorldFollowAdapter::~WorldFollowAdapter() = default;
void WorldFollowAdapter::reset() {impl_ = std::make_unique<Impl>();}
WorldFollowResult WorldFollowAdapter::solve(WorldFollowInput input)
{
  const WorldContext context{input.preceding_world_velocity, input.preceding_world_stamp_ns, input.velocity_frame};
  auto raw = input.source_state;
  FollowInput internal{std::move(input.prediction), std::move(input.route), input.body, input.limits, input.identity,
    raw, input.progress, input.acquired, input.reset_warm};
  const auto result = solve_follow<false, true>(std::move(internal), *impl_, &context);
  WorldFollowResult out; out.source_state = std::move(raw); out.reason = result.reason; out.solver_status = result.solver_status;
  out.iterations = result.iterations; out.solver_seconds = result.solver_seconds; out.elapsed_seconds = result.elapsed_seconds;
  out.nominal_dynamic_cost = result.nominal_dynamic_cost; out.solved_dynamic_cost = result.solved_dynamic_cost;
  out.minimum_constraint_slack = result.minimum_constraint_slack; out.used_warm = result.used_warm;
  if (result.proposal) {
    const auto & p = *result.proposal; WorldFollowProposal q;
    q.identity = p.identity; q.input_digest = p.input_digest; q.receipt_digest = p.receipt_digest;
    q.path_digest = p.path_digest; q.map_digest = p.map_digest; q.limits_digest = p.limits_digest; q.velocity_frame = context.frame;
    q.epoch_ns = p.epoch_ns; q.acquired = p.acquired; q.source_deadline = p.source_deadline; q.world_velocity = p.body_velocity;
    for (size_t k = 0; k < 15; ++k) {q.controls[k] = {p.controls[k].body_velocity, p.controls[k].progress_rate};}
    for (size_t k = 0; k < 31; ++k) {q.stages[k] = {p.stages[k].position, p.stages[k].body_velocity, p.stages[k].progress};}
    out.proposal = std::move(q);
  }
  return out;
}
}  // namespace rm_r4_prediction_consumption
