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
constexpr int nodes = 15, stages = 31;
constexpr double period = .05, decision_dt = .1, cycle_budget = .04, solver_budget = .015;
template<int S> using VectorFor = Eigen::Matrix<double, S * nodes, 1>;
template<int S> using RowFor = Eigen::Matrix<double, 1, S * nodes>;
template<int S> using XYMapFor = Eigen::Matrix<double, 2, S * nodes>;
template<int S> struct Maps
{
  std::array<XYMapFor<S>, stages> xy, velocity;
  std::array<RowFor<S>, stages> progress, yaw;
  std::array<Eigen::Vector2d, stages> origin;
  std::array<Eigen::Matrix2d, stages> rotation;
};
const FollowLimits & linear_limits(const FollowInput & in) {return in.limits;}
const FollowLimits & linear_limits(const RotatingFollowInput & in) {return in.limits.translation;}
namespace execution = rm_navigation_execution_adapters;
execution::Pose stage_zero(const FollowInput & in)
{return {in.state.position.x, in.state.position.y, in.state.yaw};}
execution::Pose stage_zero(const RotatingFollowInput & in)
{
  execution::Pose p{in.state.position.x, in.state.position.y, in.state.yaw};
  int64_t remaining = in.prediction.epoch_ns() - in.state.pose_stamp_ns;
  while (remaining > 0) {
    const auto step = std::min<int64_t>(remaining, 50000000);
    p = execution::integrate_held_body_twist(p,
      {in.state.measured_body_velocity.x, in.state.measured_body_velocity.y, in.state.measured_yaw_rate}, step * 1e-9);
    remaining -= step;
  }
  return p;
}
Eigen::Matrix2d rotation(double yaw)
{Eigen::Matrix2d r; r << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw); return r;}
double seconds(FollowClock::time_point begin) {return std::chrono::duration<double>(FollowClock::now() - begin).count();}
Eigen::Vector2d vec(Vec2 p) {return {p.x, p.y};}
Vec2 point(Eigen::Vector2d p) {return {p.x(), p.y()};}
bool finite(Vec2 p) {return std::isfinite(p.x) && std::isfinite(p.y);}
bool bounded(Vec2 p, const FollowLimits & l)
{return p.x >= l.lower.x && p.x <= l.upper.x && p.y >= l.lower.y && p.y <= l.upper.y;}
void text_identity(Digest & d, const FollowIdentity & id)
{d.text(id.host_instance); d.text(id.execution_id); d.text(id.base_frame); d.integer(id.authority_epoch);}

template<class Input> std::string validate(const Input & in)
{
  constexpr bool rotating = std::is_same_v<Input, RotatingFollowInput>;
  const auto & limits = linear_limits(in);
  const auto & s = in.state; const auto epoch = in.prediction.epoch_ns();
  bool yaw_valid = std::isfinite(s.measured_yaw_rate) && std::isfinite(s.last_applied_yaw_rate);
  if constexpr (rotating) {
    yaw_valid = yaw_valid && std::abs(s.measured_yaw_rate) <= 3. &&
      s.last_applied_yaw_rate >= in.limits.yaw_lower && s.last_applied_yaw_rate <= in.limits.yaw_upper;
  } else {yaw_valid = yaw_valid && std::abs(s.measured_yaw_rate) <= 1e-6 && s.last_applied_yaw_rate == 0.;}
  in.limits.digest();
  for (const auto & text : {in.identity.host_instance, in.identity.execution_id, in.identity.base_frame}) {
    if (text.empty() || text.size() > 128) {throw ContractError("host identity/frame");}
  }
  if (in.identity.cycle_sequence == 0 || !finite(s.position) ||
    !finite(s.measured_body_velocity) || !finite(s.last_applied_body_velocity) ||
    std::hypot(s.measured_body_velocity.x, s.measured_body_velocity.y) > 3. ||
    !bounded(s.last_applied_body_velocity, limits) || !std::isfinite(s.yaw) ||
    !yaw_valid || std::abs(s.yaw - in.body.yaw) > 1e-9 ||
    in.prediction.frame() != in.route.frame() || in.prediction.body_digest() != in.body.digest() ||
    in.route.body_digest() != in.body.digest() || !std::isfinite(in.progress) ||
    in.progress < 0 || in.progress > in.route.arcs().back() || s.tf_stamp_ns != s.pose_stamp_ns ||
    s.frame != in.route.frame() || s.body_frame != in.identity.base_frame)
  {throw ContractError(rotating ? "rotating coherent control values" : "fixed-yaw coherent control values");}
  for (auto stamp : {s.pose_stamp_ns, s.velocity_stamp_ns, s.tf_stamp_ns, s.applied_stamp_ns}) {
    if (stamp <= 0 || stamp > epoch || epoch - stamp > 100000000) {
      throw ContractError("state/TF/applied stamp age");
    }
  }
  if constexpr (rotating) {
    auto body = in.body; const auto p = stage_zero(in); body.yaw = p.yaw;
    in.route.local_free_bounds({p.x, p.y}, body);
  } else {in.route.local_bounds(s.position);}
  Digest d; d.text(rotating ? "r4_follow_input/se2_v2" : "r4_follow_input/v1"); text_identity(d, in.identity);
  d.integer(in.identity.cycle_sequence); d.integer(epoch);
  d.text(in.prediction.receipt_digest()); d.text(in.prediction.policy_digest());
  d.text(in.route.path_digest()); d.text(in.route.map_digest()); d.text(in.route.policy_digest());
  d.integer(in.route.generation()); d.text(in.body.digest()); d.text(in.limits.digest());
  d.point(s.position); d.point(s.measured_body_velocity); d.point(s.last_applied_body_velocity);
  d.text(s.frame); d.text(s.body_frame);
  d.number(s.yaw); d.number(s.measured_yaw_rate); d.number(s.last_applied_yaw_rate);
  for (auto stamp : {s.pose_stamp_ns, s.velocity_stamp_ns, s.tf_stamp_ns, s.applied_stamp_ns}) {d.integer(stamp);}
  d.number(in.progress); d.integer(in.reset_warm);
  d.integer(std::chrono::duration_cast<std::chrono::nanoseconds>(in.acquired.time_since_epoch()).count());
  return d.finish();
}
template<class Input> std::string warm_key(const Input & in)
{
  Digest d; text_identity(d, in.identity); d.text(in.route.path_digest()); d.text(in.route.map_digest());
  if constexpr (std::is_same_v<Input, RotatingFollowInput>) {
    d.text("se2_v2"); d.text(in.route.rotating_policy_digest()); d.text(in.body.geometry_digest());
    d.text(in.prediction.policy().digest());
  } else {d.text(in.route.policy_digest()); d.text(in.body.digest()); d.text(in.prediction.policy_digest());}
  d.integer(in.route.generation()); d.text(in.limits.digest());
  d.text(in.prediction.producer_id()); d.integer(in.prediction.generation()); return d.finish();
}
template<int S, class Input>
std::array<FollowStage, stages> rollout(const Input & in, const Maps<S> & map, const VectorFor<S> & z)
{
  std::array<FollowStage, stages> out;
  auto pose = stage_zero(in); double progress = in.progress;
  for (int k = 0; k < stages; ++k) {
    const int j = std::min(k / 2, nodes - 1);
    if constexpr (S == 3) {
      out[k] = {point(vec(in.state.position) + map.xy[k] * z),
        k == 0 ? in.state.measured_body_velocity : Vec2{z[S * j], z[S * j + 1]},
        in.state.yaw, in.progress + (map.progress[k] * z).value()};
    } else {
      out[k] = {{pose.x, pose.y}, k == 0 ? in.state.measured_body_velocity : Vec2{z[S * j], z[S * j + 1]},
        pose.yaw, progress};
      if (k < stages - 1) {
        pose = execution::integrate_held_body_twist(pose, {z[S * j], z[S * j + 1], z[S * j + 2]}, period);
        progress += period * z[S * j + 3];
      }
    }
  }
  return out;
}
template<int S, class Input> Maps<S> maps(const Input & in, const VectorFor<S> & seed)
{
  Maps<S> out;
  if constexpr (S == 3) {
    for (int k = 0; k < stages; ++k) {
      out.xy[k].setZero(); out.progress[k].setZero(); out.yaw[k].setZero();
      out.rotation[k] = rotation(in.state.yaw); out.origin[k] = vec(in.state.position);
      out.velocity[k].setZero(); const int control = std::min(k / 2, nodes - 1);
      out.velocity[k].template block<2, 2>(0, S * control) = out.rotation[k];
      for (int j = 0; j < nodes; ++j) {
        const double dt = std::clamp(k * period - j * decision_dt, 0., decision_dt);
        out.xy[k].template block<2, 2>(0, S * j) = dt * out.rotation[k]; out.progress[k][S * j + 2] = dt;
      }
    }
  } else {
    const auto nominal = rollout(in, out, seed);
    out.xy[0].setZero(); out.yaw[0].setZero(); out.progress[0].setZero();
    for (int k = 0; k < stages; ++k) {
      const int j = std::min(k / 2, nodes - 1); const auto r = rotation(nominal[k].yaw);
      out.rotation[k] = r;
      out.origin[k] = vec(nominal[k].position) - out.xy[k] * seed;
      const Eigen::Vector2d v = seed.template segment<2>(S * j), world = r * v;
      out.velocity[k] = Eigen::Vector2d(-world.y(), world.x()) * out.yaw[k];
      out.velocity[k].template block<2, 2>(0, S * j) += r;
      if (k == stages - 1) {break;}
      const double angle = period * seed[S * j + 2];
      double a, b, da, db;
      if (std::abs(angle) < 1e-4) {
        a = period * (1 - angle * angle / 6 + std::pow(angle, 4) / 120);
        b = period * (angle / 2 - std::pow(angle, 3) / 24 + std::pow(angle, 5) / 720);
        da = period * period * (-angle / 3 + std::pow(angle, 3) / 30);
        db = period * period * (.5 - angle * angle / 8 + std::pow(angle, 4) / 144);
      } else {
        a = period * std::sin(angle) / angle; b = period * (1 - std::cos(angle)) / angle;
        da = period * period * (angle * std::cos(angle) - std::sin(angle)) / (angle * angle);
        db = period * period * (angle * std::sin(angle) - (1 - std::cos(angle))) / (angle * angle);
      }
      Eigen::Matrix2d d, dd; d << a, -b, b, a; dd << da, -db, db, da;
      const Eigen::Vector2d displacement = r * d * v;
      out.xy[k + 1] = out.xy[k] + Eigen::Vector2d(-displacement.y(), displacement.x()) * out.yaw[k];
      out.xy[k + 1].template block<2, 2>(0, S * j) += r * d;
      out.xy[k + 1].col(S * j + 2) += r * dd * v;
      out.yaw[k + 1] = out.yaw[k]; out.yaw[k + 1][S * j + 2] += period;
      out.progress[k + 1] = out.progress[k]; out.progress[k + 1][S * j + 3] += period;
    }
  }
  return out;
}
template<int S, class Input> VectorFor<S> cold_seed(const Input & in)
{
  const auto & limits = linear_limits(in); const auto r = rotation(stage_zero(in).yaw);
  VectorFor<S> z = VectorFor<S>::Zero(); Vec2 last = in.state.last_applied_body_velocity; double progress = in.progress;
  for (int j = 0; j < nodes; ++j) {
    const auto ref = in.route.sample(progress);
    const double cruise = std::min(limits.cruise, std::sqrt(2 * std::max(0., in.route.arcs().back() - progress)));
    const Eigen::Vector2d target = r.transpose() * vec(ref.tangent) * cruise;
    const double dt = j == 0 ? period : decision_dt;
    last = {std::clamp(target.x(), std::max(limits.lower.x, last.x - dt * limits.command_rate.x),
        std::min(limits.upper.x, last.x + dt * limits.command_rate.x)),
      std::clamp(target.y(), std::max(limits.lower.y, last.y - dt * limits.command_rate.y),
        std::min(limits.upper.y, last.y + dt * limits.command_rate.y))};
    const double speed = std::min({limits.progress_upper,
        std::max(0., vec(ref.tangent).dot(r * vec(last))),
        std::max(0., (in.route.arcs().back() - progress) / decision_dt)});
    z.template segment<2>(S * j) << last.x, last.y;
    z[S * j + S - 1] = speed; progress += decision_dt * speed;
    if constexpr (S == 4) {
      // A zero target yaw rate respects the same command slew from the virtual command.
      const double prior = j == 0 ? in.state.last_applied_yaw_rate : z[S * (j - 1) + 2];
      z[S * j + 2] = std::clamp(0., std::max(in.limits.yaw_lower, prior - dt * in.limits.yaw_command_rate),
        std::min(in.limits.yaw_upper, prior + dt * in.limits.yaw_command_rate));
    }
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
std::string RotatingFollowLimits::digest() const
{
  translation.digest();
  if (!std::isfinite(yaw_lower) || !std::isfinite(yaw_upper) || yaw_lower > 0 || yaw_upper < 0 ||
    yaw_lower > yaw_upper || yaw_lower < -3 || yaw_upper > 3 ||
    !std::isfinite(yaw_command_rate) || yaw_command_rate <= 0 || yaw_command_rate > 5)
  {throw ContractError("explicit rotating Follow limits");}
  Digest d; d.text("r4_follow_limits/se2_v2"); d.text(translation.digest());
  d.number(yaw_lower); d.number(yaw_upper); d.number(yaw_command_rate); return d.finish();
}
template<int S> struct SolverCache
{
  std::optional<VectorFor<S>> previous;
  std::string key, seen_host, seen_execution;
  uint64_t seen_authority{}, seen_sequence{};
  int64_t previous_epoch{}, seen_epoch{};
  void clear_warm() {previous.reset(); key.clear(); previous_epoch = 0;}
};
struct FollowSolver::Impl : SolverCache<3> {};
struct RotatingFollowSolver::Impl : SolverCache<4> {};
std::string fingerprint_follow_input(const FollowInput & input) {return validate(input);}
FollowSolver::FollowSolver() : impl_(std::make_unique<Impl>()) {}
FollowSolver::~FollowSolver() = default;
void FollowSolver::reset() {impl_ = std::make_unique<Impl>();}
RotatingFollowSolver::RotatingFollowSolver() : impl_(std::make_unique<Impl>()) {}
RotatingFollowSolver::~RotatingFollowSolver() = default;
void RotatingFollowSolver::reset() {impl_ = std::make_unique<Impl>();}

template<int S, class Input, class Proposal>
FollowResultValue<Proposal> solve_model(Input in, SolverCache<S> & cache)
{
  constexpr int variables = S * nodes, axes = S - 1;
  constexpr int constraints = variables + axes * nodes + (S == 3 ? 2 : 4) * stages + stages;
  using Vector = VectorFor<S>; using Row = RowFor<S>;
  using Matrix = Eigen::Matrix<double, variables, variables>;
  const auto & limits = linear_limits(in);
  auto * impl_ = &cache;
  FollowResultValue<Proposal> result;
  auto fail = [&](const std::string & reason) {
      impl_->clear_warm(); result.proposal.reset(); result.reason = reason;
      result.elapsed_seconds = in.acquired.time_since_epoch().count() > 0 && in.acquired <= FollowClock::now() ?
        seconds(in.acquired) : -1.; return result;
    };
  try {
    if (in.acquired.time_since_epoch().count() <= 0 || in.acquired > FollowClock::now()) {return fail("invalid_acquisition");}
    if (seconds(in.acquired) >= cycle_budget) {return fail("cycle_deadline_before_assembly");}
    if (std::strcmp(osqp_version(), "0.6.3") != 0) {return fail("solver_version");}
    const auto digest = validate(in); const auto key = warm_key(in);
    if (impl_->seen_host == in.identity.host_instance && impl_->seen_execution == in.identity.execution_id &&
      impl_->seen_authority == in.identity.authority_epoch &&
      (in.identity.cycle_sequence <= impl_->seen_sequence || in.prediction.epoch_ns() <= impl_->seen_epoch))
    {return fail("nonincreasing_cycle");}
    impl_->seen_host = in.identity.host_instance; impl_->seen_execution = in.identity.execution_id;
    impl_->seen_authority = in.identity.authority_epoch; impl_->seen_sequence = in.identity.cycle_sequence;
    impl_->seen_epoch = in.prediction.epoch_ns();
    Vector seed = cold_seed<S>(in);
    const double age = (in.prediction.epoch_ns() - impl_->previous_epoch) * 1e-9;
    if (!in.reset_warm && impl_->previous && impl_->key == key && age > 0 && age <= .15) {
      seed.setZero();
      for (int k = 0; k < nodes; ++k) {
        for (int j = 0; j < nodes; ++j) {
          const double overlap = std::max(0., std::min((k + 1) * decision_dt + age, (j + 1) * decision_dt) -
              std::max(k * decision_dt + age, j * decision_dt));
          seed.template segment<S>(S * k) += overlap / decision_dt * impl_->previous->template segment<S>(S * j);
        }
        seed.template segment<S>(S * k) += std::clamp((k + 1) * decision_dt + age - 1.5, 0., decision_dt) /
          decision_dt * impl_->previous->template segment<S>(S * (nodes - 1));
      }
      result.used_warm = true;
    }
    const auto map = maps<S>(in, seed); const auto nominal = rollout(in, map, seed); TemporalSoftField field(in.prediction);
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
      velocity = tangent.transpose() * map.velocity[k]; rate[S * j + S - 1] = 1.;
      residual(velocity - rate,
        tangent.dot(map.rotation[k] * seed.template segment<2>(S * j)) - seed[S * j + S - 1], period * 2.);
      const double cruise = std::min(limits.cruise, std::sqrt(2 * std::max(0., in.route.arcs().back() - s)));
      residual(rate, (rate * seed).value() - cruise, period * 2.);
      const auto soft = [&]() {
          if constexpr (S == 3) {return field.sample(nominal[k].position, k);}
          else {return field.sample(nominal[k].position, nominal[k].yaw, k);}
        }();
      residual(vec(soft.gradient).transpose() * map.xy[k] + soft.yaw_gradient * map.yaw[k], soft.residual, period);
      result.nominal_dynamic_cost += .5 * period * soft.residual * soft.residual;
    }
    Eigen::Matrix<double, axes * nodes, variables> difference = Eigen::Matrix<double, axes * nodes, variables>::Zero();
    Eigen::Matrix<double, axes * nodes, 1> previous = Eigen::Matrix<double, axes * nodes, 1>::Zero();
    for (int j = 0; j < nodes; ++j) {
      q[S * j + S - 1] -= .1 * decision_dt;
      for (int axis = 0; axis < axes; ++axis) {
        const int row = axes * j + axis; difference(row, S * j + axis) = 1.;
        if (j) {difference(row, S * (j - 1) + axis) = -1.;}
        else {previous[row] = axis == 0 ? in.state.last_applied_body_velocity.x :
          axis == 1 ? in.state.last_applied_body_velocity.y : in.state.last_applied_yaw_rate;}
        const double dt = j == 0 ? period : decision_dt;
        // Preserve existing translational terms; no added yaw tracking/regularizer.
        if (axis < 2) {
          residual(difference.row(row) / dt, ((difference.row(row) * seed).value() - previous[row]) / dt, .1 * decision_dt);
          if (j) {
            const double prior_dt = j == 1 ? period : decision_dt;
            const Row jerk = difference.row(row) / dt - difference.row(row - axes) / prior_dt;
            residual(jerk, (jerk * seed).value() + previous[row - axes] / prior_dt, .02);
          }
        }
      }
    }
    Eigen::Matrix<double, constraints, variables> A;
    Eigen::Matrix<double, constraints, 1> lo, hi;
    A.setZero(); lo.setConstant(-OSQP_INFTY); hi.setConstant(OSQP_INFTY);
    A.template topRows<variables>().setIdentity(); A.template middleRows<axes * nodes>(variables) = difference;
    for (int j = 0; j < nodes; ++j) {
      lo.template segment<2>(S * j) = vec(limits.lower); hi.template segment<2>(S * j) = vec(limits.upper);
      lo[S * j + S - 1] = 0.; hi[S * j + S - 1] = limits.progress_upper;
      if constexpr (S == 4) {lo[S * j + 2] = in.limits.yaw_lower; hi[S * j + 2] = in.limits.yaw_upper;}
      const double dt = j == 0 ? period : decision_dt;
      for (int axis = 0; axis < axes; ++axis) {
        double rate = axis == 0 ? limits.command_rate.x : limits.command_rate.y;
        if constexpr (S == 4) {if (axis == 2) {rate = in.limits.yaw_command_rate;}}
        lo[variables + axes * j + axis] = previous[axes * j + axis] - dt * rate;
        hi[variables + axes * j + axis] = previous[axes * j + axis] + dt * rate;
      }
    }
    const double reserve = .5 * period * std::hypot(std::max(-limits.lower.x, limits.upper.x),
        std::max(-limits.lower.y, limits.upper.y));
    constexpr int static_row = variables + axes * nodes;
    constexpr int progress_row = static_row + (S == 3 ? 2 : 4) * stages;
    Bounds box;
    if constexpr (S == 3) {box = in.route.local_bounds(in.state.position);}
    else {auto body = in.body; body.yaw = nominal[0].yaw; box = in.route.local_free_bounds(nominal[0].position, body);}
    for (int k = 0; k < stages; ++k) {
      if constexpr (S == 3) {
        A.row(static_row + k) = map.xy[k].row(0); A.row(static_row + stages + k) = map.xy[k].row(1);
        lo[static_row + k] = box.xmin + reserve - in.state.position.x;
        hi[static_row + k] = box.xmax - reserve - in.state.position.x;
        lo[static_row + stages + k] = box.ymin + reserve - in.state.position.y;
        hi[static_row + stages + k] = box.ymax - reserve - in.state.position.y;
      } else {
        const auto support = in.body.support_at(nominal[k].yaw);
        const std::array<double, 4> b{support.bounds.xmin, support.bounds.xmax, support.bounds.ymin, support.bounds.ymax};
        const std::array<double, 4> db{support.yaw_derivative.xmin, support.yaw_derivative.xmax,
          support.yaw_derivative.ymin, support.yaw_derivative.ymax};
        const std::array<double, 4> face{box.xmin, box.xmax, box.ymin, box.ymax};
        for (int f = 0; f < 4; ++f) {
          const int row = static_row + f * stages + k;
          A.row(row) = map.xy[k].row(f / 2) + db[f] * map.yaw[k];
          const double nominal_value = (f < 2 ? nominal[k].position.x : nominal[k].position.y) + b[f];
          const double constant = nominal_value - (A.row(row) * seed).value();
          if (f % 2 == 0) {lo[row] = face[f] + in.body.static_clearance + reserve - constant;}
          else {hi[row] = face[f] - in.body.static_clearance - reserve - constant;}
        }
      }
      A.row(progress_row + k) = map.progress[k];
      lo[progress_row + k] = -in.progress; hi[progress_row + k] = in.route.arcs().back() - in.progress;
    }
    if ((lo.array() > hi.array()).any()) {return fail("empty_static_bounds");}
    Matrix transform = Matrix::Zero(); Vector offset = Vector::Zero();
    for (int k = 0; k < nodes; ++k) {
      offset.template segment<2>(S * k) = vec(in.state.last_applied_body_velocity);
      if constexpr (S == 4) {offset[S * k + 2] = in.state.last_applied_yaw_rate;}
      transform(S * k + S - 1, S * k + S - 1) = 1.;
      for (int j = 0; j <= k; ++j) {
        for (int axis = 0; axis < axes; ++axis) {transform(S * k + axis, S * j + axis) = j == 0 ? period : decision_dt;}
      }
    }
    const Matrix rate_P = transform.transpose() * P * transform;
    const Vector rate_q = transform.transpose() * (P * offset + q);
    const Eigen::MatrixXd rate_A = A * transform;
    Eigen::VectorXd rate_lo = lo - A * offset, rate_hi = hi - A * offset;
    Vector warm = seed;
    for (int k = 0; k < nodes; ++k) {
      for (int axis = 0; axis < axes; ++axis) {
        const double last = k == 0 ? offset[S * k + axis] : seed[S * (k - 1) + axis];
        warm[S * k + axis] = (seed[S * k + axis] - last) / (k == 0 ? period : decision_dt);
      }
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
    Vector z = offset + transform * solution; Vec2 last = in.state.last_applied_body_velocity;
    for (int k = 0; k < nodes; ++k) {
      const double dt = k == 0 ? period : decision_dt;
      last = {std::clamp(z[S * k], std::max(limits.lower.x, last.x - dt * limits.command_rate.x),
          std::min(limits.upper.x, last.x + dt * limits.command_rate.x)),
        std::clamp(z[S * k + 1], std::max(limits.lower.y, last.y - dt * limits.command_rate.y),
          std::min(limits.upper.y, last.y + dt * limits.command_rate.y))};
      z.template segment<2>(S * k) << last.x, last.y;
      z[S * k + S - 1] = std::clamp(z[S * k + S - 1], 0., limits.progress_upper);
      if constexpr (S == 4) {
        const double prior = k == 0 ? in.state.last_applied_yaw_rate : z[S * (k - 1) + 2];
        z[S * k + 2] = std::clamp(z[S * k + 2], std::max(in.limits.yaw_lower, prior - dt * in.limits.yaw_command_rate),
          std::min(in.limits.yaw_upper, prior + dt * in.limits.yaw_command_rate));
      }
    }
    const Eigen::VectorXd values = A * z;
    result.minimum_constraint_slack = std::min((values - lo).minCoeff(), (hi - values).minCoeff());
    if (!z.allFinite() || *result.minimum_constraint_slack < -1e-5) {return fail("hard_static_or_bounds");}
    Proposal proposal; proposal.identity = in.identity; proposal.input_digest = digest;
    proposal.receipt_digest = in.prediction.receipt_digest(); proposal.path_digest = in.route.path_digest();
    proposal.map_digest = in.route.map_digest(); proposal.limits_digest = in.limits.digest();
    proposal.epoch_ns = in.prediction.epoch_ns(); proposal.acquired = in.acquired;
    proposal.source_deadline = in.acquired + std::chrono::milliseconds(75);
    proposal.body_velocity = {z[0], z[1]}; proposal.stages = rollout(in, map, z);
    for (int k = 0; k < nodes; ++k) {
      if constexpr (S == 3) {proposal.controls[k] = {{z[S * k], z[S * k + 1]}, z[S * k + 2]};}
      else {proposal.controls[k] = {{z[S * k], z[S * k + 1]}, z[S * k + 2], z[S * k + 3]};}
    }
    if constexpr (S == 4) {
      proposal.yaw_rate = z[2];
      // Continuous held-twist support containment in the original certified rectangle.
      for (int k = 0; k < stages - 1; ++k) {
        const auto & a = proposal.stages[k]; const auto & b = proposal.stages[k + 1];
        const auto c = proposal.controls[k / 2]; const auto support = in.body.swept_support(a.yaw, b.yaw);
        const double chord = std::hypot(c.body_velocity.x, c.body_velocity.y) * std::abs(c.yaw_rate) * period * period / 8;
        const double margin = chord + in.body.static_clearance;
        if (std::min(a.position.x, b.position.x) + support.xmin - margin < box.xmin - 1e-5 ||
          std::max(a.position.x, b.position.x) + support.xmax + margin > box.xmax + 1e-5 ||
          std::min(a.position.y, b.position.y) + support.ymin - margin < box.ymin - 1e-5 ||
          std::max(a.position.y, b.position.y) + support.ymax + margin > box.ymax + 1e-5)
        {return fail("nonlinear_static_support");}
      }
    }
    for (int k = 0; k < 30; ++k) {
      const auto soft = [&]() {
          if constexpr (S == 3) {return field.sample(proposal.stages[k].position, k);}
          else {return field.sample(proposal.stages[k].position, proposal.stages[k].yaw, k);}
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
{return solve_model<3, FollowInput, FollowProposal>(std::move(in), *impl_);}
RotatingFollowResult RotatingFollowSolver::solve(RotatingFollowInput in)
{return solve_model<4, RotatingFollowInput, RotatingFollowProposal>(std::move(in), *impl_);}
}  // namespace rm_r4_prediction_consumption
