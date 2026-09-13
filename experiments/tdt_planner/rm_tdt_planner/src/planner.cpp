// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#include "rm_tdt_planner/planner.hpp"
#include "YAstar/yastar.hpp"
#include "MinimumSnapOsqp/minimumSnap.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rm_tdt_planner
{
namespace
{
using Clock = std::chrono::steady_clock;
bool finite(Point p) {return std::isfinite(p.x) && std::isfinite(p.y);}
double distance(Point a, Point b) {return std::hypot(a.x - b.x, a.y - b.y);}

void validate(const Grid & g, const Options & o)
{
  if (g.width < 3 || g.height < 3 || g.width > 4096 || g.height > 4096 ||
    static_cast<size_t>(g.width) * g.height > 1000000 ||
    g.costs.size() != static_cast<size_t>(g.width) * g.height ||
    !std::isfinite(g.resolution) || g.resolution < 0.001 || g.resolution > 1.0 ||
    !std::isfinite(g.origin_x) || !std::isfinite(g.origin_y))
  {
    throw std::invalid_argument("invalid or oversized axis-aligned costmap");
  }
  for (double value : {o.radius, o.clearance, o.potential_weight, o.simplify_tolerance,
      o.output_spacing, o.time_budget, o.nominal_speed, o.nominal_acceleration,
      o.corridor_range})
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("nonfinite or negative planner parameter");
    }
  }
  if (o.radius > 5.0 || o.clearance > 2.0 || o.potential_weight > 100.0 ||
    o.simplify_tolerance > 2.0 || o.output_spacing < 0.001 || o.output_spacing > 1.0 ||
    o.time_budget < 0.001 || o.time_budget > 10.0 ||
    o.nominal_speed < 0.05 || o.nominal_speed > 10.0 ||
    o.nominal_acceleration < 0.05 || o.nominal_acceleration > 20.0 ||
    o.corridor_range > 5.0 || o.max_waypoints < 2 || o.max_waypoints > 32 ||
    o.collision_iterations < 1 || o.collision_iterations > 4)
  {
    throw std::invalid_argument("planner parameter outside experiment bounds");
  }
}

struct CollisionGrid
{
  int width;
  int height;
  double resolution;
  std::vector<uint8_t> free;

  bool inside(Point p) const
  {
    return finite(p) && p.x >= 0.0 && p.y >= 0.0 &&
           p.x < width * resolution && p.y < height * resolution;
  }
  bool cell(int x, int y) const
  {
    return x >= 0 && x < width && y >= 0 && y < height && free[y * width + x] != 0;
  }
  bool point(Point p) const
  {
    return inside(p) && cell(static_cast<int>(std::floor(p.x / resolution)),
      static_cast<int>(std::floor(p.y / resolution)));
  }

  // Supercover DDA: include both cells when crossing exactly through a corner.
  // Inflation also reserves a full cell diagonal, including boundary-touch error.
  bool segment(Point a, Point b) const
  {
    if (!point(a) || !point(b)) {return false;}
    a.x /= resolution; a.y /= resolution;
    b.x /= resolution; b.y /= resolution;
    int x = static_cast<int>(std::floor(a.x));
    int y = static_cast<int>(std::floor(a.y));
    const int end_x = static_cast<int>(std::floor(b.x));
    const int end_y = static_cast<int>(std::floor(b.y));
    const double dx = b.x - a.x, dy = b.y - a.y;
    const int sx = (dx > 0.0) - (dx < 0.0), sy = (dy > 0.0) - (dy < 0.0);
    const double inf = std::numeric_limits<double>::infinity();
    const double delta_x = sx ? 1.0 / std::abs(dx) : inf;
    const double delta_y = sy ? 1.0 / std::abs(dy) : inf;
    double tx = sx ? ((sx > 0 ? x + 1.0 : x) - a.x) / dx : inf;
    double ty = sy ? ((sy > 0 ? y + 1.0 : y) - a.y) / dy : inf;
    for (int step = 0; step <= width + height + 2; ++step) {
      if (!cell(x, y)) {return false;}
      if (x == end_x && y == end_y) {return true;}
      // A grid-line endpoint can round into the preceding cell on one axis.
      // Do not step past an axis whose destination cell is already reached.
      if (x == end_x) {tx = inf;}
      if (y == end_y) {ty = inf;}
      if (std::abs(tx - ty) <= 1e-12) {
        if (!cell(x + sx, y) || !cell(x, y + sy)) {return false;}
        x += sx; y += sy; tx += delta_x; ty += delta_y;
      } else if (tx < ty) {
        x += sx; tx += delta_x;
      } else {
        y += sy; ty += delta_y;
      }
    }
    return false;
  }
  bool path(const std::vector<Point> & points) const
  {
    if (points.empty() || !point(points.front())) {return false;}
    for (size_t i = 1; i < points.size(); ++i) {
      if (!segment(points[i - 1], points[i])) {return false;}
    }
    return true;
  }
};

CollisionGrid inflate(const Grid & g, const Options & o)
{
  CollisionGrid c{g.width, g.height, g.resolution, g.costs};
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x < g.width; ++x) {
      const int i = y * g.width + x;
      c.free[i] = (g.costs[i] >= 253 || x == 0 || y == 0 ||
        x == g.width - 1 || y == g.height - 1) ? 0 : 255;
    }
  }
  YAstar field(g.width, g.height, g.resolution, 0.0f, 0.0f);
  field.setMap(g.width, g.height, c.free.data());
  const auto sdf = field.getSDF();
  // Distance transform measures cell centres. Reserve both half diagonals to
  // cover any robot centre in the free cell and any point in the obstacle cell.
  const double blocked_distance = o.radius + o.clearance + std::sqrt(2.0) * g.resolution;
  for (size_t i = 0; i < c.free.size(); ++i) {
    c.free[i] = sdf.data()[i] <= blocked_distance + 1e-6 ? 0 : 255;
  }
  return c;
}

std::vector<Point> local_path(const Grid & g, const std::vector<Point> & path)
{
  auto result = path;
  for (auto & p : result) {p.x -= g.origin_x; p.y -= g.origin_y;}
  return result;
}

std::vector<Point> resample(const std::vector<Point> & path, double spacing)
{
  if (path.empty()) {return {};}
  std::vector<Point> out{path.front()};
  for (size_t i = 1; i < path.size(); ++i) {
    const Point a = path[i - 1], b = path[i];
    const double length = distance(a, b);
    if (length <= 1e-9) {continue;}
    const double steps = std::ceil(length / spacing);
    if (steps > 100000 || out.size() + steps > 100000) {
      throw std::runtime_error("output sample budget exceeded");
    }
    for (int j = 1; j <= static_cast<int>(steps); ++j) {
      const double t = j / steps;
      out.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
    }
  }
  return out;
}
}  // namespace

bool collision_free(const Grid & g, const std::vector<Point> & path, const Options & o)
{
  try {
    validate(g, o);
    return inflate(g, o).path(local_path(g, path));
  } catch (const std::exception &) {return false;}
}

PreparedGrid::PreparedGrid(Grid grid, double radius, double clearance)
: grid_(std::move(grid)), radius_(radius), clearance_(clearance) {}

PreparedGrid prepare_grid(const Grid & g, const Options & o)
{
  validate(g, o);
  const auto collision = inflate(g, o);
  Grid prepared = g;
  for (size_t i = 0; i < prepared.costs.size(); ++i) {
    prepared.costs[i] = collision.free[i] ? 0 : 254;
  }
  return PreparedGrid(std::move(prepared), o.radius, o.clearance);
}

static CollisionGrid prepared_collision(const Grid & g)
{
  CollisionGrid c{g.width, g.height, g.resolution, g.costs};
  for (auto & cost : c.free) {cost = cost >= 253 ? 0 : 255;}
  return c;
}

bool collision_free_prepared(const PreparedGrid & prepared, const std::vector<Point> & path)
{
  return prepared_collision(prepared.grid()).path(local_path(prepared.grid(), path));
}

static Result plan_impl(const Grid & g, Point start, Point goal, const Options & o,
  bool already_prepared)
{
  const auto begin = Clock::now();
  Result result;
  auto elapsed = [&]() {return std::chrono::duration<double>(Clock::now() - begin).count();};
  auto finish = [&]() {
      result.elapsed_seconds = elapsed();
      if (result.elapsed_seconds > o.time_budget) {
        result.success = false; result.optimized = false; result.path.clear();
        result.reason = "planning deadline exceeded (late output discarded)";
      }
      return result;
    };
  try {
    validate(g, o);
    start.x -= g.origin_x; start.y -= g.origin_y;
    goal.x -= g.origin_x; goal.y -= g.origin_y;
    const auto collision = already_prepared ? prepared_collision(g) : inflate(g, o);
    if (!collision.point(start) || !collision.point(goal)) {
      result.reason = "start or goal outside conservative free space";
      return finish();
    }
    if (distance(start, goal) < 1e-8) {
      result.path = {{start.x + g.origin_x, start.y + g.origin_y}};
      result.success = true; result.reason = "already at goal";
      return finish();
    }

    YAstar astar(g.width, g.height, g.resolution, 0.0f, 0.0f);
    auto mask = collision.free;
    astar.setMap(g.width, g.height, mask.data());
    astar.setCostField(1.0f, [&](float d) {return o.potential_weight / (0.1 + d);});
    astar.initCostMap(true);
    auto raw = astar.search({static_cast<float>(start.x), static_cast<float>(start.y)},
      {static_cast<float>(goal.x), static_cast<float>(goal.y)},
      [&]() {return elapsed() > o.time_budget;});
    if (raw.empty()) {result.reason = "no A* path"; return finish();}

    std::vector<Point> base{start};
    // YAstar returns cell corners. Move to centres, preserve exact requested endpoints.
    for (const auto & p : raw) {
      Point centre{p.x() + g.resolution * 0.5, p.y() + g.resolution * 0.5};
      if (distance(base.back(), centre) > 1e-7) {base.push_back(centre);}
    }
    if (distance(base.back(), goal) > 1e-7) {base.push_back(goal);} else {base.back() = goal;}
    if (!collision.path(base)) {
      result.reason = "A* failed independent segment validation"; return finish();
    }
    std::vector<Eigen::Vector2f> controls;
    for (const auto & p : base) {controls.emplace_back(p.x, p.y);}
    auto simplified = astar.simplifyPath(controls, o.simplify_tolerance);
    std::vector<Point> simple;
    for (const auto & p : simplified) {simple.push_back({p.x(), p.y()});}
    if (!simple.empty()) {simple.front() = start; simple.back() = goal;}
    if (collision.path(simple)) {base = simple;}

    result.reason = o.optimize ? "validated A* fallback: waypoint or time budget" : "validated A*";
    if (o.optimize && base.size() >= 2 && base.size() <= static_cast<size_t>(o.max_waypoints) &&
      elapsed() < o.time_budget)
    {
      MinimumSnap backend;
      const Eigen::Map<const MinimumSnap::Map> map(mask.data(), g.height, g.width);
      backend.setMap(map, g.resolution, 0.0f, 0.0f);
      backend.setOrder(6); backend.setMaxDx(3);  // six coefficients, minimum jerk
      backend.setStrictCollision(true);
      backend.setTL(std::max(0.001, o.time_budget - elapsed()));
      MinimumSnap::SolveInput input;
      controls.clear();
      for (const auto & p : base) {controls.emplace_back(p.x, p.y);}
      input.setPath(controls); input.setInitVel(Eigen::Vector2f::Zero());
      input.setMaxSpeed(o.nominal_speed); input.setMaxAcc(o.nominal_acceleration);
      input.setNormTime(true); input.setBackend(MinimumSnap::Backend::OSQPCorridor);
      input.setMaxCorridorRange(o.corridor_range);
      input.setCollisionCheckIter(o.collision_iterations);
      const auto solution = backend.solve(input);
      result.reason = "validated A* fallback: optimizer failed";
      std::vector<Point> smooth;
      for (const auto & p : solution.path) {smooth.push_back({p.x(), p.y()});}
      if (solution.success && smooth.size() >= 2 &&
        distance(smooth.front(), start) <= 1e-3 && distance(smooth.back(), goal) <= 1e-3)
      {
        smooth.front() = start; smooth.back() = goal;
        if (collision.path(smooth)) {
          base = std::move(smooth); result.optimized = true;
          result.reason = "validated minimum-jerk polyline";
        } else {
          result.reason = "validated A* fallback: optimizer geometry rejected";
        }
      }
    }
    result.path = resample(base, std::min(o.output_spacing, g.resolution * 0.5));
    if (!collision.path(result.path)) {
      result.path.clear(); result.optimized = false;
      result.reason = "final output failed segment validation";
      return finish();
    }
    for (auto & p : result.path) {p.x += g.origin_x; p.y += g.origin_y;}
    result.success = true;
  } catch (const std::exception & error) {
    result.success = false; result.optimized = false; result.path.clear();
    result.reason = error.what();
  }
  return finish();
}

Result plan(const Grid & g, Point start, Point goal, const Options & o)
{
  return plan_impl(g, start, goal, o, false);
}

Result plan_prepared(const PreparedGrid & prepared, Point start, Point goal, const Options & o)
{
  if (o.radius != prepared.radius_ || o.clearance != prepared.clearance_) {
    Result result;
    result.reason = "prepared footprint or clearance differs from planning options";
    return result;
  }
  return plan_impl(prepared.grid(), start, goal, o, true);
}
}  // namespace rm_tdt_planner
