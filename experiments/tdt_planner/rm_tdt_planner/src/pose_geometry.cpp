// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#include "rm_tdt_planner/pose_geometry.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rm_tdt_planner::pose_geometry
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kGuard = 1e-7;
using Clock = std::chrono::steady_clock;
bool finite(Point p) {return std::isfinite(p.x) && std::isfinite(p.y);}
double cross(Point a, Point b, Point c)
{return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);}

std::vector<Cell> cells(const RawCostmapInput & g)
{
  if (g.width < 3 || g.height < 3 || g.width > 4096 || g.height > 4096 ||
    static_cast<size_t>(g.width)*g.height > 1000000 ||
    g.costs.size() != static_cast<size_t>(g.width)*g.height ||
    !std::isfinite(g.resolution) || g.resolution < .001 || g.resolution > 1.0 ||
    !finite({g.origin_x,g.origin_y}) || std::abs(g.origin_x) > 1e6 || std::abs(g.origin_y) > 1e6)
  {throw std::invalid_argument("invalid raw Nav2 snapshot");}
  std::vector<Cell> out;
  for (int y=0; y<g.height; ++y) {
    for (int x=0; x<g.width; ++x) {
      const bool boundary = x==0 || y==0 || x==g.width-1 || y==g.height-1;
      const auto cost=g.costs[y*g.width+x];
      if (boundary || cost>=253) {out.push_back({x,y,cost,boundary});}
    }
  }
  return out;
}

bool valid_footprint(const std::vector<Point> & p, double & radius)
{
  if (p.size()<3 || p.size()>32) {return false;}
  radius=0.0;
  for (auto v:p) {
    if (!finite(v) || std::hypot(v.x,v.y)>5.0) {return false;}
    radius=std::max(radius,std::hypot(v.x,v.y));
  }
  const double orientation=cross(p[0],p[1],p[2])>0.0 ? 1.0 : -1.0;
  for (size_t i=0; i<p.size(); ++i) {
    const auto a=p[i], b=p[(i+1)%p.size()];
    if (std::hypot(b.x-a.x,b.y-a.y)<1e-6 || orientation*cross(a,b,{0,0})<=1e-12) {
      return false;
    }
    // All non-edge vertices strictly on the interior side. This also rejects
    // self-intersecting stars that can pass a successive-turn-only convexity test.
    for (size_t j=0; j<p.size(); ++j) {
      if (j!=i && j!=(i+1)%p.size() && orientation*cross(a,b,p[j])<=1e-12) {return false;}
    }
  }
  return true;
}

double point_edge(Point p, Point a, Point b)
{
  const double dx=b.x-a.x, dy=b.y-a.y;
  const double norm=dx*dx+dy*dy;
  const double t=norm>0.0 ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/norm,0.0,1.0) : 0.0;
  return std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
}

double polygon_square(const std::vector<Point> & polygon, double x, double y, double r)
{
  const std::array<Point,4> box{{{x,y},{x+r,y},{x+r,y+r},{x,y+r}}};
  auto separate = [&](Point axis) {
      double pmin=std::numeric_limits<double>::infinity(), pmax=-pmin;
      double bmin=pmin, bmax=-bmin;
      for (auto p:polygon) {const double v=p.x*axis.x+p.y*axis.y; pmin=std::min(pmin,v); pmax=std::max(pmax,v);}
      for (auto p:box) {const double v=p.x*axis.x+p.y*axis.y; bmin=std::min(bmin,v); bmax=std::max(bmax,v);}
      return pmax<bmin || bmax<pmin;
    };
  bool separated=separate({1,0}) || separate({0,1});
  for (size_t i=0; i<polygon.size() && !separated; ++i) {
    const auto a=polygon[i], b=polygon[(i+1)%polygon.size()];
    separated=separate({a.y-b.y,b.x-a.x});
  }
  if (!separated) {return 0.0;}  // Closed overlap, containment and contact.
  double distance=std::numeric_limits<double>::infinity();
  for (size_t i=0; i<polygon.size(); ++i) {
    for (size_t j=0; j<box.size(); ++j) {
      distance=std::min({distance,
        point_edge(polygon[i],box[j],box[(j+1)%box.size()]),
        point_edge(box[j],polygon[i],polygon[(i+1)%polygon.size()])});
    }
  }
  return distance;
}
}  // namespace

RawSnapshot::RawSnapshot(RawCostmapInput input)
: raw_(std::move(input)), blocked_(cells(raw_)) {}

Result certify(const RawSnapshot & snapshot, const std::vector<Point> & footprint,
  const Motion & motion, const Limits & limits)
{
  const auto begin=Clock::now();
  Result result;
  const auto & g=snapshot.raw();
  auto elapsed=[&]() {return std::chrono::duration<double>(Clock::now()-begin).count();};
  bool valid_budget=std::isfinite(limits.time_budget_seconds) &&
    limits.time_budget_seconds>0.0 && limits.time_budget_seconds<=30.0;
  auto finish=[&]() {
      result.elapsed_seconds=elapsed();
      if (valid_budget && result.elapsed_seconds>limits.time_budget_seconds) {
        result.status=Status::Unresolved; result.reason="time budget exhausted"; result.witness.reset();
      }
      if (result.status!=Status::Certified) {result.intervals.clear();}
      return result;
    };
  auto bounded_pose=[](Pose p) {
      return finite({p.x,p.y}) && std::abs(p.x)<=1e6 && std::abs(p.y)<=1e6 &&
             std::isfinite(p.yaw) && std::abs(p.yaw)<=100*kPi;
    };
  double radius=0.0;
  if (!valid_budget || !std::isfinite(limits.clearance) || limits.clearance<0.0 || limits.clearance>2.0 ||
    limits.max_intervals<1 || limits.max_intervals>65535 ||
    limits.max_cell_checks<1 || limits.max_cell_checks>100000000 || limits.max_depth>30 ||
    !bounded_pose(motion.start) || !bounded_pose(motion.goal) ||
    !std::isfinite(motion.yaw_delta) || std::abs(motion.yaw_delta)>8*kPi ||
    std::abs(std::remainder(motion.start.yaw+motion.yaw_delta-motion.goal.yaw,2*kPi))>1e-10 ||
    !valid_footprint(footprint,radius))
  {result.reason="invalid motion, footprint or limits"; return finish();}

  const Point a{motion.start.x-g.origin_x,motion.start.y-g.origin_y};
  const Point b{motion.goal.x-g.origin_x,motion.goal.y-g.origin_y};
  const double translation=std::hypot(b.x-a.x,b.y-a.y);
  auto sample = [&](double t, double translation_bound, double rotation_bound, double & margin) {
      const Point centre{a.x+t*(b.x-a.x), a.y+t*(b.y-a.y)};
      const double yaw=motion.start.yaw+t*motion.yaw_delta;
      const Pose world{centre.x+g.origin_x,centre.y+g.origin_y,yaw};
      if (elapsed()>limits.time_budget_seconds) {result.reason="time budget exhausted"; return false;}
      if (centre.x<0 || centre.y<0 || centre.x>=g.width*g.resolution || centre.y>=g.height*g.resolution) {
        result.status=Status::Collision; result.reason="centre outside map extent";
        result.witness=Witness{Constraint::MapExtent,-1,-1,0,true,world,t,0.0,0.0};
        return false;
      }
      std::vector<Point> polygon; polygon.reserve(footprint.size());
      const double c=std::cos(yaw), s=std::sin(yaw);
      // Relative to centre to avoid large-origin cancellation in polygon projections.
      for (auto p:footprint) {polygon.push_back({c*p.x-s*p.y,s*p.x+c*p.y});}
      margin=std::numeric_limits<double>::infinity();
      // First implementation deliberately checks every blocking cell. No spatial
      // pruning can accidentally omit a square crossed between sample poses.
      for (const auto & cell:snapshot.blocked_cells()) {
        if (result.cell_checks>=limits.max_cell_checks) {result.reason="cell check budget exhausted"; return false;}
        if (elapsed()>limits.time_budget_seconds) {result.reason="time budget exhausted"; return false;}
        ++result.cell_checks;
        const double x=cell.x*g.resolution-centre.x, y=cell.y*g.resolution-centre.y;
        const bool hard=cell.boundary || cell.cost>=254;
        const double d=hard ? polygon_square(polygon,x,y,g.resolution) :
          std::hypot(std::max({x,0.0,-x-g.resolution}),std::max({y,0.0,-y-g.resolution}));
        const double required=hard ? limits.clearance : 0.0;
        if (d<=required) {
          result.status=Status::Collision; result.reason=hard ? "footprint clearance violated" : "centre touches inscribed cell";
          result.witness=Witness{hard ? Constraint::Footprint : Constraint::InscribedCentre,
            cell.x,cell.y,cell.cost,cell.boundary,world,t,d,required};
          return false;
        }
        if (d<=required+kGuard) {
          result.reason="distance within numerical guard"; return false;
        }
        const double displacement=translation_bound+(hard ? rotation_bound : 0.0);
        margin=std::min(margin,d-required-displacement);
      }
      return true;
    };
  double margin=0.0;
  if (!sample(0.0,0.0,0.0,margin) || !sample(1.0,0.0,0.0,margin)) {return finish();}
  struct Pending {double begin,end; unsigned int depth;};
  std::vector<Pending> stack{{0.0,1.0,0}};
  while (!stack.empty()) {
    if (result.intervals_examined>=limits.max_intervals) {
      result.reason="interval budget exhausted"; return finish();
    }
    const auto interval=stack.back(); stack.pop_back(); ++result.intervals_examined;
    const double half=(interval.end-interval.begin)*.5;
    const double middle=interval.begin+half;
    if (!sample(middle,translation*half,radius*std::abs(motion.yaw_delta)*half,margin)) {return finish();}
    if (margin>kGuard) {
      result.intervals.push_back({interval.begin,interval.end,margin});
    } else {
      if (interval.depth>=limits.max_depth) {result.reason="subdivision depth exhausted"; return finish();}
      stack.push_back({middle,interval.end,interval.depth+1});
      stack.push_back({interval.begin,middle,interval.depth+1});
    }
  }
  result.status=Status::Certified;
  result.reason="complete continuous linear-pose interval cover (offline only)";
  return finish();
}
}  // namespace rm_tdt_planner::pose_geometry
