// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#include "rm_tdt_planner/path_heading.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace rm_tdt_planner
{
std::vector<double> continuous_path_heading(const std::vector<Point> & path,
  double start_yaw, double goal_yaw)
{
  constexpr double tau=6.28318530717958647692;
  if (path.empty() || !std::isfinite(start_yaw) || !std::isfinite(goal_yaw)) {
    throw std::invalid_argument("heading requires a finite nonempty path");
  }
  std::vector<double> arc(path.size(),0), yaw(path.size(),0);
  for (size_t i=0;i<path.size();++i) {
    if (!std::isfinite(path[i].x) || !std::isfinite(path[i].y)) {
      throw std::invalid_argument("non-finite heading path");
    }
    if (i) {arc[i]=arc[i-1]+std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);}
    if (!std::isfinite(arc[i])) {throw std::invalid_argument("heading arc overflow");}
  }
  const double length=arc.back();
  if (length<1e-9) {
    // No translational interval can encode a turn. Preserve goal, do not invent motion.
    std::fill(yaw.begin(),yaw.end(),goal_yaw); return yaw;
  }
  auto at=[&](double s) {
      const auto it=std::upper_bound(arc.begin(),arc.end(),s);
      if (it==arc.end()) {return path.back();}
      const size_t j=it-arc.begin(),i=j-1;
      const double t=(s-arc[i])/(arc[j]-arc[i]);
      return Point{path[i].x+t*(path[j].x-path[i].x),path[i].y+t*(path[j].y-path[i].y)};
    };
  for (size_t i=0;i<path.size();++i) {
    const auto a=at(std::max(0.,arc[i]-.10)),b=at(std::min(length,arc[i]+.10));
    double value=std::atan2(b.y-a.y,b.x-a.x);
    if (std::hypot(b.x-a.x,b.y-a.y)<1e-9) {value=i ? yaw[i-1] : start_yaw;}
    yaw[i]=i ? yaw[i-1]+std::remainder(value-yaw[i-1],tau) : value;
  }
  const double start=start_yaw, aligned_start=yaw.front()+std::remainder(start-yaw.front(),tau);
  const double goal=yaw.back()+std::remainder(goal_yaw-yaw.back(),tau);
  auto smooth=[](double t) {t=std::clamp(t,0.,1.); return t*t*(3.-2.*t);};
  for (size_t i=0;i<path.size();++i) {
    const double start_weight=1.-smooth(arc[i]/std::min(.40,length*.5));
    const double goal_weight=smooth((arc[i]-(length-std::min(1.,length*.5)))/std::min(1.,length*.5));
    yaw[i]=(1.-start_weight-goal_weight)*yaw[i]+start_weight*aligned_start+goal_weight*goal;
  }
  return yaw;
}
}
