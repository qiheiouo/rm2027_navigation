// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#pragma once
#include "rm_tdt_planner/planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace benchmark_geometry
{
using rm_tdt_planner::Point;
inline double point_box(Point p,double x0,double y0,double x1,double y1)
{
  return std::hypot(std::max({x0-p.x,0.0,p.x-x1}),std::max({y0-p.y,0.0,p.y-y1}));
}
inline double point_segment(Point p,Point a,Point b)
{
  const double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
  const double t=l2>0?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l2,0.0,1.0):0.0;
  return std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
}
inline double segment_box(Point a,Point b,double x0,double y0,double x1,double y1)
{
  double lo=0,hi=1;
  auto slab=[&](double a0,double d,double lower,double upper){
    if(std::abs(d)<1e-15){return a0>=lower&&a0<=upper;}
    double t0=(lower-a0)/d,t1=(upper-a0)/d;if(t0>t1){std::swap(t0,t1);}
    lo=std::max(lo,t0);hi=std::min(hi,t1);return lo<=hi;
  };
  if(slab(a.x,b.x-a.x,x0,x1)&&slab(a.y,b.y-a.y,y0,y1)){return 0;}
  double minimum=std::min(point_box(a,x0,y0,x1,y1),point_box(b,x0,y0,x1,y1));
  for(Point p: {Point{x0,y0},Point{x1,y0},Point{x0,y1},Point{x1,y1}}){
    minimum=std::min(minimum,point_segment(p,a,b));
  }
  return minimum;
}
// Exact segment-to-occupied-square distance, capped at a positive reporting band.
// Unlike an EDT/sampling bound, this can distinguish an actual disk collision
// from a conservative configuration-space or interpolation rejection.
inline double clearance(const rm_tdt_planner::Grid & g,const std::vector<Point> & path,
  double radius,double cap=0.25)
{
  if(path.empty()){return -radius;}
  double minimum=cap;
  const double reach=radius+cap;
  for(size_t i=0;i<path.size();++i){
    Point a=path[i?i-1:0],b=path[i];
    a.x-=g.origin_x;a.y-=g.origin_y;b.x-=g.origin_x;b.y-=g.origin_y;
    if(!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(b.x)||!std::isfinite(b.y)||
      a.x<0||a.y<0||b.x<0||b.y<0||a.x>=g.width*g.resolution||b.x>=g.width*g.resolution||
      a.y>=g.height*g.resolution||b.y>=g.height*g.resolution){return -radius;}
    int x0=std::max(0,static_cast<int>(std::floor((std::min(a.x,b.x)-reach)/g.resolution)));
    int y0=std::max(0,static_cast<int>(std::floor((std::min(a.y,b.y)-reach)/g.resolution)));
    int x1=std::min(g.width-1,static_cast<int>(std::floor((std::max(a.x,b.x)+reach)/g.resolution)));
    int y1=std::min(g.height-1,static_cast<int>(std::floor((std::max(a.y,b.y)+reach)/g.resolution)));
    for(int y=y0;y<=y1;++y){
      for(int x=x0;x<=x1;++x){
        if(g.costs[y*g.width+x]<253 && x!=0 && y!=0 && x!=g.width-1 && y!=g.height-1){continue;}
        double d=segment_box(a,b,x*g.resolution,y*g.resolution,(x+1)*g.resolution,(y+1)*g.resolution);
        minimum=std::min(minimum,d-radius);
      }
    }
  }
  return minimum;
}
}  // namespace benchmark_geometry
