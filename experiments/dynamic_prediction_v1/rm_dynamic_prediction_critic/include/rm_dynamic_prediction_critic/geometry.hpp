#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rm_dynamic_prediction_critic {
struct Point {double x, y;};
struct Box {double min_x, min_y, max_x, max_y;};
inline bool finite(double x) {return std::isfinite(x);}
inline Box predicted_box(Point center, Point velocity, Point visible_size,
  Point object_extent, double age, double horizon, double reference_acceleration)
{
  const double values[] = {center.x, center.y, velocity.x, velocity.y,
    visible_size.x, visible_size.y, object_extent.x, object_extent.y,
    age, horizon, reference_acceleration};
  for (double value : values) if (!finite(value)) throw std::invalid_argument("nonfinite occupancy");
  if (visible_size.x < 0 || visible_size.y < 0 || object_extent.x <= 0 ||
      object_extent.y <= 0 || age < 0 || horizon < 0 || reference_acceleration < 0)
    throw std::invalid_argument("invalid occupancy dimension/time");
  const double duration = age + horizon;
  const double growth = .5 * reference_acceleration * duration * duration;
  const Point predicted{center.x + velocity.x * duration, center.y + velocity.y * duration};
  const Point half{visible_size.x / 2 + object_extent.x + growth,
                   visible_size.y / 2 + object_extent.y + growth};
  return {predicted.x - half.x, predicted.y - half.y,
          predicted.x + half.x, predicted.y + half.y};
}
inline std::vector<Point> transform(const std::vector<Point> & polygon,
  double x, double y, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<Point> out; out.reserve(polygon.size());
  for (const auto & p : polygon) out.push_back({x + c*p.x - s*p.y, y + s*p.x + c*p.y});
  return out;
}
inline double segment_distance(Point a, Point b, Point p)
{
  const double dx = b.x-a.x, dy = b.y-a.y;
  const double len2 = dx*dx+dy*dy;
  const double u = len2 > 0 ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/len2,0.0,1.0) : 0;
  return std::hypot(p.x-a.x-u*dx,p.y-a.y-u*dy);
}
inline bool separated_on_axis(const std::vector<Point> & poly, const std::vector<Point> & box,
  double ax, double ay)
{
  double pmin=std::numeric_limits<double>::infinity(),pmax=-pmin;
  double bmin=pmin,bmax=-pmin;
  for (const auto & p:poly) {const double v=p.x*ax+p.y*ay;pmin=std::min(pmin,v);pmax=std::max(pmax,v);}
  for (const auto & p:box) {const double v=p.x*ax+p.y*ay;bmin=std::min(bmin,v);bmax=std::max(bmax,v);}
  return pmax < bmin || bmax < pmin;
}
inline double polygon_box_distance(const std::vector<Point> & poly, const Box & b)
{
  if (poly.size() < 3 || b.min_x > b.max_x || b.min_y > b.max_y)
    throw std::invalid_argument("invalid polygon/box");
  const std::vector<Point> box{{b.min_x,b.min_y},{b.max_x,b.min_y},
                               {b.max_x,b.max_y},{b.min_x,b.max_y}};
  bool separated=separated_on_axis(poly,box,1,0)||separated_on_axis(poly,box,0,1);
  for (size_t i=0;i<poly.size();++i) {
    const auto & a=poly[i];const auto & c=poly[(i+1)%poly.size()];
    separated=separated||separated_on_axis(poly,box,-(c.y-a.y),c.x-a.x);
  }
  if (!separated) return 0;
  double distance=std::numeric_limits<double>::infinity();
  for (size_t i=0;i<poly.size();++i) {
    const auto & a=poly[i];const auto & c=poly[(i+1)%poly.size()];
    for (const auto & p:box) distance=std::min(distance,segment_distance(a,c,p));
  }
  for (size_t i=0;i<box.size();++i) {
    const auto & a=box[i];const auto & c=box[(i+1)%box.size()];
    for (const auto & p:poly) distance=std::min(distance,segment_distance(a,c,p));
  }
  return distance;
}
} // namespace rm_dynamic_prediction_critic
