#pragma once
#include <algorithm>
#include <array>
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
inline double polygon_area(const std::vector<Point> & polygon)
{
  if (polygon.size() < 3) return 0;
  double twice_area = 0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    twice_area += a.x * b.y - a.y * b.x;
  }
  return std::abs(twice_area) / 2;
}

inline std::vector<Point> clip_boundary(
  const std::vector<Point> & polygon, int axis, double limit, bool keep_above)
{
  if (polygon.empty()) return {};
  const auto component = [axis](Point p) {return axis == 0 ? p.x : p.y;};
  const auto inside = [&](Point p) {
      return keep_above ? component(p) >= limit : component(p) <= limit;
    };
  std::vector<Point> clipped;
  clipped.reserve(polygon.size() + 2);
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const bool a_inside = inside(a);
    const bool b_inside = inside(b);
    if (a_inside != b_inside) {
      const double fraction = (limit - component(a)) / (component(b) - component(a));
      clipped.push_back({a.x + fraction * (b.x - a.x),
                         a.y + fraction * (b.y - a.y)});
    }
    if (b_inside) clipped.push_back(b);
  }
  return clipped;
}

inline double polygon_box_overlap_area(const std::vector<Point> & polygon, const Box & box)
{
  if (polygon.size() < 3 || box.min_x > box.max_x || box.min_y > box.max_y)
    throw std::invalid_argument("invalid overlap geometry");
  auto clipped = clip_boundary(polygon, 0, box.min_x, true);
  clipped = clip_boundary(clipped, 0, box.max_x, false);
  clipped = clip_boundary(clipped, 1, box.min_y, true);
  clipped = clip_boundary(clipped, 1, box.max_y, false);
  return polygon_area(clipped);
}

inline double uniform_center_overlap_fraction(
  const std::vector<Point> & robot_polygon, Point observed, Point velocity,
  Point visible_size, Point object_extent, double age, double horizon,
  double reference_acceleration)
{
  // Validate the same inputs as the hard conservative occupancy box.
  (void)predicted_box(
    observed, velocity, visible_size, object_extent, age, horizon,
    reference_acceleration);
  const double robot_area = polygon_area(robot_polygon);
  if (!(robot_area > 0)) throw std::invalid_argument("invalid robot polygon area");
  const double duration = age + horizon;
  const double mismatch = .5 * reference_acceleration * duration * duration;
  const Point center{observed.x + velocity.x * duration,
                     observed.y + velocity.y * duration};
  // The union of these physical boxes is the existing hard occupancy box.
  const Point support{visible_size.x / 2 + object_extent.x / 2 + mismatch,
                      visible_size.y / 2 + object_extent.y / 2 + mismatch};
  constexpr std::array<double, 5> nodes{
    -0.906179845938664, -0.538469310105683, 0.,
    0.538469310105683, 0.906179845938664};
  constexpr std::array<double, 5> weights{
    0.118463442528095, 0.239314335249683, 0.284444444444444,
    0.239314335249683, 0.118463442528095};
  double expected_area = 0;
  for (size_t ix = 0; ix < nodes.size(); ++ix) {
    for (size_t iy = 0; iy < nodes.size(); ++iy) {
      const double cx = center.x + nodes[ix] * support.x;
      const double cy = center.y + nodes[iy] * support.y;
      const Box physical{cx - object_extent.x / 2, cy - object_extent.y / 2,
                         cx + object_extent.x / 2, cy + object_extent.y / 2};
      expected_area += weights[ix] * weights[iy] *
        polygon_box_overlap_area(robot_polygon, physical);
    }
  }
  return std::clamp(expected_area / robot_area, 0., 1.);
}
} // namespace rm_dynamic_prediction_critic
