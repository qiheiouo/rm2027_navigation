#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include "fixtures.hpp"
using namespace r4_test;
int main()
{
  std::cout << std::setprecision(17);
  for (double yaw : {0., std::acos(-1.) / 2}) {
    auto b = body(); b.yaw = yaw;
    TemporalSoftField f(PredictionSnapshot::freeze(envelope(), source + 50000000, "map", b));
    for (size_t stage : {0u, 5u, 15u, 30u}) {
      const auto cell = f.translated_cells(0, stage).back();
      for (Vec2 delta : {Vec2{0., 0.}, {.3, .4}, {.55, 0.}, {-.3, -.4}, {-.55, 0.}, {.8, .8}}) {
        const Vec2 point{cell.x + delta.x, cell.y + delta.y}; const auto s = f.sample(point, stage);
        std::cout << yaw << ',' << stage << ',' << point.x << ',' << point.y << ',' <<
          s.residual << ',' << s.gradient.x << ',' << s.gradient.y << ',' <<
          s.clearance.value_or(std::numeric_limits<double>::quiet_NaN()) << ',' << s.plateau << '\n';
      }
    }
  }
}
