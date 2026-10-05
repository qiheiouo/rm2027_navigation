#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include "fixtures.hpp"
#include "rm_r4_prediction_consumption/follow.hpp"
using namespace r4_test;
int main()
{
  std::cout << std::setprecision(17) << '[';
  for (int scenario = 0; scenario < 4; ++scenario) {
    auto b = body(); b.yaw = scenario == 2 ? std::acos(-1.) / 2 : 0.;
    auto e = envelope(source + 50000000);
    if (scenario != 3) {e.prediction.tracks.clear(); e.prediction.total_track_count = 0; e.tracks.clear();}
    else {e.prediction.tracks[0].velocity.x = 0.; e.tracks[0].centroid_at_observation.x = .7; align(e);}
    auto path_value = path(); if (scenario == 1) {std::reverse(path_value.poses.begin(), path_value.poses.end());}
    auto prediction = PredictionSnapshot::freeze(e, source + 50000000, "map", b);
    auto route = PreparedCorridor::prepare(path_value, grid(), b, 4);
    const auto epoch = prediction.epoch_ns();
    FollowInput in{prediction, route, b, {{-.3, -.5}, {.5, .5}, {.8, .8}, .4, .5},
      {"host-A", "action-A", "base_link", 1, 1},
      {{0., 0.}, {0., 0.}, {0., 0.}, b.yaw, 0., 0., epoch, epoch, epoch, epoch, "map", "base_link"},
      1., FollowClock::now(), false};
    FollowSolver solver; const auto r = solver.solve(in);
    if (!r.proposal) {std::cerr << r.reason << '/' << r.solver_status; return 1;}
    const auto bounds = route.local_bounds(in.state.position);
    if (scenario) {std::cout << ',';}
    std::cout << "{\"scenario\":" << scenario << ",\"yaw\":" << b.yaw << ",\"bounds\":[" <<
      bounds.xmin << ',' << bounds.xmax << ',' << bounds.ymin << ',' << bounds.ymax << "],\"solver_s\":" <<
      r.solver_seconds << ",\"elapsed_s\":" << r.elapsed_seconds << ",\"iterations\":" << r.iterations <<
      ",\"nominal_dynamic_cost\":" << r.nominal_dynamic_cost << ",\"solved_dynamic_cost\":" <<
      r.solved_dynamic_cost << ",\"controls\":[";
    for (size_t k = 0; k < 15; ++k) {
      if (k) {std::cout << ',';} const auto c = r.proposal->controls[k];
      std::cout << '[' << c.body_velocity.x << ',' << c.body_velocity.y << ',' << c.progress_rate << ']';
    }
    std::cout << "],\"stages\":[";
    for (size_t k = 0; k < 31; ++k) {
      if (k) {std::cout << ',';} const auto s = r.proposal->stages[k];
      std::cout << '[' << s.position.x << ',' << s.position.y << ',' << s.yaw << ',' <<
        s.body_velocity.x << ',' << s.body_velocity.y << ",0," << s.progress << ']';
    }
    std::cout << "]}";
  }
  std::cout << "]\n";
}
