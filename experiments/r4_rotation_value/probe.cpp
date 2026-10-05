// Controlled value probes only. No node, tracker, output owner or plant claim.
#include "../../src/rm_r4_prediction_consumption/test/fixtures.hpp"
#include <rm_r4_prediction_consumption/follow.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace r4_test;

auto fixture(int64_t epoch, bool obstacle, bool crossing = false)
{
  auto e = envelope(epoch);
  if (!obstacle) {e.prediction.tracks.clear(); e.prediction.total_track_count = 0; e.tracks.clear(); return e;}
  auto & t = e.prediction.tracks[0]; auto & m = e.tracks[0];
  t.state = t.STATE_CONFIRMED; t.miss_count = 0; t.last_observation_stamp = stamp(epoch);
  t.velocity.x = 0.; t.velocity.y = crossing ? -.6 : 0.;
  t.size.x = .3; t.size.y = .4;
  m.last_observation_stamp = stamp(epoch); m.association_sequence = e.sequence;
  m.centroid_at_observation.x = .85; m.centroid_at_observation.y = crossing ? .6 : 0.;
  m.local_endpoints.clear(); m.source_member_ids.clear();
  for (double x : {-.15, .15}) {for (double y : {-.2, 0., .2}) {
    geometry_msgs::msg::Point p; p.x = x; p.y = y; m.local_endpoints.push_back(p);
    m.source_member_ids.push_back(m.source_member_ids.size());
  }}
  align(e); return e;
}
int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  try {
    std::ofstream out(argv[1]); out << std::setprecision(17);
    out << "case,cycle,valid,reason,vx,vy,wz,progress_end,yaw_end,nominal_cost,solved_cost,solver_ms,elapsed_ms,slice_error,gradient_error,minimum_clearance,plateau_stages,used_warm\n";
    const FollowLimits limits{{-.5,-.5},{.8,.5},{1.,1.},.4,.5};
    auto path_value = path();
    // Rotation increases footprint erosion; 1m fixture spacing no longer covers
    // every segment. Use a normal dense path, without changing Sfc or its gate.
    auto midpoint=path_value.poses.front(); midpoint.pose.position.x=-.5;
    path_value.poses.insert(path_value.poses.begin()+1,midpoint);
    midpoint.pose.position.x=.5; path_value.poses.insert(path_value.poses.begin()+3,midpoint);
    for (const std::string kind : {"zero_slice", "rotating_clear", "rotating_hold", "rotating_cross", "hold_clear_sequence"}) {
      RotatingFollowSolver solver; Vec2 seed{.35,0.}; double seed_wz = 0.;
      const int count = kind == "hold_clear_sequence" ? 60 : 1;
      for (int i = 0; i < count; ++i) {
        const auto epoch = source + i * 50000000;
        auto b = body(); b.yaw = .35;
        const bool occupied = kind == "rotating_hold" || kind == "rotating_cross" || (kind == "hold_clear_sequence" && i < 30);
        auto snapshot = PredictionSnapshot::freeze(fixture(epoch, occupied, kind == "rotating_cross"), epoch, "map", b);
        auto route = PreparedCorridor::prepare(path_value, grid(), b, 1);
        RotatingFollowInput in{snapshot, route, b, {limits, kind == "zero_slice" ? 0. : -1.2,
          kind == "zero_slice" ? 0. : 1.2, 2.}, {"probe",kind,"base_link",1,uint64_t(i+1)},
          {{0.,0.},{.35,0.},seed,b.yaw,kind == "zero_slice" ? 0. : .6,seed_wz,
          epoch,epoch,epoch,epoch,"map","base_link"}, 1., FollowClock::now(),false};
        const auto result = solver.solve(in); double slice_error = 0., gradient_error = 0.;
        if (kind == "zero_slice") {
          FollowSolver fixed; FollowInput old{snapshot,route,b,limits,in.identity,in.state,1.,FollowClock::now(),false};
          const auto baseline = fixed.solve(old);
          if (!result.proposal || !baseline.proposal) {throw std::runtime_error("zero slice unavailable");}
          for (size_t k=0;k<15;++k) {
            const auto a=result.proposal->controls[k]; const auto c=baseline.proposal->controls[k];
            slice_error=std::max({slice_error,std::abs(a.body_velocity.x-c.body_velocity.x),
              std::abs(a.body_velocity.y-c.body_velocity.y),std::abs(a.progress_rate-c.progress_rate),std::abs(a.yaw_rate)});
          }
          if (slice_error > 2e-5) {throw std::runtime_error("zero slice differs from baseline");}
        }
        if (occupied) {
          TemporalSoftField field(snapshot); const Vec2 query{.05,.02}; const double yaw=.35, h=1e-6;
          const auto sample=field.sample(query,yaw,3);
          const double numeric=(field.sample(query,yaw+h,3).residual-field.sample(query,yaw-h,3).residual)/(2*h);
          gradient_error=std::abs(numeric-sample.yaw_gradient);
          if (gradient_error > 1e-5) {throw std::runtime_error("yaw residual gradient differs");}
        }
        Vec2 command{}; double omega=0., progress=0., yaw=0., clearance=1e9; int plateau=0;
        if (result.proposal) {
          command=result.proposal->body_velocity; omega=result.proposal->yaw_rate;
          progress=result.proposal->stages.back().progress; yaw=result.proposal->stages.back().yaw;
          TemporalSoftField field(snapshot);
          for(size_t k=0;k<30;++k) {const auto st=result.proposal->stages[k];
            const auto sample=field.sample(st.position,st.yaw,k);
            if(sample.clearance) {clearance=std::min(clearance,*sample.clearance);} plateau+=sample.plateau;
          }
          seed=command; seed_wz=omega;
        } else {seed={}; seed_wz=0.;}
        out<<kind<<','<<i<<','<<bool(result.proposal)<<','<<result.reason<<','<<command.x<<','<<command.y<<','<<omega<<','
          <<progress<<','<<yaw<<','<<result.nominal_dynamic_cost<<','<<result.solved_dynamic_cost<<','
          <<result.solver_seconds*1000<<','<<result.elapsed_seconds*1000<<','<<slice_error<<','<<gradient_error<<','
          <<clearance<<','<<plateau<<','<<result.used_warm<<'\n';
      }
    }
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
