// A21 value-only probes reuse observed-member fixtures. No output or plant.
#define R4_ROTATION_PROBE_LIBRARY_ONLY
#include "../r4_rotation_value/probe.cpp"
bool same_source(const FollowState & a, const FollowState & b)
{
  return a.position.x == b.position.x && a.position.y == b.position.y &&
    a.measured_body_velocity.x == b.measured_body_velocity.x && a.measured_body_velocity.y == b.measured_body_velocity.y &&
    a.last_applied_body_velocity.x == b.last_applied_body_velocity.x &&
    a.last_applied_body_velocity.y == b.last_applied_body_velocity.y &&
    a.yaw == b.yaw && a.measured_yaw_rate == b.measured_yaw_rate && a.last_applied_yaw_rate == b.last_applied_yaw_rate &&
    a.pose_stamp_ns == b.pose_stamp_ns && a.velocity_stamp_ns == b.velocity_stamp_ns &&
    a.tf_stamp_ns == b.tf_stamp_ns && a.applied_stamp_ns == b.applied_stamp_ns && a.frame == b.frame && a.body_frame == b.body_frame;
}


#ifndef R4_WORLD_PROBE_LIBRARY_ONLY
int main(int argc, char ** argv)
{
  if (argc != 2 && argc != 3) {return 2;}
  try {
    std::ofstream out(argv[1]); out << std::setprecision(17);
    out << "case,cycle,valid,reason,status,vx,vy,progress_end,nominal_cost,solved_cost,solver_ms,elapsed_ms,current_clearance,minimum_clearance,plateau_stages,used_warm,source_preserved,model_x,model_y,rollout_error,paired_difference,plant_x,plant_y,mechanical_clearance\n";
    const FollowLimits limits{{-.5,-.5},{.8,.5},{1.,1.},.4,.5};
    auto dense = path(); auto midpoint = dense.poses.front(); midpoint.pose.position.x = -.5;
    dense.poses.insert(dense.poses.begin()+1,midpoint);
    midpoint.pose.position.x = .5; dense.poses.insert(dense.poses.begin()+3,midpoint);
    for (const std::string kind : {"world_clear", "world_hold", "world_cross", "world_hold_clear"}) {
      if(argc==3) {continue;}
      WorldFollowAdapter solver, equivalent; Vec2 seed{.35,0.};
      const int count = kind == "world_hold_clear" ? 60 : 1;
      for (int i = 0; i < count; ++i) {
        const int64_t epoch = source+int64_t{i}*50000000; auto b = body(); b.yaw = .35+i*.03; b.yaw_invariant_circle = true;
        const bool occupied = kind == "world_hold" || kind == "world_cross" || (kind == "world_hold_clear" && i < 30);
        auto envelope_value = fixture(epoch,occupied,kind == "world_cross");
        auto snapshot = PredictionSnapshot::freeze(envelope_value,epoch,"map",b);
        auto route = PreparedCorridor::prepare(dense,grid(),b,1);
        const auto source_epoch = epoch-30000000;
        // Same measured world vector across spinning source orientations.
        const Vec2 measured{.35*std::cos(b.yaw),-.35*std::sin(b.yaw)};
        FollowState raw{{0.,0.},measured,{.13,-.07},b.yaw,.6,.3,
          source_epoch,source_epoch,source_epoch,epoch,"map","base_link"};
        WorldFollowInput in{snapshot,route,b,limits,{"probe",kind,"base_link",1,uint64_t(i+1)},
          raw,seed,epoch,"map",1.,FollowClock::now(),false};
        const auto result = solver.solve(in);
        if (!same_source(result.source_state,raw)) {throw std::runtime_error("raw source changed");}
        // Change yaw, measured/history wz and body representation, holding world XY fixed.
        auto paired = in; paired.body.yaw += .7; paired.source_state.yaw = paired.body.yaw;
        paired.source_state.measured_body_velocity = {.35*std::cos(paired.body.yaw),-.35*std::sin(paired.body.yaw)};
        paired.source_state.measured_yaw_rate = -1.1; paired.source_state.last_applied_yaw_rate = -.8;
        paired.prediction = PredictionSnapshot::freeze(envelope_value,epoch,"map",paired.body);
        if (paired.body.digest() != b.digest()) {throw std::runtime_error("circle depends on yaw");}
        paired.acquired = FollowClock::now(); const auto other = equivalent.solve(paired);
        if (!result.proposal || !other.proposal) {throw std::runtime_error("world probe unavailable: "+result.reason+" / "+other.reason);}
        const auto & p = *result.proposal; const auto & q = *other.proposal;
        if (p.velocity_frame != "map" || p.input_digest == q.input_digest) {throw std::runtime_error("world/source identity");}
        double error=0.,difference=0.,clearance=1e9,current=1e9; int plateau=0; Vec2 predicted{.0105,0.};
        if (std::hypot(p.stages[0].position.x-predicted.x,p.stages[0].position.y-predicted.y)>1e-12)
        {throw std::runtime_error("source-to-epoch world model");}
        TemporalSoftField field(snapshot);
        for (size_t k=0;k<31;++k) {
          error=std::max(error,std::hypot(predicted.x-p.stages[k].position.x,predicted.y-p.stages[k].position.y));
          difference=std::max({difference,std::abs(p.stages[k].position.x-q.stages[k].position.x),
            std::abs(p.stages[k].position.y-q.stages[k].position.y),std::abs(p.stages[k].progress-q.stages[k].progress)});
          if(k==30) {continue;}
          const auto soft=field.sample(p.stages[k].position,k);
          if(soft.clearance) {clearance=std::min(clearance,*soft.clearance);if(k==0) {current=*soft.clearance;}}
          plateau+=soft.plateau;
          predicted.x+=.05*p.controls[k/2].world_velocity.x; predicted.y+=.05*p.controls[k/2].world_velocity.y;
        }
        if(error>1e-12 || difference>2e-5) {throw std::runtime_error("world rollout or yaw-independent solution");}
        seed=p.world_velocity;
        out<<kind<<','<<i<<",1,"<<result.reason<<','<<result.solver_status<<','<<seed.x<<','<<seed.y<<','
          <<p.stages.back().progress<<','<<result.nominal_dynamic_cost<<','<<result.solved_dynamic_cost<<','
          <<1000*result.solver_seconds<<','<<1000*result.elapsed_seconds<<','<<current<<','<<clearance<<','<<plateau<<','
          <<result.used_warm<<",1,"<<p.stages[0].position.x<<','<<p.stages[0].position.y<<','<<error<<','<<difference<<",0,0,\n";
      }
    }
    // Minimum feedback check: ideal world-velocity plant, same fixture/weights.
    // This is a test harness, not a controller, tracker or physical output owner.
    for (const std::string kind : {"feedback_observed", "feedback_no_dynamic", "feedback_long_hold"}) {
      if(argc==3 && kind!=argv[2]) {continue;}
      const int hold_cycles=kind=="feedback_long_hold"?120:30;
      WorldFollowAdapter solver; Vec2 plant{}, previous{.35,0.};
      for (int i=0;i<hold_cycles+60;++i) {
        const int64_t epoch=source+int64_t{i}*50000000;auto b=body();b.yaw=.35+i*.03;b.yaw_invariant_circle=true;
        const bool held=i<hold_cycles;auto e=fixture(epoch,held&&kind!="feedback_no_dynamic");
        auto snapshot=PredictionSnapshot::freeze(e,epoch,"map",b);auto route=PreparedCorridor::prepare(dense,grid(),b,1);
        const Vec2 measured{std::cos(b.yaw)*previous.x+std::sin(b.yaw)*previous.y,
          -std::sin(b.yaw)*previous.x+std::cos(b.yaw)*previous.y};
        FollowState raw{{plant.x-.03*previous.x,plant.y-.03*previous.y},measured,measured,b.yaw,.6,.6,
          epoch-30000000,epoch-30000000,epoch-30000000,epoch,"map","base_link"};
        WorldFollowInput in{snapshot,route,b,limits,{"probe",kind,"base_link",1,uint64_t(i+1)},
          raw,previous,epoch,"map",route.project(plant),FollowClock::now(),false};
        const auto result=solver.solve(in);
        if(!same_source(result.source_state,raw)) {throw std::runtime_error("raw feedback source changed");}
        if(!result.proposal) {
          // Record the first failed solve and stop this plant: no invented fallback.
          const double dx=std::abs(plant.x-.85)-.15,dy=std::abs(plant.y)-.2;
          const double mechanical=held?std::hypot(std::max(dx,0.),std::max(dy,0.))+std::min(std::max(dx,dy),0.)-b.support().xmax:1e9;
          out<<kind<<','<<i<<",0,"<<result.reason<<','<<result.solver_status<<",0,0,0,"<<result.nominal_dynamic_cost<<','
            <<result.solved_dynamic_cost<<','<<1000*result.solver_seconds<<','<<1000*result.elapsed_seconds
            <<",1000000000,1000000000,0,"<<result.used_warm<<",1,"<<plant.x<<','<<plant.y<<",0,0,"<<plant.x<<','<<plant.y<<','<<mechanical<<'\n';
          break;
        }
        const auto & p=*result.proposal;TemporalSoftField field(snapshot);
        double current=1e9,clearance=1e9;int plateau=0;
        for(size_t k=0;k<30;++k) {const auto soft=field.sample(p.stages[k].position,k);
          if(soft.clearance) {clearance=std::min(clearance,*soft.clearance);if(k==0) {current=*soft.clearance;}}plateau+=soft.plateau;
        }
        // Oracle is only the explicitly known test rectangle, never a solver input.
        const double dx=std::abs(plant.x-.85)-.15,dy=std::abs(plant.y)-.2;
        const double mechanical=held?std::hypot(std::max(dx,0.),std::max(dy,0.))+std::min(std::max(dx,dy),0.)-b.support().xmax:1e9;
        previous=p.world_velocity;
        out<<kind<<','<<i<<",1,"<<result.reason<<','<<result.solver_status<<','<<previous.x<<','<<previous.y<<','
          <<p.stages.back().progress<<','<<result.nominal_dynamic_cost<<','<<result.solved_dynamic_cost<<','
          <<1000*result.solver_seconds<<','<<1000*result.elapsed_seconds<<','<<current<<','<<clearance<<','<<plateau<<','
          <<result.used_warm<<",1,"<<p.stages[0].position.x<<','<<p.stages[0].position.y<<",0,0,"<<plant.x<<','<<plant.y<<','<<mechanical<<'\n';
        plant.x+=.05*previous.x;plant.y+=.05*previous.y;
      }
    }
  } catch (const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}

#endif
