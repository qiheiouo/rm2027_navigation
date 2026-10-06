// Same A21 fixtures, ideal plant and raw body/world frame separation.
#define R4_WORLD_PROBE_LIBRARY_ONLY
#include "../r4_world_xy/probe.cpp"
#include "trace.hpp"
int main(int argc,char ** argv) {
  if(argc!=3 && argc!=4) {return 2;}
  r4_trace::mode=argv[2];const std::filesystem::path output=argv[1];
  std::filesystem::create_directories(output);std::ofstream out(output/"endpoint.csv");out<<std::setprecision(17);
  out<<"mode,condition,cycle,valid,reason,status,iterations,x,y,vx,vy,progress,progress_end,nominal_cost,solved_cost,solver_ms,elapsed_ms,used_warm,mechanical_clearance,minimum_predicted_clearance,epoch_ns\n";
  try {
    const FollowLimits limits{{-.5,-.5},{.8,.5},{1.,1.},.4,.5};
    auto dense=path();auto middle=dense.poses.front();middle.pose.position.x=-.5;dense.poses.insert(dense.poses.begin()+1,middle);
    middle.pose.position.x=.5;dense.poses.insert(dense.poses.begin()+3,middle);
    // First stage: only the already failed no-dynamic feedback, not a scene batch.
    for(const std::string condition : {"clear","hold30","hold120"}) {
      if(argc==3 && condition!="clear") {continue;}
      r4_trace::condition=condition;WorldFollowAdapter solver;Vec2 plant{},previous{.35,0.};
      const int hold=condition=="hold120"?120:(condition=="hold30"?30:0);
      for(int i=0;i<220;++i) {
        r4_trace::cycle=i;const int64_t epoch=source+int64_t{i}*50000000;auto b=body();b.yaw=.35+i*.03;b.yaw_invariant_circle=true;
        auto snapshot=PredictionSnapshot::freeze(fixture(epoch,i<hold),epoch,"map",b);
        auto route=PreparedCorridor::prepare(dense,grid(),b,1);
        const Vec2 measured{std::cos(b.yaw)*previous.x+std::sin(b.yaw)*previous.y,-std::sin(b.yaw)*previous.x+std::cos(b.yaw)*previous.y};
        FollowState raw{{plant.x-.03*previous.x,plant.y-.03*previous.y},measured,measured,b.yaw,.6,.6,
          epoch-30000000,epoch-30000000,epoch-30000000,epoch,"map","base_link"};
        WorldFollowInput in{snapshot,route,b,limits,{"probe",condition,"base_link",1,uint64_t(i+1)},raw,previous,epoch,"map",
          route.project(plant),FollowClock::now(),false};
        const auto result=solver.solve(in);if(!same_source(result.source_state,raw)) {throw std::runtime_error("raw state changed");}
        Vec2 command{};double end=0.,minimum=1e9;
        if(result.proposal) {command=result.proposal->world_velocity;end=result.proposal->stages.back().progress;TemporalSoftField field(snapshot);
          for(size_t k=0;k<30;++k) {auto soft=field.sample(result.proposal->stages[k].position,k);if(soft.clearance) {minimum=std::min(minimum,*soft.clearance);}}
        }
        const double dx=std::abs(plant.x-.85)-.15,dy=std::abs(plant.y)-.2;
        const double mechanical=i<hold?std::hypot(std::max(dx,0.),std::max(dy,0.))+std::min(std::max(dx,dy),0.)-b.support().xmax:1e9;
        out<<r4_trace::mode<<','<<condition<<','<<i<<','<<bool(result.proposal)<<','<<result.reason<<','<<result.solver_status<<','<<result.iterations<<','
          <<plant.x<<','<<plant.y<<','<<command.x<<','<<command.y<<','<<in.progress<<','<<end<<','<<result.nominal_dynamic_cost<<','<<result.solved_dynamic_cost<<','
          <<1000*result.solver_seconds<<','<<1000*result.elapsed_seconds<<','<<result.used_warm<<','<<mechanical<<','<<minimum<<','<<epoch<<'\n';
        if(!result.proposal) {break;}
        previous=command;plant.x+=.05*command.x;plant.y+=.05*command.y;
        if(r4_trace::mode=="rho_at42" && i==42) {break;}
        if(std::hypot(1.-plant.x,plant.y)<=.05) {break;} // Offline XY criterion; no angular/controller implementation.
      }
    }
    r4_trace::save(output);
  }catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
