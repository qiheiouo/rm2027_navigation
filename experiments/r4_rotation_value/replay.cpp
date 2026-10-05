// Reuse the A14 recorded ROS-message decoder; no parallel prediction pipeline.
#define R4_REPLAY_LIBRARY_ONLY
#include "../r4_input_applicability_audit/replay.cpp"

int main(int argc,char ** argv)
{
  if(argc!=4) return 2;
  try {
    std::ofstream out(std::filesystem::path(argv[3])/"replay.csv");
    out<<std::setprecision(17)<<"scene,cycle,epoch_ns,baseline_valid,valid,reason,status,iterations,vx,vy,wz,progress_end,yaw_delta,nominal_cost,solved_cost,solver_ms,elapsed_ms,used_warm,minimum_clearance,plateau_stages\n";
    for(const std::string scene:{"S0","S1","S2"}) {
      const auto data=load(std::filesystem::path(argv[1])/scene/"rosbag");
      const auto input=rows(std::filesystem::path(argv[2])/(scene+"_native.csv"));
      v::ReceiptGate gate; v::RotatingFollowSolver solver;
      std::optional<v::PredictionSnapshot> previous;
      std::optional<v::PreparedCorridor> corridor; uint64_t revision=0;
      v::Vec2 seed{}; double seed_wz=0.; int64_t seed_epoch=0;
      const v::RotatingFollowLimits limits{{{-.5,-.5},{.8,.5},{1.,1.},.4,.5},-1.2,1.2,2.};
      for(const auto & r:input) {
        const auto acquired=v::FollowClock::now(); const int64_t epoch=n(r,"acquire_ros_ns");
        std::string reason,status="not_run"; int iterations=0,plateau=0;
        bool valid=false,warm=false; v::Vec2 command{};
        double omega=0.,progress=0.,yaw_delta=0.,nominal=0.,cost=0.,solver_ms=0.,elapsed_ms=0.,clearance=1e9;
        try {
          const v::BodyPolicy b{{{-.3,-.25},{-.3,.25},{.3,.25},{.3,-.25}},.03,d(r,"yaw"),.05};
          const auto path_revision=uint64_t(n(r,"path_revision"));
          const bool new_path=!corridor||revision!=path_revision;
          if(new_path||corridor->body_digest()!=b.digest()) {
            corridor=v::PreparedCorridor::prepare(data.paths.at(n(r,"path_stamp_ns")),data.map,b,path_revision); revision=path_revision;
          }
          auto snapshot=v::PredictionSnapshot::freeze(data.envelopes.at(uint64_t(n(r,"receipt_sequence"))),epoch,"map",b);
          const bool gate_reset=gate.accept(snapshot);
          // Thin value adapter: pose-yaw changes do not invalidate geometry-only warm identity.
          const bool context_changed=!previous||snapshot.producer_id()!=previous->producer_id()||
            snapshot.generation()!=previous->generation()||snapshot.policy().digest()!=previous->policy().digest()||
            snapshot.body_policy().geometry_digest()!=previous->body_policy().geometry_digest();
          const bool stale_seed=seed_epoch<=0||epoch<=seed_epoch||epoch-seed_epoch>100000000;
          if(stale_seed) {seed={};seed_wz=0.;seed_epoch=epoch;}
          const bool reset=new_path||stale_seed||(gate_reset&&context_changed);
          previous=snapshot;
          v::RotatingFollowInput in{snapshot,*corridor,b,limits,{"recorded/"+scene,scene,"base_link",1,uint64_t(n(r,"cycle"))},
            {{d(r,"x"),d(r,"y")},{d(r,"measured_vx"),d(r,"measured_vy")},seed,d(r,"yaw"),d(r,"measured_wz"),seed_wz,
            n(r,"pose_source_ns"),n(r,"velocity_source_ns"),n(r,"tf_source_ns"),seed_epoch,"map","base_link"},
            d(r,"progress_input"),acquired,reset};
          const auto result=solver.solve(in); reason=result.reason; status=result.solver_status; iterations=result.iterations;
          nominal=result.nominal_dynamic_cost; cost=result.solved_dynamic_cost;
          solver_ms=result.solver_seconds*1000; elapsed_ms=result.elapsed_seconds*1000; warm=result.used_warm;
          if(result.proposal) {
            valid=true;command=result.proposal->body_velocity;omega=result.proposal->yaw_rate;
            progress=result.proposal->stages.back().progress; yaw_delta=result.proposal->stages.back().yaw-d(r,"yaw");
            seed=command;seed_wz=omega;seed_epoch=epoch;
            v::TemporalSoftField field(snapshot);
            for(size_t k=0;k<30;++k) {const auto st=result.proposal->stages[k];
              const auto soft=field.sample(st.position,st.yaw,k);
              if(soft.clearance) {clearance=std::min(clearance,*soft.clearance);} plateau+=soft.plateau;
            }
          } else {seed_epoch=0;}
        } catch(const std::exception & e) {reason=e.what();solver.reset();seed_epoch=0;gate.reset();previous.reset();}
        out<<scene<<','<<r.at("cycle")<<','<<epoch<<','<<r.at("valid")<<','<<valid<<','<<quote(reason)<<','<<quote(status)<<','
          <<iterations<<','<<command.x<<','<<command.y<<','<<omega<<','<<progress<<','<<yaw_delta<<','<<nominal<<','<<cost<<','
          <<solver_ms<<','<<elapsed_ms<<','<<warm<<','<<clearance<<','<<plateau<<'\n';
      }
    }
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
