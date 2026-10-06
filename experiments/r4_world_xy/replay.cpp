// Recorded-value world XY consumption. Native commands remain evidence only.
#define R4_REPLAY_LIBRARY_ONLY
#include "../r4_input_applicability_audit/replay.cpp"
#include <algorithm>

struct Condition
{
  v::WorldFollowAdapter solver; v::ReceiptGate gate;
  std::optional<v::PredictionSnapshot> previous;
  v::Vec2 seed{}; int64_t seed_epoch{};
};
int main(int argc, char ** argv)
{
  if (argc!=5 && argc!=6) {return 2;}
  try {
    std::ofstream out(std::filesystem::path(argv[3])/"replay.csv");
    out<<std::setprecision(17)<<"scene,condition,cycle,epoch_ns,legacy_fixed_shadow_valid,valid,reason,status,iterations,vx,vy,forward,progress_input,progress_end,progress_rate,nominal_cost,solved_cost,solver_ms,elapsed_ms,used_warm,current_clearance,minimum_clearance,plateau_stages,raw_measured_wz,raw_history_wz,history_age_ms,seed_reset,model_x,model_y,velocity_frame,path_revision,reset_warm,seed_vx,seed_vy,seed_stamp_ns\n";
    for (const std::string scene : {"S1","S2"}) {
      if(argc==6 && std::string(argv[5]).rfind(scene+":",0)!=0) {continue;}
      const auto data=load(std::filesystem::path(argv[1])/scene/"rosbag");
      const auto input=rows(std::filesystem::path(argv[2])/(scene+"_native.csv"));
      auto receipts=rows(std::filesystem::path(argv[4])/(scene+"_references.csv"));
      receipts.erase(std::remove_if(receipts.begin(),receipts.end(),[](const Row & r){return r.at("kind")!="actual_output";}),receipts.end());
      std::sort(receipts.begin(),receipts.end(),[](const Row & a,const Row & b){return n(a,"receipt_ros_ns")<n(b,"receipt_ros_ns");});
      std::array<Condition,2> conditions;
      std::optional<v::PreparedCorridor> corridor; uint64_t revision=0;
      const v::FollowLimits limits{{-.5,-.5},{.8,.5},{1.,1.},.4,.5};
      for (const auto & r : input) {
        if(argc==6 && n(r,"cycle")>std::stoll(std::string(argv[5]).substr(scene.size()+1))) {break;}
        const int64_t epoch=n(r,"acquire_ros_ns");
        const auto receipt=std::upper_bound(receipts.begin(),receipts.end(),epoch,
          [](int64_t t,const Row & row){return t<n(row,"receipt_ros_ns");});
        for (size_t which=0;which<2;++which) {
          auto & c=conditions[which]; const auto acquired=v::FollowClock::now();
          std::string reason,status="not_run"; bool valid=false,warm=false,reset=false,seed_reset=false;
          int iterations=0,plateau=0; v::Vec2 command{},seed_value{}; int64_t seed_stamp=0;
          double forward=0.,progress=0.,rate=0.,nominal=0.,cost=0.,solver_ms=0.,elapsed_ms=0.,current=1e9,clearance=1e9,history_wz=0.,history_age=-1.,model_x=0.,model_y=0.;
          try {
            if(receipt==receipts.begin()) {throw std::runtime_error("missing_original_output_receipt_proxy");}
            const auto & history=*(receipt-1); const auto history_stamp=n(history,"receipt_ros_ns");
            history_wz=d(history,"wz");history_age=(epoch-history_stamp)*1e-6;
            const v::BodyPolicy b{{{-.3,-.25},{-.3,.25},{.3,.25},{.3,-.25}},.03,d(r,"yaw"),.05,true};
            const auto path_revision=uint64_t(n(r,"path_revision")); const bool new_path=!corridor||revision!=path_revision;
            if(new_path) {corridor=v::PreparedCorridor::prepare(data.paths.at(n(r,"path_stamp_ns")),data.map,b,path_revision);revision=path_revision;}
            auto envelope=data.envelopes.at(uint64_t(n(r,"receipt_sequence")));
            if(which==1) {envelope.prediction.tracks.clear();envelope.prediction.total_track_count=0;envelope.tracks.clear();}
            auto snapshot=v::PredictionSnapshot::freeze(envelope,epoch,"map",b);
            const bool gate_reset=c.gate.accept(snapshot);
            const bool context_changed=!c.previous||snapshot.producer_id()!=c.previous->producer_id()||
              snapshot.generation()!=c.previous->generation()||snapshot.policy().digest()!=c.previous->policy().digest()||
              snapshot.body_policy().geometry_digest()!=c.previous->body_policy().geometry_digest();
            const bool stale=c.seed_epoch<=0||epoch<=c.seed_epoch||epoch-c.seed_epoch>100000000;
            if(stale) {c.seed={};c.seed_epoch=epoch;}
            seed_reset=stale;seed_value=c.seed;seed_stamp=c.seed_epoch;
            reset=new_path||stale||(gate_reset&&context_changed);
            if(argc==6 && scene+":"+r.at("cycle")==argv[5]) {reset=true;}c.previous=snapshot;
            v::FollowState raw{{d(r,"x"),d(r,"y")},{d(r,"measured_vx"),d(r,"measured_vy")},
              {d(history,"vx"),d(history,"vy")},d(r,"yaw"),d(r,"measured_wz"),history_wz,
              n(r,"pose_source_ns"),n(r,"velocity_source_ns"),n(r,"tf_source_ns"),history_stamp,"map","base_link"};
            v::WorldFollowInput in{snapshot,*corridor,b,limits,{"recorded/world/"+scene,scene,"base_link",1,uint64_t(n(r,"cycle"))},
              raw,c.seed,c.seed_epoch,"map",d(r,"progress_input"),acquired,reset};
            const auto result=c.solver.solve(in);reason=result.reason;status=result.solver_status;iterations=result.iterations;
            nominal=result.nominal_dynamic_cost;cost=result.solved_dynamic_cost;solver_ms=1000*result.solver_seconds;
            elapsed_ms=1000*result.elapsed_seconds;warm=result.used_warm;
            if(result.source_state.measured_yaw_rate!=raw.measured_yaw_rate||result.source_state.last_applied_yaw_rate!=raw.last_applied_yaw_rate)
            {throw std::runtime_error("raw angular evidence changed");}
            if(result.proposal) {
              const auto & p=*result.proposal;if(p.velocity_frame!="map") {throw std::runtime_error("world frame missing");}
              valid=true;command=p.world_velocity;progress=p.stages.back().progress;rate=p.controls[0].progress_rate;
              const auto ref=corridor->sample(in.progress);forward=command.x*ref.tangent.x+command.y*ref.tangent.y;
              model_x=p.stages[0].position.x;model_y=p.stages[0].position.y;
              if(std::abs(command.x-c.seed.x)>.05002||std::abs(command.y-c.seed.y)>.05002) {throw std::runtime_error("XY proposal rate bound");}
              c.seed=command;c.seed_epoch=epoch;
              v::TemporalSoftField field(snapshot);
              for(size_t k=0;k<30;++k) {const auto soft=field.sample(p.stages[k].position,k);
                if(soft.clearance) {clearance=std::min(clearance,*soft.clearance);if(k==0) {current=*soft.clearance;}}plateau+=soft.plateau;
              }
            } else {c.seed_epoch=0;}
          } catch(const std::exception & e) {valid=false;reason=e.what();c.solver.reset();c.seed_epoch=0;c.gate.reset();c.previous.reset();}
          out<<scene<<','<<(which==0?"observed":"no_observed_dynamic")<<','<<r.at("cycle")<<','<<epoch<<','<<r.at("valid")<<','<<valid<<','
            <<quote(reason)<<','<<quote(status)<<','<<iterations<<','<<command.x<<','<<command.y<<','<<forward<<','<<r.at("progress_input")<<','
            <<progress<<','<<rate<<','<<nominal<<','<<cost<<','<<solver_ms<<','<<elapsed_ms<<','<<warm<<','<<current<<','<<clearance<<','<<plateau<<','
            <<r.at("measured_wz")<<','<<history_wz<<','<<history_age<<','<<seed_reset<<','<<model_x<<','<<model_y<<",map,"<<r.at("path_revision")<<','<<reset<<','<<seed_value.x<<','<<seed_value.y<<','<<seed_stamp<<'\n';
        }
      }
    }
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
