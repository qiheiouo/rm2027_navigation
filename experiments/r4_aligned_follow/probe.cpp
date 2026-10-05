// Same A18 fixtures, virtual preceding proposal and held source pose; no plant.
#define R4_ROTATION_PROBE_LIBRARY_ONLY
#include "../r4_rotation_value/probe.cpp"
#include <rm_navigation_execution_adapters/current_geometry.hpp>

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
int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  try {
    std::ofstream out(argv[1]); out << std::setprecision(17);
    out << "case,cycle,valid,reason,status,vx,vy,wz,progress_end,yaw_end,nominal_cost,solved_cost,solver_ms,elapsed_ms,slice_error,minimum_clearance,plateau_stages,used_warm,source_preserved,legacy_accepts,model_x,model_y,model_yaw\n";
    const FollowLimits limits{{-.5,-.5},{.8,.5},{1.,1.},.4,.5};
    auto path_value = path();
    auto midpoint = path_value.poses.front(); midpoint.pose.position.x = -.5;
    path_value.poses.insert(path_value.poses.begin()+1,midpoint);
    midpoint.pose.position.x = .5; path_value.poses.insert(path_value.poses.begin()+3,midpoint);
    for (const std::string kind : {"zero_slice", "locked_future_clear", "locked_future_hold", "locked_future_hold_clear", "nonzero_applied"}) {
      AlignedFollowAdapter solver; Vec2 seed{.35,0.};
      const int count = kind == "locked_future_hold_clear" ? 60 : 1;
      for (int i = 0; i < count; ++i) {
        const auto epoch = source+i*50000000; auto b = body(); b.yaw = .35;
        const bool occupied = kind == "locked_future_hold" || (kind == "locked_future_hold_clear" && i < 30);
        auto snapshot = PredictionSnapshot::freeze(fixture(epoch,occupied),epoch,"map",b);
        auto route = PreparedCorridor::prepare(path_value,grid(),b,1);
        const bool zero = kind == "zero_slice";
        const int64_t pose_epoch = zero ? epoch : epoch-30000000;
        FollowInput in{snapshot,route,b,limits,{"probe",kind,"base_link",1,uint64_t(i+1)},
          {{0.,0.},{.35,0.},seed,b.yaw,zero ? 0. : .6,kind == "nonzero_applied" ? .3 : 0.,
          pose_epoch,pose_epoch,pose_epoch,epoch,"map","base_link"},1.,FollowClock::now(),false};
        bool legacy_accepts = true;
        try {fingerprint_follow_input(in);} catch (const ContractError &) {legacy_accepts = false;}
        const auto adapted = solver.solve({in}); const auto & result = adapted.value;
        if (!same_source(adapted.source_state,in.state)) {throw std::runtime_error("raw source changed");}
        double slice_error = 0.;
        if (kind == "nonzero_applied") {
          if (result.proposal || result.solver_status != "not_run" || adapted.source_state.last_applied_yaw_rate != .3 ||
            result.reason.find("zero last-applied yaw command") == std::string::npos)
          {throw std::runtime_error("nonzero actual yaw command was admitted");}
        } else {
          const auto digest = fingerprint_aligned_follow_input({in});
          if (!result.proposal || result.proposal->input_digest != digest) {throw std::runtime_error("missing/bad aligned value");}
          auto changed = in; changed.state.measured_yaw_rate += .01;
          if (fingerprint_aligned_follow_input({changed}) == digest) {throw std::runtime_error("raw measurement not bound");}
          changed = in; changed.state.velocity_stamp_ns -= 1;
          if (fingerprint_aligned_follow_input({changed}) == digest) {throw std::runtime_error("raw timestamp not bound");}
          if (!zero && legacy_accepts) {throw std::runtime_error("legacy host accepted rotating source");}
          if (zero) {
            FollowSolver baseline; const auto fixed = baseline.solve(in);
            if (!fixed.proposal || fixed.proposal->input_digest == digest) {throw std::runtime_error("legacy identity not distinct");}
            for (size_t k = 0; k < 15; ++k) {
              const auto a = result.proposal->controls[k], c = fixed.proposal->controls[k];
              slice_error = std::max({slice_error,std::abs(a.body_velocity.x-c.body_velocity.x),
                std::abs(a.body_velocity.y-c.body_velocity.y),std::abs(a.progress_rate-c.progress_rate)});
            }
            if (slice_error > 2e-5) {throw std::runtime_error("zero slice changed");}
          }
        }
        Vec2 command{}; double omega=0.,progress=0.,yaw=0.,clearance=1e9,model_x=0.,model_y=0.,model_yaw=0.; int plateau=0;
        if (result.proposal) {
          const auto & proposal = *result.proposal; command = proposal.body_velocity; omega = proposal.yaw_rate;
          progress = proposal.stages.back().progress; yaw = proposal.stages.back().yaw;
          model_x = proposal.stages[0].position.x; model_y = proposal.stages[0].position.y; model_yaw = proposal.stages[0].yaw;
          const auto expected = rm_navigation_execution_adapters::integrate_held_body_twist(
            {0.,0.,.35},{.35,0.,zero ? 0. : .6},zero ? 0. : .03);
          if (std::abs(model_x-expected.x) > 1e-12 || std::abs(model_y-expected.y) > 1e-12 ||
            std::abs(model_yaw-expected.yaw) > 1e-12 || omega != 0.) {throw std::runtime_error("epoch model mismatch");}
          TemporalSoftField field(snapshot);
          for (size_t k = 0; k < 31; ++k) {
            const auto & st = proposal.stages[k];
            if (st.yaw != model_yaw) {throw std::runtime_error("future yaw was not held");}
            if (k == 30) {continue;}
            const auto sample = field.sample(st.position,st.yaw,k);
            if (sample.clearance) {clearance = std::min(clearance,*sample.clearance);} plateau += sample.plateau;
          }
          seed = command;
        } else {seed={};}
        out << kind << ',' << i << ',' << bool(result.proposal) << ',' << result.reason << ',' << result.solver_status << ','
          << command.x << ',' << command.y << ',' << omega << ',' << progress << ',' << yaw << ','
          << result.nominal_dynamic_cost << ',' << result.solved_dynamic_cost << ',' << result.solver_seconds*1000 << ','
          << result.elapsed_seconds*1000 << ',' << slice_error << ',' << clearance << ',' << plateau << ',' << result.used_warm << ','
          << same_source(adapted.source_state,in.state) << ',' << legacy_accepts << ',' << model_x << ',' << model_y << ',' << model_yaw << '\n';
      }
    }
  } catch (const std::exception & e) {std::cerr << e.what() << '\n';return 1;}
}
