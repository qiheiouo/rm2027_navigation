// Controlled failure fixture, not a candidate generator or controller.
// All manager dispatch and fallback/reset below call the installed 1.1.20 library.
#include <nav2_mppi_controller/optimizer.hpp>
#include <nav2_core/exceptions.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
using J=nlohmann::json;
struct Stats {int calls=0,after=0;};
struct Gate: mppi::critics::CriticFunction {
 Stats &s;std::string mode;
 Gate(Stats &st,std::string m):s(st),mode(m){}
 void initialize() override {}
 void score(mppi::CriticData &d) override {
  ++s.calls;
  bool bad=mode=="persistent_failure"||(mode=="first_score_fail_second_score_recovers"&&s.calls==1);
  if(bad){d.fail_flag=true;return;}
  d.costs(0)+=mode=="finite_risk_no_safe"?4000000.f:1.f;
  d.costs(1)+=mode=="finite_risk_no_safe"?5000000.f:2.f;
 }
};
struct After: mppi::critics::CriticFunction {
 Stats &s;explicit After(Stats &st):s(st){}void initialize() override{}
 void score(mppi::CriticData&) override{++s.after;}
};
struct Manager: mppi::CriticManager {
 explicit Manager(Stats &s,const std::string &m){critics_.push_back(std::make_unique<Gate>(s,m));critics_.push_back(std::make_unique<After>(s));}
};
struct Fixture: mppi::Optimizer {
 Fixture(){settings_.batch_size=2;settings_.time_steps=3;settings_.retry_attempt_limit=1;setMotionModel("Omni");reset();}
 J evaluate(const std::string &mode,bool clear){
  Stats s;Manager mgr(s,mode);critics_data_.fail_flag=false;int attempts=0;std::string error="none";bool accepted=false;
  try{do{++attempts;mgr.evalTrajectoriesScores(critics_data_);bool retry=fallback(critics_data_.fail_flag);if(!retry){accepted=true;break;}if(clear)critics_data_.fail_flag=false;}while(attempts<3);}
  catch(const nav2_core::PlannerException&){error="PlannerException";}catch(const std::runtime_error&){error="runtime_error";}
  return J{{"mode",mode},{"clear_only_after_retry",clear},{"eval_attempts",attempts},{"critic_calls",s.calls},{"subsequent_critic_calls",s.after},{"accepted_by_native_failure_flag",accepted},{"exception",error},{"last_fail_flag",critics_data_.fail_flag},{"finite_costs",std::vector<float>(costs_.begin(),costs_.end())}};
 }
};
int main(){J a=J::array();for(const std::string mode:{"normal","first_score_fail_second_score_recovers","persistent_failure","finite_risk_no_safe"})for(bool clear:{false,true}){Fixture f;a.push_back(f.evaluate(mode,clear));}
 bool ok=a[0]["accepted_by_native_failure_flag"]==true&&a[1]["accepted_by_native_failure_flag"]==true&&a[2]["critic_calls"]==1&&a[3]["critic_calls"]==2&&a[2]["accepted_by_native_failure_flag"]==false&&a[3]["accepted_by_native_failure_flag"]==true&&a[4]["accepted_by_native_failure_flag"]==false&&a[5]["accepted_by_native_failure_flag"]==false&&a[6]["accepted_by_native_failure_flag"]==true&&a[7]["accepted_by_native_failure_flag"]==true;
 std::cout<<J{{"native_version","1.1.20"},{"pairs",a},{"predeclared_acceptance_pass",ok},{"scope","controlled critic data; installed manager/reset/fallback, no optimized Twist or motion"}}.dump()<<'\n';return ok?0:2;
}
