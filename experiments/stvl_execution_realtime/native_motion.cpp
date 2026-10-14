// Call actual installed SDK propagation in a no-node, no-output harness.
#include <nav2_mppi_controller/optimizer.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <array>
#include <cmath>
using J=nlohmann::json;using V=std::array<double,3>;
class MotionFixture:public mppi::Optimizer {
public:
 J check(V initial,V command,double yaw){
  settings_.model_dt=.1;settings_.time_steps=30;settings_.batch_size=1;
  state_.reset(1,30);control_sequence_.reset(30);setMotionModel("Omni");
  state_.pose.pose.orientation.z=std::sin(yaw/2);state_.pose.pose.orientation.w=std::cos(yaw/2);
  state_.speed.linear.x=initial[0];state_.speed.linear.y=initial[1];state_.speed.angular.z=initial[2];
  for(size_t k=0;k<30;k++){
   state_.cvx(0,k)=command[0];state_.cvy(0,k)=command[1];state_.cwz(0,k)=command[2];
   control_sequence_.vx(k)=command[0];control_sequence_.vy(k)=command[1];control_sequence_.wz(k)=command[2];
  }
  updateStateVelocities(state_);mppi::models::Trajectories raw;raw.reset(1,30);integrateStateVelocities(raw,state_);auto sdk=getOptimizedTrajectory();
  J samples=J::array();double x=0,y=0,h=yaw;double maxsdk=0,maxraw=0,rx=0,ry=0,rh=yaw;
  for(size_t k=0;k<30;k++){
   double px=rx,py=ry,ph=rh;V rv{state_.vx(0,k),state_.vy(0,k),state_.wz(0,k)};
   rx+=(std::cos(rh)*rv[0]-std::sin(rh)*rv[1])*.1;ry+=(std::sin(rh)*rv[0]+std::cos(rh)*rv[1])*.1;rh+=rv[2]*.1;
   x+=(std::cos(h)*command[0]-std::sin(h)*command[1])*.1;y+=(std::sin(h)*command[0]+std::cos(h)*command[1])*.1;h+=command[2]*.1;
   double se=std::hypot(x-sdk(k,0),y-sdk(k,1)),re=std::hypot(rx-raw.x(0,k),ry-raw.y(0,k));maxsdk=std::max(maxsdk,se);maxraw=std::max(maxraw,re);
   if(k<3)samples.push_back({{"step",k},{"raw_velocity",rv},{"raw_pose",{raw.x(0,k),raw.y(0,k),raw.yaws(0,k)}},{"sdk_pose",{sdk(k,0),sdk(k,1),sdk(k,2)}},{"previous_yaw_selected_view",{x,y,h}},{"SDK_view_error_m",se},{"raw_reference_error_m",re}});
  }
  return {{"initial_feedback",initial},{"selected_target",command},{"first_samples",samples},{"max_raw_reference_error_m",maxraw},{"max_sdk_previous_yaw_error_m",maxsdk}};
 }
};
int main(){MotionFixture f;J out=J::array();
 for(auto spec:std::vector<std::tuple<std::string,V,V,double>>{{"A1_start",{0,0,0},{.3,0,0},0},{"A2_straight",{.3,0,0},{.3,0,0},0},{"A3_brake",{.4,0,0},{0,0,0},0},{"A4_lateral",{0,.2,0},{0,.2,0},.6},{"A5_coupled",{.2,.1,.7},{.2,.1,.7},.6},{"A6_wait",{.1,0,0},{0,0,0},0}}){auto r=f.check(std::get<1>(spec),std::get<2>(spec),std::get<3>(spec));r["fixture"]=std::get<0>(spec);out.push_back(r);}
 std::cout<<out.dump(2)<<'\n';
}
