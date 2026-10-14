// Research-only read access to native wheel targets and physics state.
// No command/TF publishers, no ECM writes, no velocity/collision modifications.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/plugin/Register.hh>
#include <fstream>
#include <iomanip>
#include <array>
#include <chrono>
#include <cmath>
namespace ig=ignition::gazebo;
class BoundaryObserver:public ig::System,public ig::ISystemConfigure,public ig::ISystemPreUpdate,public ig::ISystemPostUpdate {
 ig::Model model{ig::kNullEntity};ig::Entity link=ig::kNullEntity;std::array<ig::Entity,4> joints{};
 std::array<double,4> target{};bool valid=false;std::ofstream out;uint64_t cycle=0;
 const std::array<std::string,4> names{"front_left_wheel_joint","front_right_wheel_joint","rear_left_wheel_joint","rear_right_wheel_joint"};
public:
 void Configure(const ig::Entity&e,const std::shared_ptr<const sdf::Element>&s,ig::EntityComponentManager&,ig::EventManager&)override{
  model=ig::Model(e);out.open(s->Get<std::string>("output"));out<<std::setprecision(17);
  out<<"cycle,sim_ns,dt_ns,steady_ns,valid,x,y,z,roll,pitch,yaw,vx,vy,wz,applied_vx,applied_vy,applied_wz,fl,fr,bl,br\n";
 }
 void PreUpdate(const ig::UpdateInfo&i,ig::EntityComponentManager&e)override{
  if(i.paused)return;valid=true;
  if(link==ig::kNullEntity)link=model.LinkByName(e,"base_link");
  for(size_t k=0;k<4;k++){
   if(joints[k]==ig::kNullEntity)joints[k]=model.JointByName(e,names[k]);
   auto c=e.Component<ig::components::JointVelocityCmd>(joints[k]);
   if(!c||c->Data().empty()){valid=false;target[k]=NAN;}else target[k]=c->Data()[0];
  }
 }
 void PostUpdate(const ig::UpdateInfo&i,const ig::EntityComponentManager&e)override{
  if(i.paused||link==ig::kNullEntity)return;ig::Link body(link);auto p=ig::worldPose(link,e);
  auto v=body.WorldLinearVelocity(e),w=body.WorldAngularVelocity(e);valid=valid&&bool(v)&&bool(w);
  ignition::math::Vector3d vb(NAN,NAN,NAN),wb(NAN,NAN,NAN);
  if(v&&w){vb=p.Rot().Inverse().RotateVector(*v);wb=p.Rot().Inverse().RotateVector(*w);}
  double vx=.075*(target[0]+target[1]+target[2]+target[3])/4;
  double vy=.075*(-target[0]+target[1]+target[2]-target[3])/4;
  double wz=.075*(-target[0]+target[1]-target[2]+target[3])/(4*.475);
  out<<cycle++<<','<<std::chrono::duration_cast<std::chrono::nanoseconds>(i.simTime).count()<<','<<std::chrono::duration_cast<std::chrono::nanoseconds>(i.dt).count()<<','<<std::chrono::steady_clock::now().time_since_epoch().count()<<','<<valid<<','<<p.Pos().X()<<','<<p.Pos().Y()<<','<<p.Pos().Z()<<','<<p.Rot().Roll()<<','<<p.Rot().Pitch()<<','<<p.Rot().Yaw()<<','<<vb.X()<<','<<vb.Y()<<','<<wb.Z()<<','<<vx<<','<<vy<<','<<wz;
  for(auto x:target)out<<','<<x;out<<'\n';if(cycle%1000==0)out.flush();
 }
};
IGNITION_ADD_PLUGIN(BoundaryObserver,ig::System,BoundaryObserver::ISystemConfigure,BoundaryObserver::ISystemPreUpdate,BoundaryObserver::ISystemPostUpdate)
