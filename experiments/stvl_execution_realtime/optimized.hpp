// Exact snapshot-lifetime optimization of 039273ae diagnosis, not a controller.
// Same 30*11 coverage, same R2 geometry, doubles, margins and Unknown rules.
#pragma once
#include "profile_reference.hpp"
namespace prepared {
using joint::Pose;using joint::combine;using joint::wrap;namespace g=joint::g;
struct SparseGrid:joint::Grid {
 std::vector<std::vector<int>> occupied,unknown;
 explicit SparseGrid(const JointJson&j):joint::Grid(j),occupied(h),unknown(h){
  for(unsigned b=0;b<h;b++)for(unsigned a=0;a<w;a++){int v=cells[b*w+a];if(v==255)unknown[b].push_back(a);else if(v>=254)occupied[b].push_back(a);}
 }
 double distance(const std::vector<g::Point>&p,double cap=.1)const{
  double ax=INFINITY,ay=INFINITY,bx=-INFINITY,by=-INFINITY;
  for(auto q:p){ax=std::min(ax,q.x);ay=std::min(ay,q.y);bx=std::max(bx,q.x);by=std::max(by,q.y);}
  if(ax<x||ay<y||bx>x+w*r||by>y+h*r)return -1;
  int ix=std::max(0,int(std::floor((ax-cap-x)/r))),iy=std::max(0,int(std::floor((ay-cap-y)/r))),ex=std::min(int(w)-1,int(std::floor((bx+cap-x)/r))),ey=std::min(int(h)-1,int(std::floor((by+cap-y)/r)));
  // Unknown anywhere in this identical old rectangular support wins regardless
  // of occupied-cell order. Never skip unknown support on an exact zero hit.
  for(int b=iy;b<=ey;b++){auto it=std::lower_bound(unknown[b].begin(),unknown[b].end(),ix);if(it!=unknown[b].end()&&*it<=ex)return -1;}
  double d=cap;
  for(int b=iy;b<=ey;b++){
   auto it=std::lower_bound(occupied[b].begin(),occupied[b].end(),ix);
   for(;it!=occupied[b].end()&&*it<=ex;++it){int a=*it;g::Box box{x+a*r,y+b*r,x+(a+1)*r,y+(b+1)*r};if(g::polygon_box_may_be_within(p,box,d))d=std::min(d,g::polygon_box_distance(p,box));if(d==0)return 0;}
  }
  return d;
 }
 std::string classify(const std::vector<g::Point>&p,double reserve)const{
  double ax=INFINITY,ay=INFINITY,bx=-INFINITY,by=-INFINITY;
  for(auto q:p){ax=std::min(ax,q.x);ay=std::min(ay,q.y);bx=std::max(bx,q.x);by=std::max(by,q.y);}
  if(ax<x||ay<y||bx>x+w*r||by>y+h*r)return "Unknown";
  // Keep old support/unknown semantics even though precise clearance telemetry
  // is unnecessary for this predicate. The sampling margin is unchanged.
  constexpr double cap=.1;
  int ix=std::max(0,int(std::floor((ax-cap-x)/r))),iy=std::max(0,int(std::floor((ay-cap-y)/r))),ex=std::min(int(w)-1,int(std::floor((bx+cap-x)/r))),ey=std::min(int(h)-1,int(std::floor((by+cap-y)/r)));
  for(int b=iy;b<=ey;b++){auto it=std::lower_bound(unknown[b].begin(),unknown[b].end(),ix);if(it!=unknown[b].end()&&*it<=ex)return "Unknown";}
  std::string result=reserve>=cap?"Unknown":"Safe";double limit=reserve>=cap?1e-6:reserve;
  for(int b=iy;b<=ey;b++)for(auto it=std::lower_bound(occupied[b].begin(),occupied[b].end(),ix);it!=occupied[b].end()&&*it<=ex;++it){int a=*it;g::Box box{x+a*r,y+b*r,x+(a+1)*r,y+(b+1)*r};if(!g::polygon_box_may_be_within(p,box,limit))continue;
   double d=g::polygon_box_distance(p,box);if(d<=1e-6)return "Unsafe";if(d<=reserve)result="Unknown";
  }
  return result;
 }
};
struct Obstacle {g::Point point;double radius;};
struct Slot {bool covered=false;std::vector<Obstacle> obstacles;};
struct Trajectory {std::string id;std::vector<Pose> poses,velocities;bool has_velocity=false;};
struct Snapshot {
 SparseGrid map;std::vector<g::Point> footprint;Pose pose,speed;double dt,radius=0,age=0,obstacle_speed=0;
 std::string input="Safe";bool prediction_valid=false,execution_known=false;std::array<std::array<Slot,11>,30> slots;std::vector<Trajectory> trajectories;
 explicit Snapshot(const JointJson&ctx,bool selected_only=false):map(ctx.at("static_map")),pose(ctx.at("pose").get<Pose>()),speed(ctx.at("speed").get<Pose>()),dt(ctx.value("dt",.1)){
  for(auto p:ctx.at("footprint"))footprint.push_back({p[0],p[1]});g::validate_footprint(footprint);for(auto p:footprint)radius=std::max(radius,std::hypot(p.x,p.y));
  if(!ctx.value("identity_frame_fixture",false)||!ctx.value("feedback_consistent",true)||!std::isfinite(dt)||dt<=0)input="Unknown";
  execution_known=ctx.value("execution_mapping_known",false);
  const auto&pr=ctx.at("prediction");const auto source=pr.value("source",JointJson::object());int64_t epoch=ctx.at("pose_epoch_ns"),consume=pr.value("consume_ns",int64_t(0)),stamp=source.value("source_ns",int64_t(0));age=(consume-stamp)*1e-9;
  prediction_valid=source.value("schema",std::string{})=="rm_time_prediction/research_v1"&&source.value("frame",std::string{})=="map"&&source.value("complete",false)&&stamp>0&&age>=0&&age<=.4&&std::abs(source.value("dt",0.)-.1)<1e-9;
  if(prediction_valid)for(const auto&t:source.at("tracks")){
   if(t.at("points").size()!=36||t.value("observed_ns",int64_t(0))<=0||consume-t.value("observed_ns",int64_t(0))>400000000||t.value("observed_ns",int64_t(0))>stamp)prediction_valid=false;
   for(size_t k=0;k<t.at("points").size();k++){for(auto v:t["points"][k])if(!std::isfinite(v.get<double>()))prediction_valid=false;if(k)obstacle_speed=std::max(obstacle_speed,std::hypot(t["points"][k][0].get<double>()-t["points"][k-1][0].get<double>(),t["points"][k][1].get<double>()-t["points"][k-1][1].get<double>())/.1);}
  }
  if(prediction_valid)for(size_t k=0;k<30;k++)for(int s=0;s<=10;s++){
   double a=double(s)/10,index=((epoch-stamp)*1e-9+(k+a)*dt)/.1;auto&slot=slots[k][s];if(index<0||index>35)continue;slot.covered=true;
   size_t lo=std::min<size_t>(35,size_t(index)),hi=std::min<size_t>(35,lo+1);double alpha=index-lo;
   for(const auto&t:source.at("tracks"))if(t["state"]==2||t["state"]==3){g::Point q{t["points"][lo][0].get<double>()+alpha*(t["points"][hi][0].get<double>()-t["points"][lo][0].get<double>()),t["points"][lo][1].get<double>()+alpha*(t["points"][hi][1].get<double>()-t["points"][lo][1].get<double>())};double r=std::max(.36,std::hypot(t["size"][0].get<double>(),t["size"][1].get<double>())/2);slot.obstacles.push_back({q,r});}
  }
  size_t begin=selected_only?ctx.at("trajectories").size()-1:0;
  for(size_t i=begin;i<ctx.at("trajectories").size();i++){
   const auto&t=ctx.at("trajectories")[i];Trajectory out;out.id=t.at("id");out.poses=t.at("poses").get<std::vector<Pose>>();if(out.poses.size()!=30)throw std::runtime_error("fixed 30-step contract required");
   auto vel=t.value("velocities",JointJson());out.has_velocity=!vel.is_null()&&vel.size()==out.poses.size();if(out.has_velocity)out.velocities=vel.get<std::vector<Pose>>();trajectories.push_back(std::move(out));
  }
 }
};
// Predicate-only path: do not pretend it produces the omitted telemetry.
inline JointJson classify(const Snapshot&ctx,const Trajectory&traj){
 Pose prev=ctx.pose;std::string stat="Safe",dyn=ctx.prediction_valid?"Safe":"Unknown",input=ctx.input;std::vector<g::Point>poly(ctx.footprint.size());size_t visited=0;
 for(size_t k=0;k<traj.poses.size();k++){
  auto end=traj.poses[k];for(double v:end)if(!std::isfinite(v))input="Unknown";double angle=wrap(end[2]-prev[2]),speed=std::hypot(end[0]-prev[0],end[1]-prev[1])/ctx.dt+ctx.radius*std::abs(angle)/ctx.dt;
  double sr=speed*ctx.dt/10/2+1e-6,dr=(speed+ctx.obstacle_speed)*ctx.dt/10/2+1e-6;
  for(int s=0;s<=10;s++){
   visited++;double a=double(s)/10;Pose p{prev[0]+a*(end[0]-prev[0]),prev[1]+a*(end[1]-prev[1]),prev[2]+a*angle};double c=std::cos(p[2]),sn=std::sin(p[2]);for(size_t j=0;j<poly.size();j++){auto q=ctx.footprint[j];poly[j]={p[0]+c*q.x-sn*q.y,p[1]+sn*q.x+c*q.y};}
   stat=combine(stat,ctx.map.classify(poly,sr));
   if(stat=="Unsafe")return {{"id",traj.id},{"joint_geometry","Unsafe"},{"joint_complete","Unsafe"},{"visited_samples",visited},{"witness_step",k},{"witness","static"}};
   if(ctx.prediction_valid){auto&slot=ctx.slots[k][s];if(!slot.covered){dyn=combine(dyn,"Unknown");continue;}double d=INFINITY;for(auto obstacle:slot.obstacles)d=std::min(d,g::circle_clearance(poly,obstacle.point,obstacle.radius));
    if(d<=.02)return {{"id",traj.id},{"joint_geometry","Unsafe"},{"joint_complete","Unsafe"},{"visited_samples",visited},{"witness_step",k},{"witness","dynamic"}};
    if(d<=.02+dr)dyn=combine(dyn,"Unknown");
   }
  }prev=end;
 }
 std::string execution="Unknown";
 if(ctx.execution_known&&traj.has_velocity){execution="Safe";auto last=ctx.speed;for(auto next:traj.velocities){for(int j=0;j<3;j++){double low=j==0?-.3:j==1?-.5:-1.2,high=j==2?1.2:.5;if(next[j]<low-1e-4||next[j]>high+1e-4||std::abs(next[j]-last[j])/ctx.dt>(j==2?2.:.8)+.01)execution="Unsafe";}last=next;}}
 auto geometry=combine(stat,dyn);return {{"id",traj.id},{"joint_geometry",geometry},{"joint_complete",combine(combine(geometry,input),execution)},{"visited_samples",visited},{"witness",nullptr}};
}
template<bool Profile=false>JointJson diagnose(const Snapshot&ctx,const Trajectory&traj,timing::Stats&stats){
 Pose prev=ctx.pose;double minstatic=.1,mindyn=INFINITY,pointdyn=INFINITY;std::string stat="Safe",dyn=ctx.prediction_valid?"Safe":"Unknown",input=ctx.input;
 int static_hits=0,dynamic_hits=0,sweep_unresolved=0,prefix_uncovered=0,first_bad=-1;std::vector<g::Point> poly(ctx.footprint.size());
 for(size_t k=0;k<traj.poses.size();k++){
  auto sweep_begin=timing::Clock::time_point{};if constexpr(Profile)sweep_begin=timing::Clock::now();auto end=traj.poses[k];for(double v:end)if(!std::isfinite(v))input="Unknown";
  double angle=wrap(end[2]-prev[2]),speed=std::hypot(end[0]-prev[0],end[1]-prev[1])/ctx.dt+ctx.radius*std::abs(angle)/ctx.dt;
  double static_reserve=speed*ctx.dt/10/2+1e-6,dynamic_reserve=(speed+ctx.obstacle_speed)*ctx.dt/10/2+1e-6;
  if constexpr(Profile)stats.sweep_ms+=timing::elapsed(sweep_begin);
  for(int s=0;s<=10;s++){
   if constexpr(Profile){sweep_begin=timing::Clock::now();stats.pose_samples++;}
   double a=double(s)/10;Pose p{prev[0]+a*(end[0]-prev[0]),prev[1]+a*(end[1]-prev[1]),prev[2]+a*angle};double c=std::cos(p[2]),sn=std::sin(p[2]);
   for(size_t j=0;j<poly.size();j++){auto q=ctx.footprint[j];poly[j]={p[0]+c*q.x-sn*q.y,p[1]+sn*q.x+c*q.y};}
   auto t=timing::Clock::time_point{};if constexpr(Profile){stats.sweep_ms+=timing::elapsed(sweep_begin);t=timing::Clock::now();}
   double ds=ctx.map.distance(poly);
   if(ds<0)stat=combine(stat,"Unknown");else {minstatic=std::min(minstatic,ds);if(ds<=1e-6){stat="Unsafe";static_hits++;if(first_bad<0)first_bad=int(k);}else if(ds<=static_reserve){stat=combine(stat,"Unknown");sweep_unresolved++;}}
   if constexpr(Profile){stats.static_ms+=timing::elapsed(t);t=timing::Clock::now();}
   if(ctx.prediction_valid){const auto&slot=ctx.slots[k][s];if(!slot.covered){dyn=combine(dyn,"Unknown");prefix_uncovered++;if constexpr(Profile)stats.dynamic_ms+=timing::elapsed(t);continue;}
    double d=INFINITY;for(auto obstacle:slot.obstacles)d=std::min(d,g::circle_clearance(poly,obstacle.point,obstacle.radius));mindyn=std::min(mindyn,d);if(s==10)pointdyn=std::min(pointdyn,d);
    if(d<=.02){dyn="Unsafe";dynamic_hits++;if(first_bad<0)first_bad=int(k);}else if(d<=.02+dynamic_reserve){dyn=combine(dyn,"Unknown");sweep_unresolved++;}
   }
   if constexpr(Profile)stats.dynamic_ms+=timing::elapsed(t);
  }prev=end;
 }
 auto t=timing::Clock::time_point{};if constexpr(Profile)t=timing::Clock::now();int velocity_bad=0,accel_bad=0;double maxacc=0;std::string rate="Unknown";
 if(traj.has_velocity){auto last=ctx.speed;rate="Safe";for(auto next:traj.velocities){for(int j=0;j<3;j++){double low=j==0?-.3:j==1?-.5:-1.2,high=j==2?1.2:.5;velocity_bad+=next[j]<low-1e-4||next[j]>high+1e-4;double acc=std::abs(next[j]-last[j])/ctx.dt;maxacc=std::max(maxacc,acc);accel_bad+=acc>(j==2?2.:.8)+.01;}last=next;}if(velocity_bad||accel_bad)rate="Unsafe";}
 if constexpr(Profile){stats.rate_ms+=timing::elapsed(t);t=timing::Clock::now();}
 std::string execution=ctx.execution_known?rate:"Unknown",geometry=combine(stat,dyn),complete=combine(combine(geometry,input),execution);
 JointJson result={{"static",stat},{"dynamic",dyn},{"nominal_rate",rate},{"mapped_execution",execution},{"input",input},{"joint_geometry",geometry},{"joint_complete",complete},{"static_min_capped_m",minstatic},{"dynamic_min_m",std::isfinite(mindyn)?JointJson(mindyn):JointJson()},{"dynamic_discrete_min_m",std::isfinite(pointdyn)?JointJson(pointdyn):JointJson()},{"velocity_bad",velocity_bad},{"acceleration_bad",accel_bad},{"nominal_max_acceleration",maxacc},{"static_hit_samples",static_hits},{"dynamic_hit_samples",dynamic_hits},{"sweep_unresolved",sweep_unresolved},{"uncovered_prediction_samples",prefix_uncovered},{"first_bad_step",first_bad},{"prediction_source_age_s",ctx.age},{"id",traj.id}};
 if constexpr(Profile)stats.result_ms+=timing::elapsed(t);return result;
}
}
