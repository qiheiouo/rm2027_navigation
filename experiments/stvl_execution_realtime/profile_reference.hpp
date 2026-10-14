// Profiling copy of 039273ae read-only checker; equations/coverage unchanged.
#pragma once
#include "../stvl_joint_execution/joint_core.hpp"
namespace timing {
using joint::Pose;using joint::combine;using joint::wrap;namespace g=joint::g;
using Clock=std::chrono::steady_clock;
inline double elapsed(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
struct Stats {double preparation_ms=0,sweep_ms=0,static_ms=0,dynamic_ms=0,rate_ms=0,result_ms=0;size_t pose_samples=0,cell_visits=0,occupied_candidates=0,exact_box_calls=0;};
struct ProfileGrid:joint::Grid {Stats* stats;ProfileGrid(const JointJson&j,Stats&s):joint::Grid(j),stats(&s){}
 double distance(const std::vector<g::Point>&p,double cap=.1) const {
  double ax=INFINITY,ay=INFINITY,bx=-INFINITY,by=-INFINITY;
  for(auto q:p){ax=std::min(ax,q.x);ay=std::min(ay,q.y);bx=std::max(bx,q.x);by=std::max(by,q.y);}
  if(ax<x||ay<y||bx>x+w*r||by>y+h*r)return -1; // Unknown map support.
  int ix=std::max(0,int(std::floor((ax-cap-x)/r))),iy=std::max(0,int(std::floor((ay-cap-y)/r))),ex=std::min(int(w)-1,int(std::floor((bx+cap-x)/r))),ey=std::min(int(h)-1,int(std::floor((by+cap-y)/r)));double d=cap;
  for(int b=iy;b<=ey;b++)for(int a=ix;a<=ex;a++) {
   stats->cell_visits++;int v=cells[b*w+a];if(v==255)return -1;
   if(v>=254){stats->occupied_candidates++;g::Box box{x+a*r,y+b*r,x+(a+1)*r,y+(b+1)*r};if(g::polygon_box_may_be_within(p,box,d)){stats->exact_box_calls++;d=std::min(d,g::polygon_box_distance(p,box));}}
  }
  return d;
 }
};
inline JointJson diagnose_profiled(const JointJson &ctx,const JointJson &traj,const ProfileGrid &map,Stats &stats) {
 auto begin=std::chrono::steady_clock::now();std::vector<g::Point> fp;for(auto p:ctx.at("footprint"))fp.push_back({p[0],p[1]});g::validate_footprint(fp);double radius=0;for(auto p:fp)radius=std::max(radius,std::hypot(p.x,p.y));
 auto poses=traj.at("poses").get<std::vector<Pose>>();Pose prev=ctx.at("pose").get<Pose>();double dt=ctx.value("dt",.1);int sub=10;double minstatic=.1,mindyn=INFINITY,pointdyn=INFINITY;std::string stat="Safe",dyn="Safe",input="Safe";
 const auto &pr=ctx.at("prediction");auto source=pr.value("source",JointJson::object());int64_t epoch=ctx.at("pose_epoch_ns"),consume=pr.value("consume_ns",int64_t(0)),stamp=source.value("source_ns",int64_t(0));double age=(consume-stamp)*1e-9;
 if(!ctx.value("identity_frame_fixture",false)||!ctx.value("feedback_consistent",true)||!std::isfinite(dt)||dt<=0)input="Unknown";
 bool valid=source.value("schema",std::string{})=="rm_time_prediction/research_v1"&&source.value("frame",std::string{})=="map"&&source.value("complete",false)&&stamp>0&&age>=0&&age<=.4&&std::abs(source.value("dt",0.)-.1)<1e-9;
 double obstacle_speed=0;
 if(valid)for(const auto&t:source.at("tracks")){
  if(t.at("points").size()!=36||t.value("observed_ns",int64_t(0))<=0||consume-t.value("observed_ns",int64_t(0))>400000000||t.value("observed_ns",int64_t(0))>stamp)valid=false;
  for(size_t k=0;k<t.at("points").size();k++){for(auto v:t["points"][k])if(!std::isfinite(v.get<double>()))valid=false;if(k)obstacle_speed=std::max(obstacle_speed,std::hypot(t["points"][k][0].get<double>()-t["points"][k-1][0].get<double>(),t["points"][k][1].get<double>()-t["points"][k-1][1].get<double>())/.1);}
 }
 if(!valid)dyn="Unknown";
 stats.preparation_ms+=elapsed(begin);
 int static_hits=0,dynamic_hits=0,sweep_unresolved=0,prefix_uncovered=0;int first_bad=-1;
 for(size_t k=0;k<poses.size();k++) {
  auto sweep_begin=Clock::now();auto end=poses[k];for(double v:end)if(!std::isfinite(v))input="Unknown";
  double angle=wrap(end[2]-prev[2]);double speed=std::hypot(end[0]-prev[0],end[1]-prev[1])/dt+radius*std::abs(angle)/dt;
  double static_reserve=speed*dt/sub/2+1e-6,dynamic_reserve=(speed+obstacle_speed)*dt/sub/2+1e-6;
  stats.sweep_ms+=elapsed(sweep_begin);
  for(int s=0;s<=sub;s++) {
   auto sample_begin=Clock::now();stats.pose_samples++;
   double a=double(s)/sub;Pose p{prev[0]+a*(end[0]-prev[0]),prev[1]+a*(end[1]-prev[1]),prev[2]+a*angle};auto poly=g::transform(fp,p[0],p[1],p[2]);stats.sweep_ms+=elapsed(sample_begin);auto static_begin=Clock::now();double ds=map.distance(poly);
   if(ds<0)stat=combine(stat,"Unknown");else {minstatic=std::min(minstatic,ds);if(ds<=1e-6){stat="Unsafe";static_hits++;if(first_bad<0)first_bad=int(k);}else if(ds<=static_reserve){stat=combine(stat,"Unknown");sweep_unresolved++;}}
   stats.static_ms+=elapsed(static_begin);auto dynamic_begin=Clock::now();
   if(valid) {
    double index=((epoch-stamp)*1e-9+(k+a)*dt)/.1;
    if(index<0||index>35){dyn=combine(dyn,"Unknown");prefix_uncovered++;stats.dynamic_ms+=elapsed(dynamic_begin);continue;}
    size_t lo=std::min<size_t>(35,size_t(index)),hi=std::min<size_t>(35,lo+1);double alpha=index-lo,d=INFINITY;
    for(const auto&t:source.at("tracks"))if(t["state"]==2||t["state"]==3){g::Point q{t["points"][lo][0].get<double>()+alpha*(t["points"][hi][0].get<double>()-t["points"][lo][0].get<double>()),t["points"][lo][1].get<double>()+alpha*(t["points"][hi][1].get<double>()-t["points"][lo][1].get<double>())};double r=std::max(.36,std::hypot(t["size"][0].get<double>(),t["size"][1].get<double>())/2);d=std::min(d,g::circle_clearance(poly,q,r));}
    mindyn=std::min(mindyn,d);if(s==sub)pointdyn=std::min(pointdyn,d);
    if(d<=.02){dyn="Unsafe";dynamic_hits++;if(first_bad<0)first_bad=int(k);}else if(d<=.02+dynamic_reserve){dyn=combine(dyn,"Unknown");sweep_unresolved++;}
   }
   stats.dynamic_ms+=elapsed(dynamic_begin);
  }prev=end;
 }
 auto rate_begin=Clock::now();
 int velocity_bad=0,accel_bad=0;double maxacc=0;std::string rate="Unknown";auto velocities=traj.value("velocities",JointJson());
 if(!velocities.is_null()&&velocities.size()==poses.size()) {
  auto last=ctx.at("speed").get<Pose>();rate="Safe";
  for(const auto&v:velocities){auto next=v.get<Pose>();for(int j=0;j<3;j++){double low=j==0?-.3:j==1?-.5:-1.2,high=j==2?1.2:.5;velocity_bad+=next[j]<low-1e-4||next[j]>high+1e-4;double acc=std::abs(next[j]-last[j])/dt;maxacc=std::max(maxacc,acc);accel_bad+=acc>(j==2?2.:.8)+.01;}last=next;}
  if(velocity_bad||accel_bad)rate="Unsafe";
 }
 stats.rate_ms+=elapsed(rate_begin);auto result_begin=Clock::now();
 // A raw rate violation proves its nominal pose/velocity rollout untrackable,
 // not that its Twist cannot be transformed by the downstream smoother.
 std::string execution=ctx.value("execution_mapping_known",false)?rate:"Unknown";std::string geometry=combine(stat,dyn),complete=combine(combine(geometry,input),execution);
 JointJson result={{"static",stat},{"dynamic",dyn},{"nominal_rate",rate},{"mapped_execution",execution},{"input",input},{"joint_geometry",geometry},{"joint_complete",complete},{"static_min_capped_m",minstatic},{"dynamic_min_m",std::isfinite(mindyn)?JointJson(mindyn):JointJson()},{"dynamic_discrete_min_m",std::isfinite(pointdyn)?JointJson(pointdyn):JointJson()},{"velocity_bad",velocity_bad},{"acceleration_bad",accel_bad},{"nominal_max_acceleration",maxacc},{"static_hit_samples",static_hits},{"dynamic_hit_samples",dynamic_hits},{"sweep_unresolved",sweep_unresolved},{"uncovered_prediction_samples",prefix_uncovered},{"first_bad_step",first_bad},{"prediction_source_age_s",age},{"compute_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()}};
 stats.result_ms+=elapsed(result_begin);return result;
}
}
