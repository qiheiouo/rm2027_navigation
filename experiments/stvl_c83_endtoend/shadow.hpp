// Read-only same-cycle observer. Native MPPI selection is never changed.
#pragma once
#include "../stvl_execution_realtime/optimized.hpp"
#include <memory>
namespace shadow {
using J=JointJson;
using Clock=std::chrono::steady_clock;
inline int64_t ns(Clock::time_point p){return std::chrono::duration_cast<std::chrono::nanoseconds>(p.time_since_epoch()).count();}
inline double ms(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
struct State {bool enabled=false;size_t cycle=0,attempts=0;Clock::time_point entry;double raw_read_ms=0;J map,context;std::vector<prepared::Trajectory> raw;std::ofstream timing,snapshots;};
// One controller instance, configure-before-activate, one compute thread.
inline State s;
inline void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr&node,const std::string&name){
 for(auto key:{"shadow_enabled","shadow_static_map","shadow_log"})if(!node->has_parameter(name+"."+key)){if(std::string(key)=="shadow_enabled")node->declare_parameter(name+"."+key,false);else node->declare_parameter(name+"."+key,std::string{});}
 std::string map,log;node->get_parameter(name+".shadow_enabled",s.enabled);node->get_parameter(name+".shadow_static_map",map);node->get_parameter(name+".shadow_log",log);std::ifstream f(map);f>>s.map;if(log.empty())throw std::runtime_error("explicit shadow log required");s.timing.open(log);s.snapshots.open(log+".raw",std::ios::binary);
}
inline void begin(const geometry_msgs::msg::PoseStamped&p,const geometry_msgs::msg::Twist&v,Clock::time_point entry){s.entry=entry;s.raw_read_ms=0;s.attempts=0;s.raw.clear();s.context={{"cycle_id",s.cycle},{"pose",{p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)}},{"speed",{v.linear.x,v.linear.y,v.angular.z}},{"pose_epoch_ns",rclcpp::Time(p.header.stamp).nanoseconds()},{"dt",.1},{"identity_frame_fixture",true},{"feedback_consistent",true},{"execution_mapping_known",false},{"trajectories",J::array()}};}
inline void read_raw(const mppi::CriticData&d){auto a=Clock::now();s.attempts++;s.raw.clear();s.raw.reserve(d.trajectories.x.shape(0));for(size_t i=0;i<d.trajectories.x.shape(0);i++){prepared::Trajectory t;t.id="raw_"+std::to_string(i);t.has_velocity=true;t.poses.reserve(30);t.velocities.reserve(30);for(size_t k=0;k<30;k++){t.poses.push_back({d.trajectories.x(i,k),d.trajectories.y(i,k),d.trajectories.yaws(i,k)});t.velocities.push_back({d.state.vx(i,k),d.state.vy(i,k),d.state.wz(i,k)});}s.raw.push_back(std::move(t));}s.raw_read_ms+=ms(a,Clock::now());}
inline void finish(J&record,const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>&map,Clock::time_point native_end,bool failed){
 auto prepare_begin=Clock::now();J ctx=s.context;ctx["case"]="live_"+std::to_string(s.cycle);ctx["prediction"]=record.value("prediction",J::object());ctx["static_map"]=s.map;ctx["footprint"]=J::array();for(auto p:map->getRobotFootprint())ctx["footprint"].push_back({p.x,p.y});
 // Source snapshot is the one the final native scoring attempt actually consumed.
 // Failed static-first attempts leave prediction empty: preserve Unknown.
 J checks=J::array();double prepare_ms=0,check_ms=0;size_t visited=0;std::string selected="unavailable";
 if(s.enabled){prepared::Snapshot snap(ctx);snap.trajectories=s.raw;auto check_begin=Clock::now();prepare_ms=ms(prepare_begin,check_begin);
 for(auto&t:snap.trajectories){auto q=prepared::classify(snap,t);visited+=q["visited_samples"].get<size_t>();checks.push_back(q);}
 if(!failed){prepared::Trajectory t;t.id="selected_previous_yaw";t.has_velocity=true;t.velocities=record.at("selected_post_sg_shift_controls").get<std::vector<joint::Pose>>();auto pose=ctx["pose"].get<joint::Pose>();for(auto v:t.velocities){double c=std::cos(pose[2]),sn=std::sin(pose[2]);pose={pose[0]+(v[0]*c-v[1]*sn)*.1,pose[1]+(v[0]*sn+v[1]*c)*.1,pose[2]+v[2]*.1};t.poses.push_back(pose);}auto q=prepared::classify(snap,t);selected=q["joint_geometry"];checks.push_back(q);}
 check_ms=ms(check_begin,Clock::now());}
 auto necessary_end=Clock::now();record["cycle_id"]=s.cycle;record["shadow_checks"]=checks;record["shadow_enabled"]=s.enabled;record["native_scoring_attempts"]=s.attempts;record["shadow_static_map_scope"]=s.map.value("scope",std::string{});
 // Lossless binary candidate export is diagnostic and explicitly timed outside
 // the potential necessary path. Record the actual wrapper return boundary too.
 auto export_begin=Clock::now();uint64_t count=s.raw.size(),cycle=s.cycle;s.snapshots.write(reinterpret_cast<char*>(&cycle),8);s.snapshots.write(reinterpret_cast<char*>(&count),8);for(auto&t:s.raw)for(size_t k=0;k<30;k++)for(auto v:{t.poses[k],t.velocities[k]})s.snapshots.write(reinterpret_cast<char*>(v.data()),24);
 s.snapshots.flush();auto export_end=Clock::now();record["timing"]={{"entry_steady_ns",ns(s.entry)},{"native_end_steady_ns",ns(native_end)},{"necessary_end_steady_ns",ns(necessary_end)},{"raw_read_ms",s.raw_read_ms},{"T_mppi_including_trace_ms",ms(s.entry,native_end)},{"T_mppi_excluding_added_raw_read_ms",ms(s.entry,native_end)-s.raw_read_ms},{"T_input_ms",prepare_ms},{"T_check_ms",check_ms},{"T_other_prelog_ms",ms(native_end,necessary_end)-prepare_ms-check_ms},{"T_total_necessary_boundary_ms",ms(s.entry,necessary_end)},{"candidate_export_ms",ms(export_begin,export_end)},{"check_samples",visited},{"failed",failed}};
 s.context=record["timing"];s.context["cycle_id"]=s.cycle;s.context["epoch_ns"]=record["epoch_ns"];s.context["command"]=record.value("command",J());s.context["shadow_enabled"]=s.enabled;s.cycle++;
}
inline void end_log(Clock::time_point start){auto end=Clock::now();s.context["control_log_ms"]=ms(start,end);s.context["wrapper_pre_return_steady_ns"]=ns(end);s.context["T_actual_wrapper_pre_return_ms"]=ms(s.entry,end);s.timing<<s.context.dump()<<'\n';s.timing.flush();}
}
