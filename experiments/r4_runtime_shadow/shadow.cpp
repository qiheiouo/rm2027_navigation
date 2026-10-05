// Experimental ROS input/telemetry adapter. No robot velocity publisher.
#include <rm_r4_prediction_consumption/follow.hpp>
#include "shadow_seed.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

namespace v = rm_r4_prediction_consumption;
using Clock = v::FollowClock;
using Row = std::map<std::string, std::string>;
int64_t steady_ns(Clock::time_point t = Clock::now())
{return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();}
int64_t stamp(const builtin_interfaces::msg::Time & t)
{return static_cast<int64_t>(t.sec) * 1000000000 + t.nanosec;}
template<class T> std::string value(const T & x)
{std::ostringstream s; s << std::setprecision(17) << x; return s.str();}
std::string quoted(const std::string & x)
{std::string out="\""; for(char c:x) {if(c=='"') out+='"';out+=c;} return out+'"';}
tf2::Quaternion quat(const geometry_msgs::msg::Quaternion & q) {return {q.x,q.y,q.z,q.w};}
const std::vector<std::string> columns={
 "cycle","acquire_ros_ns","callback_entry_steady_ns","acquire_steady_ns","proposal_observed_ros_ns","record_steady_ns","record_ros_ns",
 "scan_source_ns","latest_scan_ns","prediction_source_ns","prediction_processing_ns","members_oldest_observation_ns","members_latest_observation_ns",
 "producer_id","producer_generation","receipt_sequence","receipt_digest","tracks","members_json",
 "pose_source_ns","velocity_source_ns","tf_requested_ns","tf_source_ns","latest_odom_ns","x","y","yaw","measured_vx","measured_vy","measured_wz",
 "path_revision","path_stamp_ns","static_revision","path_digest","map_digest","body_digest","limits_digest","corridor_rebuilt","corridor_prepare_ms",
 "seed_kind","seed_vx","seed_vy","seed_stamp_ns","reset_seed","reset_warm","used_warm","progress_input","route_tangent_x","route_tangent_y",
 "solve_invoked","follow_start_steady_ns","follow_finish_steady_ns","solver_status","reason","iterations","solver_ms","follow_call_ms","acquire_to_solve_finish_ms",
 "valid","vx","vy","wz","progress_next","progress_end","progress_rate","nominal_dynamic_cost","dynamic_cost","slack","min_predicted_observed_clearance","plateau_stages","stages_json","controls_json",
 "observation_age_at_acquire_ms","prediction_age_at_acquire_ms","state_age_at_acquire_ms","velocity_age_at_acquire_ms","observation_to_proposal_age_ms","prediction_to_proposal_age_ms","remaining_75ms_at_proposal_ms","diagnostics_ms"};

class Shadow : public rclcpp::Node
{
public:
 Shadow():Node("r4_runtime_shadow")
 {
  const auto out=declare_parameter<std::string>("output_directory");
  run_=declare_parameter<std::string>("run_id");
  if(out.empty()||run_.empty()) throw std::invalid_argument("explicit output/run required");
  const auto fp=declare_parameter<std::vector<double>>("footprint");
  if(fp.size()!=8) throw std::invalid_argument("actual profile polygon required");
  for(size_t i=0;i<fp.size();i+=2) body_.footprint.push_back({fp[i],fp[i+1]});
  body_.padding=declare_parameter<double>("padding"); body_.static_clearance=.05;
  const auto lo=declare_parameter<std::vector<double>>("lower"), hi=declare_parameter<std::vector<double>>("upper"), rate=declare_parameter<std::vector<double>>("rate");
  if(lo.size()!=2||hi.size()!=2||rate.size()!=2) throw std::invalid_argument("actual limits required");
  limits_={{lo[0],lo[1]},{hi[0],hi[1]},{rate[0],rate[1]},.4,.5};
  cycles_.open(out+"/cycles.csv"); references_.open(out+"/references.csv");
  if(!cycles_||!references_) throw std::runtime_error("output files");
  for(size_t i=0;i<columns.size();++i) {cycles_<<(i?",":"")<<columns[i];} cycles_<<'\n';
  references_<<"kind,receipt_ros_ns,receipt_steady_ns,vx,vy,wz\n";
  buffer_=std::make_unique<tf2_ros::Buffer>(get_clock()); listener_=std::make_unique<tf2_ros::TransformListener>(*buffer_);
  subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scan_=stamp(m->header.stamp);}));
  subs_.push_back(create_subscription<rm_r4_interfaces::msg::ObservedPredictionEnvelope>("/perception/dynamic_obstacles_shadow/observed_predictions",10,[this](rm_r4_interfaces::msg::ObservedPredictionEnvelope::ConstSharedPtr m){envelope_=m;}));
  subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local().reliable(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){grid_=m;++static_revision_;}));
  subs_.push_back(create_subscription<nav_msgs::msg::Path>("/plan",10,[this](nav_msgs::msg::Path::ConstSharedPtr m){path_=m;++path_revision_;}));
  subs_.push_back(create_subscription<nav_msgs::msg::Odometry>("/odometry/lio",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){odoms_.push_back(m);while(odoms_.size()>30) odoms_.pop_front();}));
  for(const auto & kind:{std::string("mppi_upstream"),std::string("actual_output")}) {
   subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(kind=="mppi_upstream"?"/cmd_vel_nav":"/cmd_vel",10,[this,kind](geometry_msgs::msg::Twist::ConstSharedPtr m){
    references_<<kind<<','<<now().nanoseconds()<<','<<steady_ns()<<','<<std::setprecision(17)<<m->linear.x<<','<<m->linear.y<<','<<m->angular.z<<'\n';}));
  }
  timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{cycle();});
  RCLCPP_INFO(get_logger(),"Shadow only: frozen A08, no publisher, no active controller or enforcement");
 }
private:
 void write(Row & r) {
  r["record_steady_ns"]=value(steady_ns());r["record_ros_ns"]=value(now().nanoseconds());
  for(size_t i=0;i<columns.size();++i) {cycles_<<(i?",":"")<<quoted(r[columns[i]]);} cycles_<<'\n';
 }
 void cycle() {
  const auto acquired=Clock::now();const auto epoch=now().nanoseconds();Row r;
  r["cycle"]=value(++sequence_);r["acquire_ros_ns"]=value(epoch);r["callback_entry_steady_ns"]=r["acquire_steady_ns"]=value(steady_ns(acquired));
  r["valid"]="0";r["solve_invoked"]="0";r["latest_scan_ns"]=value(scan_);r["path_revision"]=value(path_revision_);r["static_revision"]=value(static_revision_);
  if(epoch<=previous_epoch_) {solver_.reset();gate_.reset();seed_.reset();++generation_;} previous_epoch_=epoch;
  try {
   if(envelope_) {
    const auto & e=*envelope_;r["scan_source_ns"]=r["prediction_source_ns"]=value(stamp(e.prediction.header.stamp));
    r["prediction_processing_ns"]=value(stamp(e.prediction.processing_stamp));r["producer_id"]=e.producer_id;
    r["producer_generation"]=value(e.producer_generation);r["receipt_sequence"]=value(e.sequence);r["tracks"]=value(e.tracks.size());
    int64_t oldest=0,latest=0;std::ostringstream members;members<<'[';
    for(size_t i=0;i<e.tracks.size();++i) {
     const auto & m=e.tracks[i];const auto ns=stamp(m.last_observation_stamp);
     oldest=oldest?std::min(oldest,ns):ns;latest=std::max(latest,ns);
     members<<(i?",":"")<<'['<<m.track_id<<','<<ns<<','<<m.association_sequence<<','<<m.local_endpoints.size()<<']';
    }
    members<<']';r["members_json"]=members.str();
    if(oldest) {r["members_oldest_observation_ns"]=value(oldest);r["members_latest_observation_ns"]=value(latest);r["observation_age_at_acquire_ms"]=value((epoch-oldest)/1e6);}
    r["prediction_age_at_acquire_ms"]=value((epoch-stamp(e.prediction.header.stamp))/1e6);
   }
   if(!envelope_||!grid_||!path_||odoms_.empty()) throw v::ContractError("missing_prediction_map_path_or_odom");
   r["path_stamp_ns"]=value(stamp(path_->header.stamp));r["latest_odom_ns"]=value(stamp(odoms_.back()->header.stamp));
   nav_msgs::msg::Odometry::ConstSharedPtr odom;geometry_msgs::msg::TransformStamped tf;
   for(auto it=odoms_.rbegin();it!=odoms_.rend();++it) {
    const auto ns=stamp((*it)->header.stamp);if(ns<=0||ns>epoch||epoch-ns>100000000) continue;
    try {tf=buffer_->lookupTransform("map",(*it)->header.frame_id,rclcpp::Time(ns,RCL_ROS_TIME),rclcpp::Duration::from_seconds(0));odom=*it;break;} catch(const tf2::TransformException &) {}
   }
   if(!odom) throw v::ContractError("no_source_time_TF_within_state_age");
   if(odom->child_frame_id!="base_link") throw v::ContractError("noncanonical_velocity_frame");
   const auto & op=odom->pose.pose;
   tf2::Transform transform(quat(tf.transform.rotation),{tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z});
   const auto xy=transform*tf2::Vector3(op.position.x,op.position.y,op.position.z);
   const auto yaw=tf2::getYaw(transform.getRotation()*quat(op.orientation));
   auto body=body_;body.yaw=yaw;
   const auto ns=stamp(odom->header.stamp);const auto & twist=odom->twist.twist;
   r["pose_source_ns"]=r["velocity_source_ns"]=r["tf_requested_ns"]=value(ns);r["tf_source_ns"]=value(stamp(tf.header.stamp));
   r["x"]=value(xy.x());r["y"]=value(xy.y());r["yaw"]=value(yaw);r["measured_vx"]=value(twist.linear.x);r["measured_vy"]=value(twist.linear.y);r["measured_wz"]=value(twist.angular.z);
   r["state_age_at_acquire_ms"]=r["velocity_age_at_acquire_ms"]=value((epoch-ns)/1e6);
   r["body_digest"]=body.digest();r["limits_digest"]=limits_.digest();
   const bool rebuild=!route_||prepared_path_!=path_revision_||prepared_static_!=static_revision_||route_->body_digest()!=body.digest();
   r["corridor_rebuilt"]=value(rebuild);const auto prep=Clock::now();
   if(rebuild) {route_=v::PreparedCorridor::prepare(*path_,*grid_,body,path_revision_);prepared_path_=path_revision_;prepared_static_=static_revision_;}
   r["corridor_prepare_ms"]=value((steady_ns()-steady_ns(prep))/1e6);r["path_digest"]=route_->path_digest();r["map_digest"]=route_->map_digest();
   auto consumed=gate_.consume(*envelope_,epoch,"map",body);r["receipt_digest"]=consumed.snapshot.receipt_digest();
   const auto decision=r4_runtime_shadow::decide_seed(seed_,seed_epoch_,epoch,rebuild||consumed.reset_warm);
   const auto seed=decision.velocity;const auto seed_stamp=decision.stamp_ns;
   r["seed_kind"]=decision.reset_seed?"shadow_virtual_cold_zero":"shadow_virtual_previous_proposal";
   r["seed_vx"]=value(seed.x);r["seed_vy"]=value(seed.y);r["seed_stamp_ns"]=value(seed_stamp);
   r["reset_seed"]=value(decision.reset_seed);r["reset_warm"]=value(decision.reset_warm);
   const double progress=route_->project({xy.x(),xy.y()});r["progress_input"]=value(progress);
   const auto tangent=route_->sample(progress).tangent;r["route_tangent_x"]=value(tangent.x);r["route_tangent_y"]=value(tangent.y);
   v::FollowInput input{consumed.snapshot,*route_,body,limits_,{"shadow_no_actuation/"+run_,run_,"base_link",generation_,sequence_},
    {{xy.x(),xy.y()},{twist.linear.x,twist.linear.y},seed,yaw,twist.angular.z,0.,ns,ns,stamp(tf.header.stamp),seed_stamp,"map","base_link"},progress,acquired,decision.reset_warm};
   const auto start=Clock::now();r["solve_invoked"]="1";r["follow_start_steady_ns"]=value(steady_ns(start));
   const auto result=solver_.solve(std::move(input));const auto finish=Clock::now();const auto finish_ros=now().nanoseconds();
   r["follow_finish_steady_ns"]=value(steady_ns(finish));r["follow_call_ms"]=value((steady_ns(finish)-steady_ns(start))/1e6);
   r["acquire_to_solve_finish_ms"]=value((steady_ns(finish)-steady_ns(acquired))/1e6);r["solver_status"]=result.solver_status;r["reason"]=result.reason;
   r["iterations"]=value(result.iterations);r["solver_ms"]=value(result.solver_seconds*1000);r["used_warm"]=value(result.used_warm);
   r["nominal_dynamic_cost"]=value(result.nominal_dynamic_cost);r["dynamic_cost"]=value(result.solved_dynamic_cost);
   if(result.minimum_constraint_slack) r["slack"]=value(*result.minimum_constraint_slack);
   if(result.proposal) {
    const auto & p=*result.proposal;r["valid"]="1";r["vx"]=value(p.body_velocity.x);r["vy"]=value(p.body_velocity.y);r["wz"]=value(p.yaw_rate);
    r["progress_next"]=value(p.stages[1].progress);r["progress_end"]=value(p.stages.back().progress);r["progress_rate"]=value(p.controls[0].progress_rate);
    seed_=p.body_velocity;seed_epoch_=epoch;
    r["remaining_75ms_at_proposal_ms"]=value((steady_ns(p.source_deadline)-steady_ns(finish))/1e6);
    r["proposal_observed_ros_ns"]=value(finish_ros);
    // Both operands are ROS stamps; wall solver duration is recorded separately.
    r["prediction_to_proposal_age_ms"]=value((finish_ros-consumed.snapshot.source_ns())/1e6);
    if(!r["members_oldest_observation_ns"].empty()) r["observation_to_proposal_age_ms"]=value((finish_ros-std::stoll(r["members_oldest_observation_ns"]))/1e6);
    v::TemporalSoftField field(consumed.snapshot);std::optional<double> clearance;int plateau=0;
    std::ostringstream stages,controls;stages<<std::setprecision(17)<<'[';controls<<std::setprecision(17)<<'[';
    for(size_t k=0;k<p.stages.size();++k) {
     const auto & s=p.stages[k];stages<<(k?",":"")<<'['<<s.position.x<<','<<s.position.y<<','<<s.body_velocity.x<<','<<s.body_velocity.y<<','<<s.yaw<<','<<s.progress<<']';
     const auto sample=field.sample(s.position,k);if(sample.clearance) clearance=clearance?std::min(*clearance,*sample.clearance):sample.clearance;if(sample.plateau) ++plateau;
    }
    for(size_t k=0;k<p.controls.size();++k) {const auto & c=p.controls[k];controls<<(k?",":"")<<'['<<c.body_velocity.x<<','<<c.body_velocity.y<<','<<c.progress_rate<<']';}
    stages<<']';controls<<']';r["stages_json"]=stages.str();r["controls_json"]=controls.str();r["plateau_stages"]=value(plateau);
    if(clearance) r["min_predicted_observed_clearance"]=value(*clearance);
    r["diagnostics_ms"]=value((steady_ns()-steady_ns(finish))/1e6);
   } else seed_.reset();
  } catch(const std::exception & e) {r["reason"]=e.what();solver_.reset();seed_.reset();}
  write(r);
 }
 std::string run_;std::ofstream cycles_,references_;std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
 std::unique_ptr<tf2_ros::Buffer> buffer_;std::unique_ptr<tf2_ros::TransformListener> listener_;rclcpp::TimerBase::SharedPtr timer_;
 rm_r4_interfaces::msg::ObservedPredictionEnvelope::ConstSharedPtr envelope_;nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;nav_msgs::msg::Path::ConstSharedPtr path_;
 std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odoms_;std::optional<v::PreparedCorridor> route_;v::BodyPolicy body_;v::FollowLimits limits_;
 v::ReceiptGate gate_;v::FollowSolver solver_;std::optional<v::Vec2> seed_;
 int64_t scan_{},previous_epoch_{},seed_epoch_{};uint64_t sequence_{},generation_{1},path_revision_{},static_revision_{},prepared_path_{},prepared_static_{};
};
int main(int argc,char ** argv) {rclcpp::init(argc,argv);try {rclcpp::spin(std::make_shared<Shadow>());}catch(const std::exception & e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}rclcpp::shutdown();}
