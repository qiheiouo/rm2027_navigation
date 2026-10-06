// Gazebo Research only. Same native pipeline; no owner/lease/fallback implementation.
#include <nav2_core/controller.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rm_r4_prediction_consumption/follow.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <deque>
namespace r4_gazebo_comparison {
namespace v=rm_r4_prediction_consumption;
int64_t stamp(const builtin_interfaces::msg::Time & t) {return int64_t(t.sec)*1000000000+t.nanosec;}
tf2::Quaternion quat(const geometry_msgs::msg::Quaternion & q) {return {q.x,q.y,q.z,q.w};}
class Controller : public nav2_core::Controller {
public:
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
  node_=parent.lock();name_=name;tf_=tf;
  const auto recovery=std::getenv("R4_RESEARCH_NATIVE_RECOVERY");
  research_native_recovery_=recovery && std::string(recovery)=="1";
  loader_=std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>("nav2_core","nav2_core::Controller");
  native_=loader_->createSharedInstance("nav2_mppi_controller::MPPIController");native_->configure(parent,name,tf,costmap);
  if(!node_->has_parameter(name+".research_mode")) node_->declare_parameter(name+".research_mode",std::string{});
  if(!node_->has_parameter(name+".research_log")) node_->declare_parameter(name+".research_log",std::string{});
  node_->get_parameter(name+".research_mode",mode_);std::string file;node_->get_parameter(name+".research_log",file);
  if((mode_!="B0"&&mode_!="R4")||file.empty()) throw std::runtime_error("explicit Gazebo comparison mode/log required");
  double xmin,xmax,vy;node_->get_parameter(name+".vx_min",xmin);node_->get_parameter(name+".vx_max",xmax);node_->get_parameter(name+".vy_max",vy);
  limits_={{xmin,-vy},{xmax,vy},{.8,.8},.4,.5};body_.footprint={{-.32,-.27},{-.32,.27},{.32,.27},{.32,-.27}};body_.padding=.02;body_.static_clearance=.05;body_.yaw_invariant_circle=true;
  log_.open(file);log_<<std::setprecision(17)<<"epoch_ns,mode,valid,reason,native_ms,solver_ms,total_ms,iterations,x,y,yaw,world_vx,world_vy,body_vx,body_vy,wz,nominal_cost,solved_cost,progress_end,tracks,history_stamp_ns,state_stamp_ns\n";
  if(!log_) throw std::runtime_error("research log");
  failure_=node_->create_publisher<std_msgs::msg::String>("/research/failure",rclcpp::QoS(1).transient_local());
  subs_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>("/odometry/lio",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){std::lock_guard<std::mutex> l(mutex_);odoms_.push_back(m);while(odoms_.size()>30) odoms_.pop_front();}));
  subs_.push_back(node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){std::lock_guard<std::mutex> l(mutex_);map_=m;}));
  subs_.push_back(node_->create_subscription<rm_r4_interfaces::msg::ObservedPredictionEnvelope>("/perception/dynamic_obstacles_shadow/observed_predictions",10,[this](rm_r4_interfaces::msg::ObservedPredictionEnvelope::ConstSharedPtr m){std::lock_guard<std::mutex> l(mutex_);envelope_=m;}));
  subs_.push_back(node_->create_subscription<geometry_msgs::msg::Twist>("/simulation/chassis/cmd_vel",10,[this](geometry_msgs::msg::Twist::ConstSharedPtr m){std::lock_guard<std::mutex> l(mutex_);output_=*m;output_ns_=node_->now().nanoseconds();}));
 }
 void cleanup() override {native_->cleanup();subs_.clear();route_.reset();}
 void activate() override {native_->activate();failure_->on_activate();}
 void deactivate() override {native_->deactivate();failure_->on_deactivate();}
 void setPlan(const nav_msgs::msg::Path & path) override {native_->setPlan(path);std::lock_guard<std::mutex> l(mutex_);path_=path;++revision_;}
 void setSpeedLimit(const double & limit,const bool & percentage) override {native_->setSpeedLimit(limit,percentage);}
 geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,nav2_core::GoalChecker * checker) override {
  auto acquired=v::FollowClock::now();int64_t epoch=node_->now().nanoseconds();++sequence_;
  double native_ms=0.;v::WorldFollowResult result;geometry_msgs::msg::TwistStamped command;
  if(failed_) throw std::runtime_error("latched Research trial failure");
  try {
   const auto start=v::FollowClock::now();command=native_->computeVelocityCommands(pose,velocity,checker);
   native_ms=1000*std::chrono::duration<double>(v::FollowClock::now()-start).count();
   if(mode_=="B0") {write(epoch,true,"native",native_ms,result,pose,{},command,0,0,0);return command;}
   // Native delegation precedes acquisition of the R4 input set. Otherwise
   // newly received odometry is compared with the older callback-entry epoch.
   acquired=v::FollowClock::now();
   nav_msgs::msg::Odometry::ConstSharedPtr odom;nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
   std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odoms;
   rm_r4_interfaces::msg::ObservedPredictionEnvelope::ConstSharedPtr envelope;nav_msgs::msg::Path path;
   geometry_msgs::msg::Twist output;int64_t sent;uint64_t revision;
   {std::lock_guard<std::mutex> l(mutex_);odoms=odoms_;map=map_;envelope=envelope_;path=path_;output=output_;sent=output_ns_;revision=revision_;epoch=node_->now().nanoseconds();}
   if(odoms.empty()||!map||!envelope||path.poses.size()<2||sent<=0) throw std::runtime_error("missing_owned_input");
   // Reuse A16 shadow.cpp's source-time selection: newest canonical odometry
   // with its exact TF already available, still under the original 100ms age.
   geometry_msgs::msg::TransformStamped tf;
   for(auto it=odoms.rbegin();it!=odoms.rend();++it) {
    const auto source=stamp((*it)->header.stamp);if(source>epoch||epoch-source>100000000) continue;
    try {tf=tf_->lookupTransform("map",(*it)->header.frame_id,rclcpp::Time(source,RCL_ROS_TIME),rclcpp::Duration::from_seconds(0));odom=*it;break;} catch(const tf2::TransformException &) {}
   }
   if(!odom||odom->child_frame_id!="base_link") throw std::runtime_error("no_canonical_source_time_TF_within_state_age");
   const auto ns=stamp(odom->header.stamp);
   tf2::Transform transform(quat(tf.transform.rotation),{tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z});
   const auto & p=odom->pose.pose;auto point=transform*tf2::Vector3(p.position.x,p.position.y,p.position.z);
   const auto yaw=tf2::getYaw(transform.getRotation()*quat(p.orientation));auto body=body_;body.yaw=yaw;
   if(!route_||prepared_!=revision) {route_=v::PreparedCorridor::prepare(path,*map,body,revision);prepared_=revision;}
   auto consumed=gate_.consume(*envelope,epoch,"map",body);const auto & measured=odom->twist.twist;
   // Actual ROS output target receipt, not physical applied velocity or a send grant.
   const auto current_base=tf_->lookupTransform("map","base_link",tf2::TimePointZero);
   const auto latest_yaw=tf2::getYaw(quat(current_base.transform.rotation));const double c=std::cos(latest_yaw),s=std::sin(latest_yaw);
   v::Vec2 preceding{c*output.linear.x-s*output.linear.y,s*output.linear.x+c*output.linear.y};
   v::FollowState raw{{point.x(),point.y()},{measured.linear.x,measured.linear.y},{output.linear.x,output.linear.y},yaw,measured.angular.z,output.angular.z,ns,ns,stamp(tf.header.stamp),sent,"map","base_link"};
   v::WorldFollowInput in{consumed.snapshot,*route_,body,limits_,{"gazebo_research",mode_,"base_link",1,sequence_},raw,preceding,sent,"map",route_->project({point.x(),point.y()}),acquired,consumed.reset_warm};
   result=solver_.solve(std::move(in));if(!result.proposal) throw std::runtime_error("R4/"+result.reason);
   const auto world=result.proposal->world_velocity;command.twist.linear.x=c*world.x+s*world.y;command.twist.linear.y=-s*world.x+c*world.y;
   write(epoch,true,result.reason,native_ms,result,pose,world,command,envelope->tracks.size(),sent,ns);return command;
  } catch(const std::exception & e) {
   const bool native_unavailable=std::string(e.what())=="Optimizer fail to compute path";
   if (!(research_native_recovery_ && native_unavailable)) failed_=true;
   std_msgs::msg::String message;message.data=e.what();failure_->publish(message);
   write(epoch,false,e.what(),native_ms,result,pose,{},command,0,0,0);log_.flush();throw;
  }
 }
private:
 void write(int64_t epoch,bool valid,const std::string & reason,double native_ms,const v::WorldFollowResult & r,
  const geometry_msgs::msg::PoseStamped & pose,v::Vec2 world,const geometry_msgs::msg::TwistStamped & cmd,size_t tracks,int64_t history,int64_t state) {
  std::string clean=reason;std::replace(clean.begin(),clean.end(),',',';');std::replace(clean.begin(),clean.end(),'\n',' ');
  log_<<epoch<<','<<mode_<<','<<valid<<','<<clean<<','<<native_ms<<','<<1000*r.solver_seconds<<','<<1000*r.elapsed_seconds<<','<<r.iterations<<','
   <<pose.pose.position.x<<','<<pose.pose.position.y<<','<<tf2::getYaw(quat(pose.pose.orientation))<<','<<world.x<<','<<world.y<<','<<cmd.twist.linear.x<<','<<cmd.twist.linear.y<<','<<cmd.twist.angular.z<<','
   <<r.nominal_dynamic_cost<<','<<r.solved_dynamic_cost<<','<<(r.proposal?r.proposal->stages.back().progress:0.)<<','<<tracks<<','<<history<<','<<state<<'\n';
 }
 rclcpp_lifecycle::LifecycleNode::SharedPtr node_;std::string name_,mode_;std::ofstream log_;std::mutex mutex_;
 std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;std::shared_ptr<nav2_core::Controller> native_;
 std::shared_ptr<tf2_ros::Buffer> tf_;std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
 rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr failure_;
 std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odoms_;nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
 rm_r4_interfaces::msg::ObservedPredictionEnvelope::ConstSharedPtr envelope_;nav_msgs::msg::Path path_;geometry_msgs::msg::Twist output_;int64_t output_ns_{};
 v::BodyPolicy body_;v::FollowLimits limits_;v::ReceiptGate gate_;v::WorldFollowAdapter solver_;std::optional<v::PreparedCorridor> route_;
 uint64_t sequence_{},revision_{},prepared_{};bool failed_{};bool research_native_recovery_{};
};
}
PLUGINLIB_EXPORT_CLASS(r4_gazebo_comparison::Controller,nav2_core::Controller)
