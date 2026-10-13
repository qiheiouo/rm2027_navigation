// Research only. Native optimizer, critics, lifecycle and command selection stay
// in the installed Nav2 1.1.20 library. Observers never change data.costs.
#include <fstream>
#include <mutex>
#include <chrono>
#include <nlohmann/json.hpp>
#include <xtensor/xrandom.hpp>
#include <nav2_mppi_controller/controller.hpp>
#include <nav2_mppi_controller/critic_function.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include "vendor/model.hpp"
using J=nlohmann::json;
namespace dyn=rm_dynamic_obstacle_critic;
namespace study {
struct Trace {std::vector<float> previous;J deltas=J::object(),prediction=J::object();J raw;bool capture=false;int64_t epoch=0,pose_epoch=0;};
thread_local Trace trace;
void observe(mppi::CriticData &d,const std::string &label) {
 auto &t=trace;J changes=J::array();
 for(size_t i=0;i<d.costs.size();i++) {changes.push_back(d.costs(i)-(i<t.previous.size()?t.previous[i]:0));}
 t.deltas[label]=changes;t.previous.assign(d.costs.begin(),d.costs.end());
}
}
namespace mppi::critics {
class TraceBase:public CriticFunction {
 void initialize() override {}
 void score(CriticData &d) override {
  auto label=name_.substr(name_.find_last_of('.')+1);
  if(label=="InputTraceCritic") {
   study::trace.previous.assign(d.costs.size(),0);study::trace.deltas=J::object();study::trace.prediction=J::object();
   if(study::trace.capture) {
    study::trace.raw=J::array();
    for(size_t i=0;i<d.trajectories.x.shape(0);i++) {J row=J::array();for(size_t k=0;k<d.trajectories.x.shape(1);k++) row.push_back({d.trajectories.x(i,k),d.trajectories.y(i,k),d.trajectories.yaws(i,k)});study::trace.raw.push_back(row);}
   }
  } else study::observe(d,label);
 }
};
#define OBS(NAME) class NAME final:public TraceBase{};
OBS(InputTraceCritic) OBS(ConstraintTraceCritic) OBS(CostTraceCritic) OBS(GoalTraceCritic) OBS(GoalAngleTraceCritic) OBS(PathAlignTraceCritic) OBS(PathFollowTraceCritic) OBS(PathAngleTraceCritic)
class TemporalResearchCritic final:public CriticFunction {
 std::mutex mutex_;J latest_;int64_t source_=0,receipt_=0;uint64_t rejected_=0;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
 dyn::Limits limits_;dyn::CostParameters cost_;double dt_=.1;int steps_=30;std::string topic_,prediction_epoch_;
public:
 void initialize() override {
  auto get=parameters_handler_->getParamGetter(name_);
  get(topic_,"topic",std::string("/research/time_predictions"),ParameterType::Static);
  get(limits_.max_age,"max_age",.4,ParameterType::Static);limits_.max_observation_age=.4;
  limits_.input_schema=dyn::Array::SCHEMA_OBSERVATION_ANCHOR;
  get(cost_.weight,"cost_weight",1.,ParameterType::Static);
  get(prediction_epoch_,"prediction_epoch",std::string("consume"),ParameterType::Static);
  if(prediction_epoch_!="consume" && prediction_epoch_!="input_pose")throw std::runtime_error("unknown prediction epoch");
  auto parent=parameters_handler_->getParamGetter(parent_name_);
  parent(dt_,"model_dt",.1,ParameterType::Static);parent(steps_,"time_steps",30,ParameterType::Static);
  auto node=parent_.lock();
  // Native CriticFunction defaults every plugin to true. This research plugin
  // requires an explicit enabled override in an isolated parameter file.
  if(!node->get_node_parameters_interface()->get_parameter_overrides().count(name_+".enabled")) {
   node->set_parameter(rclcpp::Parameter(name_+".enabled",false));enabled_=false;
  }
  sub_=node->create_subscription<std_msgs::msg::String>(topic_,rclcpp::QoS(1),[this,node](std_msgs::msg::String::ConstSharedPtr m){
   try {auto parsed=J::parse(m->data);int64_t source=parsed.at("source_ns");const int64_t received=node->now().nanoseconds();std::lock_guard<std::mutex> lock(mutex_);
    // An older callback cannot overwrite the accepted snapshot. A clock reset
    // causes consumer rejection until a new valid epoch is received.
    if(source<=0 || source>received){latest_=J::object();receipt_=received;rejected_++;return;}
    if(source<=source_){rejected_++;return;}latest_=std::move(parsed);source_=source;receipt_=received;
   } catch(...) {std::lock_guard<std::mutex> lock(mutex_);latest_=J::object();rejected_++;}
  });
 }
 void score(CriticData &d) override {
  if(!enabled_)return;
  auto node=parent_.lock();const int64_t now=node->now().nanoseconds();J message;int64_t received;uint64_t rejected;
  {std::lock_guard<std::mutex> lock(mutex_);message=latest_;received=receipt_;rejected=rejected_;}
  std::string reason;dyn::Array array;std::vector<std::vector<dyn::Point>> predictions;
  try {
   if(message.empty())throw std::runtime_error("missing_input");
   if(message.at("schema")!="rm_time_prediction/research_v1" || message.at("frame")!="map" || !message.at("complete").get<bool>())throw std::runtime_error("schema_frame_completeness");
   int64_t source=message.at("source_ns");array.header.stamp.sec=source/1000000000;array.header.stamp.nanosec=source%1000000000;
   array.header.frame_id="map";array.schema=dyn::Array::SCHEMA_OBSERVATION_ANCHOR;array.authority=dyn::Array::AUTHORITY_SHADOW_ONLY;array.complete=true;
   array.prediction_dt=dt_;array.prediction_steps=steps_;
   for(auto &item:message.at("tracks")) {
    dyn::Obstacle t;t.track_id=item.at("id");t.state=item.at("state");t.position.x=item.at("position")[0];t.position.y=item.at("position")[1];t.velocity.x=item.at("velocity")[0];t.velocity.y=item.at("velocity")[1];t.size.x=item.at("size")[0];t.size.y=item.at("size")[1];
    int64_t observed=item.at("observed_ns");t.last_observation_stamp.sec=observed/1000000000;t.last_observation_stamp.nanosec=observed%1000000000;array.tracks.push_back(t);
    std::vector<dyn::Point> points;for(auto &p:item.at("points"))points.push_back({p[0],p[1]});if(points.size()!=36)throw std::runtime_error("grid_length");predictions.push_back(points);
   }
   array.total_track_count=array.tracks.size();reason=dyn::validate(array,now*1e-9,limits_);if(!reason.empty())throw std::runtime_error(reason);
   if(std::abs(d.model_dt-dt_)>1e-6||d.trajectories.x.shape(1)!=size_t(steps_)||std::abs(message.at("dt").get<double>()-dt_)>1e-9)throw std::runtime_error("grid_dt");
  } catch(const std::exception &e){reason=e.what();}
  auto &trace=study::trace;trace.prediction={{"consume_ns",now},{"receipt_ns",received},{"rejected_callbacks",rejected},{"reason",reason},{"source",message}};
  if(!reason.empty()){d.fail_flag=true;return;}
  const int64_t candidate_epoch=prediction_epoch_=="input_pose" ? (trace.pose_epoch>0?trace.pose_epoch:trace.epoch) : now;
  const double age=(candidate_epoch-message.at("source_ns").get<int64_t>())*1e-9;
  trace.prediction["candidate_epoch_ns"]=candidate_epoch;trace.prediction["epoch_basis"]=prediction_epoch_;trace.prediction["alignment_age_s"]=age;
  // 3.5s source coverage includes the 3.0s candidate horizon plus <=.4s age.
  std::vector<dyn::Point> fp;for(auto p:costmap_ros_->getRobotFootprint())fp.push_back({p.x,p.y});
  auto tr=dyn::frame_transform(*costmap_ros_->getTfBuffer(),costmap_ros_->getGlobalFrameID(),"map",node->now(),.1);
  J clearances=J::array();size_t safe=0;double mincost=INFINITY,maxcost=0;
  for(size_t i=0;i<d.costs.size();i++) {
   double cost=0,minclear=INFINITY;
   for(int k=0;k<steps_;k++) {
    const double future=age+(k+1)*dt_;auto polygon=dyn::transform(fp,d.trajectories.x(i,k),d.trajectories.y(i,k),d.trajectories.yaws(i,k));
    double stage_clearance=INFINITY;
    for(size_t j=0;j<array.tracks.size();j++) {
     auto &obstacle=array.tracks[j];if(obstacle.state==dyn::Obstacle::STATE_TENTATIVE)continue;
     const double index=future/dt_;
     if(index<0 || index>35){d.fail_flag=true;trace.prediction["reason"]="candidate_outside_source_grid";return;}
     const size_t a=std::min<size_t>(35,static_cast<size_t>(index));const size_t b=std::min<size_t>(35,a+1);double alpha=index-a;
     dyn::Point p={predictions[j][a].x+alpha*(predictions[j][b].x-predictions[j][a].x),predictions[j][a].y+alpha*(predictions[j][b].y-predictions[j][a].y)};
     double clearance=dyn::circle_clearance(polygon,tr.apply(p),dyn::obstacle_radius(obstacle,limits_));stage_clearance=std::min(stage_clearance,clearance);
    }
    minclear=std::min(minclear,stage_clearance);cost+=dyn::penalty(stage_clearance,cost_)*dt_;
   }
   d.costs(i)+=cost;mincost=std::min(mincost,cost);maxcost=std::max(maxcost,cost);if(minclear>.02)safe++;clearances.push_back(minclear);
  }
  trace.prediction["age_s"]=(now-message.at("source_ns").get<int64_t>())*1e-9;trace.prediction["cost_min"]=mincost;trace.prediction["cost_max"]=maxcost;trace.prediction["safe_candidates"]=safe;
  if(trace.capture)trace.prediction["candidate_min_clearance"]=clearances;
  study::observe(d,"TemporalResearchCritic");
 }
};
}
namespace rm_stvl_temporal_research {
class Controller final:public nav2_mppi_controller::MPPIController {
 std::ofstream log_;rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr failures_;size_t count_=0,captures_=0;int64_t start_=0;
public:
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,const std::shared_ptr<tf2_ros::Buffer> tf,const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
  auto node=parent.lock();if(!node->has_parameter(name+".research_log"))node->declare_parameter(name+".research_log",std::string{});if(!node->has_parameter(name+".research_seed"))node->declare_parameter(name+".research_seed",13);
  std::string file;int seed;node->get_parameter(name+".research_log",file);node->get_parameter(name+".research_seed",seed);if(file.empty())throw std::runtime_error("explicit research log required");log_.open(file);xt::random::seed(seed);
  MPPIController::configure(parent,name,tf,costmap);
  failures_=node->create_publisher<std_msgs::msg::String>("/research/failure",rclcpp::QoS(1).transient_local());failures_->on_activate();
 }
 geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,nav2_core::GoalChecker *goal) override {
  auto begin=std::chrono::steady_clock::now();int64_t now=clock_->now().nanoseconds();if(!start_)start_=now;
  study::trace.capture=captures_<6 && (count_==0 || (now-start_)*1e-9>=captures_*2.);if(study::trace.capture)captures_++;
  study::trace.epoch=now;study::trace.pose_epoch=rclcpp::Time(pose.header.stamp,clock_->get_clock_type()).nanoseconds();
  J record={{"epoch_ns",now},{"pose_epoch_ns",study::trace.pose_epoch},{"pose",{pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)}},{"speed",{speed.linear.x,speed.linear.y,speed.angular.z}}};
  try {
   auto command=MPPIController::computeVelocityCommands(pose,speed,goal);record["command"]={command.twist.linear.x,command.twist.linear.y,command.twist.angular.z};
   auto optimal=optimizer_.getOptimizedTrajectory();record["aggregate"]=J::array();for(size_t k=0;k<optimal.shape(0);k++)record["aggregate"].push_back({optimal(k,0),optimal(k,1),optimal(k,2)});
   record["compute_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();record["prediction"]=study::trace.prediction;
   if(study::trace.capture){record["raw"]=study::trace.raw;record["critic_deltas"]=study::trace.deltas;}log_<<record.dump()<<'\n';count_++;return command;
  } catch(const std::exception &e) {
   record["error"]=e.what();record["prediction"]=study::trace.prediction;record["compute_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();if(study::trace.capture){record["raw"]=study::trace.raw;record["critic_deltas"]=study::trace.deltas;}log_<<record.dump()<<'\n';log_.flush();std_msgs::msg::String msg;msg.data=e.what();failures_->publish(msg);count_++;throw;
  }
 }
};
}
#define EXPORT_OBS(NAME) PLUGINLIB_EXPORT_CLASS(mppi::critics::NAME,mppi::critics::CriticFunction)
EXPORT_OBS(InputTraceCritic) EXPORT_OBS(ConstraintTraceCritic) EXPORT_OBS(CostTraceCritic) EXPORT_OBS(GoalTraceCritic) EXPORT_OBS(GoalAngleTraceCritic) EXPORT_OBS(PathAlignTraceCritic) EXPORT_OBS(PathFollowTraceCritic) EXPORT_OBS(PathAngleTraceCritic) EXPORT_OBS(TemporalResearchCritic)
PLUGINLIB_EXPORT_CLASS(rm_stvl_temporal_research::Controller,nav2_core::Controller)
