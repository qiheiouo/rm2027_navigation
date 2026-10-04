// Copyright 2026 RM Navigation. SPDX-License-Identifier: Apache-2.0
// No optimizer, global search, TF publisher or cmd_vel publisher lives here.
#include <nav2_core/controller.hpp>
#include "../include/stopping.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <rm_temporal_mpc_msgs/msg/plan.hpp>
#include <rm_temporal_mpc_msgs/msg/state_request.hpp>
#include <rm_temporal_mpc_msgs/msg/proposal.hpp>
#include <rm_competition_interfaces/msg/dynamic_obstacle_prediction_array.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <set>
#include <sstream>
#include <iomanip>
#include <limits>
#include <vector>

namespace rm_temporal_mpc {
using Proposal = rm_temporal_mpc_msgs::msg::Proposal;
using Predictions = rm_competition_interfaces::msg::DynamicObstaclePredictionArray;
using Clock = std::chrono::steady_clock;
static int64_t monotonic_ns(Clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}
const double period = .05, reach = std::hypot(.355,.330),
  nominal = 1.6970562748477143;
static int64_t ns(const builtin_interfaces::msg::Time &t) {
  if (t.sec < 0 || t.nanosec >= 1000000000) return -1;
  return int64_t(t.sec)*1000000000+t.nanosec;
}
static double reduce(double v, double change) {
  return std::copysign(std::max(0., std::abs(v)-change),v);
}
static bool finite(const geometry_msgs::msg::Twist &v) {
  return std::isfinite(v.linear.x) && std::isfinite(v.linear.y) && std::isfinite(v.angular.z);
}
struct Obstacle {uint64_t id; double x,y,vx,vy,r; int64_t observation;};

class Controller : public nav2_core::Controller {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent.lock(); if(!node_) throw std::runtime_error("expired lifecycle parent");
    tf_=tf; costmap_=costmap; name_=name;
    auto parameter=[&](const std::string &key, const std::string &value) {
      if(!node_->has_parameter(name+"."+key)) node_->declare_parameter(name+"."+key,value);
      return node_->get_parameter(name+"."+key).as_string();
    };
    const auto mode=parameter("geometry_mode","nominal_diameter");
    if(mode!="nominal_diameter" && mode!="visible_extent_proxy") throw std::runtime_error("invalid geometry mode");
    nominal_=mode=="nominal_diameter";
    const auto odom=parameter("odom_topic","/odometry/lio");
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    plan_pub_=node_->create_publisher<rm_temporal_mpc_msgs::msg::Plan>("temporal_mpc/plan",qos);
    request_pub_=node_->create_publisher<rm_temporal_mpc_msgs::msg::StateRequest>("temporal_mpc/state",1);
    health_pub_=node_->create_publisher<std_msgs::msg::String>("temporal_mpc/health",1);
    proposal_sub_=node_->create_subscription<Proposal>("temporal_mpc/proposal",1,[this](Proposal::SharedPtr p) {
      std::lock_guard<std::mutex> lock(mutex_); proposal_=p; receipt_=Clock::now();
    });
    prediction_sub_=node_->create_subscription<Predictions>("dynamic_obstacle_predictions",1,[this](Predictions::SharedPtr p) {
      std::lock_guard<std::mutex> lock(mutex_); accept_prediction(*p);
    });
    odom_sub_=node_->create_subscription<nav_msgs::msg::Odometry>(odom,1,[this](nav_msgs::msg::Odometry::SharedPtr p) {
      std::lock_guard<std::mutex> lock(mutex_);
      odom_=p; odom_receipt_=Clock::now();
    });
    map_sub_=node_->create_subscription<nav_msgs::msg::OccupancyGrid>("map",qos,[this](nav_msgs::msg::OccupancyGrid::SharedPtr p) {
      std::lock_guard<std::mutex> lock(mutex_); accept_map(*p);
    });
    path_sub_=node_->create_subscription<nav_msgs::msg::Path>("plan",rclcpp::QoS(1).reliable(),[this](nav_msgs::msg::Path::SharedPtr p) {
      std::lock_guard<std::mutex> lock(mutex_); adopt_plan(*p);
    });
    timer_=node_->create_wall_timer(std::chrono::milliseconds(50),[this]() {shadow();});
  }
  void activate() override {
    std::lock_guard<std::mutex> lock(mutex_); active_=true;
    plan_pub_->on_activate(); request_pub_->on_activate(); health_pub_->on_activate();
    publish_plan();
  }
  void deactivate() override {
    std::lock_guard<std::mutex> lock(mutex_); active_=false; proposal_.reset();
    plan_pub_->on_deactivate(); request_pub_->on_deactivate(); health_pub_->on_deactivate();
  }
  void cleanup() override {
    timer_.reset(); proposal_sub_.reset(); prediction_sub_.reset(); map_sub_.reset(); odom_sub_.reset(); path_sub_.reset();
    proposal_.reset(); odom_.reset(); plan_pub_.reset(); request_pub_.reset(); health_pub_.reset();
    costmap_.reset(); tf_.reset(); node_.reset();
  }
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(mutex_); adopt_plan(path);
  }
  void setSpeedLimit(const double &limit, const bool &percentage) override {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_scale_=limit==0. ? 1. : (!std::isfinite(limit) || limit<0 ? 0. :
      std::clamp(percentage ? limit/100. : limit/.8,0.,1.));
    proposal_.reset();
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker *) override {
    auto started=Clock::now(); std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::TwistStamped out; out.header.stamp=node_->now(); out.header.frame_id=costmap_->getBaseFrameID();
    geometry_msgs::msg::PoseStamped world;
    std::string reason="invalid state"; bool ready=false; reset_diagnostic();
    if(active_ && to_map(pose,world) && finite(velocity)) {
      // Revalidation uses THIS callback's measured state, never a cached safety verdict.
      request(world.pose,velocity,out.header.stamp);
      ready=validate(world.pose,velocity,ns(out.header.stamp),out.twist,reason);
    }
    if(!ready) {
      // Braking is continuous output, but is not called collision-free when no certificate exists.
      if(finite(velocity)) {
        out.twist.linear.x=reduce(velocity.linear.x,period);
        out.twist.linear.y=reduce(velocity.linear.y,period);
        out.twist.angular.z=reduce(velocity.angular.z,2*period);
      }
    }
    ++computations_; health(ready,reason,true,std::chrono::duration<double>(Clock::now()-started).count());
    return out;
  }
private:
  bool to_map(const geometry_msgs::msg::PoseStamped &in, geometry_msgs::msg::PoseStamped &out) {
    try {out=in.header.frame_id=="map" ? in : tf_->transform(in,"map",tf2::durationFromSec(0.));}
    catch(const tf2::TransformException &) {return false;}
    const auto &p=out.pose; const auto &q=p.orientation;
    double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(norm) &&
      std::abs(norm-1)<1e-3 && std::abs(q.x)<1e-6 && std::abs(q.y)<1e-6;
  }
  void adopt_plan(const nav_msgs::msg::Path &path) {
    if(path.header.frame_id!="map" || path.poses.size()<2 || path.poses.size()>4096) {
      plan_=nav_msgs::msg::Path(); ++generation_; proposal_.reset(); publish_plan(); return;
    }
    for(const auto &p:path.poses) if(!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y)) {
      plan_=nav_msgs::msg::Path(); ++generation_; proposal_.reset(); publish_plan(); return;
    }
    bool same=path.header.frame_id==plan_.header.frame_id && path.poses.size()==plan_.poses.size();
    if(same) for(size_t i=0;i<path.poses.size();i++) {
      if(path.poses[i].pose.position.x!=plan_.poses[i].pose.position.x ||
        path.poses[i].pose.position.y!=plan_.poses[i].pose.position.y) {same=false; break;}
    }
    if(!same) {plan_=path; ++generation_; proposal_.reset();}
    publish_plan();
  }
  void publish_plan() {
    if(active_ && plan_pub_) {rm_temporal_mpc_msgs::msg::Plan p; p.generation=generation_; p.path=plan_; plan_pub_->publish(p);}
  }
  void accept_map(const nav_msgs::msg::OccupancyGrid &p) {
    map_valid_=false; proposal_.reset(); blocked_.clear();
    const auto &info=p.info; const auto &q=info.origin.orientation;
    if(p.header.frame_id!="map" || ns(p.header.stamp)<0 || info.width<3 || info.height<3 ||
      info.width>1000 || info.height>1000 || uint64_t(info.width)*info.height>100000 ||
      p.data.size()!=uint64_t(info.width)*info.height || !std::isfinite(info.resolution) ||
      info.resolution<.01 || info.resolution>.2 || !std::isfinite(info.origin.position.x) ||
      !std::isfinite(info.origin.position.y) || q.x!=0 || q.y!=0 || q.z!=0 || q.w!=1) return;
    resolution_=info.resolution; origin_={info.origin.position.x,info.origin.position.y};
    map_right_=origin_[0]+info.width*resolution_; map_top_=origin_[1]+info.height*resolution_;
    for(unsigned y=0;y<info.height;y++) for(unsigned x=0;x<info.width;x++) {
      const int cost=p.data[y*info.width+x]; if(cost < -1 || cost>100) return;
      if(cost<0 || cost>=65 || x==0 || y==0 || x==info.width-1 || y==info.height-1)
        blocked_.push_back({origin_[0]+x*resolution_,origin_[1]+y*resolution_});
    }
    revision_=ns(p.header.stamp); map_valid_=true; ++generation_; publish_plan();
  }
  void accept_prediction(const Predictions &p) {
    predictions_valid_=false; obstacles_.clear(); const auto source=ns(p.header.stamp);
    const int64_t now=node_->now().nanoseconds();
    if(source<0 || source>now || now-source>400000000 || source<=last_source_ || p.header.frame_id!="map" ||
      p.schema!="rm_dynamic_obstacle_predictions/v2_observation_anchor" || p.authority!="shadow_only" ||
      !p.complete || p.total_track_count!=p.tracks.size() || p.tracks.size()>64 ||
      p.prediction_steps!=15 || std::abs(p.prediction_dt-.1)>1e-12 || ns(p.processing_stamp)<source) return;
    std::set<uint64_t> ids;
    for(const auto &t:p.tracks) {
      const auto observation=ns(t.last_observation_stamp);
      const double fields[]={t.position.x,t.position.y,t.position.z,t.velocity.x,t.velocity.y,t.velocity.z,t.size.x,t.size.y,t.size.z};
      for(double v:fields) if(!std::isfinite(v)) return;
      if(!ids.insert(t.track_id).second || t.state<1 || t.state>3 || observation<0 || observation>source ||
        ((t.state==3 && observation>=source) || (t.state==2 && observation!=source)) || t.size.x<=0 || t.size.y<=0 ||
        t.size.x>3 || t.size.y>3 || t.size.z<0 || std::abs(t.position.z)>1e-6 ||
        std::abs(t.velocity.z)>1e-6 || std::hypot(t.velocity.x,t.velocity.y)>3 || t.prediction.size()!=15) return;
      for(const auto &v:t.prediction) if(!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) return;
      for(const auto &old:last_obstacles_) if(old.id==t.track_id && std::hypot(t.position.x-old.x,t.position.y-old.y)>.5+3*(source-last_source_)*1e-9) return;
      obstacles_.push_back({t.track_id,t.position.x,t.position.y,t.state==1?0.:t.velocity.x,
        t.state==1?0.:t.velocity.y,nominal_?nominal:std::max(.36,.5*std::hypot(t.size.x,t.size.y)),observation});
    }
    last_source_=source; last_obstacles_=obstacles_; predictions_valid_=true;
  }
  bool static_certificate(const std::array<double,4> &b) {
    if(!map_valid_ || b[0]<origin_[0] || b[1]>map_right_ || b[2]<origin_[1] || b[3]>map_top_) return false;
    for(const auto &cell:blocked_) {
      if(Clock::now()-validation_started_>std::chrono::milliseconds(10)) return false;
      const auto dx=std::max({cell[0]-b[1],b[0]-cell[0]-resolution_,0.});
      const auto dy=std::max({cell[1]-b[3],b[2]-cell[1]-resolution_,0.});
      if(std::hypot(dx,dy)<reach+.02-1e-6) return false;
    }
    return true;
  }
  bool current_grid_clear(double x,double y,double yaw, const geometry_msgs::msg::Twist &cmd) {
    // Check an enclosing circle against every occupied cell INSIDE the footprint,
    // including unknown and inscribed cells, not just the perimeter.
    if(costmap_->getGlobalFrameID()!="map") return false;
    auto grid=costmap_->getCostmap();
    // Controller Server already owns this recursive map mutex during compute.
    // Shadow callbacks must not invert map -> plugin mutex acquisition order.
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex(),std::try_to_lock);
    if(!lock.owns_lock()) return false;
    const auto c=std::cos(yaw),s=std::sin(yaw); double radius=reach+.02+.005*std::hypot(cmd.linear.x,cmd.linear.y);
    for(int k=0;k<=5;k++) {
      if(Clock::now()-validation_started_>std::chrono::milliseconds(10)) return false;
      const double t=k*.01, px=x+t*(c*cmd.linear.x-s*cmd.linear.y),py=y+t*(s*cmd.linear.x+c*cmd.linear.y);
      unsigned x0,y0,x1,y1;
      if(!grid->worldToMap(px-radius,py-radius,x0,y0) || !grid->worldToMap(px+radius,py+radius,x1,y1)) return false;
      for(unsigned iy=y0;iy<=y1;iy++) for(unsigned ix=x0;ix<=x1;ix++) {
        if(grid->getCost(ix,iy)<253) continue;
        double cx,cy; grid->mapToWorld(ix,iy,cx,cy); const double half=.5*grid->getResolution();
        if(std::hypot(std::max(std::abs(px-cx)-half,0.),std::max(std::abs(py-cy)-half,0.))<radius) return false;
      }
    }
    return true;
  }
  bool validate(const geometry_msgs::msg::Pose &pose,const geometry_msgs::msg::Twist &v,int64_t now,
    geometry_msgs::msg::Twist &cmd,std::string &reason) {
    validation_started_=Clock::now();
    evaluation_ns_=now; step_=-1; track_=0; slack_=std::numeric_limits<double>::quiet_NaN();
    constraint_="input"; handoff_=false;
    checked_state_={pose.position.x,pose.position.y,tf2::getYaw(pose.orientation),v.linear.x,v.linear.y,v.angular.z};
    initial_state_=checked_state_;
    auto reject=[&](const char *text) {reason=text; return false;};
    if(!proposal_ || !odom_ || !finite(v)) return reject("missing proposal/state");
    if(std::chrono::duration<double>(Clock::now()-odom_receipt_).count()>.15 ||
      now-ns(odom_->header.stamp)>150000000 || now<ns(odom_->header.stamp) || odom_->child_frame_id!=costmap_->getBaseFrameID()) return reject("stale measured velocity");
    const auto &p=*proposal_; const auto age=(now-ns(p.header.stamp))*1e-9;
    if(p.header.frame_id!="map" || ns(p.header.stamp)<0 || age<0 || age>.15 ||
      std::chrono::duration<double>(Clock::now()-receipt_).count()>.15 || p.generation!=generation_ ||
      !map_valid_ || p.map_revision!=revision_ || !p.model_feasible ||
      p.period!=period || p.accelerations.size()!=30) return reject("proposal stale/rejected/generation");
    if(!predictions_valid_ || now<last_source_ || now-last_source_>400000000) return reject("prediction degraded/stale");
    for(const auto &o:obstacles_) if(now<o.observation || now-o.observation>400000000) return reject("observation stale");
    const auto yaw=tf2::getYaw(pose.orientation);
    if(!std::isfinite(p.fixed_yaw) || std::abs(std::remainder(yaw-p.fixed_yaw,2*M_PI))>.001 || std::abs(v.angular.z)>1e-6) return reject("fixed yaw domain");
    std::array<double,4> b; std::copy(p.centre_bounds.begin(),p.centre_bounds.end(),b.begin());
    for(double a:b) if(!std::isfinite(a)) return reject("nonfinite corridor");
    if(b[0]>=b[1] || b[2]>=b[3] || !static_certificate(b)) return reject("raw static corridor certificate");
    for(const auto &a:p.accelerations) if(!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) || std::abs(a.x)>1.000001 || std::abs(a.y)>1.000001 || a.z!=0) return reject("acceleration bounds");
    double x=pose.position.x,y=pose.position.y,vx=v.linear.x,vy=v.linear.y;
    const double c=std::cos(yaw),s=std::sin(yaw); const double reserve=.5*period*std::hypot(.8,.5);
    if(!std::isfinite(x) || !std::isfinite(y) || std::abs(vx)>.8 || vx<-.5 || std::abs(vy)>.5) return reject("state bounds");
    auto point_clear=[&](double t) {
      checked_state_={x,y,yaw,vx,vy,0.};
      const double values[]={x-b[0]-reserve,b[1]-x-reserve,y-b[2]-reserve,b[3]-y-reserve,
        vx+.5*speed_scale_,.8*speed_scale_-vx,.5*speed_scale_-std::abs(vy)};
      const char *names[]={"corridor_x_lower","corridor_x_upper","corridor_y_lower","corridor_y_upper",
        "velocity_x_lower","velocity_x_upper","velocity_y"};
      for(size_t i=0;i<7;i++) if(values[i] < (i<4?0.:-1e-6)) {
        constraint_=names[i]; slack_=values[i]; return false;
      }
      for(const auto &o:obstacles_) {
        double ox=o.x+((now-last_source_)*1e-9+t)*o.vx, oy=o.y+((now-last_source_)*1e-9+t)*o.vy;
        const double dx=c*(ox-x)+s*(oy-y),dy=-s*(ox-x)+c*(oy-y);
        const double distance=std::hypot(std::max(std::abs(dx)-.355,0.),std::max(std::abs(dy)-.330,0.));
        const double slack=distance-o.r-.02-.5*period*(std::hypot(.8,.5)+std::hypot(o.vx,o.vy));
        if(slack<0.) {constraint_="dynamic_clearance"; track_=o.id; slack_=slack; return false;}
      }
      return true;
    };
    step_=0;
    if(!point_clear(0)) return reject("current dynamic/corridor bounds");
    // Align to elapsed proposal time, re-anchor to NEW measured state, and end
    // with bounded braking. The last 150 ms are reserved for terminal correction.
    for(int k=0;k<30;k++) {
      if(Clock::now()-validation_started_>std::chrono::milliseconds(10)) return reject("validation deadline");
      const double from=age+k*period,to=from+period;
      double ax=0,ay=0;
      if(k>=27) {ax=(reduce(vx,period)-vx)/period; ay=(reduce(vy,period)-vy)/period;}
      else for(size_t i=0;i<30;i++) {
        const double overlap=std::max(0.,std::min(to,(i+1)*period)-std::max(from,i*period));
        ax+=overlap/period*p.accelerations[i].x; ay+=overlap/period*p.accelerations[i].y;
      }
      // A newly held velocity may differ from the proposal's ramp state. Reserve
      // enough remaining acceleration authority to stop INSIDE this horizon.
      // All repaired positions are subsequently rechecked against every track.
      const double remaining=(29-k)*period;
      ax=bounded_stopping_acceleration(vx,ax,remaining,-.5,.8);
      ay=bounded_stopping_acceleration(vy,ay,remaining,-.5,.5);
      if(k==0) {cmd.linear.x=vx+period*ax; cmd.linear.y=vy+period*ay;}
      // Same velocity ZOH model as the condensed worker QP. Acceleration
      // bounds describe command differences; they do not certify wheel transients.
      vx+=period*ax; vy+=period*ay;
      x+=period*(c*vx-s*vy); y+=period*(s*vx+c*vy); step_=k+1;
      if(!point_clear((k+1)*period)) return reject("reanchored trajectory rejected");
    }
    if(std::max(std::abs(vx),std::abs(vy))>1e-5) return reject("terminal stop");
    // Independently check the first held-command interval against current STVL.
    if(!current_grid_clear(pose.position.x,pose.position.y,yaw,cmd)) return reject("execution grid blocked");
    handoff_=p.fallback_requested; constraint_="accepted"; slack_=0.;
    reason=handoff_?"accepted_feasible_handoff":"accepted_reanchored"; return true;
  }
  void request(const geometry_msgs::msg::Pose &pose,const geometry_msgs::msg::Twist &v,const builtin_interfaces::msg::Time &stamp) {
    rm_temporal_mpc_msgs::msg::StateRequest p; p.header.frame_id="map"; p.header.stamp=stamp;
    p.generation=generation_; p.pose=pose; p.velocity=v;
    request_epoch_ns_=ns(stamp); request_publish_clock_ns_=node_->now().nanoseconds();
    request_publish_ns_=monotonic_ns(Clock::now()); request_pub_->publish(p);
    request_publish_return_ns_=monotonic_ns(Clock::now());
  }
  void reset_diagnostic() {
    evaluation_ns_=node_->now().nanoseconds(); step_=-1; track_=0; handoff_=false;
    slack_=std::numeric_limits<double>::quiet_NaN(); constraint_="input";
    initial_state_.fill(slack_); checked_state_.fill(slack_);
    request_epoch_ns_=request_publish_clock_ns_=request_publish_ns_=request_publish_return_ns_=-1;
  }
  void health(bool ready,const std::string &reason,bool executed,double elapsed) {
    std_msgs::msg::String p; std::ostringstream out;
    // Reasons are internal fixed strings, never supplied by a message.
    out<<std::setprecision(17)<<"{\"ready\":"<<(ready?"true":"false")<<",\"fallback_requested\":"<<(!ready || handoff_?"true":"false")
       <<",\"reason\":\""<<reason<<"\",\"executed\":"<<(executed?"true":"false")
       <<",\"compute_count\":"<<computations_<<",\"elapsed_s\":"<<elapsed
       <<",\"evaluation_ns\":"<<evaluation_ns_
       <<",\"proposal_ns\":"<<(proposal_?ns(proposal_->header.stamp):-1)
       <<",\"prediction_ns\":"<<last_source_<<",\"generation\":"<<generation_
       <<",\"request_epoch_ns\":"<<request_epoch_ns_
       <<",\"request_publish_clock_ns\":"<<request_publish_clock_ns_
       <<",\"request_publish_monotonic_ns\":"<<request_publish_ns_
       <<",\"request_publish_return_monotonic_ns\":"<<request_publish_return_ns_
       <<",\"proposal_receipt_monotonic_ns\":"<<(proposal_?monotonic_ns(receipt_):-1)
       <<",\"model\":\"fixed_yaw_velocity_zoh/v1\",\"constraint\":\""<<constraint_
       <<"\",\"reanchor_projection\":\"velocity_and_stop/v2"
       <<"\",\"step\":"<<step_<<",\"track_id\":"<<track_<<",\"slack\":";
    if(std::isfinite(slack_)) out<<slack_; else out<<"null";
    auto state=[&](const char *name,const std::array<double,6> &v) {
      out<<",\""<<name<<"\":[";
      for(size_t i=0;i<v.size();i++) {if(i) out<<','; if(std::isfinite(v[i])) out<<v[i]; else out<<"null";}
      out<<']';
    };
    state("initial_state",initial_state_); state("checked_state",checked_state_); out<<'}';
    p.data=out.str(); health_pub_->publish(p);
  }
  void shadow() {
    std::lock_guard<std::mutex> lock(mutex_); if(!active_) return;
    geometry_msgs::msg::PoseStamped pose,world; geometry_msgs::msg::Twist cmd;
    std::string reason="missing TF/odom"; bool ready=false; reset_diagnostic(); const builtin_interfaces::msg::Time now=node_->now();
    if(odom_) {pose.header=odom_->header; pose.pose=odom_->pose.pose;}
    if(odom_ && to_map(pose,world)) {
      request(world.pose,odom_->twist.twist,now);
      ready=validate(world.pose,odom_->twist.twist,ns(now),cmd,reason);
    }
    health(ready,reason,false,0.);
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_; std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::string name_; std::mutex mutex_; bool active_=false,nominal_=true,map_valid_=false,predictions_valid_=false;
  uint64_t generation_=0,revision_=0,computations_=0; int64_t last_source_=-1;
  int64_t evaluation_ns_=-1; int step_=-1; uint64_t track_=0; bool handoff_=false;
  int64_t request_epoch_ns_=-1,request_publish_clock_ns_=-1,request_publish_ns_=-1,request_publish_return_ns_=-1;
  double slack_=0.; std::string constraint_="input";
  std::array<double,6> initial_state_{},checked_state_{};
  double resolution_=0,map_right_=0,map_top_=0,speed_scale_=1.; std::array<double,2> origin_{};
  std::vector<std::array<double,2>> blocked_; std::vector<Obstacle> obstacles_,last_obstacles_;
  nav_msgs::msg::Path plan_; nav_msgs::msg::Odometry::SharedPtr odom_; Proposal::SharedPtr proposal_;
  Clock::time_point receipt_,odom_receipt_,validation_started_; rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<rm_temporal_mpc_msgs::msg::Plan>::SharedPtr plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<rm_temporal_mpc_msgs::msg::StateRequest>::SharedPtr request_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Subscription<Proposal>::SharedPtr proposal_sub_; rclcpp::Subscription<Predictions>::SharedPtr prediction_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};
} // namespace rm_temporal_mpc
PLUGINLIB_EXPORT_CLASS(rm_temporal_mpc::Controller,nav2_core::Controller)
