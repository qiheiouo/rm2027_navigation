// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
// Offline only: no costmap activation, TF broadcast, action goals or cmd_vel.
#include "rm_tdt_planner/planner.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_smac_planner/smac_planner_2d.hpp"
#include "nav2_map_server/map_io.hpp"
#include "benchmark_geometry.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <sys/resource.h>

using namespace rm_tdt_planner;
using Clock = std::chrono::steady_clock;
namespace
{
struct Scenario {std::string name; Grid raw; bool crossing = false;};
Point point(const Grid & g, int cell)
{
  return {g.origin_x + (cell%g.width + 0.5)*g.resolution,
    g.origin_y + (cell/g.width + 0.5)*g.resolution};
}
Grid blank()
{
  return {200,160,0.05,-3.25,-1.75,std::vector<uint8_t>(32000,0)};
}
std::vector<Scenario> scenarios(const std::string & map_yaml)
{
  std::vector<Scenario> out{{"open",blank()},{"wall_detour",blank()}};
  for (int y=0; y<110; ++y) {out.back().raw.costs[y*200+100]=254;}
  for (auto spec : {std::make_pair("passage_080m",16),std::make_pair("passage_140m",28),
      std::make_pair("blocker_present",0),std::make_pair("blocker_removed",28)})
  {
    auto g=blank();
    for (int y=0; y<g.height; ++y) {
      if (y<80-spec.second/2 || y>=80+spec.second/2) {g.costs[y*200+100]=254;}
    }
    out.push_back({spec.first,std::move(g),true});
  }
  auto unknown=blank();
  for (int y=0; y<160; ++y) {unknown.costs[y*200+100]=255;}
  out.push_back({"unknown_barrier",std::move(unknown),true});
  if (!map_yaml.empty()) {
    nav_msgs::msg::OccupancyGrid occupancy;
    if (nav2_map_server::loadMapFromYaml(map_yaml,occupancy) != nav2_map_server::LOAD_MAP_SUCCESS) {
      throw std::runtime_error("map_server could not load benchmark map");
    }
    const auto & q=occupancy.info.origin.orientation;
    if (std::abs(q.x)+std::abs(q.y)+std::abs(q.z)>1e-8 || std::abs(q.w-1)>1e-8) {
      throw std::runtime_error("rotated map origin unsupported");
    }
    Grid g{static_cast<int>(occupancy.info.width),static_cast<int>(occupancy.info.height),
      occupancy.info.resolution,occupancy.info.origin.position.x,occupancy.info.origin.position.y,{}};
    for (auto c:occupancy.data) {g.costs.push_back(c<0?255:(c==0?0:254));}
    out.push_back({"field_map",std::move(g)});
  }
  return out;
}
std::vector<int> largest_component(const Grid & g)
{
  std::vector<bool> seen(g.costs.size(),false);
  std::vector<int> largest;
  for (int i=0; i<static_cast<int>(g.costs.size()); ++i) {
    if (seen[i] || g.costs[i]>=253) {continue;}
    std::vector<int> component{i}; seen[i]=true;
    for (size_t j=0; j<component.size(); ++j) {
      int x=component[j]%g.width,y=component[j]/g.width;
      for (auto d:{std::make_pair(-1,0),{1,0},{0,-1},{0,1}}) {
        int nx=x+d.first,ny=y+d.second;
        if (nx<0 || ny<0 || nx>=g.width || ny>=g.height) {continue;}
        int next=ny*g.width+nx;
        if (!seen[next] && g.costs[next]<253) {seen[next]=true;component.push_back(next);}
      }
    }
    if (component.size()>largest.size()) {largest=std::move(component);}
  }
  return largest;
}
std::pair<Point,Point> request(const Scenario & scenario,const PreparedGrid & prepared,
  std::mt19937 & rng,const std::vector<int> & component)
{
  const auto & g=prepared.grid();
  if (scenario.crossing) {
    const int sy=60+static_cast<int>(rng()%40),gy=60+static_cast<int>(rng()%40);
    const int sx=25+static_cast<int>(rng()%35),gx=140+static_cast<int>(rng()%35);
    return {point(g,sy*g.width+sx),point(g,gy*g.width+gx)};
  }
  if (component.size()<2) {throw std::runtime_error("no free component for requests");}
  for (int attempt=0; attempt<1000; ++attempt) {
    auto a=point(g,component[rng()%component.size()]);
    auto b=point(g,component[rng()%component.size()]);
    if (std::hypot(a.x-b.x,a.y-b.y)>2.0) {return {a,b};}
  }
  throw std::runtime_error("map component too small for 2m requests");
}

double length(const std::vector<Point> & p)
{
  double total=0;for(size_t i=1;i<p.size();++i){total+=std::hypot(p[i].x-p[i-1].x,p[i].y-p[i-1].y);}return total;
}
double turning(const std::vector<Point> & p)
{
  // Geometric turning only, independent of pose yaw; not physical jerk.
  double total=0,last=0;bool have=false;
  for(size_t i=1;i<p.size();++i){
    double dx=p[i].x-p[i-1].x,dy=p[i].y-p[i-1].y;
    if(std::hypot(dx,dy)<1e-7){continue;}
    double angle=std::atan2(dy,dx);
    if(have){total+=std::abs(std::remainder(angle-last,2*M_PI));}
    have=true;last=angle;
  }
  return total;
}
geometry_msgs::msg::PoseStamped pose(Point p)
{
  geometry_msgs::msg::PoseStamped out;out.header.frame_id="map";
  out.pose.position.x=p.x;out.pose.position.y=p.y;out.pose.orientation.w=1;return out;
}
}  // namespace

int main(int argc,char ** argv)
{
  if (argc<3 || argc>4) {
    std::cerr<<"usage: benchmark_planners OUTPUT.csv TRIALS [MAP.yaml]\n";return 2;
  }
  try {
    const std::string output=argv[1],map_yaml=argc==4?argv[3]:"";
    const int trials=std::stoi(argv[2]);if(trials<1||trials>1000){return 2;}
    {std::ifstream exists(output);if(exists.good()){throw std::runtime_error("output exists");}}
    std::ofstream csv(output);if(!csv){throw std::runtime_error("cannot create CSV");}
    rclcpp::init(0,nullptr);
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("tdt_p2_benchmark");
    auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("benchmark_costmap");
    map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
    map->set_parameter(rclcpp::Parameter("footprint","[[-0.32,-0.27],[-0.32,0.27],[0.32,0.27],[0.32,-0.27]]"));
    map->set_parameter(rclcpp::Parameter("footprint_padding",0.02));
    map->on_configure(rclcpp_lifecycle::State());
    node->declare_parameter("Navfn.tolerance",0.0);
    node->declare_parameter("Navfn.allow_unknown",false);
    node->declare_parameter("Smac.tolerance",0.0);
    node->declare_parameter("Smac.allow_unknown",false);
    node->declare_parameter("Smac.max_planning_time",0.25);
    // Humble 1.1.20 always invokes its built-in smoother; no smooth_path parameter.
    nav2_navfn_planner::NavfnPlanner navfn;
    nav2_smac_planner::SmacPlanner2D smac;
    navfn.configure(node,"Navfn",{},map);smac.configure(node,"Smac",{},map);
    navfn.activate();smac.activate();
    Options options;options.radius=std::hypot(0.34,0.29);
    csv<<"scenario,trial,planner,seed,start_x,start_y,goal_x,goal_y,returned,accepted,optimized,"
       <<"common_geometry_ok,physical_clearance_m,length_m,turning_rad,prepare_ms,planner_ms,"
       <<"prepare_plus_plan_ms,rss_max_kb,reason\n";
    csv<<std::setprecision(10);
    for(const auto & scenario:scenarios(map_yaml)){
      // Reset so blocker present/removed and passage widths share the same requests.
      std::mt19937 rng(20260913);
      auto begin=Clock::now();
      const auto prepared=prepare_grid(scenario.raw,options);
      double prep_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
      const auto & g=prepared.grid();
      const auto component=largest_component(g);
      std::cout<<"scenario "<<scenario.name<<" free_component="<<component.size()<<std::endl;
      for(int trial=0;trial<trials;++trial){
        auto [start,goal]=request(scenario,prepared,rng,component);
        // Rotate execution order to reduce cache/thermal bias.
        for(int offset=0;offset<4;++offset){
          const int which=(trial+offset)%4;
          map->getCostmap()->resizeMap(g.width,g.height,g.resolution,g.origin_x,g.origin_y);
          std::copy(g.costs.begin(),g.costs.end(),map->getCostmap()->getCharMap());
          const char * name=which==0?"navfn":which==1?"smac2d":which==2?"tdt_astar":"tdt_qp";
          Result result;begin=Clock::now();
          try {
            if(which<2){
              auto p=which==0?navfn.createPlan(pose(start),pose(goal)):smac.createPlan(pose(start),pose(goal));
              for(const auto & q:p.poses){result.path.push_back({q.pose.position.x,q.pose.position.y});}
              result.success=!result.path.empty();result.reason=result.success?"returned":"empty";
            }else{
              auto o=options;o.optimize=which==3;
              result=plan_prepared(prepared,start,goal,o);
            }
          }catch(const std::exception &){result.success=false;result.reason="planner_exception";}
          double ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
          bool geometry=result.success&&collision_free_prepared(prepared,result.path);
          double clearance=benchmark_geometry::clearance(scenario.raw,result.path,options.radius+options.clearance);
          bool endpoints=!result.path.empty()&&
            std::hypot(result.path.front().x-start.x,result.path.front().y-start.y)<=g.resolution&&
            std::hypot(result.path.back().x-goal.x,result.path.back().y-goal.y)<=g.resolution;
          bool accepted=result.success&&clearance>=-1e-9&&endpoints&&ms+prep_ms<=250.0;
          struct rusage usage{};getrusage(RUSAGE_SELF,&usage);
          csv<<scenario.name<<','<<trial<<','<<name<<",20260913,"<<start.x<<','<<start.y<<','
             <<goal.x<<','<<goal.y<<','<<result.success<<','<<accepted<<','<<result.optimized<<','
             <<geometry<<','<<clearance<<','<<length(result.path)<<','<<turning(result.path)<<','
             <<prep_ms<<','<<ms<<','<<ms+prep_ms<<','<<usage.ru_maxrss<<','<<result.reason<<'\n';
        }
      }
      csv.flush();
    }
    smac.deactivate();navfn.deactivate();smac.cleanup();navfn.cleanup();
    map->on_cleanup(rclcpp_lifecycle::State());map.reset();node.reset();
    rclcpp::shutdown();return 0;
  } catch(const std::exception & e){std::cerr<<e.what()<<'\n';return 1;}
}
