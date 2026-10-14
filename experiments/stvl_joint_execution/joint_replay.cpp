// No ROS initialization, optimizer mutation, node, command publisher or owner.
#include "joint_core.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char**argv){
 try {
  if(argc!=2)throw std::runtime_error("expected JSON context path");std::ifstream f(argv[1]);JointJson in;f>>in;joint::Grid grid(in.at("static_map"));auto begin=std::chrono::steady_clock::now();JointJson result=JointJson::array();
  nav2_costmap_2d::Costmap2D native(grid.w,grid.h,grid.r,grid.x,grid.y,0);
  for(unsigned y=0;y<grid.h;y++)for(unsigned x=0;x<grid.w;x++)native.setCost(x,y,grid.cells[y*grid.w+x]);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(&native);std::vector<geometry_msgs::msg::Point> fp;
  for(auto v:in.at("footprint")){geometry_msgs::msg::Point p;p.x=v[0];p.y=v[1];fp.push_back(p);}
  for(const auto&t:in.at("trajectories")){auto row=joint::diagnose(in,t,grid);row["id"]=t.at("id");int hits=0;for(auto p:t.at("poses"))hits+=checker.footprintCostAtPose(p[0],p[1],p[2],fp)==254;row["native_same_static_grid_discrete_hits"]=hits;result.push_back(row);}
  std::cout<<JointJson{{"case",in.at("case")},{"epoch_ns",in.at("pose_epoch_ns")},{"scope","nominal (X,U); Unknown downstream execution is not a safe command certificate"},{"results",result},{"total_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count()}}.dump()<<'\n';
 } catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
}
