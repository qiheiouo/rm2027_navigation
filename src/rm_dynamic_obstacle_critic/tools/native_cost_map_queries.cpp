// File-only queries of the unchanged installed footprint checker, without ROS
// initialization or a Costmap2DROS wrapper. Never an actual CostCritic score.
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>

template<typename T> void read_values(std::istream &s,T *values,size_t count) {
  s.read(reinterpret_cast<char *>(values),count*sizeof(T));
  if(!s)throw std::runtime_error("truncated native map query");
}
template<typename T> void finite_values(const T &values) {
  for(const auto v:values)if(!std::isfinite(v))throw std::runtime_error("nonfinite native map query");
}
struct Trace { uint32_t kind,row,epoch; double start; std::vector<std::array<double,4>> poses; };
struct Input {
  float dt; std::array<double,4> map;
  nav2_costmap_2d::Footprint footprint;
  std::array<unsigned char,10000> raw;
  std::vector<Trace> traces;
};
Input read_input(const char *path) {
  if(std::filesystem::file_size(path)!=689868)throw std::runtime_error("native map query exact byte budget");
  std::ifstream s(path,std::ios::binary);std::array<char,8> magic;read_values(s,magic.data(),8);
  if(std::memcmp(magic.data(),"RMCOSTQ1",8))throw std::runtime_error("native map query private schema");
  std::array<uint32_t,8> h;read_values(s,h.data(),h.size());
  if(h!=std::array<uint32_t,8>{298,100,100,4,102,21165,2,30})throw std::runtime_error("native map query source/map/trace budget");
  Input input;read_values(s,&input.dt,1);read_values(s,input.map.data(),input.map.size());finite_values(input.map);
  if(input.dt!=float(.1) || input.map[0]!=.05 || input.map[3]!=30*double(input.dt))throw std::runtime_error("native map query time/resolution mismatch");
  input.footprint.resize(4);
  for(auto &p:input.footprint) {
    std::array<double,2> xy;read_values(s,xy.data(),2);finite_values(xy);p.x=xy[0];p.y=xy[1];p.z=0;
  }
  read_values(s,input.raw.data(),input.raw.size());std::vector<std::pair<uint32_t,uint32_t>> first_ids;size_t count=0;
  for(size_t i=0;i<102;++i) {
    std::array<uint32_t,4> m;read_values(s,m.data(),m.size());Trace trace;trace.kind=m[0];trace.row=m[1];trace.epoch=m[2];read_values(s,&trace.start,1);
    const size_t epoch=i/51,index=i%51,samples=epoch?207:208;
    if(trace.epoch!=epoch || m[3]!=samples || !std::isfinite(trace.start) || trace.start!=(epoch?47.98:48.004) ||
        (index<8 && (trace.kind!=0 || trace.row!=index)) || (index>=8 && (trace.kind!=1 || trace.row>=300)))
      throw std::runtime_error("native map query trace row/epoch identity");
    if(epoch==0) {
      if(index>8 && trace.row<=first_ids.back().second)throw std::runtime_error("native map query individual order/duplicate");
      first_ids.emplace_back(trace.kind,trace.row);
    } else if(first_ids[index]!=std::make_pair(trace.kind,trace.row))throw std::runtime_error("native map query epoch row mapping changed");
    trace.poses.resize(samples);double last=-1;
    for(auto &pose:trace.poses) {
      read_values(s,pose.data(),pose.size());finite_values(pose);
      if(pose[0]<=last || pose[0]<0 || pose[0]>input.map[3]+1e-12)throw std::runtime_error("native map query full monotonic grid");
      last=pose[0];
    }
    if(trace.poses.front()[0]!=0 || std::abs(trace.poses.back()[0]-input.map[3])>1e-12)throw std::runtime_error("native map query complete horizon required");
    count+=samples;input.traces.push_back(std::move(trace));
  }
  if(count!=21165 || s.peek()!=std::char_traits<char>::eof())throw std::runtime_error("native map query trailing/budget mismatch");
  return input;
}
std::string loaded_library() {
  std::ifstream stream("/proc/self/maps");std::string line,path;
  while(std::getline(stream,line))if(line.find("libnav2_costmap_2d_core.so")!=std::string::npos){path=line.substr(line.find('/'));break;}
  if(path!="/opt/ros/humble/lib/libnav2_costmap_2d_core.so")throw std::runtime_error("pinned native map core path mismatch");
  return path;
}
using Checker=nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>;
void synthetic(const char *path) {
  nav2_costmap_2d::Footprint fp(4);size_t i=0;
  for(const auto xy:std::array<std::array<double,2>,4>{{{-.25,-.25},{.25,-.25},{.25,.25},{-.25,.25}}}) {fp[i].x=xy[0];fp[i].y=xy[1];++i;}
  std::ofstream out(path);out.exceptions(std::ios::failbit|std::ios::badbit);out<<std::setprecision(17);
  out<<"{\"schema\":\"native_map_synthetic_v1\",\"native_library\":\""<<loaded_library()<<"\",\"scope\":\"Artificial uninﬂated maps; native checker only, no CostCritic or actual map classification\"}\n";
  for(const auto placement:{"edge","interior","center"})for(const unsigned int value:{203u,216u,253u,254u,255u}) {
    nav2_costmap_2d::Costmap2D map(40,40,.05,-1,-1,0);Checker checker(&map);
    const unsigned int x=std::strcmp(placement,"edge")==0?25:std::strcmp(placement,"interior")==0?22:20;
    const unsigned int y=std::strcmp(placement,"interior")==0?22:20;map.setCost(x,y,value);
    out<<"{\"placement\":\""<<placement<<"\",\"cell_value\":"<<value<<",\"center_raw\":"<<unsigned(map.getCost(20,20))
       <<",\"native_footprint_cost\":"<<checker.footprintCostAtPose(0,0,0,fp)<<"}\n";
  }
  nav2_costmap_2d::Costmap2D map(40,40,.05,-1,-1,0);Checker checker(&map);
  out<<"{\"placement\":\"outside_map\",\"native_footprint_cost\":"<<checker.footprintCostAtPose(.9,0,0,fp)<<"}\n";
}
int main(int argc,char **argv) {
  try {
    if(argc!=3)throw std::invalid_argument("INPUT NEW_JSONL or --synthetic NEW_JSONL");
    if(std::filesystem::exists(argv[2]))throw std::runtime_error("refusing to overwrite native map output");
    if(std::strcmp(argv[1],"--synthetic")==0){synthetic(argv[2]);return 0;}
    const auto input=read_input(argv[1]);nav2_costmap_2d::Costmap2D map(100,100,input.map[0],input.map[1],input.map[2],0);
    std::memcpy(map.getCharMap(),input.raw.data(),input.raw.size());Checker checker(&map);
    std::ofstream out(argv[2]);out.exceptions(std::ios::failbit|std::ios::badbit);out<<std::setprecision(17);
    out<<"{\"schema\":\"native_map_queries_v1\",\"source_ordinal\":298,\"trace_count\":102,\"query_count\":21165,\"native_library\":\""<<loaded_library()
       <<"\",\"actual_CostCritic_state\":\"unknown\",\"scope\":\"Installed checker queries on frozen map; not actual critic costs or collision events\"}\n";
    for(size_t t=0;t<input.traces.size();++t) {
      const auto &trace=input.traces[t];
      for(size_t sample=0;sample<trace.poses.size();++sample) {
        const auto &p=trace.poses[sample];unsigned int mx=0,my=0;const bool inside=map.worldToMap(p[1],p[2],mx,my);
        const double cost=checker.footprintCostAtPose(p[1],p[2],p[3],input.footprint);
        if(!std::isfinite(cost))throw std::runtime_error("nonfinite native map result");
        out<<"{\"trace\":"<<t<<",\"path_kind\":"<<trace.kind<<",\"path_row\":"<<trace.row<<",\"epoch_index\":"<<trace.epoch
           <<",\"sample\":"<<sample<<",\"future_seconds\":"<<p[0]<<",\"pose\":["<<p[1]<<','<<p[2]<<','<<p[3]<<"],\"center_in_map\":"<<(inside?"true":"false")
           <<",\"center_raw\":"<<(inside?int(map.getCost(mx,my)):-1)<<",\"native_footprint_cost\":"<<cost<<"}\n";
      }
    }
    std::cout<<"native_map_queries=21165 traces=102 actual_CostCritic_state=unknown\n";return 0;
  }catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}
