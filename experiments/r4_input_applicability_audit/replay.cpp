// Recorded-value replay, not a ROS runtime host. No node or publisher.
#include <rm_r4_prediction_consumption/follow.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#ifndef R4_REPLAY_LIBRARY_ONLY
#include "shadow_seed.hpp"
#endif
#include <filesystem>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace v=rm_r4_prediction_consumption;
using Row=std::map<std::string,std::string>;
int64_t ns(const builtin_interfaces::msg::Time & s) {return int64_t(s.sec)*1000000000+s.nanosec;}
std::vector<std::string> parse(const std::string & line)
{
  std::vector<std::string> values;std::string value;bool quoted=false;
  for(size_t i=0;i<line.size();++i) {
    const char c=line[i];
    if(c=='"') {if(quoted&&i+1<line.size()&&line[i+1]=='"') {value+='"';++i;}else quoted=!quoted;}
    else if(c==','&&!quoted) {values.push_back(value);value.clear();}else value+=c;
  }
  if(quoted) throw std::runtime_error("unterminated CSV field");
  values.push_back(value);return values;
}
std::vector<Row> rows(const std::filesystem::path & path)
{
  std::ifstream f(path);if(!f) throw std::runtime_error("CSV input missing");
  std::string line;std::getline(f,line);const auto keys=parse(line);std::vector<Row> result;
  while(std::getline(f,line)) {const auto values=parse(line);if(values.size()!=keys.size()) throw std::runtime_error("CSV width");
    Row row;for(size_t i=0;i<keys.size();++i) row[keys[i]]=values[i];result.push_back(std::move(row));}
  return result;
}
template<class T> T decode(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & m)
{
  rclcpp::SerializedMessage bytes(*m->serialized_data);T value;
  rclcpp::Serialization<T> codec;codec.deserialize_message(&bytes,&value);return value;
}
struct Recorded
{
  nav_msgs::msg::OccupancyGrid map;
  std::map<int64_t,nav_msgs::msg::Path> paths;
  std::map<uint64_t,rm_r4_interfaces::msg::ObservedPredictionEnvelope> envelopes;
};
Recorded load(const std::filesystem::path & bag)
{
  Recorded r;rosbag2_cpp::Reader reader;reader.open(bag.string());
  while(reader.has_next()) {const auto m=reader.read_next();
    if(m->topic_name=="/map") r.map=decode<nav_msgs::msg::OccupancyGrid>(m);
    if(m->topic_name=="/plan") {auto p=decode<nav_msgs::msg::Path>(m);if(!r.paths.emplace(ns(p.header.stamp),std::move(p)).second) throw std::runtime_error("ambiguous path stamp");}
    if(m->topic_name=="/perception/dynamic_obstacles_shadow/observed_predictions") {
      auto e=decode<rm_r4_interfaces::msg::ObservedPredictionEnvelope>(m);if(!r.envelopes.emplace(e.sequence,std::move(e)).second) throw std::runtime_error("ambiguous receipt");}
  }
  if(r.map.data.empty()||r.paths.empty()||r.envelopes.empty()) throw std::runtime_error("incomplete recorded input");
  return r;
}
double d(const Row & r,const std::string & key) {return std::stod(r.at(key));}
int64_t n(const Row & r,const std::string & key) {return std::stoll(r.at(key));}
std::string quote(const std::string & s) {std::string o="\"";for(char c:s) {if(c=='"') o+='"';o+=c;}return o+'"';}
#ifndef R4_REPLAY_LIBRARY_ONLY
struct Policy
{
  v::ReceiptGate gate;v::FollowSolver solver;std::optional<v::Vec2> seed;int64_t seed_epoch{},previous_epoch{};
  std::optional<v::PreparedCorridor> corridor;uint64_t path_revision{};bool legacy;
};
void replay(const Recorded & recorded,const std::vector<Row> & input,const std::string & scene,bool legacy,std::ofstream & out)
{
  Policy p;p.legacy=legacy;uint64_t generation=1;
  v::FollowLimits limits{{-.5,-.5},{.8,.5},{1.,1.},.4,.5}; // Unchanged actual sim profile.
  for(const auto & r:input) {
    const int64_t epoch=n(r,"acquire_ros_ns");
    if(epoch<=p.previous_epoch) {p.gate.reset();p.solver.reset();p.seed.reset();++generation;}p.previous_epoch=epoch;
    // Startup inputs have no owned coherent tuple; preserve as NA, never fabricate.
    if(r.at("pose_source_ns").empty()||r.at("receipt_sequence").empty()||r.at("path_stamp_ns").empty()||r.at("progress_input").empty()) {p.seed.reset();p.solver.reset();continue;}
    const auto acquired=v::FollowClock::now();std::string reason,status="not_run";bool valid=false,reset_seed=false,reset_warm=false,used_warm=false;
    v::Vec2 seed{},command{};int64_t seed_stamp=epoch;double cost=0.;int iterations=0;
    try {
      v::BodyPolicy body{{{-.3,-.25},{-.3,.25},{.3,.25},{.3,-.25}},.03,d(r,"yaw"),.05};
      const auto path_revision=uint64_t(n(r,"path_revision"));
      const bool rebuild=!p.corridor||p.path_revision!=path_revision||p.corridor->body_digest()!=body.digest();
      if(rebuild) {p.corridor=v::PreparedCorridor::prepare(recorded.paths.at(n(r,"path_stamp_ns")),recorded.map,body,path_revision);p.path_revision=path_revision;}
      auto c=p.gate.consume(recorded.envelopes.at(uint64_t(n(r,"receipt_sequence"))),epoch,"map",body);
      if(p.corridor->path_digest()!=r.at("path_digest")||p.corridor->map_digest()!=r.at("map_digest")||body.digest()!=r.at("body_digest")||c.snapshot.receipt_digest()!=r.at("receipt_digest")) throw std::runtime_error("recorded digest mismatch");
      const auto decision=r4_runtime_shadow::decide_seed(p.seed,p.seed_epoch,epoch,rebuild||c.reset_warm);
      reset_warm=decision.reset_warm;reset_seed=legacy?decision.reset_warm:decision.reset_seed;
      seed=reset_seed?v::Vec2{}:decision.velocity;seed_stamp=reset_seed?epoch:decision.stamp_ns;
      v::FollowInput in{c.snapshot,*p.corridor,body,limits,{"offline_replay/"+scene,scene,"base_link",generation,uint64_t(n(r,"cycle"))},
        {{d(r,"x"),d(r,"y")},{d(r,"measured_vx"),d(r,"measured_vy")},seed,d(r,"yaw"),d(r,"measured_wz"),0.,
         n(r,"pose_source_ns"),n(r,"velocity_source_ns"),n(r,"tf_source_ns"),seed_stamp,"map","base_link"},d(r,"progress_input"),acquired,reset_warm};
      auto result=p.solver.solve(std::move(in));reason=result.reason;status=result.solver_status;iterations=result.iterations;used_warm=result.used_warm;
      if(result.proposal) {valid=true;command=result.proposal->body_velocity;cost=result.solved_dynamic_cost;p.seed=command;p.seed_epoch=epoch;
        if(std::abs(command.x-seed.x)>.05002||std::abs(command.y-seed.y)>.05002) throw std::runtime_error("rate constraint violation");
      }else p.seed.reset();
    }catch(const std::exception & e) {valid=false;reason=e.what();p.solver.reset();p.seed.reset();}
    out<<scene<<','<<(legacy?"legacy":"separate_seed_warm")<<','<<r.at("cycle")<<','<<epoch<<','<<valid<<','<<quote(reason)<<','<<quote(status)<<','<<iterations<<','
       <<seed.x<<','<<seed.y<<','<<seed_stamp<<','<<reset_seed<<','<<reset_warm<<','<<used_warm<<','<<command.x<<','<<command.y<<','<<cost<<'\n';
  }
}
int main(int argc,char ** argv)
{
  if(argc!=4) return 2;
  try {
    std::ofstream out(std::filesystem::path(argv[3])/"seed_replay.csv");if(!out) throw std::runtime_error("explicit output required");
    out<<std::setprecision(17)<<"scene,policy,original_cycle,source_epoch_ns,valid,reason,status,iterations,seed_vx,seed_vy,seed_stamp_ns,reset_seed,reset_warm,used_warm,vx,vy,dynamic_cost\n";
    for(const auto & scene:{std::string("S0"),std::string("S1"),std::string("S2")}) {
      const auto data=load(std::filesystem::path(argv[1])/scene/"rosbag");const auto cycles=rows(std::filesystem::path(argv[2])/(scene+"_cycles.csv"));
      replay(data,cycles,scene,true,out);replay(data,cycles,scene,false,out);
    }
  }catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
  return 0;
}
#endif
