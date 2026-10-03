#pragma once
#include "nav2_mppi_controller/critic_data.hpp"
#include "rm_dynamic_obstacle_critic/evidence_json.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <vector>

namespace rm_dynamic_obstacle_critic {
// Plain snapshot of public map context. The plugin copies it under the map
// mutex; the serializer owns no ROS objects, subscriptions or TF state.
struct CostAccumulatorContext {
  std::string stage,evaluation_frame,base_frame;
  std::array<size_t,2> map_size{};
  std::array<double,2> origin{};
  double resolution{};
  std::vector<uint8_t> map_bytes;
  std::vector<std::array<double,2>> padded_footprint;
  bool map_snapshot_mutex_locked{false};
  // Observer-owned token identifies the committed before record. It is not
  // an optimizer cycle id and never changes CriticData.
  std::optional<size_t> paired_before_ordinal;
};

inline void write_cost_accumulator_observation(
    const std::filesystem::path &directory,size_t ordinal,double capture_stamp,
    const mppi::CriticData &data,const CostAccumulatorContext &context) {
  static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
  const uint32_t endian=1;
  if(*reinterpret_cast<const uint8_t *>(&endian)!=1)throw std::runtime_error("cost observation requires little endian");
  const auto &tr=data.trajectories;const auto &state=data.state;
  const size_t batch=tr.x.shape(0),steps=tr.x.shape(1);
  if(ordinal>=1000 || !batch || batch>512 || !steps || steps>64 ||
      tr.x.shape()!=tr.y.shape() || tr.x.shape()!=tr.yaws.shape() || data.costs.size()!=batch ||
      data.path.x.size()>2000 || data.path.x.shape()!=data.path.y.shape() || data.path.x.shape()!=data.path.yaws.shape() ||
      !std::isfinite(data.model_dt) || data.model_dt<=0 || !std::isfinite(capture_stamp) ||
      (context.stage!="before_CostCritic" && context.stage!="after_CostCritic") ||
      (context.paired_before_ordinal && *context.paired_before_ordinal>=1000) ||
      (context.stage=="before_CostCritic" && context.paired_before_ordinal!=std::optional<size_t>(ordinal)))
    throw std::runtime_error("cost observation stage/grid budget");
  for(const auto *tensor:{&state.cvx,&state.cvy,&state.cwz,&state.vx,&state.vy,&state.wz})
    if(tensor->shape()!=tr.x.shape())throw std::runtime_error("cost observation state shape");
  if(!context.map_size[0] || !context.map_size[1] || context.map_size[0]>65536 ||
      context.map_size[1]>65536 || context.map_size[0]*context.map_size[1]>65536 ||
      context.map_bytes.size()!=context.map_size[0]*context.map_size[1] ||
      !std::isfinite(context.resolution) || context.resolution<=0 ||
      !std::isfinite(context.origin[0]) || !std::isfinite(context.origin[1]) ||
      context.padded_footprint.size()<3 || context.padded_footprint.size()>64 ||
      state.pose.header.frame_id!=context.evaluation_frame || context.evaluation_frame.empty() ||
      context.base_frame.empty() || state.pose.header.stamp.nanosec>=1000000000u)
    throw std::runtime_error("cost observation map/frame context");
  const auto &pose=state.pose.pose;const auto &speed=state.speed;
  for(double v:{pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,
      pose.orientation.z,pose.orientation.w,speed.linear.x,speed.linear.y,speed.linear.z,
      speed.angular.x,speed.angular.y,speed.angular.z})
    if(!std::isfinite(v))throw std::runtime_error("cost observation nonfinite metadata");
  for(const auto &p:context.padded_footprint)for(double v:p)
    if(!std::isfinite(v))throw std::runtime_error("cost observation nonfinite footprint");
  std::ostringstream json;json.imbue(std::locale::classic());json<<std::setprecision(17);
  json<<"{\"schema\":\"native_cost_accumulator_observation_v1\",\"ordinal\":"<<ordinal
      <<",\"stage\":"<<json_string(context.stage)<<",\"capture_stamp\":"<<capture_stamp
      <<",\"paired_before_ordinal\":";
  if(context.paired_before_ordinal)json<<*context.paired_before_ordinal;else json<<"null";
  json
      <<",\"pose_stamp_sec\":"<<state.pose.header.stamp.sec<<",\"pose_stamp_nanosec\":"<<state.pose.header.stamp.nanosec
      <<",\"pose_frame\":"<<json_string(state.pose.header.frame_id)<<",\"evaluation_frame\":"<<json_string(context.evaluation_frame)
      <<",\"base_frame\":"<<json_string(context.base_frame)<<",\"model_dt\":"<<data.model_dt
      <<",\"fail_flag\":"<<(data.fail_flag?"true":"false")<<",\"pose\":["<<pose.position.x<<','<<pose.position.y<<','<<pose.position.z
      <<','<<pose.orientation.x<<','<<pose.orientation.y<<','<<pose.orientation.z<<','<<pose.orientation.w
      <<"],\"speed\":["<<speed.linear.x<<','<<speed.linear.y<<','<<speed.linear.z<<','<<speed.angular.x<<','<<speed.angular.y<<','<<speed.angular.z
      <<"],\"map\":{\"size\":["<<context.map_size[0]<<','<<context.map_size[1]<<"],\"origin\":["<<context.origin[0]<<','<<context.origin[1]
      <<"],\"resolution\":"<<context.resolution<<"},\"map_snapshot_mutex_locked\":"<<(context.map_snapshot_mutex_locked?"true":"false")
      <<",\"padded_footprint\":[";
  bool comma=false;for(const auto &p:context.padded_footprint){if(comma)json<<',';json<<'['<<p[0]<<','<<p[1]<<']';comma=true;}
  json<<"],\"blocks\":[";comma=false;std::vector<uint8_t> payload;payload.reserve(9*batch*steps*4+batch*4+context.map_bytes.size());
  auto block=[&](const std::string &name,const void *bytes,size_t size,const std::string &dtype,const auto &shape){
    if(comma)json<<',';
    comma=true;
    json<<"{\"name\":"<<json_string(name)<<",\"dtype\":"<<json_string(dtype)<<",\"offset\":"<<payload.size()<<",\"bytes\":"<<size<<",\"shape\":[";
    bool dim=false;for(size_t value:shape){if(dim)json<<',';json<<value;dim=true;}json<<"]}";
    const auto *begin=static_cast<const uint8_t *>(bytes);if(size)payload.insert(payload.end(),begin,begin+size);
  };
  auto floats=[&](const std::string &name,const auto &tensor){
    if(!tensor.is_contiguous() || tensor.layout()!=xt::layout_type::row_major)throw std::runtime_error("cost observation tensor layout");
    for(size_t i=0;i<tensor.size();++i)if(!std::isfinite(tensor.data()[i]))throw std::runtime_error("cost observation nonfinite tensor");
    block(name,tensor.data(),tensor.size()*4,"<f4",tensor.shape());
  };
  floats("x",tr.x);floats("y",tr.y);floats("yaw",tr.yaws);
  floats("cvx",state.cvx);floats("cvy",state.cvy);floats("cwz",state.cwz);
  floats("vx",state.vx);floats("vy",state.vy);floats("wz",state.wz);
  floats("cumulative_critic_costs",data.costs);
  floats("path_x",data.path.x);floats("path_y",data.path.y);floats("path_yaw",data.path.yaws);
  block("raw_costmap",context.map_bytes.data(),context.map_bytes.size(),"|u1",std::array<size_t,2>{context.map_size[1],context.map_size[0]});
  json<<"],\"payload_bytes\":"<<payload.size()<<",\"unavailable\":[\"unrounded_individual_increment\",\"inflation_cache\",\"actual_CostCritic_branch_and_query_selection\",\"other_public_CriticData_cache_fields\",\"physical_cell_source\"]}\n";
  const auto prefix=directory/("cost_"+std::to_string(ordinal));const auto binary=prefix.string()+".bin",metadata=prefix.string()+".json";
  for(const auto &p:{binary,metadata,binary+".tmp",metadata+".tmp"})if(std::filesystem::exists(p))throw std::runtime_error("refusing to overwrite cost observation");
  std::ofstream bytes(binary+".tmp",std::ios::binary);bytes.exceptions(std::ios::failbit|std::ios::badbit);
  bytes.write(reinterpret_cast<const char *>(payload.data()),payload.size());bytes.close();
  std::ofstream meta(metadata+".tmp");meta.exceptions(std::ios::failbit|std::ios::badbit);meta<<json.str();meta.close();
  std::filesystem::rename(binary+".tmp",binary);std::filesystem::rename(metadata+".tmp",metadata);
}
} // namespace rm_dynamic_obstacle_critic
