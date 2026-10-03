#pragma once
#include "nav2_mppi_controller/critic_data.hpp"
#include "rm_dynamic_obstacle_critic/evidence_json.hpp"
#include "rm_dynamic_obstacle_critic/model.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstdint>
#include <limits>
#include <vector>

namespace rm_dynamic_obstacle_critic {
// Private file evidence, not a public tracker interface or a control input.
// The caller passes the very message/transform/footprint used in this score.
class DynamicConsumptionEvidence {
public:
  void configure(const std::string &directory,int maximum) {
    if(maximum<1 || maximum>1000)throw std::invalid_argument("dynamic evidence record budget");
    maximum_=size_t(maximum);
    if(directory.empty())return;
    directory_=directory;
    if(!directory_.is_absolute() || (std::filesystem::exists(directory_) &&
       (!std::filesystem::is_directory(directory_) || !std::filesystem::is_empty(directory_))))
      throw std::invalid_argument("dynamic evidence requires an empty/new absolute directory");
    std::filesystem::create_directories(directory_);
    if(!std::filesystem::create_directory(directory_/"writer.lock"))
      throw std::invalid_argument("dynamic evidence directory already owned");
  }
  bool active() const {return !directory_.empty() && !stopped_ && count_<maximum_;}
  static bool bounded(size_t batch,size_t steps,size_t tracks,size_t footprint_points) {
    return batch>0 && batch<=512 && steps>0 && steps<=64 && tracks<=64 && footprint_points<=128;
  }
  void stop() {stopped_=true;}
  size_t count() const {return count_;}
  void record(const mppi::CriticData &data,const Array &message,
              const std::vector<Point> &footprint,const Rigid2D &transform,
              const Limits &limits,const CostParameters &parameters,double now,double age,
              const std::string &evaluation_frame,const std::string &base_frame,
              const std::vector<float> &before,const std::vector<double> &risk) {
    if(!active())return;
    static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
    static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559);
    const uint32_t endian=1;
    if(*reinterpret_cast<const uint8_t*>(&endian)!=1)throw std::runtime_error("dynamic evidence requires little endian");
    const auto &tr=data.trajectories;const size_t batch=tr.x.shape(0),steps=tr.x.shape(1);
    if(!bounded(batch,steps,message.tracks.size(),footprint.size()) || tr.x.shape()!=tr.y.shape() ||
       tr.x.shape()!=tr.yaws.shape() || data.costs.size()!=batch || before.size()!=batch ||
       risk.size()!=batch || message.tracks.size()>64 || footprint.size()>128 ||
       !std::isfinite(data.model_dt) || data.model_dt<=0)
      throw std::runtime_error("dynamic evidence bounded grid/track/footprint mismatch");
    const auto &pose=data.state.pose.pose;const auto &speed=data.state.speed;
    for(double value:{now,age,pose.position.x,pose.position.y,pose.position.z,
        pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w,
        speed.linear.x,speed.linear.y,speed.linear.z,speed.angular.x,speed.angular.y,speed.angular.z,
        transform.x,transform.y,transform.yaw,limits.minimum_radius,limits.max_age,limits.max_observation_age,
        limits.max_speed,limits.max_extent,limits.jump_tolerance,limits.max_tf_age,
        parameters.weight,parameters.influence_distance,parameters.safety_margin,parameters.collision_cost,
        double(message.prediction_dt)})
      if(!std::isfinite(value))throw std::runtime_error("nonfinite dynamic evidence metadata");
    std::ostringstream json;json.imbue(std::locale::classic());json<<std::setprecision(17);
    json<<"{\"schema\":1,\"kind\":\"dynamic_consumption_fields_v1\",\"ordinal\":"<<count_
        <<",\"score_stamp\":"<<now<<",\"source_age_used\":"<<age
        <<",\"pose_stamp_sec\":"<<data.state.pose.header.stamp.sec
        <<",\"pose_stamp_nanosec\":"<<data.state.pose.header.stamp.nanosec
        <<",\"pose_frame\":"<<json_string(data.state.pose.header.frame_id)
        <<",\"evaluation_frame\":"<<json_string(evaluation_frame)<<",\"base_frame\":"<<json_string(base_frame)
        <<",\"model_dt\":"<<data.model_dt<<",\"pose\":["<<pose.position.x<<','<<pose.position.y<<','<<pose.position.z
        <<','<<pose.orientation.x<<','<<pose.orientation.y<<','<<pose.orientation.z<<','<<pose.orientation.w
        <<"],\"speed\":["<<speed.linear.x<<','<<speed.linear.y<<','<<speed.linear.z<<','<<speed.angular.x<<','<<speed.angular.y<<','<<speed.angular.z
        <<"],\"world_transform\":["<<transform.x<<','<<transform.y<<','<<transform.yaw<<"],\"padded_footprint\":[";
    bool comma=false;
    for(const auto &point:footprint) {
      if(!std::isfinite(point.x) || !std::isfinite(point.y))throw std::runtime_error("nonfinite evidence footprint");
      if(comma)json<<',';
      json<<'['<<point.x<<','<<point.y<<']';comma=true;
    }
    json<<"],\"limits\":{\"minimum_radius\":"<<limits.minimum_radius<<",\"max_age\":"<<limits.max_age
        <<",\"max_observation_age\":"<<limits.max_observation_age<<",\"max_speed\":"<<limits.max_speed
        <<",\"max_extent\":"<<limits.max_extent<<",\"max_tracks\":"<<limits.max_tracks
        <<",\"jump_tolerance\":"<<limits.jump_tolerance<<",\"max_tf_age\":"<<limits.max_tf_age
        <<",\"input_frame\":"<<json_string(limits.input_frame)<<"},\"cost_parameters\":{\"weight\":"<<parameters.weight
        <<",\"influence_distance\":"<<parameters.influence_distance<<",\"safety_margin\":"<<parameters.safety_margin
        <<",\"collision_cost\":"<<parameters.collision_cost<<"},\"input_used\":{\"stamp_sec\":"<<message.header.stamp.sec
        <<",\"stamp_nanosec\":"<<message.header.stamp.nanosec<<",\"frame\":"<<json_string(message.header.frame_id)
        <<",\"schema\":"<<json_string(message.schema)<<",\"authority\":"<<json_string(message.authority)
        <<",\"complete\":"<<(message.complete?"true":"false")<<",\"total_track_count\":"<<message.total_track_count
        <<",\"prediction_dt\":"<<message.prediction_dt<<",\"prediction_steps\":"<<message.prediction_steps<<",\"tracks\":[";
    comma=false;
    for(const auto &track:message.tracks) {
      for(double value:{track.position.x,track.position.y,track.velocity.x,track.velocity.y,track.size.x,track.size.y})
        if(!std::isfinite(value))throw std::runtime_error("nonfinite consumed track fields");
      if(comma)json<<',';
      json<<"{\"id\":"<<track.track_id<<",\"state\":"<<unsigned(track.state)
          <<",\"xy\":["<<track.position.x<<','<<track.position.y<<"],\"vxy\":["<<track.velocity.x<<','<<track.velocity.y
          <<"],\"size_xy\":["<<track.size.x<<','<<track.size.y<<"],\"observed_sec\":"<<track.last_observation_stamp.sec
          <<",\"observed_nanosec\":"<<track.last_observation_stamp.nanosec<<'}';comma=true;
    }
    json<<"]},\"blocks\":[";comma=false;
    std::vector<uint8_t> payload;payload.reserve(3*batch*steps*4+batch*16);
    auto block=[&](const std::string &name,const void *bytes,size_t length,const std::string &dtype,const auto &shape) {
      if(comma)json<<',';
      json<<"{\"name\":"<<json_string(name)<<",\"dtype\":"<<json_string(dtype)<<",\"offset\":"<<payload.size()
          <<",\"bytes\":"<<length<<",\"shape\":[";bool separator=false;
      for(size_t dim:shape){if(separator)json<<',';json<<dim;separator=true;}
      json<<"]}";comma=true;const auto *start=static_cast<const uint8_t*>(bytes);
      payload.insert(payload.end(),start,start+length);
    };
    auto floats=[&](const std::string &name,const auto &tensor) {
      if(!tensor.is_contiguous() || tensor.layout()!=xt::layout_type::row_major)
        throw std::runtime_error("dynamic evidence requires contiguous row-major tensors");
      for(float value:tensor)if(!std::isfinite(value))throw std::runtime_error("nonfinite dynamic evidence tensor");
      block(name,tensor.data(),tensor.size()*sizeof(float),"<f4",tensor.shape());
    };
    for(float value:before)if(!std::isfinite(value))throw std::runtime_error("nonfinite pre-dynamic cost");
    for(double value:risk)if(!std::isfinite(value))throw std::runtime_error("nonfinite dynamic risk cost");
    floats("x",tr.x);floats("y",tr.y);floats("yaw",tr.yaws);
    block("costs_before_dynamic",before.data(),before.size()*4,"<f4",std::vector<size_t>{batch});
    block("dynamic_risk_double",risk.data(),risk.size()*8,"<f8",std::vector<size_t>{batch});
    floats("costs_after_dynamic",data.costs);
    json<<"],\"payload_bytes\":"<<payload.size()
        <<",\"scope\":\"used public fields and actual dynamic score only; no new control or tracker API\","
        <<"\"unavailable\":[\"unconsumed message fields\",\"TF correction source stamp\",\"native map source stamp\",\"odometry subscriber source stamp\"]}\n";
    if(payload.size()>2'000'000 || json.str().size()>200'000)throw std::runtime_error("dynamic evidence payload budget");
    const auto prefix=directory_/("score_"+std::to_string(count_));
    const auto binary=prefix.string()+".bin",metadata=prefix.string()+".json";
    if(std::filesystem::exists(binary) || std::filesystem::exists(metadata) ||
       std::filesystem::exists(binary+".tmp") || std::filesystem::exists(metadata+".tmp"))
      throw std::runtime_error("refusing to overwrite dynamic evidence");
    std::ofstream output(binary+".tmp",std::ios::binary);output.exceptions(std::ios::failbit|std::ios::badbit);
    output.write(reinterpret_cast<const char*>(payload.data()),payload.size());output.close();
    std::ofstream meta(metadata+".tmp");meta.exceptions(std::ios::failbit|std::ios::badbit);meta<<json.str();meta.close();
    std::filesystem::rename(binary+".tmp",binary);std::filesystem::rename(metadata+".tmp",metadata);
    ++count_;
  }
private:
  std::filesystem::path directory_;
  size_t count_{0},maximum_{400};
  bool stopped_{false};
};
} // namespace rm_dynamic_obstacle_critic
