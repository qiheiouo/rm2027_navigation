// Offline only. Recompute the existing pure CV/geometry score with the runtime
// compiler policy. No optimizer, subscriptions, TF, goals or control publisher.
#include "rm_dynamic_obstacle_critic/model.hpp"
#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

template<typename T> void read_values(std::istream &input,T *data,size_t count) {
  input.read(reinterpret_cast<char*>(data),count*sizeof(T));
  if(!input)throw std::runtime_error("truncated dynamic replay");
}
void number(std::ostream &out,double value) {
  if(std::isfinite(value))out<<value;else out<<"null";
}
int main(int argc,char **argv) {
  try {
    if(argc!=3)throw std::invalid_argument("INPUT NEW_OUTPUT_JSONL");
    if(std::filesystem::exists(argv[2]))throw std::invalid_argument("refusing to overwrite dynamic replay");
    static_assert(sizeof(float)==4 && sizeof(double)==8);
    const uint32_t endian=1;if(*reinterpret_cast<const uint8_t*>(&endian)!=1)throw std::runtime_error("little endian required");
    std::ifstream input(argv[1],std::ios::binary);char magic[8];read_values(input,magic,8);
    if(std::memcmp(magic,"RMDYNRP1",8))throw std::runtime_error("dynamic replay schema");
    uint32_t records;read_values(input,&records,1);
    if(records<1 || records>1000)throw std::runtime_error("dynamic replay record budget");
    std::ofstream output(argv[2]);output.exceptions(std::ios::failbit|std::ios::badbit);
    output<<std::setprecision(17);bool all_exact=true;int64_t previous=-1;
    namespace dyn=rm_dynamic_obstacle_critic;
    for(uint32_t record=0;record<records;++record) {
      std::array<uint32_t,5> grid;read_values(input,grid.data(),grid.size());
      const auto ordinal=grid[0],batch=grid[1],steps=grid[2],tracks=grid[3],points=grid[4];
      if(ordinal>=1000 || ordinal<=previous || batch<1 || batch>512 || steps<1 || steps>64 || tracks>64 || points<3 || points>128)
        throw std::runtime_error("dynamic replay bounded grid");
      previous=ordinal;std::array<double,10> params;read_values(input,params.data(),params.size());
      for(double value:params)if(!std::isfinite(value))throw std::runtime_error("nonfinite dynamic replay parameter");
      dyn::Rigid2D transform{params[2],params[3],params[4]};dyn::Limits limits;limits.minimum_radius=params[5];
      dyn::CostParameters cost{params[6],params[7],params[8],params[9]};
      if(params[0]<=0 || params[1]<0 || params[5]<=0 || params[6]<0 || params[7]<=params[8] || params[8]<=0 || params[9]<=0)
        throw std::runtime_error("dynamic replay parameter range");
      std::vector<dyn::Point> footprint(points);
      for(auto &point:footprint){read_values(input,&point.x,1);read_values(input,&point.y,1);}
      dyn::validate_footprint(footprint);dyn::Array msg;msg.tracks.resize(tracks);
      for(auto &track:msg.tracks) {
        read_values(input,&track.track_id,1);read_values(input,&track.state,1);
        for(double *value:{&track.position.x,&track.position.y,&track.velocity.x,&track.velocity.y,&track.size.x,&track.size.y}) {
          read_values(input,value,1);if(!std::isfinite(*value))throw std::runtime_error("nonfinite dynamic replay track");
        }
        if(track.state<1 || track.state>3 || track.size.x<=0 || track.size.y<=0)throw std::runtime_error("dynamic replay track state/size");
      }
      std::array<std::vector<float>,3> rollout;
      for(auto &axis:rollout){axis.resize(batch*steps);read_values(input,axis.data(),axis.size());}
      std::vector<float> before(batch),after(batch);std::vector<double> actual_risk(batch);
      read_values(input,before.data(),batch);read_values(input,actual_risk.data(),batch);read_values(input,after.data(),batch);
      size_t exact_risk=0,exact_after=0;std::vector<dyn::Risk> risks;
      for(size_t row=0;row<batch;++row) {
        std::vector<dyn::Pose> poses;poses.reserve(steps);
        for(size_t k=0;k<steps;++k) {
          for(const auto &axis:rollout)if(!std::isfinite(axis[row*steps+k]))throw std::runtime_error("nonfinite dynamic replay rollout");
          poses.push_back({rollout[0][row*steps+k],rollout[1][row*steps+k],rollout[2][row*steps+k]});
        }
        auto risk=dyn::score(poses,footprint,msg,limits,cost,params[1],params[0],transform);
        const float added=static_cast<float>(double(before[row])+risk.cost);
        if(!std::isfinite(before[row]) || !std::isfinite(after[row]) || !std::isfinite(actual_risk[row]) || !std::isfinite(risk.cost))
          throw std::runtime_error("nonfinite dynamic replay cost");
        exact_risk+=std::memcmp(&risk.cost,&actual_risk[row],8)==0;
        exact_after+=std::memcmp(&added,&after[row],4)==0;risks.push_back(risk);
      }
      all_exact &= exact_risk==batch && exact_after==batch;
      output<<"{\"ordinal\":"<<ordinal<<",\"batch\":"<<batch<<",\"steps\":"<<steps
            <<",\"risk_double_bit_exact_rows\":"<<exact_risk<<",\"after_float_bit_exact_rows\":"<<exact_after<<",\"risk\":[";
      for(size_t row=0;row<batch;++row){if(row)output<<',';output<<risks[row].cost;}
      output<<"],\"minimum_clearance\":[";
      for(size_t row=0;row<batch;++row){if(row)output<<',';number(output,risks[row].minimum_clearance);}
      output<<"],\"collision_time\":[";
      for(size_t row=0;row<batch;++row){if(row)output<<',';number(output,risks[row].collision_time);}
      output<<"]}\n";
    }
    if(input.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing dynamic replay input");
    std::cout<<"records="<<records<<" all_risk_and_after_bits_exact="<<all_exact<<'\n';return all_exact?0:2;
  } catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}
