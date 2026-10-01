// Offline only. Uses own objects and the unchanged installed native model;
// each proposal receives the same numerically verified history independently.
#include "nav2_mppi_controller/optimizer.hpp"
#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

template<typename T> void read_values(std::istream &input,T *values,size_t count) {
  input.read(reinterpret_cast<char *>(values),count*sizeof(T));
  if(!input)throw std::runtime_error("truncated native witness input");
}
template<typename T> void write_values(std::ostream &output,const T *values,size_t count) {
  output.write(reinterpret_cast<const char *>(values),count*sizeof(T));
}

class NativeWitness : public mppi::Optimizer {
public:
  NativeWitness(size_t batch,size_t steps,const std::array<float,5> &parameters) {
    settings_.batch_size=batch;settings_.time_steps=steps;settings_.model_dt=parameters[0];
    settings_.constraints={parameters[2],parameters[1],parameters[3],parameters[4]};
    settings_.base_constraints=settings_.constraints;settings_.shift_control_sequence=true;
    motion_model_=std::make_shared<mppi::OmniMotionModel>();
    state_.reset(batch,steps);generated_trajectories_.reset(batch,steps);
    control_sequence_.reset(steps);
  }
  void cycle(std::istream &input,std::ostream &binary,std::ostream &json,uint32_t ordinal,bool sampler) {
    std::array<double,13> measured;read_values(input,measured.data(),measured.size());
    auto &pose=state_.pose.pose;auto &speed=state_.speed;
    pose.position.x=measured[0];pose.position.y=measured[1];pose.position.z=measured[2];
    pose.orientation.x=measured[3];pose.orientation.y=measured[4];
    pose.orientation.z=measured[5];pose.orientation.w=measured[6];
    speed.linear.x=measured[7];speed.linear.y=measured[8];speed.linear.z=measured[9];
    speed.angular.x=measured[10];speed.angular.y=measured[11];speed.angular.z=measured[12];
    for(auto &control:control_history_) {
      read_values(input,&control.vx,1);read_values(input,&control.vy,1);read_values(input,&control.wz,1);
    }
    const auto actual_history=control_history_;const size_t steps=settings_.time_steps;
    std::vector<float> aggregate(3*steps),aggregate_filtered(3*steps);
    read_values(input,aggregate.data(),aggregate.size());
    read_values(input,aggregate_filtered.data(),aggregate_filtered.size());
    std::array<double,3> command;read_values(input,command.data(),command.size());
    for(auto *tensor:{&state_.cvx,&state_.cvy,&state_.cwz})read_values(input,tensor->data(),tensor->size());
    std::array<std::vector<float>,6> reference;
    for(auto &tensor:reference) {tensor.resize(settings_.batch_size*steps);read_values(input,tensor.data(),tensor.size());}
    updateStateVelocities(state_); // Installed native measured prefix and propagation.
    integrateStateVelocities(generated_trajectories_,state_); // Installed native integration.
    uint32_t mask=0;size_t index=0;
    for(const auto *tensor:{&state_.vx,&state_.vy,&state_.wz,
        &generated_trajectories_.x,&generated_trajectories_.y,&generated_trajectories_.yaws}) {
      if(std::memcmp(tensor->data(),reference[index].data(),tensor->size()*sizeof(float))==0)mask|=1u<<index;
      ++index;
    }
    const size_t rows=11+(sampler?settings_.batch_size:0);
    mppi::models::State witnesses;witnesses.reset(rows,steps);
    witnesses.pose=state_.pose;witnesses.speed=state_.speed;
    const double yaw=tf2::getYaw(state_.pose.pose.orientation);
    constexpr std::array<std::array<double,2>,8> directions{{
      {{.2,0}},{{.2,.3}},{{.2,-.3}},{{.4,0}},{{.4,.3}},{{.4,-.3}},{{0,.3}},{{0,-.3}}}};
    uint32_t aggregate_exact=0,command_exact=0;
    for(size_t row=0;row<rows;++row) {
      control_sequence_.reset(steps);control_history_=actual_history;
      std::array<float,3> constant{};
      if(row==2)constant={float(speed.linear.x),float(speed.linear.y),float(speed.angular.z)};
      if(row>=3 && row<11) {
        const auto &world=directions[row-3];
        constant={float(world[0]*std::cos(yaw)+world[1]*std::sin(yaw)),
                  float(-world[0]*std::sin(yaw)+world[1]*std::cos(yaw)),0};
      }
      size_t axis=0;
      for(auto pair:{std::make_pair(&control_sequence_.vx,&state_.cvx),
          std::make_pair(&control_sequence_.vy,&state_.cvy),std::make_pair(&control_sequence_.wz,&state_.cwz)}) {
        for(size_t step=0;step<steps;++step)
          (*pair.first)(step)=row==0?aggregate[axis*steps+step]:
            row>=11?(*pair.second)(row-11,step):constant[axis];
        ++axis;
      }
      applyControlSequenceConstraints(); // Same order as native aggregation.
      mppi::utils::savitskyGolayFilter(control_sequence_,control_history_,settings_);
      if(row==0) {
        bool exact=true;axis=0;
        for(const auto *tensor:{&control_sequence_.vx,&control_sequence_.vy,&control_sequence_.wz}) {
          exact &= std::memcmp(tensor->data(),aggregate_filtered.data()+axis*steps,steps*sizeof(float))==0;
          ++axis;
        }
        aggregate_exact=exact;
        const std::array<double,3> actual{{double(control_sequence_.vx(1)),double(control_sequence_.vy(1)),double(control_sequence_.wz(1))}};
        command_exact=std::memcmp(actual.data(),command.data(),sizeof(command))==0;
      }
      for(size_t step=0;step<steps;++step) {
        witnesses.cvx(row,step)=control_sequence_.vx(step);
        witnesses.cvy(row,step)=control_sequence_.vy(step);
        witnesses.cwz(row,step)=control_sequence_.wz(step);
      }
    }
    updateStateVelocities(witnesses);
    mppi::models::Trajectories trajectories;trajectories.reset(rows,steps);
    integrateStateVelocities(trajectories,witnesses);
    write_values(binary,&ordinal,1);write_values(binary,&mask,1);
    write_values(binary,&aggregate_exact,1);write_values(binary,&command_exact,1);write_values(binary,&yaw,1);
    for(const auto *tensor:{&witnesses.cvx,&witnesses.cvy,&witnesses.cwz,
        &witnesses.vx,&witnesses.vy,&witnesses.wz,&trajectories.x,&trajectories.y,&trajectories.yaws})
      write_values(binary,tensor->data(),tensor->size());
    json << "{\"ordinal\":" << ordinal << ",\"raw_velocity_pose_exact_mask\":" << mask
         << ",\"aggregate_SG_exact\":" << (aggregate_exact?"true":"false")
         << ",\"actual_command_double_bit_exact\":" << (command_exact?"true":"false")
         << ",\"initial_yaw\":" << yaw << "}\n";
  }
};

int main(int argc,char **argv) {
  if(argc!=4 && !(argc==5 && std::string(argv[4])=="--filtered-sampler"))
    throw std::invalid_argument("usage: native_safe_control_witness INPUT OUTPUT.bin OUTPUT.jsonl [--filtered-sampler]");
  if(std::filesystem::exists(argv[2]) || std::filesystem::exists(argv[3]))
    throw std::runtime_error("refusing to overwrite witness evidence");
  std::ifstream input(argv[1],std::ios::binary);std::ofstream binary(argv[2],std::ios::binary),json(argv[3]);
  if(!input || !binary || !json)throw std::runtime_error("witness files unavailable");
  binary.exceptions(std::ios::failbit|std::ios::badbit);json.exceptions(std::ios::failbit|std::ios::badbit);
  json<<std::setprecision(17);
  std::array<char,8> magic;read_values(input,magic.data(),magic.size());
  if(std::string(magic.data(),8)!="RMSFWIT1")throw std::runtime_error("witness schema");
  uint32_t count,batch,steps;read_values(input,&count,1);read_values(input,&batch,1);read_values(input,&steps,1);
  if(!count || count>1000 || !batch || batch>500 || steps!=30)throw std::runtime_error("witness budget/grid");
  std::array<float,5> parameters;read_values(input,parameters.data(),parameters.size());
  NativeWitness witness(batch,steps,parameters);const bool sampler=argc==5;
  const uint32_t rows=11+(sampler?batch:0);
  binary.write("RMSFWTO1",8);write_values(binary,&count,1);write_values(binary,&rows,1);
  write_values(binary,&steps,1);write_values(binary,&parameters[0],1);
  std::ifstream mappings("/proc/self/maps");std::string line,library;
  while(std::getline(mappings,line))if(line.find("libmppi_controller.so")!=std::string::npos){library=line.substr(line.find('/'));break;}
  json<<"{\"kind\":\"native_witness\",\"native_library\":\""<<library<<"\",\"rows\":"<<rows<<",\"raw_mask_complete\":63,"
      <<"\"scope\":\"offline actual SDK constraints, independent actual history per proposal, measured prefix and full native integration; no safety verdict\"}\n";
  for(uint32_t cycle=0;cycle<count;++cycle) {
    uint32_t ordinal;read_values(input,&ordinal,1);
    if(ordinal!=cycle)throw std::runtime_error("witness ordinal gap");
    witness.cycle(input,binary,json,ordinal,sampler);
  }
  if(input.peek()!=std::char_traits<char>::eof())throw std::runtime_error("witness trailing input");
}
