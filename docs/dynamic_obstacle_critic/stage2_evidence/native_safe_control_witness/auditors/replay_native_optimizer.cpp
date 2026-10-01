// Offline only: own derived objects call the unchanged installed optimizer and
// NoiseGenerator functions. No live controller, fork or control publisher.
#include "nav2_mppi_controller/optimizer.hpp"
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>

class FixedNativeNoises : public mppi::NoiseGenerator {
public:
  void freeze(const mppi::models::State &state) {
    noises_vx_=state.cvx;noises_vy_=state.cvy;noises_wz_=state.cwz;
  }
};

class NativeReplay : public mppi::Optimizer {
public:
  NativeReplay(size_t batch,size_t steps,const std::array<float,10> &parameters) {
    settings_.batch_size=batch;settings_.time_steps=steps;
    settings_.model_dt=parameters[0];settings_.temperature=parameters[1];settings_.gamma=parameters[2];
    settings_.sampling_std={parameters[3],parameters[4],parameters[5]};
    settings_.constraints={parameters[7],parameters[6],parameters[8],parameters[9]};
    settings_.base_constraints=settings_.constraints;settings_.iteration_count=1;
    settings_.shift_control_sequence=true;
    motion_model_=std::make_shared<mppi::OmniMotionModel>();
    state_.reset(batch,steps);control_sequence_.reset(steps);
    clear_history();
  }
  void clear_history() {for(auto &control:control_history_)control={0,0,0};}
  void read_cycle(std::istream &input,bool reset,bool zero_history) {
    for(auto *tensor:{&state_.cvx,&state_.cvy,&state_.cwz})
      input.read(reinterpret_cast<char*>(tensor->data()),tensor->size()*sizeof(float));
    costs_=xt::zeros<float>({settings_.batch_size});
    input.read(reinterpret_cast<char*>(costs_.data()),costs_.size()*sizeof(float));
    if(!input)throw std::runtime_error("truncated native replay input");
    if(reset) {
      control_sequence_.reset(settings_.time_steps);clear_history();fixed_.freeze(state_);
    }
    if(zero_history)clear_history(); // Explicit offline negative control only.
  }
  void emit_cycle(std::ostream &out,uint32_t ordinal,bool reset) {
    mppi::models::State expected;expected.reset(settings_.batch_size,settings_.time_steps);
    fixed_.setNoisedControls(expected,control_sequence_); // Actual installed native function.
    bool exact=true;double maximum=0;
    for(auto pair:{std::make_pair(&state_.cvx,&expected.cvx),std::make_pair(&state_.cvy,&expected.cvy),std::make_pair(&state_.cwz,&expected.cwz)}) {
      exact &= std::memcmp(pair.first->data(),pair.second->data(),pair.first->size()*sizeof(float))==0;
      for(size_t k=0;k<pair.first->size();++k)
        maximum=std::max(maximum,std::abs(double(pair.first->data()[k])-pair.second->data()[k]));
    }
    out << "{\"ordinal\":" << ordinal << ",\"reset_from_recorded_events\":" << (reset?"true":"false")
        << ",\"fixed_noise_input_exact\":" << (exact?"true":"false") << ",\"fixed_noise_input_max_error\":" << maximum
        << ",\"history_before\":[";
    bool comma=false;
    for(const auto &control:control_history_) for(float v:{control.vx,control.vy,control.wz}) {
      if(comma)out<<',';
      out<<v;comma=true;
    }
    auto controls=[&](const mppi::models::ControlSequence &sequence) {
      bool separator=false;
      for(const auto *axis:{&sequence.vx,&sequence.vy,&sequence.wz})for(float value:*axis) {
        if(separator)out<<',';
        out<<value;separator=true;
      }
    };
    out << "],\"mean_before\":[";controls(control_sequence_);
    updateControlSequence(); // Actual installed gamma/softmax/aggregation/constraints.
    out << "],\"mean_before_SG\":[";controls(control_sequence_);
    mppi::utils::savitskyGolayFilter(control_sequence_,control_history_,settings_); // Pinned SDK utility.
    out << "],\"mean_after_SG\":[";controls(control_sequence_);
    out << "],\"returned_control\":[" << control_sequence_.vx(1) << ','
        << control_sequence_.vy(1) << ',' << control_sequence_.wz(1) << "]}\n";
    shiftControlSequence(); // Actual installed shift, used to verify the next captured batch.
  }
private:
  FixedNativeNoises fixed_;
};

int main(int argc,char **argv) {
  if(argc!=3 && !(argc==4 && std::string(argv[3])=="--zero-history-each-cycle"))
    throw std::invalid_argument("usage: replay_native_optimizer INPUT.bin OUTPUT.jsonl [--zero-history-each-cycle]");
  std::ifstream input(argv[1],std::ios::binary);std::ofstream out(argv[2]);
  if(!input || !out)throw std::runtime_error("native replay file open failed");
  out << std::setprecision(17);
  char magic[8];input.read(magic,8);
  if(std::string(magic,8)!="RMSGRPL1")throw std::runtime_error("native replay schema");
  uint32_t count,batch,steps;input.read(reinterpret_cast<char*>(&count),4);input.read(reinterpret_cast<char*>(&batch),4);input.read(reinterpret_cast<char*>(&steps),4);
  if(!count || count>1000 || !batch || batch>512 || steps!=30)throw std::runtime_error("native replay budget/grid");
  std::array<float,10> parameters;input.read(reinterpret_cast<char*>(parameters.data()),sizeof(parameters));
  if(!input)throw std::runtime_error("native replay settings missing");
  NativeReplay replay(batch,steps,parameters);
  std::ifstream mappings("/proc/self/maps");std::string line,library;
  while(std::getline(mappings,line))if(line.find("libmppi_controller.so")!=std::string::npos){library=line.substr(line.find('/'));break;}
  out << "{\"kind\":\"native_library\",\"path\":\"" << library << "\"}\n";
  for(uint32_t k=0;k<count;++k) {
    uint32_t ordinal,reset;input.read(reinterpret_cast<char*>(&ordinal),4);input.read(reinterpret_cast<char*>(&reset),4);
    if(!input || ordinal!=k || reset>1 || (k==0 && reset!=1))throw std::runtime_error("native replay schedule");
    replay.read_cycle(input,reset,argc==4);replay.emit_cycle(out,ordinal,reset);
  }
  if(input.peek()!=std::char_traits<char>::eof())throw std::runtime_error("native replay trailing bytes");
}
