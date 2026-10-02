// Offline only: four independent cost interventions on one actual SDK context.
// The unchanged installed optimizer performs aggregation, constraints and model
// propagation. Source/future oracles are never published or represented as data
// actually consumed by the live controller.
#include "nav2_mppi_controller/optimizer.hpp"
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

template<typename T> void read_values(std::istream &stream, T *values, size_t count) {
  stream.read(reinterpret_cast<char *>(values), count*sizeof(T));
  if (!stream) throw std::runtime_error("truncated native factor input");
}
template<typename T> void write_values(std::ostream &stream, const T *values, size_t count) {
  stream.write(reinterpret_cast<const char *>(values), count*sizeof(T));
}
bool finite_bits(float value) {
  uint32_t bits; std::memcpy(&bits, &value, sizeof(bits));
  return (bits&0x7f800000u)!=0x7f800000u;
}
bool finite_bits(double value) {
  uint64_t bits; std::memcpy(&bits, &value, sizeof(bits));
  return (bits&0x7ff0000000000000ull)!=0x7ff0000000000000ull;
}
template<typename T> void require_finite(const T &values) {
  for (const auto &value : values) if (!finite_bits(value))
    throw std::runtime_error("nonfinite native factor field");
}
using Sequence = std::array<float, 90>;
struct Input {
  std::array<float, 10> parameters;
  std::array<double, 13> measured;
  std::array<float, 12> history;
  Sequence warm, expected_mean, expected_sg;
  std::array<double, 3> expected_command;
  std::array<std::vector<float>, 3> controls;
  std::array<std::vector<float>, 6> raw_reference;
  std::array<std::vector<float>, 4> costs;
  std::vector<float> expected_gamma, expected_weights;
  std::array<std::array<float, 30>, 9> expected_path;
};
Input read_input(const char *path) {
  if (std::filesystem::file_size(path)!=333600) throw std::runtime_error("native factor exact byte budget");
  std::ifstream stream(path, std::ios::binary); std::array<char, 8> magic;
  read_values(stream, magic.data(), magic.size());
  if (std::memcmp(magic.data(), "RMCVAGH1", 8)) throw std::runtime_error("native factor hypothesis schema");
  std::array<uint32_t, 4> header; read_values(stream, header.data(), header.size());
  if (header!=std::array<uint32_t, 4>{298,300,30,4}) throw std::runtime_error("native factor source/grid/count");
  Input input;
  for (auto *array : {&input.parameters}) { read_values(stream,array->data(),array->size()); require_finite(*array); }
  read_values(stream,input.measured.data(),input.measured.size()); require_finite(input.measured);
  read_values(stream,input.history.data(),input.history.size()); require_finite(input.history);
  for (auto *array : {&input.warm,&input.expected_mean,&input.expected_sg}) {
    read_values(stream,array->data(),array->size()); require_finite(*array);
  }
  read_values(stream,input.expected_command.data(),input.expected_command.size()); require_finite(input.expected_command);
  for (auto &array : input.controls) { array.resize(9000); read_values(stream,array.data(),array.size()); require_finite(array); }
  for (auto &array : input.raw_reference) { array.resize(9000); read_values(stream,array.data(),array.size()); require_finite(array); }
  for (auto &array : input.costs) { array.resize(300); read_values(stream,array.data(),array.size()); require_finite(array); }
  for (auto *array : {&input.expected_gamma,&input.expected_weights}) {
    array->resize(300); read_values(stream,array->data(),array->size()); require_finite(*array);
  }
  for (auto &array : input.expected_path) { read_values(stream,array.data(),array.size()); require_finite(array); }
  if (stream.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("native factor trailing data");
  const auto &p=input.parameters;
  if (p[0]!=float(.1) || p[1]<=0 || p[2]<0 || p[3]<=0 || p[4]<=0 || p[5]<=0 ||
      p[6]>=0 || p[7]<=0 || p[8]<=0 || p[9]<=0)
    throw std::runtime_error("native factor settings invalid");
  return input;
}
struct Result {
  Sequence mean, sg;
  std::array<double, 3> command;
  std::vector<float> gamma, weights;
  bool reaggregate_exact=false;
};
class NativeFactors : public mppi::Optimizer {
public:
  double initial_yaw() const { return tf2::getYaw(state_.pose.pose.orientation); }
  explicit NativeFactors(const Input &input) {
    const auto &p=input.parameters;
    settings_.batch_size=300; settings_.time_steps=30; settings_.model_dt=p[0];
    settings_.temperature=p[1]; settings_.gamma=p[2]; settings_.sampling_std={p[3],p[4],p[5]};
    settings_.constraints={p[7],p[6],p[8],p[9]}; settings_.base_constraints=settings_.constraints;
    settings_.shift_control_sequence=true; settings_.iteration_count=1;
    motion_model_=std::make_shared<mppi::OmniMotionModel>();
    state_.reset(300,30); control_sequence_.reset(30); generated_trajectories_.reset(300,30);
    set_measured(state_,input.measured);
    size_t axis=0;
    for (auto *tensor : {&state_.cvx,&state_.cvy,&state_.cwz}) {
      std::memcpy(tensor->data(),input.controls[axis].data(),9000*sizeof(float)); ++axis;
    }
    updateStateVelocities(state_); integrateStateVelocities(generated_trajectories_,state_);
    axis=0;
    for (const auto *tensor : {&state_.vx,&state_.vy,&state_.wz,&generated_trajectories_.x,
        &generated_trajectories_.y,&generated_trajectories_.yaws}) {
      if (std::memcmp(tensor->data(),input.raw_reference[axis].data(),9000*sizeof(float)))
        throw std::runtime_error("actual raw native model bits mismatch");
      ++axis;
    }
  }
  static void set_measured(mppi::models::State &state,const std::array<double,13> &m) {
    auto &pose=state.pose.pose; auto &speed=state.speed;
    pose.position.x=m[0];pose.position.y=m[1];pose.position.z=m[2];
    pose.orientation.x=m[3];pose.orientation.y=m[4];pose.orientation.z=m[5];pose.orientation.w=m[6];
    speed.linear.x=m[7];speed.linear.y=m[8];speed.linear.z=m[9];
    speed.angular.x=m[10];speed.angular.y=m[11];speed.angular.z=m[12];
  }
  Sequence sequence() const {
    Sequence values; size_t axis=0;
    for (const auto *tensor : {&control_sequence_.vx,&control_sequence_.vy,&control_sequence_.wz}) {
      std::memcpy(values.data()+axis*30,tensor->data(),30*sizeof(float)); ++axis;
    }
    require_finite(values); return values;
  }
  Result evaluate(const Input &input,size_t factor) {
    size_t axis=0;
    for (auto *tensor : {&control_sequence_.vx,&control_sequence_.vy,&control_sequence_.wz}) {
      std::memcpy(tensor->data(),input.warm.data()+axis*30,30*sizeof(float)); ++axis;
    }
    size_t h=0;
    for (auto &control : control_history_) {
      control={input.history[h],input.history[h+1],input.history[h+2]};h+=3;
    }
    costs_=xt::zeros<float>({300}); std::memcpy(costs_.data(),input.costs[factor].data(),300*sizeof(float));
    updateControlSequence(); // Installed gamma, softmax, mean and bounds.
    Result result; result.mean=sequence(); result.gamma.assign(costs_.begin(),costs_.end());
    using xt::evaluation_strategy::immediate;
    auto &&normalized=costs_-xt::amin(costs_,immediate);
    auto &&exponents=xt::eval(xt::exp(-1/settings_.temperature*normalized));
    auto &&weights=xt::eval(exponents/xt::sum(exponents,immediate));
    result.weights.assign(weights.begin(),weights.end()); require_finite(result.weights);require_finite(result.gamma);
    auto actual=control_sequence_;
    auto &&expanded=xt::eval(xt::view(weights,xt::all(),xt::newaxis()));
    xt::noalias(control_sequence_.vx)=xt::sum(state_.cvx*expanded,0,immediate);
    xt::noalias(control_sequence_.vy)=xt::sum(state_.cvy*expanded,0,immediate);
    xt::noalias(control_sequence_.wz)=xt::sum(state_.cwz*expanded,0,immediate);
    applyControlSequenceConstraints();
    const auto reaggregated=sequence(); result.reaggregate_exact=std::memcmp(reaggregated.data(),result.mean.data(),sizeof(Sequence))==0;
    control_sequence_=std::move(actual);
    if (!result.reaggregate_exact) throw std::runtime_error("native factor weights do not reaggregate exact bounded mean");
    mppi::utils::savitskyGolayFilter(control_sequence_,control_history_,settings_);
    result.sg=sequence(); result.command={double(control_sequence_.vx(1)),double(control_sequence_.vy(1)),double(control_sequence_.wz(1))};
    return result;
  }
  std::array<std::vector<float>,9> paths(const Input &input,const std::array<Result,4> &results) {
#ifdef RM_NATIVE_SG_STAGES
    constexpr size_t rows=8;
    mppi::models::State states;states.reset(rows,30);set_measured(states,input.measured);
    for (size_t row=0;row<rows;++row) {
      const auto &controls=row%2?results[row/2].sg:results[row/2].mean;
      size_t axis=0;
      for (auto *tensor : {&states.cvx,&states.cvy,&states.cwz}) {
        std::memcpy(tensor->data()+row*30,controls.data()+axis*30,30*sizeof(float));++axis;
      }
    }
#else
    mppi::models::State states;states.reset(4,30);set_measured(states,input.measured);
    for (size_t row=0;row<4;++row) {
      size_t axis=0;
      for (auto *tensor : {&states.cvx,&states.cvy,&states.cwz}) {
        std::memcpy(tensor->data()+row*30,results[row].sg.data()+axis*30,30*sizeof(float));++axis;
      }
    }
#endif
    updateStateVelocities(states);
#ifdef RM_NATIVE_SG_STAGES
    mppi::models::Trajectories trajectories;trajectories.reset(rows,30);integrateStateVelocities(trajectories,states);
#else
    mppi::models::Trajectories trajectories;trajectories.reset(4,30);integrateStateVelocities(trajectories,states);
#endif
    std::array<std::vector<float>,9> output;size_t axis=0;
    for (const auto *tensor : {&states.cvx,&states.cvy,&states.cwz,&states.vx,&states.vy,&states.wz,
        &trajectories.x,&trajectories.y,&trajectories.yaws}) {
      output[axis].assign(tensor->begin(),tensor->end());require_finite(output[axis]);++axis;
    }
#ifdef RM_NATIVE_SG_STAGES
    for (size_t row=0;row<rows;++row) for (size_t a=0;a<3;++a) {
#else
    for (size_t row=0;row<4;++row) for (size_t a=0;a<3;++a) {
#endif
      const float expected=float(input.measured[a==2?12:7+a]);
      if (std::memcmp(output[3+a].data()+row*30,&expected,4)) throw std::runtime_error("factor measured prefix changed");
    }
    return output;
  }
};
template<typename T> void json_array(std::ostream &out,const T &values) {
  out<<'[';bool comma=false;
  for (const auto value : values) {if(comma)out<<',';out<<value;comma=true;}out<<']';
}
int main(int argc,char **argv) {
  try {
    if(argc!=4)throw std::invalid_argument("ACTUAL_AND_HYPOTHESIS_INPUT NEW_PATHS_BIN NEW_JSONL");
    if(std::filesystem::exists(argv[2]) || std::filesystem::exists(argv[3]))throw std::runtime_error("refusing to overwrite native factor outputs");
    const auto input=read_input(argv[1]);NativeFactors native(input);std::array<Result,4> results;
    std::ifstream mappings("/proc/self/maps");std::string line,library;
    while(std::getline(mappings,line))if(line.find("libmppi_controller.so")!=std::string::npos){library=line.substr(line.find('/'));break;}
    if(library!="/opt/ros/humble/lib/libmppi_controller.so")throw std::runtime_error("pinned native library path mismatch");
    for(size_t factor=0;factor<4;++factor)results[factor]=native.evaluate(input,factor);
    const auto paths=native.paths(input,results);const auto &baseline=results[0];
    if(std::memcmp(baseline.mean.data(),input.expected_mean.data(),sizeof(Sequence)) ||
       std::memcmp(baseline.sg.data(),input.expected_sg.data(),sizeof(Sequence)) ||
       std::memcmp(baseline.command.data(),input.expected_command.data(),24) ||
       std::memcmp(baseline.gamma.data(),input.expected_gamma.data(),1200) ||
       std::memcmp(baseline.weights.data(),input.expected_weights.data(),1200))
      throw std::runtime_error("native factor actual gamma/weights/mean/SG/command bits mismatch");
#ifdef RM_NATIVE_SG_STAGES
    for(size_t axis=0;axis<9;++axis)if(std::memcmp(paths[axis].data()+30,input.expected_path[axis].data(),120))
#else
    for(size_t axis=0;axis<9;++axis)if(std::memcmp(paths[axis].data(),input.expected_path[axis].data(),120))
#endif
      throw std::runtime_error("native factor actual aggregate path bits mismatch");
    std::ofstream binary(argv[2],std::ios::binary),json(argv[3]);
    binary.exceptions(std::ios::failbit|std::ios::badbit);json.exceptions(std::ios::failbit|std::ios::badbit);json<<std::setprecision(17);
#ifdef RM_NATIVE_SG_STAGES
    const uint32_t source=298,rows=8,steps=30,baseline_mask=127;const float dt=input.parameters[0];
#else
    const uint32_t source=298,rows=4,steps=30,baseline_mask=127;const float dt=input.parameters[0];
#endif
    const double yaw=native.initial_yaw();
#ifdef RM_NATIVE_SG_STAGES
    binary.write("RMCVAST1",8);write_values(binary,&source,1);write_values(binary,&rows,1);
#else
    binary.write("RMCVAGO1",8);write_values(binary,&source,1);write_values(binary,&rows,1);
#endif
    write_values(binary,&steps,1);write_values(binary,&baseline_mask,1);write_values(binary,&dt,1);write_values(binary,&yaw,1);
    for(const auto &values : paths)write_values(binary,values.data(),values.size());
#ifdef RM_NATIVE_SG_STAGES
    json<<"{\"source_ordinal\":298,\"rows\":8,\"steps\":30,\"model_dt\":"<<dt<<",\"native_library\":\""<<library<<'"'
        <<",\"actual_baseline_bit_exact_mask\":127,\"measured_prefix_exact_all_eight\":true,"
        <<"\"path_order\":\"factor-major; bounded_mean_before_SG then actual_history_after_SG\","
        <<"\"scope\":\"offline fixed-context stage paths; pre-SG is a model hypothesis, not a published native command\"}\n";
#else
    json<<"{\"source_ordinal\":298,\"rows\":4,\"steps\":30,\"model_dt\":"<<dt<<",\"native_library\":\""<<library<<'"'
        <<",\"actual_baseline_bit_exact_mask\":127,\"measured_prefix_exact_all_four\":true,"
        <<"\"scope\":\"offline fixed-context native factor aggregation; baseline replica and three counterfactuals, no new actual event\"}\n";
#endif
    const std::array<const char *,4> names{"actual_baseline","source_geometry_oracle","source_bracket_CV_oracle","future_circle_oracle"};
    for(size_t factor=0;factor<4;++factor) {
      const auto &r=results[factor];json<<"{\"row\":"<<factor<<",\"factor\":\""<<names[factor]<<"\",\"weights_reaggregate_bounded_mean_bit_exact\":true,\"post_gamma_costs\":";
      json_array(json,r.gamma);json<<",\"native_softmax_weights\":";json_array(json,r.weights);
      json<<",\"mean_before_SG\":";json_array(json,r.mean);json<<",\"mean_after_SG\":";json_array(json,r.sg);
      json<<",\"returned_control\":";json_array(json,r.command);json<<"}\n";
    }
    std::cout<<"actual_baseline_all_bits_exact=1 factors=4 measured_prefix_exact=1\n";return 0;
  }catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}
