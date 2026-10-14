// Fixed-input diagnostic noninterference fixture. No ROS node or publisher.
// Native weighted update comes from installed Nav2 1.1.20. Numeric output uses
// the installed offset rule; Twist wrapper requires a live costmap ROS node.
#include <nav2_mppi_controller/optimizer.hpp>
#include <fstream>
#include "/check/shadow_export.hpp"
#include <cstring>
#include <filesystem>
#include <iostream>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>
using J=nlohmann::json;

template<class T>void append(std::vector<float>&v,const T&t){v.insert(v.end(),t.begin(),t.end());}
bool bits(const std::vector<float>&a,const std::vector<float>&b){return a.size()==b.size()&&std::memcmp(a.data(),b.data(),a.size()*sizeof(float))==0;}
struct Fixture: mppi::Optimizer {
 Fixture(const J&record,const std::string&raw){
  settings_.batch_size=300;settings_.time_steps=30;settings_.model_dt=.1f;
  settings_.temperature=.3f;settings_.gamma=0;settings_.sampling_std={.2f,.2f,.4f};
  settings_.base_constraints={.7f,-.35f,.5f,1.8f};settings_.constraints=settings_.base_constraints;
  settings_.shift_control_sequence=false;setMotionModel("Omni");reset();
  std::ifstream f(raw,std::ios::binary);if(!f)throw std::runtime_error("missing raw slice");
  for(size_t i=0;i<300;i++)for(size_t k=0;k<30;k++){
   double x[6];f.read(reinterpret_cast<char*>(x),sizeof(x));if(!f)throw std::runtime_error("short raw slice");
   generated_trajectories_.x(i,k)=x[0];generated_trajectories_.y(i,k)=x[1];generated_trajectories_.yaws(i,k)=x[2];
   state_.vx(i,k)=state_.cvx(i,k)=x[3];state_.vy(i,k)=state_.cvy(i,k)=x[4];state_.wz(i,k)=state_.cwz(i,k)=x[5];
  }
  for(size_t i=0;i<300;i++)costs_(i)=record.at("costs")[i].get<float>();
 }
 std::vector<float> input()const{
  std::vector<float> v;for(auto*t:{&state_.vx,&state_.vy,&state_.wz,&state_.cvx,&state_.cvy,&state_.cwz,&generated_trajectories_.x,&generated_trajectories_.y,&generated_trajectories_.yaws})append(v,*t);append(v,costs_);return v;
 }
 std::vector<float> output(){
  updateControlSequence();std::vector<float> v;append(v,costs_);append(v,control_sequence_.vx);append(v,control_sequence_.vy);append(v,control_sequence_.wz);
  size_t offset=settings_.shift_control_sequence?1:0;
  v.push_back(control_sequence_.vx(offset));v.push_back(control_sequence_.vy(offset));v.push_back(control_sequence_.wz(offset));return v;
 }
 mppi::CriticData&data(){return critics_data_;}
};
int main(int argc,char**argv){try{
 signal(SIGSEGV,[](int code){void*frames[24];int n=backtrace(frames,24);backtrace_symbols_fd(frames,n,2);_exit(128+code);});
 if(argc!=3)return 2;std::ifstream in(argv[1]);J cases;in>>cases;std::filesystem::path dir(argv[2]);J rows=J::array();bool all=true;
 for(const auto&c:cases){
  auto file=(dir/("cycle_"+std::to_string(c["cycle"].get<int>())+".bin")).string();
  Fixture base(c,file);auto expected=base.output();
  for(const std::string mode:{"shadow_off_export_on","shadow_off_export_off","shadow_on_export_off"}){
   Fixture f(c,file);shadow::s=shadow::State{};shadow::s.enabled=mode=="shadow_on_export_off";shadow::s.candidate_export_enabled=mode=="shadow_off_export_on";shadow::s.cycle=c["cycle"];
   auto path=dir/("export_"+std::to_string(c["cycle"].get<int>())+"_"+mode+".bin");
   if(std::filesystem::exists(path))throw std::runtime_error("do not overwrite fixture evidence");
   if(shadow::s.candidate_export_enabled)shadow::s.snapshots.open(path,std::ios::binary);
   auto before=f.input();auto ptr=f.data().costs.data();auto flag=f.data().fail_flag;
   shadow::read_raw(f.data());auto after=f.input();auto start=shadow::Clock::now();shadow::export_candidates();auto end=shadow::Clock::now();shadow::s.snapshots.close();
   bool unchanged=bits(before,after)&&flag==f.data().fail_flag&&ptr==f.data().costs.data();
   auto actual=f.output();bool equal=bits(actual,expected);bool raw_needed=shadow::s.enabled||shadow::s.candidate_export_enabled;
   auto size=std::filesystem::exists(path)?std::filesystem::file_size(path):0;
   bool available=shadow::s.raw.size()==(raw_needed?300:0)&&shadow::s.attempts==1&&size==(shadow::s.candidate_export_enabled?432016:0)&&std::filesystem::exists(path)==shadow::s.candidate_export_enabled;
   // Two skipped reads with a real CriticData failure flag: count calls without
   // changing fail_flag or interpreting missing diagnostic candidates as safe.
   f.data().fail_flag=true;shadow::s.attempts=0;shadow::read_raw(f.data());shadow::read_raw(f.data());
   bool failure_unchanged=f.data().fail_flag&&shadow::s.attempts==2;
   all &= unchanged&&equal&&available&&failure_unchanged;
   rows.push_back(J{{"cycle",c["cycle"]},{"original_safe_candidates",c["original_safe_candidates"]},{"mode",mode},{"input_bits_and_cost_storage_unchanged",unchanged},{"native_output_bits_equal_no_observer",equal},{"availability_pass",available},{"fail_flag_and_two_attempts_preserved",failure_unchanged},{"raw_retained",shadow::s.raw.size()},{"export_bytes",size},{"raw_read_ms_including_two_flag_probes",shadow::s.raw_read_ms},{"export_ms",shadow::ms(start,end)},{"fixture_command",{actual[actual.size()-3],actual[actual.size()-2],actual.back()}}});
  }
 }
 std::cout<<J{{"predeclared_acceptance_pass",all},{"cases",rows},{"native_updates",8},{"scope","weighted update fixture; not original warm/noised/SG/shift or navigation replay"}}.dump()<<'\n';return all?0:2;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
