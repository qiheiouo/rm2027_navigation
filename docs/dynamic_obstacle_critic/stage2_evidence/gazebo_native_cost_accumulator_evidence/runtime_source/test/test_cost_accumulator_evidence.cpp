#include "rm_dynamic_obstacle_critic/cost_accumulator_evidence.hpp"
#include <gtest/gtest.h>
#include <iterator>
#include <unistd.h>

namespace {
std::filesystem::path root;
class CostAccumulatorEvidence : public ::testing::Test {
protected:
  mppi::models::State state;
  mppi::models::Trajectories trajectories;
  mppi::models::Path path;
  xt::xtensor<float,1> costs;
  float dt{.1f};
  rm_dynamic_obstacle_critic::CostAccumulatorContext context;
  void SetUp() override {
    state.reset(2,30);trajectories.reset(2,30);path.reset(2);costs=xt::zeros<float>({2});
    state.pose.header.frame_id="odom";state.pose.pose.orientation.w=1.;costs(0)=.25f;costs(1)=8.f;
    context.stage="before_CostCritic";context.evaluation_frame="odom";context.base_frame="base_link";
    context.paired_before_ordinal=0;
    context.map_size={100,100};context.origin={-2.5,-2.5};context.resolution=.05;
    context.map_bytes.resize(10000);context.padded_footprint={{{-.35,-.33}},{{.35,-.33}},{{.35,.33}},{{-.35,.33}}};
  }
  mppi::CriticData data(){return {state,trajectories,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};}
  std::filesystem::path directory(const std::string &name){auto p=root/name;std::filesystem::create_directory(p);return p;}
  std::vector<uint8_t> read(const std::filesystem::path &p){std::ifstream s(p,std::ios::binary);return {std::istreambuf_iterator<char>(s),std::istreambuf_iterator<char>()};}
  void write(const std::filesystem::path &p,mppi::CriticData &d,size_t ordinal=0){rm_dynamic_obstacle_critic::write_cost_accumulator_observation(p,ordinal,42.,d,context);}
};
TEST_F(CostAccumulatorEvidence, CopiesActualBytesAndLeavesAllInputsUnchanged){
  auto d=data();d.fail_flag=true;const auto old_costs=costs;const auto old_x=trajectories.x;const auto old_vx=state.vx;
  auto p=directory("readonly");write(p,d);const auto payload=read(p/"cost_0.bin");
  ASSERT_EQ(payload.size(),9*2*30*4+2*4+3*2*4+10000);EXPECT_EQ(std::memcmp(payload.data()+9*2*30*4,costs.data(),8),0);
  EXPECT_EQ(std::memcmp(costs.data(),old_costs.data(),8),0);EXPECT_EQ(std::memcmp(trajectories.x.data(),old_x.data(),240),0);
  EXPECT_EQ(std::memcmp(state.vx.data(),old_vx.data(),240),0);EXPECT_TRUE(d.fail_flag);EXPECT_EQ(context.stage,"before_CostCritic");
  const auto bytes=read(p/"cost_0.json");const std::string text(bytes.begin(),bytes.end());
  EXPECT_NE(text.find("\"stage\":\"before_CostCritic\""),std::string::npos);EXPECT_NE(text.find("\"fail_flag\":true"),std::string::npos);
}
TEST_F(CostAccumulatorEvidence, PreservesCommittedAndPartialEvidence){
  auto d=data();auto p=directory("committed");write(p,d);const auto original=read(p/"cost_0.bin");
  EXPECT_THROW(write(p,d),std::runtime_error);EXPECT_EQ(read(p/"cost_0.bin"),original);
  p=directory("partial");std::ofstream(p/"cost_0.bin.tmp")<<"retained partial";
  EXPECT_THROW(write(p,d),std::runtime_error);EXPECT_FALSE(std::filesystem::exists(p/"cost_0.json"));
  EXPECT_EQ(read(p/"cost_0.bin.tmp"),std::vector<uint8_t>({'r','e','t','a','i','n','e','d',' ','p','a','r','t','i','a','l'}));
}
TEST_F(CostAccumulatorEvidence, RejectsNonfiniteDataWithoutCommitting){
  auto d=data();auto p=directory("nan_cost");costs(1)=std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(write(p,d),std::runtime_error);EXPECT_FALSE(std::filesystem::exists(p/"cost_0.bin"));
  costs(1)=8;state.cvx(0,0)=std::numeric_limits<float>::infinity();p=directory("inf_control");
  EXPECT_THROW(write(p,d),std::runtime_error);EXPECT_FALSE(std::filesystem::exists(p/"cost_0.json"));
}
TEST_F(CostAccumulatorEvidence, RejectsUnboundedOrUnassociatedContext){
  auto d=data();auto p=directory("bad_context");context.stage="actual_CostCritic_branch";
  EXPECT_THROW(write(p,d),std::runtime_error);context.stage="after_CostCritic";context.map_size={65537,1};
  EXPECT_THROW(write(p,d),std::runtime_error);context.map_size={100,100};context.evaluation_frame="map";
  EXPECT_THROW(write(p,d),std::runtime_error);context.evaluation_frame="odom";
  EXPECT_THROW(write(p,d,1000),std::runtime_error);EXPECT_FALSE(std::filesystem::exists(p/"cost_0.json"));
}
} // namespace
int main(int argc,char **argv){
  ::testing::InitGoogleTest(&argc,argv);
  if(argc>2)return 2;
  root=argc==2?std::filesystem::path(argv[1]):std::filesystem::temp_directory_path()/("rm_cost_accumulator_test_"+std::to_string(getpid()));
  if(std::filesystem::exists(root))return 2;
  std::filesystem::create_directories(root);return RUN_ALL_TESTS();
}
