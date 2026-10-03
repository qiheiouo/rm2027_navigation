#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_loader.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <unistd.h>

namespace {
std::filesystem::path root;
class CostAccumulatorPlugin : public ::testing::Test {
protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::unique_ptr<mppi::ParametersHandler> handler;
  std::unique_ptr<pluginlib::ClassLoader<mppi::critics::CriticFunction>> loader;
  std::shared_ptr<mppi::critics::CriticFunction> before,after;
  std::filesystem::path bdir,adir;
  mppi::models::State state;mppi::models::Trajectories trajectories;mppi::models::Path path;
  xt::xtensor<float,1> costs;float dt{.1f};
  void SetUp() override {
    if(!rclcpp::ok())rclcpp::init(0,nullptr);
    node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("cost_observer_test");
    costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_observer_map");
    costmap->set_parameter(rclcpp::Parameter("global_frame","odom"));
    costmap->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));costmap->configure();
    costmap->getCostmap()->resizeMap(100,100,.05,-2.5,-2.5);
    const auto test=::testing::UnitTest::GetInstance()->current_test_info()->name();
    bdir=root/(std::string(test)+"_before");adir=root/(std::string(test)+"_after");
    state.reset(2,30);trajectories.reset(2,30);path.reset(2);costs=xt::zeros<float>({2});costs(0)=.25f;costs(1)=8.f;
    state.pose.header.frame_id="odom";state.pose.pose.orientation.w=1.;
    handler=std::make_unique<mppi::ParametersHandler>(node);
    loader=std::make_unique<pluginlib::ClassLoader<mppi::critics::CriticFunction>>("nav2_mppi_controller","mppi::critics::CriticFunction");
  }
  void TearDown() override {
    before.reset();after.reset();loader.reset();handler.reset();costmap->cleanup();costmap.reset();node.reset();
  }
  void parameters(bool capture,int budget=10,bool valid=true){
    std::vector<std::string> names{"ConstraintCritic","BeforeCostAccumulatorCritic","CostCritic","AfterCostAccumulatorCritic","GoalCritic","GoalAngleCritic","PathAlignCritic","PathFollowCritic","PathAngleCritic","DynamicObstacleCritic","StaticStoppingCritic","NativeCycleSnapshotCritic"};
    if(!valid)std::swap(names[1],names[2]);
    node->declare_parameter("FollowPath.critics",names);
    for(const auto &name:{"BeforeCostAccumulatorCritic","AfterCostAccumulatorCritic"}){
      const std::string prefix=std::string("FollowPath.")+name;
      node->declare_parameter(prefix+".enabled",true);
      node->declare_parameter(prefix+".capture_enabled",capture);
      node->declare_parameter(prefix+".output_directory",name==std::string("BeforeCostAccumulatorCritic")?bdir.string():adir.string());
      node->declare_parameter(prefix+".max_records",budget);
    }
  }
  void configure(){
    before=loader->createSharedInstance("mppi::critics::BeforeCostAccumulatorCritic");
    after=loader->createSharedInstance("mppi::critics::AfterCostAccumulatorCritic");
    before->on_configure(node,"FollowPath","FollowPath.BeforeCostAccumulatorCritic",costmap,handler.get());
    after->on_configure(node,"FollowPath","FollowPath.AfterCostAccumulatorCritic",costmap,handler.get());
  }
  mppi::CriticData data(){return {state,trajectories,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};}
  std::string read(const std::filesystem::path &p){std::ifstream s(p);return {std::istreambuf_iterator<char>(s),std::istreambuf_iterator<char>()};}
};
TEST_F(CostAccumulatorPlugin, DefaultOffDoesNotCreateFilesOrMutateData){
  parameters(false);configure();auto d=data();d.fail_flag=true;before->score(d);after->score(d);
  EXPECT_FALSE(std::filesystem::exists(bdir));EXPECT_FALSE(std::filesystem::exists(adir));EXPECT_EQ(costs(0),.25f);EXPECT_EQ(costs(1),8.f);EXPECT_TRUE(d.fail_flag);
}
TEST_F(CostAccumulatorPlugin, ActiveCallbacksShareCommittedTokenAndLeaveCostsUnchanged){
  parameters(true);configure();auto d=data();before->score(d);after->score(d);
  EXPECT_EQ(costs(0),.25f);EXPECT_EQ(costs(1),8.f);EXPECT_FALSE(d.fail_flag);
  EXPECT_NE(read(bdir/"cost_0.json").find("\"paired_before_ordinal\":0"),std::string::npos);
  EXPECT_NE(read(adir/"cost_0.json").find("\"paired_before_ordinal\":0"),std::string::npos);
}
TEST_F(CostAccumulatorPlugin, AfterWithoutBeforeIsExplicitlyUnpaired){
  parameters(true);configure();auto d=data();after->score(d);
  EXPECT_NE(read(adir/"cost_0.json").find("\"paired_before_ordinal\":null"),std::string::npos);
}
TEST_F(CostAccumulatorPlugin, BudgetExhaustionCannotReuseOrphanedBeforeToken){
  parameters(true,1);configure();auto d=data();before->score(d);before->score(d);after->score(d);
  EXPECT_FALSE(std::filesystem::exists(bdir/"cost_1.json"));
  EXPECT_NE(read(adir/"cost_0.json").find("\"paired_before_ordinal\":null"),std::string::npos);
}
TEST_F(CostAccumulatorPlugin, DifferentCriticDataObjectCannotReuseToken){
  parameters(true);configure();auto first=data();auto second=data();before->score(first);after->score(second);
  EXPECT_NE(read(adir/"cost_0.json").find("\"paired_before_ordinal\":null"),std::string::npos);
}
TEST_F(CostAccumulatorPlugin, ReorderedCostCriticIsRejectedBeforeDirectoryCreation){
  parameters(true,10,false);EXPECT_THROW(configure(),std::invalid_argument);
  EXPECT_FALSE(std::filesystem::exists(bdir));EXPECT_FALSE(std::filesystem::exists(adir));
}
} // namespace
int main(int argc,char **argv){
  ::testing::InitGoogleTest(&argc,argv);if(argc>2)return 2;
  root=argc==2?std::filesystem::path(argv[1]):std::filesystem::temp_directory_path()/("rm_cost_accumulator_plugin_"+std::to_string(getpid()));
  if(std::filesystem::exists(root))return 2;
  std::filesystem::create_directories(root);
  const auto result=RUN_ALL_TESTS();rclcpp::shutdown();return result;
}
