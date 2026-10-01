#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_loader.hpp"
#include "rm_dynamic_obstacle_critic/model.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <gtest/gtest.h>
#include <thread>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <rcl/time.h>

class PluginTest : public ::testing::Test {
protected:
  virtual std::string plugin_name() const { return "DynamicObstacleCritic"; }
  virtual void prepare_parameters() {}
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("critic_test");
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
    costmap->set_parameter(rclcpp::Parameter("global_frame", "map"));
    costmap->set_parameter(
        rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap->configure();
    std::vector<geometry_msgs::msg::Point> fp(4);
    fp[0].x = -.25;
    fp[0].y = -.2;
    fp[1].x = .25;
    fp[1].y = -.2;
    fp[2].x = .25;
    fp[2].y = .2;
    fp[3].x = -.25;
    fp[3].y = .2;
    costmap->setRobotFootprint(fp);
    prepare_parameters();
    handler = std::make_unique<mppi::ParametersHandler>(node);
    loader =
        std::make_unique<pluginlib::ClassLoader<mppi::critics::CriticFunction>>(
            "nav2_mppi_controller", "mppi::critics::CriticFunction");
    critic = loader->createSharedInstance("mppi::critics::" + plugin_name());
    critic->on_configure(node, "FollowPath", "FollowPath." + plugin_name(),
                         costmap, handler.get());
    pub = node->create_publisher<rm_dynamic_obstacle_critic::Array>(
        "/perception/dynamic_obstacles_shadow/predictions", 1);
    pub->on_activate();
    exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node->get_node_base_interface());
  }
  void TearDown() override {
    exec->remove_node(node->get_node_base_interface());
    critic.reset();
    handler.reset();
    costmap->cleanup();
    costmap.reset();
    node.reset();
    loader.reset();
  }
  void publish(rm_dynamic_obstacle_critic::Array msg) {
    for (int i = 0; i < 20 && pub->get_subscription_count() == 0; ++i) {
      exec->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    pub->publish(msg);
    for (int i = 0; i < 8; ++i) {
      exec->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  rm_dynamic_obstacle_critic::Array input() {
    using A = rm_dynamic_obstacle_critic::Array;
    using O = rm_dynamic_obstacle_critic::Obstacle;
    A msg;
    msg.schema = A::SCHEMA;
    msg.authority = A::AUTHORITY_SHADOW_ONLY;
    msg.complete = true;
    msg.header.frame_id = "map";
    msg.header.stamp = node->now();
    msg.prediction_dt = .1;
    msg.prediction_steps = 30;
    msg.total_track_count = 1;
    O t;
    t.track_id = 1;
    t.state = O::STATE_CONFIRMED;
    t.position.x = 1;
    t.position.y = -1;
    t.velocity.y = 1;
    t.size.x = .2;
    t.size.y = .2;
    t.last_observation_stamp = msg.header.stamp;
    msg.tracks.push_back(t);
    return msg;
  }
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::unique_ptr<mppi::ParametersHandler> handler;
  std::unique_ptr<pluginlib::ClassLoader<mppi::critics::CriticFunction>> loader;
  std::shared_ptr<mppi::critics::CriticFunction> critic;
  rclcpp_lifecycle::LifecyclePublisher<
      rm_dynamic_obstacle_critic::Array>::SharedPtr pub;
};
TEST_F(PluginTest, NativePluginScoresFutureAndWaitAndRejectsStale) {
  mppi::models::State state;
  state.reset(2, 30);
  mppi::models::Trajectories trajectories;
  trajectories.reset(2, 30);
  mppi::models::Path path;
  path.reset(2);
  xt::xtensor<float, 1> costs = xt::zeros<float>({2});
  float dt = .1;
  for (int k = 0; k < 30; ++k) {
    trajectories.x(0, k) = (k + 1) * .1f;
    trajectories.x(1, k) = 0;
  }
  mppi::CriticData data{state,        trajectories, path,    costs,
                        dt,           false,        nullptr, nullptr,
                        std::nullopt, std::nullopt};
  critic->score(data);
  EXPECT_TRUE(data.fail_flag);
  auto msg = input();
  publish(msg);
  data.fail_flag = false;
  critic->score(data);
  EXPECT_FALSE(data.fail_flag);
  EXPECT_GT(costs(0), 1000);
  EXPECT_LT(costs(1), 1);
  msg.header.stamp.sec -= 2;
  publish(msg);
  data.fail_flag = false;
  critic->score(data);
  EXPECT_TRUE(data.fail_flag);
}
TEST_F(PluginTest, GridMismatchFailsInsteadOfTruncating) {
  auto msg = input();
  msg.prediction_steps = 10;
  publish(msg);
  mppi::models::State state;
  state.reset(1, 30);
  mppi::models::Trajectories tr;
  tr.reset(1, 30);
  mppi::models::Path path;
  path.reset(2);
  xt::xtensor<float, 1> costs = xt::zeros<float>({1});
  float dt = .1;
  mppi::CriticData data{state, tr,      path,    costs,        dt,
                        false, nullptr, nullptr, std::nullopt, std::nullopt};
  critic->score(data);
  EXPECT_TRUE(data.fail_flag);
}

class StoppingPluginTest : public PluginTest {
protected:
  std::string plugin_name() const override { return "StaticStoppingCritic"; }
  void check_near_cell(float moving_cost) {
    auto grid = costmap->getCostmap();
    grid->resizeMap(120, 120, .05, -3, -3);
    std::fill(grid->getCharMap(), grid->getCharMap() + 120 * 120, 0);
    unsigned int x, y;
    ASSERT_TRUE(grid->worldToMap(.45, 0, x, y));
    grid->setCost(x, y, 203);
    mppi::models::State state;
    state.reset(2, 30);
    state.pose.header.frame_id = "map";
    state.pose.pose.orientation.w = 1;
    state.cvx(0, 1) = .2;
    mppi::models::Trajectories tr;
    tr.reset(2, 30);
    mppi::models::Path path;
    path.reset(2);
    xt::xtensor<float, 1> costs = xt::zeros<float>({2});
    float dt = .1;
    mppi::CriticData data{state, tr,      path,    costs,        dt,
                          false, nullptr, nullptr, std::nullopt, std::nullopt};
    critic->score(data);
    EXPECT_FALSE(data.fail_flag);
    EXPECT_EQ(costs(0), moving_cost);
    EXPECT_EQ(costs(1), 0); // A stopped proposal retains sufficient clearance.
  }
};
class BufferedStoppingPluginTest : public StoppingPluginTest {
protected:
  void prepare_parameters() override {
    node->declare_parameter(
        "FollowPath.StaticStoppingCritic.map_uncertainty_margin", .11);
  }
};
class SoftStoppingPluginTest : public BufferedStoppingPluginTest {
protected:
  void prepare_parameters() override {
    BufferedStoppingPluginTest::prepare_parameters();
    node->declare_parameter(
        "FollowPath.StaticStoppingCritic.map_uncertainty_mode", "soft");
  }
};
TEST_F(SoftStoppingPluginTest,
       EscapeWaitApproachStayDistinctInsidePlanningBand) {
  auto grid = costmap->getCostmap();
  grid->resizeMap(120, 120, .05, -3, -3);
  std::fill(grid->getCharMap(), grid->getCharMap() + 120 * 120, 0);
  unsigned int x, y;
  ASSERT_TRUE(grid->worldToMap(.32, 0, x, y));
  grid->setCost(x, y, 203);
  mppi::models::State state;
  state.reset(4, 30);
  state.pose.header.frame_id = "map";
  state.pose.pose.orientation.w = 1;
  state.cvx(0, 1) = -.2; // Escape from a band shared by all at t=0.
  state.cvx(1, 1) = 0;
  state.cvx(2, 1) = .03; // Still raw-clear, but moves closer.
  state.cvx(3, 1) = .2;  // Violates the original raw203 check.
  mppi::models::Trajectories tr;
  tr.reset(4, 30);
  mppi::models::Path path;
  path.reset(2);
  xt::xtensor<float, 1> costs = xt::zeros<float>({4});
  float dt = .1;
  mppi::CriticData data{state, tr,      path,    costs,        dt,
                        false, nullptr, nullptr, std::nullopt, std::nullopt};
  critic->score(data);
  EXPECT_FALSE(data.fail_flag);
  EXPECT_GT(costs(0), 0);
  EXPECT_LT(costs(0), costs(1));
  EXPECT_LT(costs(1), costs(2));
  EXPECT_LT(costs(2), 10000);
  EXPECT_GE(costs(3), 10000);
  // Measured momentum remains an independent hard rejection, even for escape.
  state.speed.linear.x = .8;
  costs.fill(0);
  critic->score(data);
  EXPECT_FALSE(data.fail_flag);
  for (float value : costs)
    EXPECT_EQ(value, 10000);
}
TEST_F(StoppingPluginTest, UnsupportedSoftModeParametersRejectInitialization) {
  for (const auto &suffix : {std::string("unknown"), std::string("zero_band"),
                             std::string("negative_weight")}) {
    const auto name = "FollowPath." + suffix;
    node->declare_parameter(name + ".map_uncertainty_mode",
                            suffix == "unknown" ? "invalid" : "soft");
    node->declare_parameter(name + ".map_uncertainty_margin",
                            suffix == "zero_band" ? 0. : .11);
    node->declare_parameter(name + ".map_uncertainty_weight",
                            suffix == "negative_weight" ? -1. : 10000.);
    auto invalid =
        loader->createSharedInstance("mppi::critics::StaticStoppingCritic");
    EXPECT_THROW(
        invalid->on_configure(node, "FollowPath", name, costmap, handler.get()),
        std::invalid_argument);
  }
}
TEST_F(StoppingPluginTest, DefaultMarginPreservesNearCellDecision) {
  check_near_cell(0);
}
TEST_F(BufferedStoppingPluginTest, PlanningAllowanceRejectsNearCellProposal) {
  check_near_cell(10000);
}
TEST_F(StoppingPluginTest, InvalidPlanningAllowancesRejectInitialization) {
  size_t i = 0;
  for (double margin : {-1., std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN()}) {
    const auto name = "FollowPath.InvalidMargin" + std::to_string(i++);
    node->declare_parameter(name + ".map_uncertainty_margin", margin);
    auto invalid =
        loader->createSharedInstance("mppi::critics::StaticStoppingCritic");
    EXPECT_THROW(
        invalid->on_configure(node, "FollowPath", name, costmap, handler.get()),
        std::invalid_argument);
  }
}
TEST_F(StoppingPluginTest, ScoresNativeOffsetOneWithUnchangedRaw203Gate) {
  auto grid = costmap->getCostmap();
  grid->resizeMap(120, 120, .05, -3, -3);
  std::fill(grid->getCharMap(), grid->getCharMap() + 120 * 120, 0);
  unsigned int x, y;
  ASSERT_TRUE(grid->worldToMap(.9, 0, x, y));
  grid->setCost(x, y, 203);
  mppi::models::State state;
  state.reset(2, 30);
  state.pose.header.frame_id = "map";
  state.pose.pose.orientation.w = 1;
  // Index zero is deliberately the opposite of the actual output proxy index.
  state.cvx(0, 0) = 0;
  state.cvx(0, 1) = .8;
  state.cvx(1, 0) = .8;
  state.cvx(1, 1) = .2;
  mppi::models::Trajectories tr;
  tr.reset(2, 30);
  mppi::models::Path path;
  path.reset(2);
  xt::xtensor<float, 1> costs = xt::zeros<float>({2});
  float dt = .1;
  mppi::CriticData data{state, tr,      path,    costs,        dt,
                        false, nullptr, nullptr, std::nullopt, std::nullopt};
  critic->score(data);
  EXPECT_FALSE(data.fail_flag);
  EXPECT_EQ(costs(0), 10000);
  EXPECT_EQ(costs(1), 0);
  grid->setCost(x, y, 255);
  costs.fill(0);
  critic->score(data);
  EXPECT_EQ(costs(0), 10000);
  state.cvx(1, 1) = NAN;
  costs.fill(12);
  critic->score(data);
  EXPECT_TRUE(data.fail_flag);
  EXPECT_EQ(costs(0), 12); // No partial writes on a malformed batch.
  EXPECT_EQ(costs(1), 12);
}
TEST_F(StoppingPluginTest, MeasuredStoppingTailAndMalformedGrid) {
  auto grid = costmap->getCostmap();
  grid->resizeMap(120, 120, .05, -3, -3);
  std::fill(grid->getCharMap(), grid->getCharMap() + 120 * 120, 0);
  unsigned int x, y;
  ASSERT_TRUE(grid->worldToMap(.6, 0, x, y));
  grid->setCost(x, y, 203);
  mppi::models::State state;
  state.reset(1, 30);
  state.pose.header.frame_id = "map";
  state.pose.pose.orientation.w = 1;
  state.speed.linear.x = .8;
  mppi::models::Trajectories tr;
  tr.reset(1, 30);
  mppi::models::Path path;
  path.reset(2);
  xt::xtensor<float, 1> costs = xt::zeros<float>({1});
  float dt = .1;
  mppi::CriticData data{state, tr,      path,    costs,        dt,
                        false, nullptr, nullptr, std::nullopt, std::nullopt};
  critic->score(data);
  EXPECT_FALSE(data.fail_flag);
  EXPECT_EQ(costs(0), 10000); // Zero proposal does not erase measured momentum.
  dt = .2;
  critic->score(data);
  EXPECT_TRUE(data.fail_flag);
}

class SnapshotPluginTest : public PluginTest {
protected:
  std::string plugin_name() const override { return "NativeCycleSnapshotCritic"; }
  void prepare_parameters() override {
    char pattern[] = "/tmp/rm-native-snapshot-test-XXXXXX";
    const auto result = mkdtemp(pattern);
    ASSERT_NE(result, nullptr);
    directory = result;
    node->declare_parameter("FollowPath.critics",std::vector<std::string>{"NativeCycleSnapshotCritic"});
    node->declare_parameter("FollowPath.NativeCycleSnapshotCritic.output_directory",directory.string());
    node->declare_parameter("FollowPath.NativeCycleSnapshotCritic.capture_period",0.0);
    node->declare_parameter("FollowPath.NativeCycleSnapshotCritic.max_snapshots",1);
  }
  void TearDown() override {
    PluginTest::TearDown();
    if (!directory.empty()) std::filesystem::remove_all(directory);
  }
  void score_and_check(bool invalid_shape=false, bool original_fail=false) {
    auto grid=costmap->getCostmap();grid->resizeMap(2,3,.2,-.4,-.6);
    for (int i=0;i<6;++i) grid->getCharMap()[i]=uint8_t(200+i);
    mppi::models::State state;state.reset(2,30);
    state.pose.header.frame_id="map";state.pose.pose.orientation.w=1;
    state.speed.linear.x=.375;state.speed.linear.y=-.125;state.speed.angular.z=.25;
    mppi::models::Trajectories tr;tr.reset(2,30);
    for (size_t i=0;i<2;++i) for(size_t k=0;k<30;++k) {
      tr.x(i,k)=float(100*i+k)/8;tr.y(i,k)=-tr.x(i,k);tr.yaws(i,k)=.25f;
      state.vx(i,k)=k?float(k)/10:.375f;state.vy(i,k)=k?-.2f:-.125f;state.wz(i,k)=.25f;
      state.cvx(i,k)=float(i+k)/10;state.cvy(i,k)=-.125f;state.cwz(i,k)=.25f;
    }
    if(invalid_shape) state.cvx.resize({2,29});
    mppi::models::Path path;path.reset(2);
    xt::xtensor<float,1> costs={12.25f,-3.5f};const auto original_costs=costs;
    float dt=.1;const auto original_state=state;const auto original_tr=tr;
    mppi::CriticData data{state,tr,path,costs,dt,original_fail,nullptr,nullptr,std::nullopt,std::nullopt};
    critic->score(data);critic->score(data);
    EXPECT_EQ(data.fail_flag,original_fail);EXPECT_EQ(dt,.1f);
    EXPECT_EQ(0,std::memcmp(costs.data(),original_costs.data(),costs.size()*sizeof(float)));
    auto same=[](const auto &a,const auto &b) {return a.shape()==b.shape() && std::memcmp(a.data(),b.data(),a.size()*sizeof(float))==0;};
    EXPECT_TRUE(same(state.vx,original_state.vx));EXPECT_TRUE(same(state.vy,original_state.vy));EXPECT_TRUE(same(state.wz,original_state.wz));
    EXPECT_TRUE(same(state.cvx,original_state.cvx));EXPECT_TRUE(same(state.cvy,original_state.cvy));EXPECT_TRUE(same(state.cwz,original_state.cwz));
    EXPECT_TRUE(same(tr.x,original_tr.x));EXPECT_TRUE(same(tr.y,original_tr.y));EXPECT_TRUE(same(tr.yaws,original_tr.yaws));
    EXPECT_EQ(grid->getCost(1,2),205);EXPECT_EQ(state.speed.linear.x,.375);
  }
  std::filesystem::path directory;
};
TEST_F(SnapshotPluginTest, CompleteNativeDataAndCostsPreservedBitwise) {
  score_and_check();
  ASSERT_TRUE(std::filesystem::exists(directory/"cycle_0.json"));
  ASSERT_TRUE(std::filesystem::exists(directory/"cycle_0.bin"));
  EXPECT_FALSE(std::filesystem::exists(directory/"cycle_1.json"));
  std::ifstream file(directory/"cycle_0.bin",std::ios::binary);
  std::vector<float> xyz(180);file.read(reinterpret_cast<char*>(xyz.data()),xyz.size()*sizeof(float));
  EXPECT_FLOAT_EQ(xyz[0],0);EXPECT_FLOAT_EQ(xyz[29],29.f/8);EXPECT_FLOAT_EQ(xyz[30],12.5);
  EXPECT_FLOAT_EQ(xyz[60+29],-29.f/8);EXPECT_FLOAT_EQ(xyz[120+29],.25f);
  file.seekg(9*60*sizeof(float));float values[2];file.read(reinterpret_cast<char*>(values),sizeof(values));
  EXPECT_FLOAT_EQ(values[0],12.25);EXPECT_FLOAT_EQ(values[1],-3.5);
  file.seekg(-6,std::ios::end);char cells[6];file.read(cells,6);
  for(int i=0;i<6;++i) EXPECT_EQ(static_cast<unsigned char>(cells[i]),200+i);
  if (const char *output = std::getenv("RM_NATIVE_SNAPSHOT_TEST_OUTPUT")) {
    std::filesystem::create_directories(output);
    for (const auto &name : {"cycle_0.json","cycle_0.bin"})
      std::filesystem::copy_file(directory/name,std::filesystem::path(output)/name);
  }
}
TEST_F(SnapshotPluginTest, MalformedStateOnlyStopsEvidence) {
  score_and_check(true);
  EXPECT_FALSE(std::filesystem::exists(directory/"cycle_0.json"));
  EXPECT_FALSE(std::filesystem::exists(directory/"cycle_0.bin"));
}
TEST_F(SnapshotPluginTest, ExistingFailureFlagIsNeverCleared) {
  score_and_check(false,true);
  ASSERT_TRUE(std::filesystem::exists(directory/"cycle_0.json"));
}
TEST_F(SnapshotPluginTest, ExistingEvidenceIsNeverOverwrittenAndControlUnchanged) {
  {std::ofstream file(directory/"cycle_0.json");file<<"preserve";}
  score_and_check();
  std::ifstream file(directory/"cycle_0.json");std::string text;file>>text;EXPECT_EQ(text,"preserve");
  EXPECT_FALSE(std::filesystem::exists(directory/"cycle_0.bin"));
}
class DisabledSnapshotPluginTest : public SnapshotPluginTest {
  void prepare_parameters() override {
    SnapshotPluginTest::prepare_parameters();
    node->declare_parameter("FollowPath.NativeCycleSnapshotCritic.enabled",false);
  }
};
TEST_F(DisabledSnapshotPluginTest, DisabledCaptureHasNoSideEffects) {
  score_and_check();EXPECT_TRUE(std::filesystem::is_empty(directory));
}

class DynamicEvidencePluginTest : public PluginTest {
protected:
  std::filesystem::path directory;
  virtual int evidence_budget() const {return 4;}
  void prepare_parameters() override {
    char pattern[]="/tmp/rm-dynamic-consumption-test-XXXXXX";
    const auto result=mkdtemp(pattern);
    ASSERT_NE(result,nullptr);
    directory=result;
    node->declare_parameter("FollowPath.DynamicObstacleCritic.consumption_evidence_directory",directory.string());
    node->declare_parameter("FollowPath.DynamicObstacleCritic.consumption_evidence_max_records",evidence_budget());
  }
  void TearDown() override {
    if(const char *destination=std::getenv("RM_DYNAMIC_CONSUMPTION_FIXTURE_DIR")) {
      const auto name=::testing::UnitTest::GetInstance()->current_test_info()->name();
      const auto target=std::filesystem::path(destination)/name;
      std::filesystem::create_directories(target);
      std::filesystem::copy(directory,target,std::filesystem::copy_options::recursive);
    }
    PluginTest::TearDown();std::filesystem::remove_all(directory);
  }
  void pin_clock(int seconds=100) {
    auto clock=node->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock,int64_t(seconds)*1'000'000'000),RCL_RET_OK);
  }
  void setup_data(mppi::models::State &state,mppi::models::Trajectories &tr,mppi::models::Path &path) {
    state.reset(2,30);tr.reset(2,30);path.reset(2);
    state.pose.header.frame_id="map";state.pose.header.stamp=node->now();state.pose.pose.orientation.w=1;
    state.speed.linear.x=.15;state.speed.linear.y=.02;state.speed.angular.z=.1;
    for(size_t k=0;k<30;++k)tr.x(0,k)=float(k+1)*.1f;
  }
};
class OneDynamicEvidencePluginTest : public DynamicEvidencePluginTest {
  int evidence_budget() const override {return 1;}
};
TEST_F(OneDynamicEvidencePluginTest, MovingInputWithCaptureAndBudgetExhaustionIsBitExact) {
  pin_clock();publish(input());
  mppi::models::State state;mppi::models::Trajectories tr;mppi::models::Path path;setup_data(state,tr,path);
  xt::xtensor<float,1> costs={10.f,20.f};float dt=.1f;
  mppi::CriticData data{state,tr,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};
  const auto original_state=state;const auto original_tr=tr;
  critic->score(data);ASSERT_FALSE(data.fail_flag);EXPECT_GT(costs(0),1000);
  const auto captured_costs=costs;
  ASSERT_TRUE(std::filesystem::exists(directory/"score_0.json"));
  costs={10.f,20.f};critic->score(data);EXPECT_FALSE(data.fail_flag);
  EXPECT_EQ(std::memcmp(costs.data(),captured_costs.data(),costs.size()*4),0);
  EXPECT_FALSE(std::filesystem::exists(directory/"score_1.json"));
  EXPECT_EQ(std::memcmp(tr.x.data(),original_tr.x.data(),tr.x.size()*4),0);
  EXPECT_EQ(std::memcmp(state.cvx.data(),original_state.cvx.data(),state.cvx.size()*4),0);
  EXPECT_EQ(state.speed.linear.x,.15);
}
TEST_F(DynamicEvidencePluginTest, EmptyInputIsRecordedAndStaleReplacementStillFailsClosed) {
  pin_clock();publish(input());
  mppi::models::State state;mppi::models::Trajectories tr;mppi::models::Path path;setup_data(state,tr,path);
  xt::xtensor<float,1> costs={10.f,20.f};float dt=.1f;
  mppi::CriticData data{state,tr,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};
  critic->score(data);ASSERT_FALSE(data.fail_flag);
  auto stale=input();stale.header.stamp.sec-=2;publish(stale);costs={10.f,20.f};critic->score(data);
  EXPECT_TRUE(data.fail_flag);EXPECT_EQ(costs(0),10.f);EXPECT_FALSE(std::filesystem::exists(directory/"score_1.json"));
  pin_clock(101);auto empty=input();empty.tracks.clear();empty.total_track_count=0;publish(empty);
  data.fail_flag=false;costs={10.f,20.f};critic->score(data);EXPECT_FALSE(data.fail_flag);
  EXPECT_EQ(costs(0),10.f);EXPECT_EQ(costs(1),20.f);EXPECT_TRUE(std::filesystem::exists(directory/"score_1.json"));
}
TEST_F(DynamicEvidencePluginTest, ExistingMetadataStopsEvidenceWithoutChangingCostsOrFailureFlag) {
  pin_clock();publish(input());
  {std::ofstream file(directory/"score_0.json");file<<"preserve";}
  mppi::models::State state;mppi::models::Trajectories tr;mppi::models::Path path;setup_data(state,tr,path);
  xt::xtensor<float,1> costs={10.f,20.f};float dt=.1f;
  mppi::CriticData data{state,tr,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};
  critic->score(data);EXPECT_FALSE(data.fail_flag);const auto expected=costs;
  costs={10.f,20.f};critic->score(data);EXPECT_FALSE(data.fail_flag);
  EXPECT_EQ(std::memcmp(costs.data(),expected.data(),costs.size()*4),0);
  std::ifstream file(directory/"score_0.json");std::string value;file>>value;EXPECT_EQ(value,"preserve");
  EXPECT_FALSE(std::filesystem::exists(directory/"score_0.bin"));
}
TEST_F(OneDynamicEvidencePluginTest, StaticInputHasIdenticalCostsWithAndWithoutRecording) {
  pin_clock();auto stationary=input();stationary.tracks[0].velocity.y=0;publish(stationary);
  mppi::models::State state;mppi::models::Trajectories tr;mppi::models::Path path;setup_data(state,tr,path);
  xt::xtensor<float,1> costs={10.f,20.f};float dt=.1f;
  mppi::CriticData data{state,tr,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};
  critic->score(data);ASSERT_FALSE(data.fail_flag);const auto expected=costs;
  ASSERT_TRUE(std::filesystem::exists(directory/"score_0.json"));
  costs={10.f,20.f};critic->score(data);EXPECT_FALSE(data.fail_flag);
  EXPECT_EQ(std::memcmp(costs.data(),expected.data(),costs.size()*4),0);
}
class DisabledDynamicEvidencePluginTest : public DynamicEvidencePluginTest {
  void prepare_parameters() override {
    DynamicEvidencePluginTest::prepare_parameters();
    node->declare_parameter("FollowPath.DynamicObstacleCritic.enabled",false);
  }
};
TEST_F(DisabledDynamicEvidencePluginTest, DisabledCriticCreatesNoEvidenceOrCostChanges) {
  pin_clock();
  mppi::models::State state;mppi::models::Trajectories tr;mppi::models::Path path;setup_data(state,tr,path);
  xt::xtensor<float,1> costs={10.f,20.f};float dt=.1f;
  mppi::CriticData data{state,tr,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};
  critic->score(data);EXPECT_FALSE(data.fail_flag);EXPECT_EQ(costs(0),10.f);EXPECT_EQ(costs(1),20.f);
  EXPECT_TRUE(std::filesystem::is_empty(directory));
}
