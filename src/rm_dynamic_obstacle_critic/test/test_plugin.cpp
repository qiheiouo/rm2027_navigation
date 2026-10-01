#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_loader.hpp"
#include "rm_dynamic_obstacle_critic/model.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <gtest/gtest.h>
#include <thread>

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
