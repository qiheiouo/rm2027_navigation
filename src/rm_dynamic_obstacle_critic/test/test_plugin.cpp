#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_loader.hpp"
#include "rm_dynamic_obstacle_critic/model.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <gtest/gtest.h>
#include <thread>

class PluginTest : public ::testing::Test {
protected:
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
    handler = std::make_unique<mppi::ParametersHandler>(node);
    loader =
        std::make_unique<pluginlib::ClassLoader<mppi::critics::CriticFunction>>(
            "nav2_mppi_controller", "mppi::critics::CriticFunction");
    critic =
        loader->createSharedInstance("mppi::critics::DynamicObstacleCritic");
    critic->on_configure(node, "FollowPath", "FollowPath.DynamicObstacleCritic",
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
