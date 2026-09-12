#include "rm_tdt_planner/planner.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "pluginlib/class_loader.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <cstdlib>

namespace
{
class PluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("tdt_test");
    map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("tdt_test_costmap");
    map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    map->set_parameter(rclcpp::Parameter("footprint", "[[-0.32,-0.27],[-0.32,0.27],[0.32,0.27],[0.32,-0.27]]"));
    map->on_configure(rclcpp_lifecycle::State());
    map->getCostmap()->resizeMap(100, 80, 0.1, -3.25, -1.75);
    std::fill(map->getCostmap()->getCharMap(), map->getCostmap()->getCharMap()+8000, 0);
    loader = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
      "nav2_core", "nav2_core::GlobalPlanner");
    planner = loader->createSharedInstance("rm_tdt_planner/TdtGlobalPlanner");
    planner->configure(node, "GridBased", {}, map);
    planner->activate();
  }
  void TearDown() override
  {
    planner->deactivate(); planner->cleanup(); planner.reset(); loader.reset();
    map->on_cleanup(rclcpp_lifecycle::State()); map.reset(); node.reset();
  }
  geometry_msgs::msg::PoseStamped pose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = -3.25+x; p.pose.position.y = -1.75+y;
    p.pose.orientation.w = 1;
    return p;
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader;
  nav2_core::GlobalPlanner::Ptr planner;
};
TEST_F(PluginTest, LoadLifecycleAndEndpointContract)
{
  const auto start = pose(1.13, 1.27), goal = pose(8.31, 6.24);
  auto p = planner->createPlan(start, goal);
  ASSERT_GT(p.poses.size(), 2u); EXPECT_EQ(p.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(p.poses.front().pose.position.x, start.pose.position.x);
  EXPECT_DOUBLE_EQ(p.poses.back().pose.position.y, goal.pose.position.y);
  EXPECT_DOUBLE_EQ(p.poses.back().pose.orientation.w, goal.pose.orientation.w);
  auto bad = goal; bad.header.frame_id = "odom";
  EXPECT_THROW(planner->createPlan(start, bad), nav2_core::PlannerException);
  bad = goal; bad.pose.orientation.w = 0;
  EXPECT_THROW(planner->createPlan(start, bad), nav2_core::PlannerException);
  planner->deactivate();
  EXPECT_THROW(planner->createPlan(start, goal), nav2_core::PlannerException);
  planner->activate();
  EXPECT_NO_THROW(planner->createPlan(start, goal));
}
TEST_F(PluginTest, MapUpdateIsUsedAndNoVelocityPublisher)
{
  auto start = pose(1, 1), goal = pose(8, 1);
  EXPECT_NO_THROW(planner->createPlan(start, goal));
  for (int y = 0; y < 80; ++y) {map->getCostmap()->setCost(50, y, 254);}
  EXPECT_THROW(planner->createPlan(start, goal), nav2_core::PlannerException);
  const auto topics = node->get_topic_names_and_types();
  EXPECT_EQ(topics.count("/cmd_vel"), 0u);
}
TEST_F(PluginTest, NavfnFrontendAndBackendSameSnapshotBenchmark)
{
  nav2_navfn_planner::NavfnPlanner navfn;
  navfn.configure(node, "NavfnBaseline", {}, map); navfn.activate();
  auto frontend = loader->createSharedInstance("rm_tdt_planner/TdtGlobalPlanner");
  node->declare_parameter("Frontend.optimize", false);
  frontend->configure(node, "Frontend", {}, map); frontend->activate();
  std::ofstream csv;
  if (const char * path = std::getenv("TDT_BENCHMARK_CSV")) {
    csv.open(path); ASSERT_TRUE(csv.is_open());
    csv << "scenario,trial,planner,success,elapsed_ms,length_m,samples\n";
  }
  for (int scenario = 0; scenario < 2; ++scenario) {
    if (scenario) {
      // Include a soft inflation field in the same snapshot seen by all three.
      for (int y = 0; y < 55; ++y) {
        for (int x = 43; x <= 57; ++x) {
          map->getCostmap()->setCost(x, y, x == 50 ? 254 : 200-20*std::abs(x-50));
        }
      }
    }
    const auto s = pose(1.13, 1.27), t = pose(8.31, 2.24);
    std::vector<std::pair<std::string, nav2_core::GlobalPlanner *>> planners{
      {"navfn_dijkstra", &navfn}, {"tdt_astar", frontend.get()}, {"tdt_jerk", planner.get()}};
    for (int trial = 0; trial < 20; ++trial) {
      for (const auto & entry : planners) {
        auto begin = std::chrono::steady_clock::now();
        nav_msgs::msg::Path p;
        try {p = entry.second->createPlan(s, t);} catch (const std::exception &) {}
        double elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now()-begin).count();
        double length = 0;
        for (size_t i = 1; i < p.poses.size(); ++i) {
          const auto & a = p.poses[i-1].pose.position;
          const auto & b = p.poses[i].pose.position;
          length += std::hypot(a.x-b.x, a.y-b.y);
        }
        EXPECT_FALSE(p.poses.empty()) << entry.first << " scenario=" << scenario;
        if (csv.is_open()) {
          csv << scenario << ',' << trial << ',' << entry.first << ',' << !p.poses.empty()
              << ',' << elapsed << ',' << length << ',' << p.poses.size() << '\n';
        }
      }
    }
  }
  frontend->deactivate(); frontend->cleanup();
  navfn.deactivate(); navfn.cleanup();
}
}  // namespace
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
