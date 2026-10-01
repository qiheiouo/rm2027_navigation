// Offline native-plugin microbenchmark. Synthetic controls are NOT sampled
// historical rollouts, SG outputs or safety/coverage witnesses.
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_loader.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>

int main(int argc, char **argv) {
  if (argc != 2)
    throw std::invalid_argument("usage: benchmark_stopping_plugin OUTPUT.jsonl");
  std::ofstream out(argv[1]);
  if (!out)
    throw std::runtime_error("cannot create benchmark output");
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  rclcpp::init(argc, argv);
  constexpr size_t batch = 300, steps = 30, repeats = 7;
  constexpr unsigned int seed = 712824;
  for (size_t mode = 0; mode < 3; ++mode) {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "offline_stopping_benchmark");
    auto costmap =
        std::make_shared<nav2_costmap_2d::Costmap2DROS>("benchmark_costmap");
    costmap->set_parameter(rclcpp::Parameter("global_frame", "odom"));
    costmap->set_parameter(rclcpp::Parameter("footprint_padding", 0.));
    costmap->set_parameter(
        rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap->configure();
    std::vector<geometry_msgs::msg::Point> fp(4);
    fp[0].x = -.33; fp[0].y = -.28;
    fp[1].x = .33; fp[1].y = -.28;
    fp[2].x = .33; fp[2].y = .28;
    fp[3].x = -.33; fp[3].y = .28;
    costmap->setRobotFootprint(fp);
    const std::string name = "FollowPath.StaticStoppingCritic";
    node->declare_parameter(name + ".map_uncertainty_margin", mode == 0 ? 0. : .11);
    node->declare_parameter(name + ".map_uncertainty_mode",
                            mode == 2 ? std::string("soft") : std::string("hard"));
    auto handler = std::make_unique<mppi::ParametersHandler>(node);
    pluginlib::ClassLoader<mppi::critics::CriticFunction> loader(
        "nav2_mppi_controller", "mppi::critics::CriticFunction");
    auto critic = loader.createSharedInstance("mppi::critics::StaticStoppingCritic");
    critic->on_configure(node, "FollowPath", name, costmap, handler.get());
    if (mode == 0) {
      // Prove which library the identical executable actually loaded.
      std::ifstream mappings("/proc/self/maps");
      std::string line, loaded;
      while (std::getline(mappings, line))
        if (line.find("librm_dynamic_obstacle_critic.so") != std::string::npos) {
          loaded = line.substr(line.find('/'));
          break;
        }
      if (loaded.empty())
        throw std::runtime_error("benchmark plugin mapping not found");
      out << "{\"kind\":\"library_mapping\",\"path\":\"" << loaded
          << "\",\"footprint_padding\":0,\"footprint\":[[-0.33,-0.28],"
             "[0.33,-0.28],[0.33,0.28],[-0.33,0.28]]}\n";
    }
    for (size_t map_case = 0; map_case < 4; ++map_case) {
      auto grid = costmap->getCostmap();
      grid->resizeMap(120, 120, .05, -3, -3);
      std::fill(grid->getCharMap(), grid->getCharMap() + 120 * 120, 0);
      // Exact indexed masks; no ray labels or hidden physics dimensions.
      if (map_case == 1)
        for (size_t y = 43; y <= 77; ++y)
          for (size_t x = 69; x <= 72; ++x)
            grid->setCost(x, y, x == 69 ? 204 : 213);
      if (map_case == 2)
        for (size_t y = 48; y <= 81; ++y)
          for (size_t x = 73; x <= 80; ++x)
            if ((x + y) % 3 != 0)
              grid->setCost(x, y, (x + y) % 7 == 0 ? 255 : 223);
      if (map_case == 3)
        grid->setCost(60, 60, 255);
      for (size_t rotation = 0; rotation < 2; ++rotation)
        for (size_t motion = 0; motion < 3; ++motion) {
          mppi::models::State state;
          state.reset(batch, steps);
          state.pose.header.frame_id = "odom";
          const double yaw = rotation == 0 ? 0. : .41;
          state.pose.pose.orientation.z = std::sin(yaw / 2);
          state.pose.pose.orientation.w = std::cos(yaw / 2);
          state.speed.linear.x = motion == 0 ? 0. : motion == 1 ? .15 : .8;
          state.speed.linear.y = motion == 1 ? .05 : 0.;
          state.speed.angular.z = motion == 1 ? .2 : 0.;
          std::mt19937 rng(seed);
          std::normal_distribution<float> vx(.35f, .2f), vy(0.f, .2f), wz(0.f, .4f);
          for (size_t i = 0; i < batch; ++i)
            for (size_t k = 0; k < steps; ++k) {
              state.cvx(i, k) = vx(rng);
              state.cvy(i, k) = vy(rng);
              state.cwz(i, k) = wz(rng);
            }
          // Explicit clipping and escape/wait/approach controls in every batch.
          state.cvx(0, 1) = -2; state.cvy(0, 1) = -2; state.cwz(0, 1) = -2;
          state.cvx(1, 1) = 2; state.cvy(1, 1) = 2; state.cwz(1, 1) = 2;
          state.cvx(2, 1) = -.2; state.cvy(2, 1) = 0; state.cwz(2, 1) = 0;
          state.cvx(3, 1) = 0; state.cvy(3, 1) = 0; state.cwz(3, 1) = 0;
          state.cvx(4, 1) = .2; state.cvy(4, 1) = 0; state.cwz(4, 1) = 0;
          mppi::models::Trajectories trajectories;
          trajectories.reset(batch, steps);
          mppi::models::Path path;
          path.reset(2);
          xt::xtensor<float, 1> costs = xt::zeros<float>({batch});
          float dt = .1;
          mppi::CriticData data{state, trajectories, path, costs, dt, false,
                              nullptr, nullptr, std::nullopt, std::nullopt};
          std::vector<double> times;
          std::vector<float> first_costs;
          // Warmup avoids charging one-time resource/diagnostic initialization.
          critic->score(data);
          if (data.fail_flag)
            throw std::runtime_error("benchmark critic failed warmup");
          for (size_t repeat = 0; repeat < repeats; ++repeat) {
            for (size_t i = 0; i < batch; ++i)
              costs(i) = static_cast<float>((i * 17) % 97) + .25f;
            const auto begin = std::chrono::steady_clock::now();
            critic->score(data);
            times.push_back(std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - begin).count());
            if (data.fail_flag)
              throw std::runtime_error("benchmark critic failed score");
            std::vector<float> current(costs.begin(), costs.end());
            if (repeat == 0)
              first_costs = current;
            else if (current != first_costs)
              throw std::runtime_error("benchmark cost not deterministic");
          }
          out << "{\"mode\":" << mode << ",\"map_case\":" << map_case
              << ",\"rotation\":" << rotation << ",\"motion\":" << motion
              << ",\"seed\":" << seed << ",\"batch\":" << batch
              << ",\"steps\":" << steps << ",\"score_ms\":[";
          for (size_t i = 0; i < times.size(); ++i)
            out << (i ? "," : "") << times[i];
          out << "],\"costs\":[";
          for (size_t i = 0; i < first_costs.size(); ++i)
            out << (i ? "," : "") << first_costs[i];
          out << "]}\n";
        }
    }
    critic.reset();
    handler.reset();
    costmap->cleanup();
  }
  rclcpp::shutdown();
}
