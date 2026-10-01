#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rm_dynamic_obstacle_critic/native_snapshot.hpp"
#include <chrono>

namespace mppi::critics {
class NativeCycleSnapshotCritic final : public CriticFunction {
public:
  void initialize() override {
    auto get = parameters_handler_->getParamGetter(name_);
    get(directory_,"output_directory",std::string(""),ParameterType::Static);
    get(period_,"capture_period",0.1,ParameterType::Static);
    get(max_snapshots_,"max_snapshots",400,ParameterType::Static);
    if (!std::isfinite(period_) || period_ < 0 || max_snapshots_ < 1 || max_snapshots_ > 1000)
      throw std::invalid_argument("invalid native snapshot budget");
    if (directory_.empty() || !enabled_) return;
    auto parent_get = parameters_handler_->getParamGetter(parent_name_);
    std::vector<std::string> critics;
    parent_get(critics,"critics",std::vector<std::string>{},ParameterType::Static);
    if (critics.empty() || critics.back() != "NativeCycleSnapshotCritic")
      throw std::invalid_argument("native snapshot plugin must be the last listed critic");
    const std::filesystem::path path(directory_);
    if (!path.is_absolute() || (std::filesystem::exists(path) &&
        (!std::filesystem::is_directory(path) || !std::filesystem::is_empty(path))))
      throw std::invalid_argument("native evidence needs an absolute empty/new directory");
    std::filesystem::create_directories(path);
    // Atomic ownership even if two nodes are accidentally configured for the
    // same empty directory. The marker remains with the frozen evidence.
    if (!std::filesystem::create_directory(path / "writer.lock"))
      throw std::invalid_argument("native evidence directory already owned");
  }
  void score(CriticData &data) override {
    if (!enabled_ || directory_.empty() || stopped_ || count_ >= size_t(max_snapshots_)) return;
    auto node = parent_.lock();
    if (!node) return;
    const double stamp = node->now().seconds();
    if (last_stamp_ >= 0 && stamp < last_stamp_) {
      stopped_ = true;
      RCLCPP_ERROR(logger_,"Native snapshot clock moved backwards; capture stopped");
      return;
    }
    if (last_stamp_ >= 0 && stamp-last_stamp_+1e-9 < period_) return;
    const auto begin = std::chrono::steady_clock::now();
    try {
      rm_dynamic_obstacle_critic::write_native_snapshot(directory_,count_,stamp,data,*costmap_ros_);
      ++count_; last_stamp_ = stamp;
      const double milliseconds = std::chrono::duration<double,std::milli>(
          std::chrono::steady_clock::now()-begin).count();
      RCLCPP_INFO_THROTTLE(logger_,*node->get_clock(),1000,
          "NativeSnapshot ordinal=%zu capture_ms=%.9f",count_-1,milliseconds);
      if (count_ == size_t(max_snapshots_))
        RCLCPP_INFO(logger_,"Native snapshot capture budget exhausted (%zu)",count_);
    } catch (const std::exception &e) {
      stopped_ = true;
      RCLCPP_ERROR(logger_,"Native evidence capture stopped: %s",e.what());
    }
    // Instrumentation never mutates data, including costs and fail_flag.
  }
private:
  std::string directory_;
  double period_{0.1},last_stamp_{-1};
  int max_snapshots_{400};
  size_t count_{0};
  bool stopped_{false};
};
} // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::NativeCycleSnapshotCritic,mppi::critics::CriticFunction)
