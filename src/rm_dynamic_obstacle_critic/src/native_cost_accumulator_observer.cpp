#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rm_dynamic_obstacle_critic/cost_accumulator_evidence.hpp"
#include <chrono>
#include <mutex>
#include <map>
#include <tuple>

namespace mppi::critics {
namespace {
struct CostObservationPairState {
  std::mutex mutex;
  std::optional<size_t> pending_before;
  const CriticData *pending_data{nullptr};
};
std::mutex registry_mutex;
using PairKey=std::tuple<const void *,std::string,const void *>;
std::map<PairKey,std::weak_ptr<CostObservationPairState>> registry;
std::shared_ptr<CostObservationPairState> pair_state(const PairKey &key){
  std::lock_guard<std::mutex> lock(registry_mutex);
  for(auto it=registry.begin();it!=registry.end();)if(it->second.expired())it=registry.erase(it);else ++it;
  auto result=registry[key].lock();
  if(!result){result=std::make_shared<CostObservationPairState>();registry[key]=result;}
  return result;
}
} // namespace
class CostAccumulatorObserver : public CriticFunction {
protected:
  explicit CostAccumulatorObserver(bool before):before_(before){}
public:
  void initialize() override {
    auto get=parameters_handler_->getParamGetter(name_);
    get(capture_,"capture_enabled",false,ParameterType::Static);
    get(directory_,"output_directory",std::string(""),ParameterType::Static);
    get(max_records_,"max_records",1000,ParameterType::Static);
    if(!capture_)return;
    if(!enabled_ || !costmap_ros_ || max_records_<1 || max_records_>1000)
      throw std::invalid_argument("enabled paired cost observer needs actual costmap and bounded records");
    const std::string own=before_?"BeforeCostAccumulatorCritic":"AfterCostAccumulatorCritic";
    const std::string peer=before_?"AfterCostAccumulatorCritic":"BeforeCostAccumulatorCritic";
    if(name_!=parent_name_+"."+own)throw std::invalid_argument("cost observer role/name mismatch");
    auto parent_get=parameters_handler_->getParamGetter(parent_name_);std::vector<std::string> critics;
    parent_get(critics,"critics",std::vector<std::string>{},ParameterType::Static);
    const std::vector<std::string> expected{"ConstraintCritic","BeforeCostAccumulatorCritic","CostCritic","AfterCostAccumulatorCritic",
      "GoalCritic","GoalAngleCritic","PathAlignCritic","PathFollowCritic","PathAngleCritic","DynamicObstacleCritic","StaticStoppingCritic","NativeCycleSnapshotCritic"};
    if(critics!=expected)throw std::invalid_argument("cost observers must be adjacent to unchanged original CostCritic order");
    auto peer_get=parameters_handler_->getParamGetter(parent_name_+"."+peer);bool peer_capture=false,peer_enabled=true;int peer_max=1000;std::string peer_directory;
    peer_get(peer_capture,"capture_enabled",false,ParameterType::Static);peer_get(peer_enabled,"enabled",true,ParameterType::Static);
    peer_get(peer_max,"max_records",1000,ParameterType::Static);peer_get(peer_directory,"output_directory",std::string(""),ParameterType::Static);
    if(!peer_capture || !peer_enabled || peer_max!=max_records_ || peer_directory.empty())throw std::invalid_argument("both cost observers require matching enabled budgets");
    const std::filesystem::path path(directory_),peer_path(peer_directory);
    if(!path.is_absolute() || !peer_path.is_absolute() || path!=path.lexically_normal() || peer_path!=peer_path.lexically_normal() || path==peer_path ||
        (std::filesystem::exists(path) && (!std::filesystem::is_directory(path) || !std::filesystem::is_empty(path))))
      throw std::invalid_argument("cost observer needs distinct absolute empty/new directories");
    std::filesystem::create_directories(path);
    if(!std::filesystem::create_directory(path/"writer.lock"))throw std::invalid_argument("cost observation directory already owned");
    auto node=parent_.lock();if(!node)throw std::invalid_argument("cost observer actual parent unavailable");
    pair_=pair_state({node.get(),parent_name_,costmap_ros_.get()});
  }
  void score(CriticData &data) override {
    // Reset/consume on every callback, including budget exhaustion and stopped
    // capture, so no earlier token can be reused by a later after callback.
    std::optional<size_t> token;
    if(pair_){
      std::lock_guard<std::mutex> lock(pair_->mutex);
      if(!before_ && pair_->pending_data==&data)token=pair_->pending_before;
      pair_->pending_before.reset();
      pair_->pending_data=nullptr;
    }
    if(!enabled_ || !capture_ || stopped_ || count_>=size_t(max_records_))return;
    auto node=parent_.lock();if(!node)return;
    const double stamp=node->now().seconds();const auto begin=std::chrono::steady_clock::now();
    try {
      if(last_stamp_>=0 && stamp<last_stamp_)throw std::runtime_error("cost observation clock moved backwards");
      rm_dynamic_obstacle_critic::CostAccumulatorContext context;
      context.stage=before_?"before_CostCritic":"after_CostCritic";
      context.paired_before_ordinal=before_?std::optional<size_t>(count_):token;
      context.evaluation_frame=costmap_ros_->getGlobalFrameID();context.base_frame=costmap_ros_->getBaseFrameID();
      auto *grid=costmap_ros_->getCostmap();if(!grid)throw std::runtime_error("cost observation actual map unavailable");
      {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
        context.map_size={grid->getSizeInCellsX(),grid->getSizeInCellsY()};
        const size_t cells=context.map_size[0]*context.map_size[1];
        if(!cells || cells>65536)throw std::runtime_error("cost observation map budget");
        context.origin={grid->getOriginX(),grid->getOriginY()};context.resolution=grid->getResolution();
        context.map_bytes.assign(grid->getCharMap(),grid->getCharMap()+cells);
        for(const auto &p:costmap_ros_->getRobotFootprint())context.padded_footprint.push_back({p.x,p.y});
        context.map_snapshot_mutex_locked=true;
      }
      rm_dynamic_obstacle_critic::write_cost_accumulator_observation(directory_,count_,stamp,data,context);
      if(before_){std::lock_guard<std::mutex> lock(pair_->mutex);pair_->pending_before=count_;pair_->pending_data=&data;}
      last_stamp_=stamp;++count_;
      const double milliseconds=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      // Separate call sites give each role its own sampled timing throttle.
      if(before_){
        RCLCPP_INFO_THROTTLE(logger_,*node->get_clock(),1000,"CostAccumulator stage=before_CostCritic ordinal=%zu capture_ms=%.9f",count_-1,milliseconds);
      }else{
        RCLCPP_INFO_THROTTLE(logger_,*node->get_clock(),1000,"CostAccumulator stage=after_CostCritic ordinal=%zu capture_ms=%.9f",count_-1,milliseconds);
      }
    }catch(const std::exception &e){stopped_=true;RCLCPP_ERROR(logger_,"Cost observation stopped: %s",e.what());}
    // No mutation, fail flag, subscriber, publisher, goal or control side effect.
  }
private:
  bool before_,capture_{false},stopped_{false};std::string directory_;int max_records_{1000};size_t count_{0};double last_stamp_{-1};
  std::shared_ptr<CostObservationPairState> pair_;
};
class BeforeCostAccumulatorCritic final : public CostAccumulatorObserver {
public:BeforeCostAccumulatorCritic():CostAccumulatorObserver(true){}
};
class AfterCostAccumulatorCritic final : public CostAccumulatorObserver {
public:AfterCostAccumulatorCritic():CostAccumulatorObserver(false){}
};
} // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::BeforeCostAccumulatorCritic,mppi::critics::CriticFunction)
PLUGINLIB_EXPORT_CLASS(mppi::critics::AfterCostAccumulatorCritic,mppi::critics::CriticFunction)
