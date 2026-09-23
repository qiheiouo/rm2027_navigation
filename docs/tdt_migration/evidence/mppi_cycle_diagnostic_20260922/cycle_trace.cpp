#include "nav2_mppi_controller/cycle_trace.hpp"
#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <exception>
#include <tf2/utils.h>
namespace tdt_trace {
int64_t steady_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Packet {rclcpp::Clock::SharedPtr clock; Json meta; std::vector<uint8_t> bytes; int64_t observer_ns=0; int exceptions=0;};
thread_local std::unique_ptr<Packet> current;
struct Writer {
  std::string directory;
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<std::unique_ptr<Packet>> queue;
  std::thread thread;
  bool stopping=false;
  std::atomic<uint64_t> next{0}, dropped{0}, errors{0}, written{0};
  Writer() {
    auto env=std::getenv("TDT_MPPI_TRACE_DIR");
    if (!env || !*env) return;
    directory=env;
    if (!std::filesystem::create_directory(directory)) throw std::runtime_error("trace directory already exists");
    std::ifstream maps("/proc/self/maps");std::ofstream provenance(directory+"/loaded_maps.txt");provenance<<maps.rdbuf();
    thread=std::thread([this] {loop();});
  }
  void loop() noexcept {
    for (;;) {
      std::unique_ptr<Packet> p;
      {std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[this]{return stopping||!queue.empty();});
       if(queue.empty()) {break;} p=std::move(queue.front());queue.pop_front();}
      try {
        auto begin=steady_ns();std::string stem=directory+"/cycle_"+std::to_string(p->meta["cycle_id"].get<uint64_t>());
        std::ofstream bin(stem+".bin",std::ios::binary);bin.exceptions(std::ios::badbit|std::ios::failbit);
        bin.write(reinterpret_cast<const char*>(p->bytes.data()),p->bytes.size());bin.close();
        p->meta["writer_ns"]=steady_ns()-begin;
        std::ofstream meta(stem+".json");meta.exceptions(std::ios::badbit|std::ios::failbit);meta<<p->meta.dump()<<'\n';meta.close();++written;
      } catch (...) {++errors;}
    }
  }
  void close() noexcept {
    {std::lock_guard<std::mutex> lock(mutex);stopping=true;}cv.notify_one();if(thread.joinable())thread.join();
    if(directory.empty())return;
    try {std::ofstream f(directory+"/writer_status.json");f<<Json{{"attempted",next.load()},{"written",written.load()},{"dropped",dropped.load()},{"errors",errors.load()},{"closed",true}}.dump()<<'\n';}catch(...){}
  }
  ~Writer(){close();}
};
Writer & writer() {static Writer w;return w;}
bool active() noexcept {return bool(current);}
void value(const std::string & key,const Json & v) noexcept {
  if(!current) {return;} auto start=steady_ns();try {current->meta["events"].push_back({{"kind",key},{"steady_ns",start},{"value",v}});}catch(...){++writer().errors;}current->observer_ns+=steady_ns()-start;
}
void stage(const std::string & name) noexcept {value("stage",name);}
void blob(const std::string & key,const void * data,size_t bytes,const std::string & dtype,const std::vector<size_t> & shape) noexcept {
  if(!current) {return;} auto start=steady_ns();try {
    auto offset=current->bytes.size();auto p=static_cast<const uint8_t*>(data);current->bytes.insert(current->bytes.end(),p,p+bytes);
    current->meta["events"].push_back({{"kind","array"},{"name",key},{"offset",offset},{"bytes",bytes},{"dtype",dtype},{"shape",shape},{"steady_ns",start}});
  }catch(...){++writer().errors;}current->observer_ns+=steady_ns()-start;
}
void controls(const std::string & name,const mppi::models::ControlSequence & c) noexcept {
  stage(name);tensor(name+".vx",c.vx);tensor(name+".vy",c.vy);tensor(name+".wz",c.wz);
}
void scored(const mppi::CriticData & d) noexcept {
  stage("scored");tensor("rollout.x",d.trajectories.x);tensor("rollout.y",d.trajectories.y);tensor("rollout.yaw",d.trajectories.yaws);tensor("scored.costs",d.costs);value("scored.fail",d.fail_flag);
}
void collision_mask(const std::vector<uint8_t> & v,bool near) noexcept {blob("cost_critic.collisions",v.data(),v.size(),"u1",{v.size()});value("cost_critic.near_goal",near);}
Cycle::Cycle(const geometry_msgs::msg::PoseStamped & p,const geometry_msgs::msg::Twist & s,const rclcpp::Clock::SharedPtr & clock) noexcept {
  try {
    auto & w=writer();if(w.directory.empty())return;
    current=std::make_unique<Packet>();current->clock=clock;current->exceptions=std::uncaught_exceptions();
    current->meta={{"schema","tdt_mppi_cycle/v1"},{"cycle_id",w.next++},{"request_steady_ns",steady_ns()},{"sim_ns",clock->now().nanoseconds()},
      {"pose_stamp_ns",rclcpp::Time(p.header.stamp).nanoseconds()},{"pose_frame",p.header.frame_id},
      {"pose",{p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)}},{"speed",{s.linear.x,s.linear.y,s.angular.z}},{"events",Json::array()}};
    current->bytes.reserve(512*1024);
  }catch(...){current.reset();}
}
void Cycle::locked(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & ros,const nav_msgs::msg::Path & path) noexcept {
  if(!current) {return;} auto start=steady_ns();try {
    auto map=ros->getCostmap();current->meta["lock_acquired_steady_ns"]=start;current->meta["map_sim_ns"]=current->clock->now().nanoseconds();
    current->meta["map"]={{"width",map->getSizeInCellsX()},{"height",map->getSizeInCellsY()},{"resolution",map->getResolution()},{"origin",{map->getOriginX(),map->getOriginY()}},{"frame",ros->getGlobalFrameID()}};
    Json footprint=Json::array();for(const auto & p:ros->getRobotFootprint())footprint.push_back({p.x,p.y});current->meta["padded_footprint"]=footprint;
    Json plan=Json::array();for(const auto & p:path.poses)plan.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});current->meta["path"]=plan;
    current->observer_ns+=steady_ns()-start;
    blob("locked.raw_map",map->getCharMap(),map->getSizeInCellsX()*map->getSizeInCellsY(),"u1",{map->getSizeInCellsY(),map->getSizeInCellsX()});
  }catch(...){++writer().errors;}
}
void Cycle::output(const geometry_msgs::msg::TwistStamped & c) noexcept {value("output",{c.twist.linear.x,c.twist.linear.y,c.twist.angular.z});}
Cycle::~Cycle() noexcept {
  if(!current)return;
  try {
    current->meta["unwinding_exception"]=std::uncaught_exceptions()>current->exceptions;
    current->meta["finish_sim_ns"]=current->clock->now().nanoseconds();current->meta["finish_steady_ns"]=steady_ns();current->meta["observer_copy_ns"]=current->observer_ns;
    auto & w=writer();{std::lock_guard<std::mutex> lock(w.mutex);
      if(w.queue.size()>=64||w.stopping){++w.dropped;current.reset();return;}
      w.queue.push_back(std::move(current));}w.cv.notify_one();
  }catch(...){current.reset();}
}
void shutdown() noexcept {try{writer().close();}catch(...){}}
}
