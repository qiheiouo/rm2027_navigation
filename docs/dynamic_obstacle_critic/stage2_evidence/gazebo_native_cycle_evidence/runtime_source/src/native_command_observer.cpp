#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rm_dynamic_obstacle_critic/evidence_json.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>

class NativeCommandObserver : public rclcpp::Node {
public:
  NativeCommandObserver() : Node("native_command_observer") {
    topic_ = declare_parameter("input_topic",std::string("/cmd_vel_nav"));
    const auto output = declare_parameter("output_file",std::string(""));
    maximum_ = declare_parameter("max_records",4096);
    if (!std::filesystem::path(output).is_absolute() || maximum_ < 1 || maximum_ > 10000)
      throw std::invalid_argument("observer requires absolute new output file and bounded record count");
    file_.reset(std::fopen(output.c_str(),"wx")); // Exclusive creation; never overwrite evidence.
    if (!file_) throw std::runtime_error("cannot exclusively create command evidence file");
    std::setvbuf(file_.get(),nullptr,_IOLBF,0);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(topic_,100,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg,const rclcpp::MessageInfo &info) {
      if (count_ >= size_t(maximum_)) return;
      const auto &rmw = info.get_rmw_message_info();
      Gid gid; std::copy(rmw.publisher_gid.data,rmw.publisher_gid.data+gid.size(),gid.begin());
      if (publishers_.find(gid) == publishers_.end()) refresh_publishers();
      std::ostringstream record; record.imbue(std::locale::classic()); record << std::setprecision(17);
      record << "{\"ordinal\":" << count_ << ",\"receive_sim\":" << now().seconds()
             << ",\"dds_source_timestamp_ns\":" << rmw.source_timestamp
             << ",\"dds_received_timestamp_ns\":" << rmw.received_timestamp << ",\"publisher_gid\":\"";
      for (uint8_t byte : gid) record << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
      record << std::dec << "\",\"publisher_node\":";
      auto found = publishers_.find(gid);
      if (found == publishers_.end()) record << "null,\"publisher_namespace\":null";
      else record << rm_dynamic_obstacle_critic::json_string(found->second.first)
                  << ",\"publisher_namespace\":" << rm_dynamic_obstacle_critic::json_string(found->second.second);
      record << ",\"velocity\":[";
      bool comma=false,finite=true;
      for (double value : {msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z}) {
        if (comma) record << ',';
        comma=true;
        if (std::isfinite(value)) record << value;
        else { record << "null"; finite=false; }
      }
      record << "],\"finite\":" << (finite?"true":"false") << "}\n";
      if (std::fputs(record.str().c_str(),file_.get()) == EOF) {
        RCLCPP_ERROR(get_logger(),"Command evidence write failed; observer stops recording");
        count_ = maximum_; return;
      }
      ++count_;
      if (count_ == size_t(maximum_)) RCLCPP_INFO(get_logger(),"Command evidence budget exhausted");
    });
    refresh_ = create_wall_timer(std::chrono::seconds(1),[this](){refresh_publishers();});
  }
private:
  using Gid = std::array<uint8_t,RMW_GID_STORAGE_SIZE>;
  void refresh_publishers() {
    for (const auto &endpoint : get_publishers_info_by_topic(topic_))
      publishers_[endpoint.endpoint_gid()] = {endpoint.node_name(),endpoint.node_namespace()};
  }
  std::string topic_;
  int maximum_{4096}; size_t count_{0};
  std::unique_ptr<FILE,decltype(&std::fclose)> file_{nullptr,&std::fclose};
  std::map<Gid,std::pair<std::string,std::string>> publishers_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr refresh_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<NativeCommandObserver>()); }
  catch (const std::exception &e) { std::fprintf(stderr,"native command observer: %s\n",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
