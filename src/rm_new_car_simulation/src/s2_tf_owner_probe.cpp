// Test-only canonical-edge observation; GIDs are diagnostics, not authentication.
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
class S2TFOwners : public rclcpp::Node {
public:
  S2TFOwners() : Node("s2_tf_owner_probe") {
    path_=declare_parameter<std::string>("output", "");
    if(path_.empty())throw std::invalid_argument("test output required");
    const auto edges=declare_parameter<std::vector<std::string>>(
      "edges", {"map>odom", "odom>base_link"});
    observed_edges_=std::set<std::string>(edges.begin(),edges.end());
    auto callback=[this](tf2_msgs::msg::TFMessage::ConstSharedPtr m,const rclcpp::MessageInfo & info) {
      std::ostringstream gid;
      for(auto b:info.get_rmw_message_info().publisher_gid.data)gid<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<int>(b);
      for(const auto & t:m->transforms) {
        auto edge=t.header.frame_id+">"+t.child_frame_id;
        if(!observed_edges_.count(edge))continue;
        if(!owners_[edge].insert(gid.str()).second)continue;
        std::ofstream out(path_);out<<"{";bool first=true;
        for(const auto & [name,ids]:owners_) {
          if(!first)out<<",";first=false;out<<"\""<<name<<"\":[";bool item=true;
          for(const auto & id:ids){if(!item)out<<",";item=false;out<<"\""<<id<<"\"";}
          out<<"]";
        }
        out<<"}\n";
      }
    };
    dynamic_=create_subscription<tf2_msgs::msg::TFMessage>("/tf",rclcpp::QoS(100),callback);
    fixed_=create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",rclcpp::QoS(100).transient_local(),callback);
  }
private:
  std::set<std::string> observed_edges_;
  std::string path_;std::map<std::string,std::set<std::string>> owners_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_,fixed_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<S2TFOwners>());rclcpp::shutdown();}
