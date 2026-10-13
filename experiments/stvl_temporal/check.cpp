// Deterministic same-candidate future-motion intervention through pluginlib/DDS.
#include <iostream>
#include <thread>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <nav2_mppi_controller/critic_function.hpp>
#include <std_msgs/msg/string.hpp>
using J=nlohmann::json;
int main(int argc,char **argv) {
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("temporal_check");
 auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("check_map");map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));map->set_parameter(rclcpp::Parameter("global_frame",std::string("map")));map->set_parameter(rclcpp::Parameter("footprint",std::string("[[-.34,-.29],[-.34,.29],[.34,.29],[.34,-.29]]")));map->set_parameter(rclcpp::Parameter("footprint_padding",0.));map->configure();
 auto handler=std::make_unique<mppi::ParametersHandler>(node);pluginlib::ClassLoader<mppi::critics::CriticFunction> loader("nav2_mppi_controller","mppi::critics::CriticFunction");auto critic=loader.createSharedInstance("mppi::critics::TemporalResearchCritic");critic->on_configure(node,"FollowPath","FollowPath.TemporalResearchCritic",map,handler.get());
 auto pub=node->create_publisher<std_msgs::msg::String>("/research/time_predictions",1);pub->on_activate();rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node->get_node_base_interface());
 for(int i=0;i<30&&pub->get_subscription_count()==0;i++){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 mppi::models::State state;state.reset(2,30);mppi::models::Trajectories tr;tr.reset(2,30);mppi::models::Path path;path.reset(2);xt::xtensor<float,1> costs=xt::zeros<float>({2});float dt=.1;
 for(int k=0;k<30;k++)tr.x(0,k)=(k+1)*.1;
 auto message=[&](double velocity,int64_t source){J points=J::array();for(int k=0;k<36;k++)points.push_back({1.,-1+velocity*k*.1});return J{{"schema","rm_time_prediction/research_v1"},{"frame","map"},{"mode","C"},{"source_ns",source},{"dt",.1},{"complete",true},{"tracks",J::array({{{"id",1},{"state",2},{"position",{1.,-1.}},{"velocity",{0.,velocity}},{"size",{.2,.2}},{"observed_ns",source},{"points",points}}})}};};
 auto evaluate=[&](J m){std_msgs::msg::String msg;msg.data=m.dump();pub->publish(msg);for(int i=0;i<6;i++){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(3));}costs.fill(0);mppi::CriticData data{state,tr,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};critic->score(data);return J{{"failed",data.fail_flag},{"forward",costs(0)},{"wait",costs(1)}};};
 int64_t source=node->now().nanoseconds();auto crossing=evaluate(message(1.,source));auto away=evaluate(message(-1.,node->now().nanoseconds()));
 if(crossing["failed"].get<bool>()||away["failed"].get<bool>()||crossing["forward"].get<double>()<=1000||crossing["forward"].get<double>()<=crossing["wait"].get<double>()||away["forward"].get<double>()>=crossing["forward"].get<double>())throw std::runtime_error("future risk intervention failed");
 auto stale=evaluate(message(1.,node->now().nanoseconds()-1000000000)); // old callback is ignored, latest stays fresh.
 std::this_thread::sleep_for(std::chrono::milliseconds(420));costs.fill(0);mppi::CriticData old{state,tr,path,costs,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};critic->score(old);if(!old.fail_flag)throw std::runtime_error("stale latest snapshot accepted");
 auto future=evaluate(message(1.,node->now().nanoseconds()+1000000000));if(!future["failed"].get<bool>())throw std::runtime_error("future epoch accepted");
 auto resumed=evaluate(message(-1.,node->now().nanoseconds()));if(resumed["failed"].get<bool>())throw std::runtime_error("future watermark poisoned recovery");
 std::cout<<J{{"crossing",crossing},{"away",away},{"old_callback_cannot_replace_new",!stale["failed"].get<bool>()},{"stale_snapshot_rejected",old.fail_flag},{"future_rejected",future["failed"]},{"fresh_recovered",!resumed["failed"].get<bool>()}}.dump(2)<<std::endl;
 critic.reset();handler.reset();map->cleanup();exec.remove_node(node->get_node_base_interface());map.reset();node.reset();rclcpp::shutdown();
}
