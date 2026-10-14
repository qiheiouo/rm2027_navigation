// Subscriptions only. DDS publication time uses system clock, not ROS sim time.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
int main(int argc,char**argv){
 if(argc<2)return 2;rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("execution_receipt_cpp",rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time",true)}));
 std::ofstream out(argv[1]);out<<std::setprecision(17)<<"topic,ROS_receipt_ns,steady_ns,system_ns,DDS_source_ns,DDS_received_ns,publisher_gid,state_source_ns,vx,vy,wz\n";
 auto emit=[&](const std::string&t,const geometry_msgs::msg::Twist&v,const rclcpp::MessageInfo&i,int64_t stamp){
  auto &m=i.get_rmw_message_info();out<<t<<','<<node->get_clock()->now().nanoseconds()<<','<<std::chrono::steady_clock::now().time_since_epoch().count()<<','<<std::chrono::system_clock::now().time_since_epoch().count()<<','<<m.source_timestamp<<','<<m.received_timestamp<<',';
  out<<std::hex<<std::setfill('0');for(auto b:m.publisher_gid.data)out<<std::setw(2)<<int(b);out<<std::dec<<','<<stamp<<','<<v.linear.x<<','<<v.linear.y<<','<<v.angular.z<<'\n';
 };
 std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> subs;
 for(auto t:{"/cmd_vel_nav","/cmd_vel","/simulation/chassis/cmd_vel"})subs.push_back(node->create_subscription<geometry_msgs::msg::Twist>(t,100,[&,topic=std::string(t)](geometry_msgs::msg::Twist::ConstSharedPtr m,const rclcpp::MessageInfo&i){emit(topic,*m,i,0);}));
 auto odom=node->create_subscription<nav_msgs::msg::Odometry>("/odometry/lio",rclcpp::SensorDataQoS(),[&](nav_msgs::msg::Odometry::ConstSharedPtr m,const rclcpp::MessageInfo&i){emit("/odometry/lio",m->twist.twist,i,int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec);});
 auto timer=node->create_wall_timer(std::chrono::seconds(1),[&]{out.flush();});rclcpp::spin(node);out.flush();rclcpp::shutdown();
}
