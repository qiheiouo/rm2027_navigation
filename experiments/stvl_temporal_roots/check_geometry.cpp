// Native 1.1.20 geometry counterexample, through actual plugins and InflationLayer.
#include <iostream>
#include <nlohmann/json.hpp>
#include <nav2_mppi_controller/critic_function.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <pluginlib/class_loader.hpp>
using J=nlohmann::json;
int main(int argc,char **argv){
 rclcpp::init(argc,argv);rclcpp::NodeOptions opt;opt.parameter_overrides({rclcpp::Parameter("FollowPath.CostCritic.consider_footprint",true),rclcpp::Parameter("FollowPath.RootCostProbe.consider_footprint",true),rclcpp::Parameter("FollowPath.RootCostProbe.check_all_poses",true)});auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("geometry_counterexample","",opt);
 auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("geometry_map");map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{"inflation_layer"}));map->declare_parameter("inflation_layer.plugin",std::string("nav2_costmap_2d::InflationLayer"));map->declare_parameter("inflation_layer.inflation_radius",.30);map->declare_parameter("inflation_layer.cost_scaling_factor",6.);map->set_parameter(rclcpp::Parameter("global_frame",std::string("map")));map->set_parameter(rclcpp::Parameter("footprint",std::string("[[-.32,-.27],[-.32,.27],[.32,.27],[.32,-.27]]")));map->set_parameter(rclcpp::Parameter("footprint_padding",.02));map->configure();auto grid=map->getCostmap();grid->resizeMap(80,80,.05,-2,-2);
 for(unsigned int j=22;j<=25;j++)for(unsigned int i=10;i<73;i++)grid->setCost(i,j,254);
 for(auto layer:*map->getLayeredCostmap()->getPlugins()){layer->matchSize();double ax=-2,ay=-2,bx=2,by=2;layer->updateBounds(0,0,0,&ax,&ay,&bx,&by);layer->updateCosts(*grid,0,0,80,80);}
 nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(grid);unsigned int ix,iy;grid->worldToMap(0,-.295,ix,iy);int center=grid->getCost(ix,iy);double exact=checker.footprintCostAtPose(0,-.295,.7,map->getRobotFootprint());if(center!=0||exact!=254)throw std::runtime_error("fixture must be free center / lethal polygon");
 auto handler=std::make_unique<mppi::ParametersHandler>(node);pluginlib::ClassLoader<mppi::critics::CriticFunction> loader("nav2_mppi_controller","mppi::critics::CriticFunction");auto native=loader.createSharedInstance("mppi::critics::CostCritic");auto repair=loader.createSharedInstance("mppi::critics::RootCostProbe");native->on_configure(node,"FollowPath","FollowPath.CostCritic",map,handler.get());repair->on_configure(node,"FollowPath","FollowPath.RootCostProbe",map,handler.get());
 mppi::models::State state;state.reset(2,30);state.pose.pose.orientation.w=1.;mppi::models::Trajectories tr;tr.reset(2,30);mppi::models::Path path;path.reset(2);path.x(1)=4.;xt::xtensor<float,1> cost=xt::zeros<float>({2});float dt=.1;
 for(int k=0;k<30;k++){tr.y(0,k)=-.295*(k+1)/30.;tr.yaws(0,k)=.7*(k+1)/30.;}
 mppi::CriticData a{state,tr,path,cost,dt,false,nullptr,nullptr,std::nullopt,std::nullopt};native->score(a);J old={{"unsafe",cost(0)},{"safe_wait",cost(1)},{"all_failed",a.fail_flag}};cost.fill(0);a.fail_flag=false;repair->score(a);J fixed={{"unsafe",cost(0)},{"safe_wait",cost(1)},{"all_failed",a.fail_flag}};
 if(old["unsafe"].get<double>()!=0||fixed["unsafe"].get<double>()<499||fixed["safe_wait"].get<double>()!=0||a.fail_flag)throw std::runtime_error("single-factor geometry check failed");
 for(int k=0;k<30;k++){tr.y(1,k)=tr.y(0,k);tr.yaws(1,k)=tr.yaws(0,k);}cost.fill(0);a.fail_flag=false;repair->score(a);if(!a.fail_flag)throw std::runtime_error("native all-collision fallback not raised");
 std::cout<<J{{"center_cost",center},{"exact_native_footprint_cost",exact},{"circumscribed_radius",map->getLayeredCostmap()->getCircumscribedRadius()},{"inflation_radius",.30},{"native",old},{"repair",fixed},{"all_unsafe_native_fail_flag",a.fail_flag}}.dump(2)<<std::endl;
 repair.reset();native.reset();handler.reset();map->cleanup();map.reset();node.reset();rclcpp::shutdown();
}
