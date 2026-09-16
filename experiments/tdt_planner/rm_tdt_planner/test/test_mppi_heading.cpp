#include "nav2_mppi_controller/critics/path_align_critic.hpp"
#include <gtest/gtest.h>
namespace
{
// Call the installed library's real scorer, with all inputs explicitly supplied.
class HeadingCritic : public mppi::critics::PathAlignCritic
{
public:
  void setup(bool orientations) {
    enabled_=true; use_path_orientations_=orientations; offset_from_furthest_=20;
    trajectory_point_step_=4; threshold_to_consider_=.5; max_path_occupancy_ratio_=.07;
    power_=1; weight_=10;
  }
};
TEST(MppiHeading, InstalledPathAlignScoresYawAndHandsOffNearGoal)
{
  mppi::models::State state; state.pose.pose.orientation.w=1;
  mppi::models::Path path; path.reset(41);
  mppi::models::Trajectories trajectories; trajectories.reset(2,30);
  for(size_t i=0;i<41;++i) {path.x(i)=i*.05;}
  for(size_t t=0;t<30;++t) {
    for(size_t b=0;b<2;++b) {trajectories.x(b,t)=t*.05; trajectories.yaws(b,t)=b ? 1.F : 0.F;}
  }
  xt::xtensor<float,1> costs=xt::zeros<float>({2}); float dt=.1;
  mppi::CriticData data{state,trajectories,path,costs,dt,false,nullptr,nullptr,
    std::vector<bool>(41,true),size_t(29)};
  HeadingCritic critic; critic.setup(false); critic.score(data);
  EXPECT_NEAR(costs(0),costs(1),1e-6);
  costs.fill(0); critic.setup(true); critic.score(data);
  EXPECT_GT(costs(1),costs(0)+5);
  costs.fill(0); state.pose.pose.position.x=1.8; critic.score(data);
  EXPECT_EQ(costs(0),0); EXPECT_EQ(costs(1),0);
}
}
