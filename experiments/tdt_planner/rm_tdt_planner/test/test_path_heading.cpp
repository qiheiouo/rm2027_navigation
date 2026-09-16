#include "rm_tdt_planner/path_heading.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
using rm_tdt_planner::Point;
using rm_tdt_planner::continuous_path_heading;
namespace {constexpr double pi=3.14159265358979323846;}
TEST(PathHeading, StraightMiddleAndSmoothExactGoalWithoutChangingXY)
{
  std::vector<Point> p; for(int i=0;i<=300;++i) {p.push_back({i*.01,0});}
  const auto original=p; // XY remains caller-owned.
  const auto h=continuous_path_heading(p,0,pi/2);
  EXPECT_EQ(h.size(),p.size()); EXPECT_NEAR(h.front(),0,1e-12); EXPECT_NEAR(h.back(),pi/2,1e-12);
  EXPECT_NEAR(h[150],0,1e-12); EXPECT_NEAR(h[250],pi/4,1e-12);
  EXPECT_LT(std::abs(h.back()-h[h.size()-2]),.001);
  for(size_t i=1;i<h.size();++i) {EXPECT_LT(std::abs(h[i]-h[i-1]),.024); EXPECT_EQ(p[i].x,original[i].x);}
}
TEST(PathHeading, WrapAndRepeatedPointsRemainFiniteAndContinuous)
{
  std::vector<Point> p; for(int i=0;i<=100;++i) {p.push_back({-i*.02,i*.0001});}
  p.insert(p.begin()+30,p[29]);
  const auto h=continuous_path_heading(p,pi-.1,-pi+.1);
  EXPECT_NEAR(std::remainder(h.back()-(-pi+.1),2*pi),0,1e-12);
  for(size_t i=1;i<h.size();++i) {EXPECT_TRUE(std::isfinite(h[i])); EXPECT_LT(std::abs(h[i]-h[i-1]),.03);}
}
TEST(PathHeading, ShortAndZeroLengthPathsDoNotDivideByZero)
{
  const auto h=continuous_path_heading({{0,0},{.01,0},{.02,0}},.3,-.7);
  EXPECT_NEAR(h.front(),.3,1e-12); EXPECT_NEAR(h.back(),-.7,1e-12);
  EXPECT_EQ(continuous_path_heading({{0,0},{0,0}},0,1),std::vector<double>({1,1}));
  EXPECT_THROW(continuous_path_heading({},0,0),std::invalid_argument);
  EXPECT_THROW(continuous_path_heading({{NAN,0}},0,0),std::invalid_argument);
}
TEST(PathHeading, RoundedCornerChangesHeadingBeforeAndAfterVertex)
{
  std::vector<Point> p; for(int i=0;i<=100;++i) {p.push_back({i*.01,0});}
  for(int i=1;i<=100;++i) {p.push_back({1,i*.01});}
  const auto h=continuous_path_heading(p,0,pi/2);
  EXPECT_NEAR(h[100],pi/4,1e-12); EXPECT_GT(h[95],0); EXPECT_LT(h[105],pi/2);
  for(size_t i=1;i<h.size();++i) {EXPECT_LT(std::abs(h[i]-h[i-1]),.20);}
}
