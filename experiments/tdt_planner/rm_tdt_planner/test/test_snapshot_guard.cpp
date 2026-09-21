// Safety regressions for latest-snapshot admission; linked into core tests.
#include "rm_tdt_planner/snapshot_guard.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
using namespace rm_tdt_planner;
namespace {
class SnapshotGuard : public ::testing::Test {
protected:
  Grid old{100,80,.1,-3.25,-1.75,std::vector<uint8_t>(8000,0),CostInterpretation::Nav2Master};
  std::vector<Point> fp{{.2,.15},{-.2,.15},{-.2,-.15},{.2,-.15}};
  // Deliberately sparse samples: a new obstacle midway must invalidate a segment.
  std::vector<Point> path{{-1.25,.30},{3.75,.30}};
  Options options;
  void SetUp() override {options.radius=.25; options.clearance=.02;}
  SnapshotAdmission check(const Grid & latest) {
    return validate_latest_candidate(old,latest,fp,fp,path,options);
  }
  Grid shifted(int cells) {
    Grid next=old;next.origin_x+=cells*old.resolution;
    next.costs.assign(next.costs.size(),0);
    for(int y=0;y<old.height;++y) for(int x=0;x<old.width;++x) {
      int src=x+cells;
      if(src>=0 && src<old.width) next.costs[y*old.width+x]=old.costs[y*old.width+src];
    }
    return next;
  }
};
TEST_F(SnapshotGuard, UnchangedValidPath) {EXPECT_EQ(check(old),SnapshotAdmission::Unchanged);}
TEST_F(SnapshotGuard, RollingOriginPreservesWorldPathAndObstacle) {
  old.costs[35*100+45]=254;const auto before=old.costs;const auto original=path;
  const auto latest=shifted(2);
  EXPECT_EQ(check(latest),SnapshotAdmission::Revalidated);
  EXPECT_EQ(old.costs,before);
  ASSERT_EQ(path.size(),original.size());
  for(size_t i=0;i<path.size();++i) {EXPECT_DOUBLE_EQ(path[i].x,original[i].x);EXPECT_DOUBLE_EQ(path[i].y,original[i].y);}
}
TEST_F(SnapshotGuard, RollingAndNewLethalRejectsBetweenSamples) {
  auto latest=shifted(2);latest.costs[20*100+43]=254;
  EXPECT_EQ(check(latest),SnapshotAdmission::Unsafe);
}
TEST_F(SnapshotGuard, RollingUnknownStripRejectsFullFootprint) {
  auto latest=shifted(2);
  for(int x=20;x<68;++x) latest.costs[22*100+x]=255; // y .45-.55, 15 cm from path centre
  EXPECT_EQ(check(latest),SnapshotAdmission::Unsafe);
}
TEST_F(SnapshotGuard, RollingOutsideWindowRejects) {EXPECT_EQ(check(shifted(30)),SnapshotAdmission::Unsafe);}
TEST_F(SnapshotGuard, NewLethalUnknownOr253OnSegmentRejects) {
  for(int cost:{253,254,255}) {
    auto latest=old;latest.costs[20*100+45]=cost;
    EXPECT_EQ(check(latest),SnapshotAdmission::Unsafe)<<cost;
  }
}
TEST_F(SnapshotGuard, SoftCostUpdateRequiresValidPath) {
  auto latest=old;latest.costs[20*100+45]=252;
  EXPECT_EQ(check(latest),SnapshotAdmission::Revalidated);
  latest.costs[22*100+45]=254;
  EXPECT_EQ(check(latest),SnapshotAdmission::Unsafe);
}
TEST_F(SnapshotGuard, RemoteHardChangeCanPassFullCheck) {
  auto latest=old;latest.costs[50*100+45]=254;
  EXPECT_EQ(check(latest),SnapshotAdmission::Revalidated);
}
TEST_F(SnapshotGuard, SameRadiusDifferentFootprintRejects) {
  auto changed=fp;changed[0]={.15,.2};
  EXPECT_EQ(validate_latest_candidate(old,old,fp,changed,path,options),SnapshotAdmission::FootprintChanged);
}
TEST_F(SnapshotGuard, SizeResolutionAndInterpretationChangesReject) {
  auto latest=old;latest.width=80;latest.height=100;
  EXPECT_EQ(check(latest),SnapshotAdmission::GeometryChanged);
  latest=old;latest.resolution=.05;EXPECT_EQ(check(latest),SnapshotAdmission::GeometryChanged);
  latest=old;latest.cost_interpretation=CostInterpretation::ObstacleSeeds;
  EXPECT_EQ(check(latest),SnapshotAdmission::GeometryChanged);
}
TEST_F(SnapshotGuard, MalformedOrNonfiniteLatestRejects) {
  auto latest=old;latest.costs.pop_back();EXPECT_EQ(check(latest),SnapshotAdmission::Unsafe);
  latest=old;latest.origin_x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(check(latest),SnapshotAdmission::Unsafe);
  latest=old;latest.origin_y=std::numeric_limits<double>::infinity();
  EXPECT_EQ(check(latest),SnapshotAdmission::Unsafe);
}
TEST_F(SnapshotGuard, InvalidFootprintOrRadiusMismatchRejects) {
  auto changed=fp;changed[0].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validate_latest_candidate(old,old,fp,changed,path,options),SnapshotAdmission::FootprintChanged);
  EXPECT_EQ(validate_latest_candidate(old,old,{},fp,path,options),SnapshotAdmission::FootprintChanged);
  options.radius=.2;EXPECT_EQ(check(old),SnapshotAdmission::FootprintChanged);
}
TEST_F(SnapshotGuard, ClearanceAndTouchRemainRejected) {
  auto latest=old;latest.costs[23*100+45]=254; // y .55-.65: exactly .25 body radius
  EXPECT_EQ(check(latest),SnapshotAdmission::Unsafe);
  options.clearance=0.;EXPECT_EQ(check(latest),SnapshotAdmission::Unsafe); // guard rejects contact
}
TEST_F(SnapshotGuard, EmptyNonfiniteAndBoundaryPathsReject) {
  path.clear();EXPECT_EQ(check(old),SnapshotAdmission::Unsafe);
  path={{std::numeric_limits<double>::quiet_NaN(),.3}};EXPECT_EQ(check(old),SnapshotAdmission::Unsafe);
  path={{-3.1,.3},{-3.1,.6}};EXPECT_EQ(check(old),SnapshotAdmission::Unsafe);
}
}
