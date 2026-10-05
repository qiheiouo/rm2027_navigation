// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
#include "rm_tdt_planner/pose_geometry.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <type_traits>

namespace pg = rm_tdt_planner::pose_geometry;
using rm_tdt_planner::Point;
namespace
{
constexpr double pi=3.14159265358979323846;
const std::vector<Point> rectangle{{-.33,-.28},{.33,-.28},{.33,.28},{-.33,.28}};
pg::RawCostmapInput map()
{return {60,60,.1,-3,-3,std::vector<uint8_t>(3600,0)};}
void obstacle(pg::RawCostmapInput & g, int x, int y, uint8_t cost=254)
{g.costs[y*g.width+x]=cost;}
pg::Motion stationary(double x=0, double y=0, double yaw=0)
{return {{x,y,yaw},{x,y,yaw},0};}
pg::Limits generous()
{pg::Limits l; l.time_budget_seconds=10; return l;}
void cover(const pg::Result & r)
{
  ASSERT_EQ(r.status,pg::Status::Certified) << r.reason;
  ASSERT_FALSE(r.intervals.empty());
  double end=0;
  for (auto i:r.intervals) {
    EXPECT_EQ(i.begin,end);
    EXPECT_GT(i.end,i.begin);
    EXPECT_GT(i.constraint_margin_lower_bound,1e-7);
    end=i.end;
  }
  EXPECT_EQ(end,1.0);
  EXPECT_FALSE(r.witness);
}

// Independent oracle: long-double segment intersections, containment, then
// vertex/edge distances. No SAT or temporal bounding from the implementation.
struct V {long double x,y;};
long double orient(V a,V b,V c)
{return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
bool on(V p,V a,V b)
{return orient(a,b,p)==0 && p.x>=std::min(a.x,b.x) && p.x<=std::max(a.x,b.x) &&
  p.y>=std::min(a.y,b.y) && p.y<=std::max(a.y,b.y);}
bool intersects(V a,V b,V c,V d)
{
  const auto u=orient(a,b,c),v=orient(a,b,d),w=orient(c,d,a),z=orient(c,d,b);
  return (u*v<0 && w*z<0) || on(c,a,b) || on(d,a,b) || on(a,c,d) || on(b,c,d);
}
long double edge(V p,V a,V b)
{
  const auto dx=b.x-a.x,dy=b.y-a.y;
  const auto t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy),0.L,1.L);
  return std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
}
bool inside(V p,const std::vector<V> & poly)
{
  bool positive=false,negative=false;
  for (size_t i=0;i<poly.size();++i) {
    const auto v=orient(poly[i],poly[(i+1)%poly.size()],p);
    positive|=v>0; negative|=v<0;
  }
  return !(positive && negative);
}
long double distance(const std::vector<V> & a,const std::vector<V> & b)
{
  if (inside(a[0],b) || inside(b[0],a)) {return 0;}
  long double d=1e30L;
  for (size_t i=0;i<a.size();++i) {
    for (size_t j=0;j<b.size();++j) {
      const auto aa=a[i],ab=a[(i+1)%a.size()],ba=b[j],bb=b[(j+1)%b.size()];
      if (intersects(aa,ab,ba,bb)) {return 0;}
      d=std::min({d,edge(aa,ba,bb),edge(ba,aa,ab)});
    }
  }
  return d;
}
long double oracle(const pg::RawSnapshot & snap,const std::vector<Point> & shape,
  const pg::Motion & m,double t,double clearance)
{
  const long double x=m.start.x+t*(m.goal.x-m.start.x),y=m.start.y+t*(m.goal.y-m.start.y);
  const long double yaw=m.start.yaw+t*m.yaw_delta,c=std::cos(yaw),s=std::sin(yaw);
  std::vector<V> poly;
  for (auto p:shape) {poly.push_back({x+c*p.x-s*p.y,y+s*p.x+c*p.y});}
  long double margin=1e30L;
  const auto & g=snap.raw();
  // Enumerate the raw input independently as well; do not reuse the candidate's cell list.
  for (int index=0;index<g.width*g.height;++index) {
    const int cx=index%g.width,cy=index/g.width;
    const bool boundary=cx==0 || cy==0 || cx==g.width-1 || cy==g.height-1;
    if (!boundary && g.costs[index]<253) {continue;}
    const pg::Cell cell{cx,cy,g.costs[index],boundary};
    const long double bx=g.origin_x+static_cast<long double>(cell.x)*g.resolution;
    const long double by=g.origin_y+static_cast<long double>(cell.y)*g.resolution,r=g.resolution;
    if (cell.boundary || cell.cost>=254) {
      margin=std::min(margin,distance(poly,{{bx,by},{bx+r,by},{bx+r,by+r},{bx,by+r}})-clearance);
    } else {
      margin=std::min(margin,std::hypot(x-std::clamp(x,bx,bx+r),y-std::clamp(y,by,by+r)));
    }
  }
  return margin;
}
}  // namespace

TEST(PoseGeometry, ExactEndpointSingleCellDistinguishesYawFromCircle)
{
  pg::RawCostmapInput g{200,160,.05,-2,-4,std::vector<uint8_t>(32000,0)};
  obstacle(g,120,87); // Closed square [4,4.05] x [.35,.4], isolated synthetic input.
  const pg::RawSnapshot snap(g);
  const double circle_distance=std::hypot(4.3-4.05,.35);
  EXPECT_LT(circle_distance,std::hypot(.33,.28)+.02);
  const auto safe=pg::certify(snap,rectangle,stationary(4.3,0),generous());
  cover(safe);
  ASSERT_EQ(safe.intervals.size(),1u);
  EXPECT_NEAR(safe.intervals[0].constraint_margin_lower_bound,.05,1e-12);
  const auto bad=pg::certify(snap,rectangle,stationary(4.3,0,1.48743203),generous());
  ASSERT_EQ(bad.status,pg::Status::Collision);
  ASSERT_TRUE(bad.witness);
  EXPECT_EQ(bad.witness->cell_x,120);
  EXPECT_EQ(bad.witness->cell_y,87);
  EXPECT_EQ(bad.witness->cost,254);
  EXPECT_EQ(bad.witness->distance,0.0);
}

TEST(PoseGeometry, TranslationFindsCollisionMissedByEndpointsAndMidpoint)
{
  auto g=map(); obstacle(g,20,30);
  pg::RawSnapshot snap(g);
  const std::vector<Point> small{{-.04,-.04},{.04,-.04},{.04,.04},{-.04,.04}};
  pg::Motion motion{{-2,.05,0},{2,.05,0},0};
  for (double t:{0.,.5,1.}) {EXPECT_GT(oracle(snap,small,motion,t,.02),0);}
  const auto r=pg::certify(snap,small,motion,generous());
  ASSERT_EQ(r.status,pg::Status::Collision);
  ASSERT_TRUE(r.witness);
  EXPECT_GT(r.intervals_examined,1u);
  EXPECT_GT(r.witness->parameter,0);
  EXPECT_LT(r.witness->parameter,.5);
  EXPECT_LE(oracle(snap,small,motion,r.witness->parameter,.02),0);
  EXPECT_TRUE(r.intervals.empty());
}

TEST(PoseGeometry, SafeEndpointsDoNotCertifyIntermediateRotation)
{
  auto g=map(); obstacle(g,30,36);
  pg::RawSnapshot snap(g);
  const std::vector<Point> rod{{-.8,-.1},{.8,-.1},{.8,.1},{-.8,.1}};
  const pg::Motion turn{{0,0,0},{0,0,pi},pi};
  EXPECT_GT(oracle(snap,rod,turn,0,.02),0);
  EXPECT_GT(oracle(snap,rod,turn,1,.02),0);
  const auto r=pg::certify(snap,rod,turn,generous());
  ASSERT_EQ(r.status,pg::Status::Collision);
  EXPECT_NEAR(r.witness->parameter,.5,1e-12);
}

TEST(PoseGeometry, ExplicitFullTurnsAreNotCollapsedToEqualYaw)
{
  auto g=map(); obstacle(g,30,36); pg::RawSnapshot snap(g);
  const std::vector<Point> rod{{-.8,-.1},{.8,-.1},{.8,.1},{-.8,.1}};
  cover(pg::certify(snap,rod,stationary(),generous()));
  for (double delta:{2*pi,-2*pi}) {
    auto m=stationary(); m.yaw_delta=delta;
    const auto r=pg::certify(snap,rod,m,generous());
    ASSERT_EQ(r.status,pg::Status::Collision);
    EXPECT_GT(std::abs(r.witness->pose.yaw),.5);
  }
}

TEST(PoseGeometry, WrappedGoalHonorsExplicitShortAndLongRotation)
{
  auto g=map(); obstacle(g,30,36); pg::RawSnapshot snap(g);
  const std::vector<Point> rod{{-.8,-.1},{.8,-.1},{.8,.1},{-.8,.1}};
  pg::Motion m{{0,0,pi-.1},{0,0,-pi+.1},.2};
  cover(pg::certify(snap,rod,m,generous()));
  m.yaw_delta=.2-2*pi;
  EXPECT_EQ(pg::certify(snap,rod,m,generous()).status,pg::Status::Collision);
}

TEST(PoseGeometry, InscribedCostConstrainsCentreWithoutInflatingFootprintAgain)
{
  auto g=map(); obstacle(g,32,30,253); pg::RawSnapshot snap(g);
  cover(pg::certify(snap,rectangle,stationary(),generous())); // Polygon overlaps 253; centre does not.
  const auto r=pg::certify(snap,rectangle,stationary(.25,.05),generous());
  ASSERT_EQ(r.status,pg::Status::Collision);
  EXPECT_EQ(r.witness->constraint,pg::Constraint::InscribedCentre);
  EXPECT_EQ(r.witness->required,0.0);
  EXPECT_EQ(r.witness->cost,253);
}

TEST(PoseGeometry, UnknownAndBoundaryRemainHardObstacles)
{
  auto g=map(); obstacle(g,32,30,255);
  auto r=pg::certify(pg::RawSnapshot(g),rectangle,stationary(),generous());
  ASSERT_EQ(r.status,pg::Status::Collision); EXPECT_EQ(r.witness->cost,255);
  g=map();
  r=pg::certify(pg::RawSnapshot(g),rectangle,stationary(-2.8,0),generous());
  ASSERT_EQ(r.status,pg::Status::Collision);
  EXPECT_TRUE(r.witness->boundary); EXPECT_EQ(r.witness->cost,0);
}

TEST(PoseGeometry, OutsideCentreIsRejectedEvenWhenEntireBodyIsOutside)
{
  const auto r=pg::certify(pg::RawSnapshot(map()),rectangle,stationary(10,0),generous());
  ASSERT_EQ(r.status,pg::Status::Collision);
  EXPECT_EQ(r.witness->constraint,pg::Constraint::MapExtent);
}

TEST(PoseGeometry, ClosedEdgeAndCornerContactRejectAtZeroClearance)
{
  auto g=map(); obstacle(g,30,30);
  const pg::RawSnapshot snap(g);
  const std::vector<Point> square{{-.125,-.125},{.125,-.125},{.125,.125},{-.125,.125}};
  auto l=generous(); l.clearance=0;
  for (auto p:std::array<pg::Pose,2>{{{-.125,.05,0},{-.125,-.125,0}}}) {
    const auto r=pg::certify(snap,square,{p,p,0},l);
    ASSERT_EQ(r.status,pg::Status::Collision); EXPECT_EQ(r.witness->distance,0);
  }
}

TEST(PoseGeometry, NumericalGuardCannotProduceSafeCertificate)
{
  auto g=map(); obstacle(g,30,30);
  const std::vector<Point> square{{-.125,-.125},{.125,-.125},{.125,.125},{-.125,.125}};
  auto l=generous(); l.clearance=0;
  const auto r=pg::certify(pg::RawSnapshot(g),square,stationary(-.125-5e-8,.05),l);
  EXPECT_EQ(r.status,pg::Status::Unresolved);
  EXPECT_EQ(r.reason,"distance within numerical guard"); EXPECT_FALSE(r.witness);
}

TEST(PoseGeometry, PolygonAndCellContainmentReject)
{
  auto g=map(); obstacle(g,30,30);
  EXPECT_EQ(pg::certify(pg::RawSnapshot(g),rectangle,stationary(),generous()).status,pg::Status::Collision);
  const std::vector<Point> tiny{{-.01,-.01},{.01,-.01},{.01,.01},{-.01,.01}};
  EXPECT_EQ(pg::certify(pg::RawSnapshot(g),tiny,stationary(.05,.05),generous()).status,pg::Status::Collision);
}

TEST(PoseGeometry, BudgetExhaustionIsUnresolvedWithoutPartialCertificate)
{
  pg::RawSnapshot snap(map()); const pg::Motion m{{-2,-2,0},{2,2,0},0};
  auto l=generous(); l.max_intervals=1;
  auto r=pg::certify(snap,rectangle,m,l);
  EXPECT_EQ(r.status,pg::Status::Unresolved); EXPECT_EQ(r.reason,"interval budget exhausted");
  EXPECT_TRUE(r.intervals.empty());
  l=generous(); l.max_depth=0; r=pg::certify(snap,rectangle,m,l);
  EXPECT_EQ(r.status,pg::Status::Unresolved); EXPECT_EQ(r.reason,"subdivision depth exhausted");
  l=generous(); l.max_cell_checks=1; r=pg::certify(snap,rectangle,m,l);
  EXPECT_EQ(r.status,pg::Status::Unresolved); EXPECT_EQ(r.reason,"cell check budget exhausted");
  l=generous(); l.time_budget_seconds=1e-12; r=pg::certify(snap,rectangle,m,l);
  EXPECT_EQ(r.status,pg::Status::Unresolved); EXPECT_EQ(r.reason,"time budget exhausted");
  cover(pg::certify(snap,rectangle,m,generous()));
}

TEST(PoseGeometry, RawSnapshotOwnsCopyAndHasNoPreparedConversion)
{
  static_assert(!std::is_constructible_v<pg::RawSnapshot,rm_tdt_planner::Grid>);
  static_assert(!std::is_constructible_v<pg::RawSnapshot,rm_tdt_planner::PreparedGrid>);
  auto g=map(); pg::RawSnapshot snap(g); obstacle(g,30,30);
  cover(pg::certify(snap,rectangle,stationary(),generous()));
  EXPECT_EQ(snap.raw().costs[30*60+30],0);
}

TEST(PoseGeometry, InvalidRawMetadataThrowsBeforeIndexing)
{
  auto g=map(); g.costs.pop_back(); EXPECT_THROW(pg::RawSnapshot{g},std::invalid_argument);
  g=map(); g.width=-1; EXPECT_THROW(pg::RawSnapshot{g},std::invalid_argument);
  g=map(); g.resolution=0; EXPECT_THROW(pg::RawSnapshot{g},std::invalid_argument);
  g=map(); g.origin_y=std::numeric_limits<double>::quiet_NaN(); EXPECT_THROW(pg::RawSnapshot{g},std::invalid_argument);
  g=map(); g.width=4096; g.height=4096; EXPECT_THROW(pg::RawSnapshot{g},std::invalid_argument);
}

TEST(PoseGeometry, InvalidMotionAndLimitsNeverCertify)
{
  pg::RawSnapshot snap(map()); auto m=stationary(); auto l=generous();
  m.yaw_delta=1;
  EXPECT_EQ(pg::certify(snap,rectangle,m,l).status,pg::Status::Unresolved);
  m=stationary(); m.start.x=std::numeric_limits<double>::infinity();
  EXPECT_EQ(pg::certify(snap,rectangle,m,l).status,pg::Status::Unresolved);
  m=stationary(); m.yaw_delta=10*pi;
  EXPECT_EQ(pg::certify(snap,rectangle,m,l).status,pg::Status::Unresolved);
  m=stationary(); l.clearance=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(pg::certify(snap,rectangle,m,l).status,pg::Status::Unresolved);
  l=generous(); l.max_cell_checks=0;
  EXPECT_EQ(pg::certify(snap,rectangle,m,l).status,pg::Status::Unresolved);
}

TEST(PoseGeometry, MalformedFootprintsNeverCertify)
{
  pg::RawSnapshot snap(map());
  std::vector<std::vector<Point>> bad{
    {}, {{0,0},{1,0}}, {{-.3,-.3},{.3,-.3},{0,0},{.3,.3},{-.3,.3}},
    {{-.3,-.3},{.3,.3},{.3,-.3},{-.3,.3}},
    {{-.3,-.3},{.3,-.3},{.3,-.3},{.3,.3},{-.3,.3}},
    {{0,0},{1,0},{0,1}}, {{1,1},{2,1},{2,2},{1,2}},
    {{-.3,-.3},{0,-.3},{.3,-.3},{.3,.3},{-.3,.3}}};
  auto nan=rectangle; nan[1].x=std::numeric_limits<double>::quiet_NaN(); bad.push_back(nan);
  std::vector<Point> star;
  for (int k=0;k<5;++k) {const double a=4*pi*k/5; star.push_back({std::cos(a),std::sin(a)});}
  bad.push_back(star);
  for (const auto & shape:bad) {
    const auto r=pg::certify(snap,shape,stationary(),generous());
    EXPECT_EQ(r.status,pg::Status::Unresolved); EXPECT_TRUE(r.intervals.empty());
  }
}

TEST(PoseGeometry, WorldTranslationAndWindingDoNotChangeCertificate)
{
  auto g=map(); obstacle(g,38,33); const pg::Motion m{{-1,-.6,-.2},{.1,-.5,.4},.6};
  const auto base=pg::certify(pg::RawSnapshot(g),rectangle,m,generous()); cover(base);
  for (double shift:{123.25,900000.0}) {
    auto shifted=g; shifted.origin_x+=shift; shifted.origin_y-=shift;
    auto motion=m; motion.start.x+=shift; motion.goal.x+=shift;
    motion.start.y-=shift; motion.goal.y-=shift;
    auto shape=rectangle; std::reverse(shape.begin(),shape.end());
    const auto r=pg::certify(pg::RawSnapshot(shifted),shape,motion,generous()); cover(r);
    ASSERT_EQ(r.intervals.size(),base.intervals.size());
    for (size_t i=0;i<r.intervals.size();++i) {
      EXPECT_NEAR(r.intervals[i].constraint_margin_lower_bound,base.intervals[i].constraint_margin_lower_bound,1e-8);
    }
  }
}

TEST(PoseGeometry, SoftCostsDoNotBecomeHardObstacles)
{
  auto g=map(); obstacle(g,30,30,252);
  cover(pg::certify(pg::RawSnapshot(g),rectangle,stationary(),generous()));
}

TEST(PoseGeometry, ContinuousCertificatesCrossCheckedByIndependentDenseOracle)
{
  auto g=map(); obstacle(g,39,37); obstacle(g,22,40,255); obstacle(g,20,21,253);
  pg::RawSnapshot snap(g); std::mt19937 rng(20260916);
  std::uniform_real_distribution<double> position(-1.4,1.4),angle(-pi,pi),rotation(-2*pi,2*pi);
  int certified=0,collisions=0;
  for (int n=0;n<48;++n) {
    auto shape=rectangle;
    if (n%2) {shape={{-.42,-.17},{.25,-.3},{.38,.23},{-.18,.4}};}
    pg::Motion m{{position(rng),position(rng),angle(rng)},{position(rng),position(rng),0},rotation(rng)};
    m.goal.yaw=std::remainder(m.start.yaw+m.yaw_delta,2*pi);
    const auto r=pg::certify(snap,shape,m,generous());
    ASSERT_NE(r.status,pg::Status::Unresolved) << n << ": " << r.reason;
    if (r.status==pg::Status::Certified) {
      ++certified; cover(r);
      for (auto interval:r.intervals) {
        for (int j=0;j<=8;++j) {
          const double t=interval.begin+(interval.end-interval.begin)*j/8;
          const auto actual=oracle(snap,shape,m,t,.02);
          EXPECT_GT(actual,0) << n << " t=" << t;
          EXPECT_LE(interval.constraint_margin_lower_bound,actual+1e-10L);
        }
      }
      for (int j=0;j<=100;++j) {EXPECT_GT(oracle(snap,shape,m,j/100.,.02),0) << n;}
    } else {
      ++collisions; ASSERT_TRUE(r.witness);
      EXPECT_LE(oracle(snap,shape,m,r.witness->parameter,.02),1e-10L) << n;
    }
  }
  EXPECT_GT(certified,5); EXPECT_GT(collisions,5);
}
