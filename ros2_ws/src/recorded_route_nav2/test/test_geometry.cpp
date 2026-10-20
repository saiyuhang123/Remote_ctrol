#include <gtest/gtest.h>
#include "recorded_route_nav2/geometry.hpp"
#include "recorded_route_nav2/entry_curve.hpp"
using namespace recorded_route_nav2;
TEST(RecordedEntry, LargeHeadingCorrectionRetainsExactCuspsAndCurvature) {
  auto start=fromJson({{"x",.11},{"y",-.03},{"yaw",-1.8}}),goal=fromJson({{"x",0.},{"y",0.},{"yaw",0.}});
  auto path=entryCurve(start,goal,.69);
  ASSERT_GT(path.poses.size(),20u);
  EXPECT_NEAR(dist(path.poses.front().pose,start),0.,1e-9);
  EXPECT_NEAR(dist(path.poses.back().pose,goal),0.,1e-9);
  int previous=0,changes=0;
  for(size_t i=1;i<path.poses.size();++i) {
    const auto &a=path.poses[i-1].pose;const auto &b=path.poses[i].pose;
    double ds=dist(a,b),yaw=tf2::getYaw(a.orientation),da=wrap(tf2::getYaw(b.orientation)-yaw);
    ASSERT_GT(ds,1e-7);EXPECT_LE(ds,.025001);
    EXPECT_LE(std::abs(da)/ds,1/.69+.01);
    double along=(b.position.x-a.position.x)*std::cos(yaw)+(b.position.y-a.position.y)*std::sin(yaw);
    int gear=along>=0?1:-1;
    if(previous && previous!=gear) ++changes;
    previous=gear;
  }
  EXPECT_GE(changes,2);
  EXPECT_TRUE(entryCurve(start,goal,.69).poses.size()==path.poses.size());
}
TEST(RecordedGeometry, FullFootprintCellIntersection) {
  Polygon p{{0.2,0.2},{1.2,0.2},{1.2,0.8},{0.2,0.8}};
  EXPECT_TRUE(polygonCell(p,0,0));
  EXPECT_TRUE(polygonCell(p,1,0)); // centre of footprint is outside this cell
  EXPECT_FALSE(polygonCell(p,0,1));
  Polygon thin{{-.5,.45},{1.5,.45},{1.5,.55},{-.5,.55}};
  EXPECT_TRUE(polygonCell(thin,0,0)); // no polygon vertex lies inside the cell
}
TEST(RecordedGeometry, ConcavePermittedRegion) {
  Polygon allowed{{0,0},{4,0},{4,4},{3,4},{3,1},{1,1},{1,4},{0,4}};
  EXPECT_TRUE(inside(.5,2,allowed));
  EXPECT_FALSE(inside(2,2,allowed));
  EXPECT_TRUE(properCross({.5,2},{3.5,2},{1,1},{1,4}));
}
TEST(RecordedGeometry, BodyHeadingAndArcLengthSurviveReversing) {
  nav_msgs::msg::Path path;
  for(auto x:{0.,.5,0.}) {geometry_msgs::msg::PoseStamped p;p.pose=fromJson({{"x",x},{"y",0.},{"yaw",0.}});path.poses.push_back(p);}
  EXPECT_DOUBLE_EQ(pathLength(path),1.);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation),0.,1e-9);
  EXPECT_NEAR(wrap(3.14-(-3.14)),-.003185307179586,1e-8);
}
TEST(RecordedGeometry, DensifyingSmacSamplesKeepsTheSameCurveAndBodyHeading) {
  nav_msgs::msg::Path path;path.header.frame_id="map";
  for(auto x:{0.,.114,.228}) {geometry_msgs::msg::PoseStamped p;p.pose=fromJson({{"x",x},{"y",0.},{"yaw",0.}});path.poses.push_back(p);}
  auto dense=densePath(path);
  EXPECT_EQ(dense.header.frame_id,"map");EXPECT_GT(dense.poses.size(),path.poses.size());
  EXPECT_NEAR(pathLength(dense),pathLength(path),1e-9);
  for(size_t i=1;i<dense.poses.size();++i) EXPECT_LE(dist(dense.poses[i-1].pose,dense.poses[i].pose),.040001);
  EXPECT_DOUBLE_EQ(dense.poses.back().pose.position.x,.228);
}
TEST(RecordedGeometry, ExactAckermannBypassReturnsWithoutPassingItsRejoinGate) {
  auto start=fromJson({{"x",2.86},{"y",0.},{"yaw",0.}}),goal=fromJson({{"x",5.86},{"y",0.},{"yaw",0.}});
  for(auto offset:{-.56,.56}) {
    auto path=laneChangeDetour(start,goal,.69,offset);ASSERT_GT(path.poses.size(),20u);
    EXPECT_NEAR(dist(path.poses.front().pose,start),0.,1e-9);EXPECT_NEAR(dist(path.poses.back().pose,goal),0.,1e-9);
    EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation),0.,1e-9);
    double max_shift=0;
    for(size_t i=1;i<path.poses.size();++i) {
      const auto &a=path.poses[i-1].pose;const auto &b=path.poses[i].pose;double ds=dist(a,b);
      EXPECT_LE(std::abs(wrap(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation)))/ds,1/.69+.01);
      EXPECT_FALSE(pastGate(b,goal));max_shift=std::max(max_shift,std::abs(b.position.y));
    }
    EXPECT_NEAR(max_shift,std::abs(offset),1e-9);
  }
  auto near=goal;near.position.x=3.2;EXPECT_TRUE(laneChangeDetour(start,near,.69,.56).poses.empty());
  auto beyond=goal;beyond.position.x+=.1;EXPECT_TRUE(pastGate(beyond,goal));
}
