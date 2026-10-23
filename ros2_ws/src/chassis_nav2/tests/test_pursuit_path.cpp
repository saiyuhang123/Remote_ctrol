#include <gtest/gtest.h>
#include <limits>
#include "chassis_nav2/pursuit_path.hpp"

namespace {
geometry_msgs::msg::PoseStamped pose(double x,double y=0.,double yaw=0.) {
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x=x; p.pose.position.y=y;
  p.pose.orientation.z=std::sin(yaw/2); p.pose.orientation.w=std::cos(yaw/2);
  return p;
}
nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> poses) {
  nav_msgs::msg::Path p; p.header.frame_id="map"; p.poses=poses; return p;
}
}

TEST(PursuitPath, ForwardAndReverseUsePathHeading) {
  auto forward=chassis_nav2::splitPursuitLegs(path({pose(0),pose(1),pose(2)}));
  ASSERT_EQ(forward.size(),1u); EXPECT_EQ(forward[0].direction,1);
  auto reverse=chassis_nav2::splitPursuitLegs(path({pose(0),pose(-1),pose(-2)}));
  ASSERT_EQ(reverse.size(),1u); EXPECT_EQ(reverse[0].direction,-1);
  // Negative world X is still forward when the vehicle faces west.
  auto west=chassis_nav2::splitPursuitLegs(path({pose(0,0,M_PI),pose(-1,0,M_PI)}));
  ASSERT_EQ(west.size(),1u); EXPECT_EQ(west[0].direction,1);
}

TEST(PursuitPath, OverlappingReturnLegCannotBePrunedIntoActiveLeg) {
  auto legs=chassis_nav2::splitPursuitLegs(path({pose(0),pose(-.5),pose(-1),pose(-.5),pose(0),pose(1)}));
  ASSERT_EQ(legs.size(),2u);
  EXPECT_EQ(legs[0].direction,-1); EXPECT_EQ(legs[1].direction,1);
  ASSERT_EQ(legs[0].path.poses.size(),3u);
  EXPECT_DOUBLE_EQ(legs[0].path.poses.back().pose.position.x,-1);
  EXPECT_DOUBLE_EQ(legs[1].path.poses.front().pose.position.x,-1);
}

TEST(PursuitPath, DuplicatedCuspsAndMultipleGearChanges) {
  auto legs=chassis_nav2::splitPursuitLegs(path({pose(0),pose(1),pose(1),pose(.5),pose(0),pose(0),pose(.5)}));
  ASSERT_EQ(legs.size(),3u);
  EXPECT_EQ(legs[0].direction,1); EXPECT_EQ(legs[1].direction,-1); EXPECT_EQ(legs[2].direction,1);
  for (const auto &leg:legs) EXPECT_GE(leg.path.poses.size(),2u);
}

TEST(PursuitPath, PreserveRequestedFinalHeadingAndSinglePose) {
  auto legs=chassis_nav2::splitPursuitLegs(path({pose(0),pose(1),pose(1,0,.3)}));
  ASSERT_EQ(legs.size(),1u);
  EXPECT_NEAR(tf2::getYaw(legs[0].path.poses.back().pose.orientation),.3,1e-9);
  EXPECT_EQ(chassis_nav2::splitPursuitLegs(path({pose(0)})).size(),1u);
}

TEST(PursuitPath, RejectMalformedPlansBeforeDriving) {
  EXPECT_THROW(chassis_nav2::splitPursuitLegs(path({})),std::invalid_argument);
  auto invalid=pose(0); invalid.pose.orientation.w=0.;
  EXPECT_THROW(chassis_nav2::splitPursuitLegs(path({invalid})),std::invalid_argument);
  invalid=pose(std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(chassis_nav2::splitPursuitLegs(path({invalid})),std::invalid_argument);
}
