#include <gtest/gtest.h>
#include "recorded_route_nav2/tracking.hpp"

using namespace recorded_route_nav2;

static nav_msgs::msg::Path line(double start, double end, double y=0.0, size_t n=50) {
  nav_msgs::msg::Path path;
  for(size_t i=0;i<=n;++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose=fromJson({{"x",start+(end-start)*i/n},{"y",y},{"yaw",0.0}});
    path.poses.push_back(p);
  }
  return path;
}

TEST(RecordedTracking, LateralErrorBeyondLookaheadNeverSelectsBehindVehicle) {
  // The old Euclidean RPP selection chooses (-.01,-.10) immediately and
  // requests reverse on this entirely forward path.
  auto local=line(-.01,1.,-.10);
  auto carrot=directedCarrot(local,.08,1);
  EXPECT_GT(carrot.position.x,0.);
  EXPECT_NEAR(carrot.position.x,.07,1e-9);
  EXPECT_NEAR(carrot.position.y,-.10,1e-9);
  auto reverse=line(.01,-1.,.10);
  EXPECT_LT(directedCarrot(reverse,.08,-1).position.x,0.);
}

TEST(RecordedTracking, CannotInventRecoveryBeyondEndpoint) {
  EXPECT_THROW(directedCarrot(line(-.10,-.01),.15,1),std::runtime_error);
  auto carrot=directedCarrot(line(0.,.05),.5,1);
  EXPECT_DOUBLE_EQ(carrot.position.x,.05);
}

TEST(RecordedTracking, LocalBodyIntegralHandlesClosedCurvesAndNoisyFirstPair) {
  nav_msgs::msg::Path circle;
  for(int i=0;i<=200;++i) {
    double yaw=2*M_PI*i/200;
    geometry_msgs::msg::PoseStamped p;
    p.pose=fromJson({{"x",std::sin(yaw)},{"y",1-std::cos(yaw)},{"yaw",yaw}});
    circle.poses.push_back(p);
  }
  EXPECT_EQ(geometryDirection(circle),1);
  std::reverse(circle.poses.begin(),circle.poses.end());
  EXPECT_EQ(geometryDirection(circle),-1);
  auto noisy=line(0.,1.);
  auto correction=noisy.poses.front();correction.pose.position.x=-.005;
  noisy.poses.insert(noisy.poses.begin()+1,correction);
  EXPECT_EQ(geometryDirection(noisy),1);
  EXPECT_THROW(geometryDirection(line(0.,0.)),std::invalid_argument);
}

TEST(RecordedTracking, ShortReversalHasSmallToleranceOnBothSidesOfCusp) {
  const double short_length=.0653642107;
  double preceding=executionTolerance(.08,7.2,blockStopCap(7.2,short_length));
  double reversing=executionTolerance(.08,short_length,blockStopCap(short_length,.088));
  EXPECT_LT(preceding,.017);
  EXPECT_DOUBLE_EQ(preceding,reversing);
  EXPECT_LT(executionTolerance(.20,.088,blockStopCap(.088)),.023);
  EXPECT_GT(short_length, reversing);  // the start cannot count as arrival
}

TEST(RecordedTracking, ProjectionUsesContinuousArcAndCannotJumpToAnotherVisit) {
  auto path=line(0.,1.,0.,10);
  auto back=line(1.,0.,0.,10);
  path.poses.insert(path.poses.end(),back.poses.begin()+1,back.poses.end());
  auto arc=cumulativeArc(path);
  Pose current=fromJson({{"x",.355},{"y",.10},{"yaw",0.0}});
  auto projection=orderedProjection(path,arc,current,.30,.15);
  EXPECT_NEAR(projection.s,.355,1e-9);
  EXPECT_NEAR(projection.error,.10,1e-9);
  EXPECT_EQ(projection.index,3u);
  current.position.x=.8;
  projection=orderedProjection(path,arc,current,.30,.15);
  EXPECT_NEAR(projection.s,.45,1e-9);
  current.position.x=.20;
  projection=orderedProjection(path,arc,current,.30,.15);
  EXPECT_NEAR(projection.s,.30,1e-9);
}

TEST(RecordedTracking, DuplicateSamplesDoNotCreateNanProjectionOrDirection) {
  auto path=line(0.,.06,0.,2);
  path.poses.insert(path.poses.begin(),path.poses.front());
  auto arc=cumulativeArc(path);
  auto q=orderedProjection(path,arc,fromJson({{"x",.023},{"y",0.},{"yaw",0.}}),0.,.20);
  EXPECT_NEAR(q.s,.023,1e-9);
  EXPECT_EQ(geometryDirection(path),1);
}
