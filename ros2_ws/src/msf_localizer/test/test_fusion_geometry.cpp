#include "msf_localizer/fusion_geometry.hpp"
#include <gtest/gtest.h>
using namespace msf_localizer;

TEST(FusionGeometry, InterpolatesAtScanTimestampAndRejectsGaps)
{
  Matrix a = Matrix::Identity(), b = Matrix::Identity();
  b(0, 3) = 2.; b.block<3, 3>(0, 0) = Eigen::AngleAxisd(1., Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::deque<TimedPose> history{{1., a}, {1.2, b}};
  const auto result = sample_pose(history, 1.1, .25);
  ASSERT_TRUE(result); EXPECT_NEAR((*result)(0, 3), 1., 1e-8);
  EXPECT_NEAR(std::atan2((*result)(1, 0), (*result)(0, 0)), .5, 1e-8);
  EXPECT_FALSE(sample_pose(history, .9, .25));
  EXPECT_FALSE(sample_pose(history, 1.6, .25));
  EXPECT_FALSE(sample_pose(history, 1.1, .1));
}

TEST(FusionGeometry, MapOdomCompositionPreservesSensorExtrinsic)
{
  Matrix odom = Matrix::Identity(), base_lidar = Matrix::Identity(), map_base = Matrix::Identity();
  odom(0, 3) = 4.; base_lidar(0, 3) = .32; base_lidar(2, 3) = .46;
  map_base(1, 3) = 7.;
  const Matrix ndt_lidar = map_base * base_lidar;
  const Matrix correction = ndt_lidar * base_lidar.inverse() * odom.inverse();
  EXPECT_TRUE((correction * odom).isApprox(map_base));
}

TEST(FusionGeometry, RejectsBadMatchesInsteadOfLegacyUnconditionalAcceptance)
{
  Matrix identity = Matrix::Identity(), jump = identity;
  jump(0, 3) = 2.;
  EXPECT_TRUE(acceptable(true, .1, identity, identity, 1.5, .8, .6));
  EXPECT_FALSE(acceptable(false, .1, identity, identity, 1.5, .8, .6));
  EXPECT_FALSE(acceptable(true, 5., identity, identity, 1.5, .8, .6));
  EXPECT_FALSE(acceptable(true, .1, jump, identity, 1.5, .8, .6));
  jump = identity; jump.block<3, 3>(0, 0) = Eigen::AngleAxisd(1., Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(acceptable(true, .1, jump, identity, 1.5, .8, .6));
}
