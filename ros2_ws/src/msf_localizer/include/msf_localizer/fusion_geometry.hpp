#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

namespace msf_localizer
{
using Matrix = Eigen::Matrix4d;
struct TimedPose { double stamp; Matrix pose; };

inline std::optional<Matrix> sample_pose(
  const std::deque<TimedPose> &history, double stamp, double max_gap)
{
  if (history.empty() || !std::isfinite(stamp)) { return std::nullopt; }
  if (stamp < history.front().stamp) { return std::nullopt; }
  if (stamp >= history.back().stamp) {
    if (stamp - history.back().stamp > max_gap) { return std::nullopt; }
    return history.back().pose;
  }
  auto right = std::lower_bound(history.begin(), history.end(), stamp,
    [](const TimedPose &p, double t) { return p.stamp < t; });
  if (right->stamp == stamp) { return right->pose; }
  const auto &left = *std::prev(right);
  if (right->stamp - left.stamp > max_gap) { return std::nullopt; }
  const double ratio = (stamp - left.stamp) / (right->stamp - left.stamp);
  Matrix result = Matrix::Identity();
  result.block<3, 1>(0, 3) = (1. - ratio) * left.pose.block<3, 1>(0, 3) +
    ratio * right->pose.block<3, 1>(0, 3);
  result.block<3, 3>(0, 0) = Eigen::Quaterniond(left.pose.block<3, 3>(0, 0)).slerp(
    ratio, Eigen::Quaterniond(right->pose.block<3, 3>(0, 0))).toRotationMatrix();
  return result;
}

inline bool acceptable(
  bool converged, double score, const Matrix &candidate, const Matrix &prediction,
  double max_score, double max_distance, double max_angle)
{
  if (!converged || !std::isfinite(score) || score > max_score || score < 0. ||
    !candidate.allFinite()) { return false; }
  const Matrix delta = prediction.inverse() * candidate;
  const double angle = std::abs(Eigen::AngleAxisd(delta.block<3, 3>(0, 0)).angle());
  return delta.block<3, 1>(0, 3).norm() <= max_distance && angle <= max_angle;
}
}  // namespace msf_localizer
