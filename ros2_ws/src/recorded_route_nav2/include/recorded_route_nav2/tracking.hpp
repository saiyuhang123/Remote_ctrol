#pragma once

#include <stdexcept>
#include "recorded_route_nav2/geometry.hpp"

namespace recorded_route_nav2 {

inline std::vector<double> cumulativeArc(const nav_msgs::msg::Path &path) {
  std::vector<double> arc(path.poses.size(), 0.0);
  for (size_t i = 1; i < path.poses.size(); ++i)
    arc[i] = arc[i-1] + dist(path.poses[i-1].pose, path.poses[i].pose);
  return arc;
}

inline Pose interpolatePose(const Pose &a, const Pose &b, double t) {
  Pose p;
  p.position.x = a.position.x + t*(b.position.x-a.position.x);
  p.position.y = a.position.y + t*(b.position.y-a.position.y);
  double yaw = tf2::getYaw(a.orientation) +
    t*wrap(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation));
  p.orientation.z = std::sin(yaw/2);
  p.orientation.w = std::cos(yaw/2);
  return p;
}

inline Pose poseAtArc(
    const nav_msgs::msg::Path &path, const std::vector<double> &arc, double s) {
  if (path.poses.size() < 2 || arc.size() != path.poses.size())
    throw std::invalid_argument("recorded path needs two poses");
  if (s >= arc.back()) return path.poses.back().pose;
  auto next = std::upper_bound(arc.begin(), arc.end(), std::max(0.0, s));
  size_t j = std::min(static_cast<size_t>(next-arc.begin()), arc.size()-1);
  return interpolatePose(path.poses[j-1].pose, path.poses[j].pose,
    (std::max(0.0, s)-arc[j-1])/(arc[j]-arc[j-1]));
}

struct Projection {
  double s = 0.0;
  double error = std::numeric_limits<double>::infinity();
  size_t index = 0;  // last fully reached sample, for BT resume/barriers
  Pose pose;
};

// Search only the current traversal occurrence. Never find a closer pose on
// a later lap or the other side of a cusp by searching the entire path.
inline Projection orderedProjection(
    const nav_msgs::msg::Path &path, const std::vector<double> &arc,
    const Pose &current, double previous_s, double advance) {
  Projection best;
  best.s = std::clamp(previous_s, 0.0, arc.back());
  best.pose = poseAtArc(path, arc, best.s);
  best.error = dist(current, best.pose);
  const double limit = std::min(arc.back(), best.s+advance);
  for (size_t i = 0; i+1 < path.poses.size(); ++i) {
    if (arc[i+1] < previous_s || arc[i] > limit) continue;
    const auto &a = path.poses[i].pose;
    const auto &b = path.poses[i+1].pose;
    double ds = arc[i+1]-arc[i];
    if (ds < 1e-9) continue;
    double dx = b.position.x-a.position.x, dy = b.position.y-a.position.y;
    double fraction = ((current.position.x-a.position.x)*dx +
      (current.position.y-a.position.y)*dy)/(ds*ds);
    double s = std::clamp(arc[i]+fraction*ds, std::max(arc[i], previous_s),
      std::min(arc[i+1], limit));
    Pose q = interpolatePose(a, b, (s-arc[i])/ds);
    double error = dist(current, q);
    if (error < best.error) {best.s=s; best.pose=q; best.error=error;}
  }
  auto next = std::upper_bound(arc.begin(), arc.end(), best.s+1e-8);
  best.index = next == arc.begin() ? 0 : static_cast<size_t>(next-arc.begin()-1);
  return best;
}

// This checks the explicit ticket direction. Integrating in each local body
// frame works on a U-turn/closed curve, unlike the first-to-last chord.
inline int geometryDirection(const nav_msgs::msg::Path &path) {
  double along = 0.0, total = 0.0;
  for (size_t i=1; i<path.poses.size(); ++i) {
    const auto &a=path.poses[i-1].pose;
    const auto &b=path.poses[i].pose;
    if (!std::isfinite(a.position.x) || !std::isfinite(a.position.y) ||
        !std::isfinite(b.position.x) || !std::isfinite(b.position.y) ||
        !std::isfinite(tf2::getYaw(a.orientation)) ||
        !std::isfinite(tf2::getYaw(b.orientation)))
      throw std::invalid_argument("non-finite recorded path");
    double yaw=tf2::getYaw(a.orientation)+.5*wrap(
      tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation));
    along+=(b.position.x-a.position.x)*std::cos(yaw)+
      (b.position.y-a.position.y)*std::sin(yaw);
    total+=dist(a,b);
  }
  if (total<1e-5 || std::abs(along)<.5*total)
    throw std::invalid_argument("recorded path has no reliable body-axis direction");
  return along>0 ? 1 : -1;
}

inline double executionTolerance(double configured, double length, double cap) {
  return std::min({configured, length*.25, cap});
}

// Limit the stop before a short next manoeuvre too. Otherwise the previous
// block can stop 8cm early and swallow an entire 6cm reversal.
inline double blockStopCap(double length, double next_length = 1e9) {
  // Budget error at BOTH ends: two 1/8-length tolerances still leave at
  // least 3/4 of a short manoeuvre to be physically travelled.
  return std::min(length, next_length)*.125;
}

// The local path begins at its ordered projection. Look ahead by ARC length,
// not distance from the robot: lateral error must not select a point behind
// the vehicle. Never extrapolate beyond the recorded block endpoint.
inline Pose directedCarrot(const nav_msgs::msg::Path &local, double lookahead, int direction) {
  auto arc=cumulativeArc(local);
  double s=std::min(lookahead, arc.back());
  Pose carrot=poseAtArc(local,arc,s);
  while (direction*carrot.position.x<=1e-5 && s<arc.back()) {
    s=std::min(s+.02,arc.back());
    carrot=poseAtArc(local,arc,s);
  }
  if (direction*carrot.position.x<=1e-5)
    throw std::runtime_error("recorded endpoint is behind authorized motion; cannot reverse to recover");
  return carrot;
}

}  // namespace recorded_route_nav2
