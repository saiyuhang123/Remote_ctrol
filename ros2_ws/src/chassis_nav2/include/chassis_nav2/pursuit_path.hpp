#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace chassis_nav2 {

struct PursuitLeg {
  nav_msgs::msg::Path path;
  int direction;
};

// Hybrid-A* poses describe the vehicle heading even on a reverse segment.
// Split BEFORE nearest-point pruning: overlapping forward/reverse legs must
// never compete for the closest point or for the lookahead target.
inline std::vector<PursuitLeg> splitPursuitLegs(const nav_msgs::msg::Path &path) {
  if (path.poses.empty() || path.header.frame_id.empty())
    throw std::invalid_argument("navigation path must have poses and a frame");
  for (const auto &p : path.poses) {
    const auto &q = p.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) || !std::isfinite(p.pose.position.z) ||
        !std::isfinite(norm) || std::abs(norm-1.0)>0.01)
      throw std::invalid_argument("navigation path has an invalid position or heading");
  }
  std::vector<PursuitLeg> legs;
  auto previous = path.poses.front();
  for (size_t i=1; i<path.poses.size(); ++i) {
    const auto &next = path.poses[i];
    const double dx = next.pose.position.x-previous.pose.position.x;
    const double dy = next.pose.position.y-previous.pose.position.y;
    if (std::hypot(dx,dy)<1e-6) {previous=next; continue;}
    const double yaw = tf2::getYaw(previous.pose.orientation);
    const int direction = dx*std::cos(yaw)+dy*std::sin(yaw)<0 ? -1 : 1;
    if (legs.empty() || legs.back().direction!=direction) {
      PursuitLeg leg;
      leg.direction=direction; leg.path.header=path.header;
      leg.path.poses.push_back(previous);
      legs.push_back(std::move(leg));
    }
    legs.back().path.poses.push_back(next);
    previous=next;
  }
  if (legs.empty()) legs.push_back({path,1});
  // Keep the exact requested final orientation, including a duplicate endpoint.
  legs.back().path.poses.back()=path.poses.back();
  return legs;
}

}  // namespace chassis_nav2
