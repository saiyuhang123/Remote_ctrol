#pragma once

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "recorded_route_nav2/tracking.hpp"

namespace recorded_route_nav2 {

// Use Humble's regulation and collision projection, but choose a target in
// traversal order and the ticket's gear BEFORE checking the actual command.
class OrderedPursuit :
  public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
public:
  geometry_msgs::msg::TwistStamped computeDirected(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity, int direction, double radius,
      double remaining, double tolerance) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto *costmap=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> cost_lock(*costmap->getMutex());
    auto local=transformGlobalPlan(pose);
    if (local.poses.size()<2) {
      // RPP may prune to the final pose. Keep a zero-length segment so the
      // same endpoint/authorized-side handling applies.
      local.poses.insert(local.poses.begin(),local.poses.front());
    }
    const double lateral=std::abs(local.poses.front().pose.position.y);
    const double lookahead=std::min(max_lookahead_dist_,
      std::max(getLookAheadDistance(velocity),std::sqrt(2*radius*lateral)));
    auto target=directedCarrot(local,lookahead,direction);
    carrot_x=target.position.x;carrot_y=target.position.y;
    geometry_msgs::msg::PoseStamped carrot;
    carrot.header=local.header;carrot.pose=target;
    carrot_pub_->publish(createCarrotMsg(carrot));
    const double d2=carrot_x*carrot_x+carrot_y*carrot_y;
    // The real chassis saturates steering. Project collisions with the same
    // feasible curvature, not an impossible tighter arc.
    curvature=std::clamp(d2>1e-9 ? 2*carrot_y/d2 : 0.0,-1/radius,1/radius);
    double linear=desired_linear_vel_,sign=direction;
    applyConstraints(curvature,velocity,costAtPose(pose.pose.position.x,pose.pose.position.y),
      local,linear,sign);
    // Bound speed before a precision stop without raising RPP's low speed.
    const double braking=std::sqrt(2*.3*std::max(0.0,remaining-tolerance*.5));
    linear=direction*std::min(std::abs(linear),braking);
    double angular=linear*curvature;
    if (!std::isfinite(linear) || !std::isfinite(angular))
      throw nav2_core::PlannerException("non-finite recorded controller command");
    if (use_collision_detection_ &&
        isCollisionImminent(pose,linear,angular,std::sqrt(d2)))
      throw nav2_core::PlannerException("recorded controller detected collision ahead");
    geometry_msgs::msg::TwistStamped result;
    result.header=pose.header;result.twist.linear.x=linear;result.twist.angular.z=angular;
    return result;
  }

  double carrot_x=0.0,carrot_y=0.0,curvature=0.0;
};
}  // namespace recorded_route_nav2
