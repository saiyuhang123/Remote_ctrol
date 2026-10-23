#include <algorithm>
#include <chrono>
#include <mutex>
#include <optional>
#include "chassis_nav2/pursuit_path.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace chassis_nav2 {

// Retain Humble RPP's obstacle regulation, footprint collision projection and
// speed-limit interface. Add the traversal/kinematic constraints of this 4WS
// chassis before collision checking, rather than clipping just v downstream.
class AckermannPursuitController : public
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    auto node=parent.lock();
    auto parameter=[&](const std::string &key, double value) {
      nav2_util::declare_parameter_if_not_declared(node,name+"."+key,rclcpp::ParameterValue(value));
      return node->get_parameter(name+"."+key).as_double();
    };
    radius_=parameter("minimum_turning_radius",.69);
    reverse_speed_=parameter("max_reverse_speed",.30);
    angular_speed_=parameter("max_angular_vel",.8);
    deceleration_=parameter("cusp_deceleration",.4);
    cusp_tolerance_=parameter("cusp_tolerance",.06);
    terminal_lookahead_=parameter("terminal_lookahead_dist",.18);
    cusp_heading_tolerance_=parameter("cusp_heading_tolerance",.25);
    stopped_speed_=parameter("stopped_speed",.03);
    stop_duration_=parameter("cusp_stop_duration",.25);
    for (double v : {radius_,reverse_speed_,angular_speed_,deceleration_,
                     cusp_tolerance_,terminal_lookahead_,cusp_heading_tolerance_,stopped_speed_,stop_duration_})
      if (!std::isfinite(v) || v<=0) throw std::invalid_argument("invalid Ackermann pursuit limit");
    RegulatedPurePursuitController::configure(parent,name,tf,costmap);
    if (use_rotate_to_heading_ || !allow_reversing_)
      throw std::invalid_argument("Ackermann pursuit requires reversing and no in-place rotation");
  }

  void setPlan(const nav_msgs::msg::Path &path) override {
    std::vector<PursuitLeg> legs;
    try {legs=splitPursuitLegs(path);}
    catch (const std::invalid_argument &error) {throw nav2_core::PlannerException(error.what());}
    std::lock_guard<std::mutex> lock(mutex_);
    legs_=std::move(legs); leg_=0; at_cusp_=false; stopped_since_.reset();
    global_plan_=legs_.front().path;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker *checker) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto *map=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> cost_lock(*map->getMutex());
    if (legs_.empty() || !std::isfinite(velocity.linear.x) || !std::isfinite(velocity.angular.z))
      throw nav2_core::PlannerException("missing path or invalid measured velocity");
    geometry_msgs::msg::TwistStamped command;
    command.header=pose.header;
    geometry_msgs::msg::Pose tolerance;
    geometry_msgs::msg::Twist unused;
    if (checker && checker->getTolerances(tolerance,unused)) goal_dist_tol_=tolerance.position.x;

    // An explicit cusp is a stop, then a direction change. Its tolerance is
    // independent of the much looser final goal tolerance.
    auto end=legs_[leg_].path.poses.back();
    end.header=legs_[leg_].path.header; end.header.stamp=pose.header.stamp;
    geometry_msgs::msg::PoseStamped local_end;
    if (!transformPose(costmap_ros_->getBaseFrameID(),end,local_end))
      throw nav2_core::PlannerException("cannot transform navigation endpoint");
    const double end_distance=std::hypot(local_end.pose.position.x,local_end.pose.position.y);
    const bool cusp=leg_+1<legs_.size();
    const double remaining=nav2_util::geometry_utils::calculate_path_length(global_plan_);
    if (cusp && (at_cusp_ || (remaining<.2 && end_distance<=cusp_tolerance_))) {
      at_cusp_=true;
      const auto now=std::chrono::steady_clock::now();
      if (std::abs(velocity.linear.x)>stopped_speed_ || std::abs(velocity.angular.z)>.08)
        stopped_since_.reset();
      else if (!stopped_since_) stopped_since_=now;
      else if (std::chrono::duration<double>(now-*stopped_since_).count()>=stop_duration_) {
        if (end_distance>2*cusp_tolerance_)
          throw nav2_core::PlannerException("overshot navigation cusp; replan required");
        if (std::abs(tf2::getYaw(local_end.pose.orientation))>cusp_heading_tolerance_)
          throw nav2_core::PlannerException("navigation cusp heading missed; replan required");
        ++leg_; global_plan_=legs_[leg_].path; at_cusp_=false; stopped_since_.reset();
      }
      return command;
    }
    stopped_since_.reset();
    if (!cusp && checker) {
      geometry_msgs::msg::PoseStamped goal;
      if (!transformPose(pose.header.frame_id,end,goal))
        throw nav2_core::PlannerException("cannot transform navigation goal");
      if (checker->isGoalReached(pose.pose,goal.pose,velocity)) return command;
    }

    auto local=transformGlobalPlan(pose);
    // Speed-based preview with a larger floor damps short-period steering
    // corrections. A leg contains no later cusp, so preview cannot skip a gear.
    // Short terminal curves need their own preview: a long cruise carrot
    // would cut across an S-shaped reverse leg and reach its cusp with the
    // wrong heading. Direction is locked, so shorter preview cannot flip gear.
    const double lookahead=std::min(getLookAheadDistance(velocity),
                                    std::max(terminal_lookahead_,remaining*.5));
    const double direction=legs_[leg_].direction;
    if (remaining<.2 && direction*local_end.pose.position.x<-std::max(.10,goal_dist_tol_))
      throw nav2_core::PlannerException("navigation endpoint passed; replan required");
    auto carrot=getLookAheadPoint(lookahead,local);
    carrot_pub_->publish(createCarrotMsg(carrot));
    const double x=carrot.pose.position.x,y=carrot.pose.position.y;
    const double distance=std::hypot(x,y);
    const double curvature=std::clamp(distance>.001 ? 2*y/(distance*distance) : 0.,
                                     -1/radius_,1/radius_);
    double sign=legs_[leg_].direction;
    const double speed_limit=sign<0 ? std::min(desired_linear_vel_,reverse_speed_) : desired_linear_vel_;
    double linear=speed_limit;
    applyConstraints(curvature,velocity,costAtPose(pose.pose.position.x,pose.pose.position.y),
                     local,linear,sign);
    double limit=speed_limit;
    if (std::abs(curvature)>1e-6) limit=std::min(limit,angular_speed_/std::abs(curvature));
    if (cusp && remaining<approach_velocity_scaling_dist_)
      limit=std::min(limit,std::sqrt(2*deceleration_*std::max(0.,end_distance-cusp_tolerance_*.5)));
    linear=sign*std::min(std::abs(linear),limit);
    const double angular=linear*curvature;
    if (!std::isfinite(linear) || !std::isfinite(angular))
      throw nav2_core::PlannerException("invalid Ackermann pursuit command");
    if (use_collision_detection_ && isCollisionImminent(pose,linear,angular,distance))
      throw nav2_core::PlannerException("Ackermann pursuit detected collision ahead");
    command.twist.linear.x=linear; command.twist.angular.z=angular;
    return command;
  }

private:
  std::vector<PursuitLeg> legs_;
  size_t leg_=0;
  bool at_cusp_=false;
  std::optional<std::chrono::steady_clock::time_point> stopped_since_;
  double radius_=.69,reverse_speed_=.3,angular_speed_=.8,deceleration_=.4;
  double cusp_tolerance_=.06,stopped_speed_=.03,stop_duration_=.25;
  double terminal_lookahead_=.18,cusp_heading_tolerance_=.25;
};
}  // namespace chassis_nav2

PLUGINLIB_EXPORT_CLASS(chassis_nav2::AckermannPursuitController,nav2_core::Controller)
