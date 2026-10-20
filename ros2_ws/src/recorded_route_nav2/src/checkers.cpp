#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "recorded_route_nav2/progress.hpp"
#include "recorded_route_nav2/tracking.hpp"

namespace recorded_route_nav2 {
class RecordedGoalChecker : public nav2_core::GoalChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,const std::string &name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {
    auto node=parent.lock();
    tolerance_=param(node,name+".xy_goal_tolerance",0.20);
    yaw_tolerance_=param(node,name+".yaw_goal_tolerance",-1.0);
    auto controller=param(node,name+".controller_id",std::string("RecordedRoute"));
    progress_=sharedProgress(std::string(node->get_node_base_interface()->get_fully_qualified_name())+"/"+controller);
  }
  void reset() override {}
  bool isGoalReached(const Pose &query,const Pose &goal,const geometry_msgs::msg::Twist &) override {
    std::lock_guard<std::mutex> lock(progress_->mutex);
    const double tolerance=executionTolerance(tolerance_,progress_->total,progress_->tolerance_cap);
    return progress_->ready && std::chrono::duration<double>(Clock::now()-progress_->updated).count()<1.0 &&
      progress_->remaining<=tolerance && dist(query,goal)<=tolerance &&
      (yaw_tolerance_<0 || std::abs(wrap(tf2::getYaw(query.orientation)-tf2::getYaw(goal.orientation)))<=yaw_tolerance_);
  }
  bool getTolerances(Pose &p,geometry_msgs::msg::Twist &v) override {
    std::lock_guard<std::mutex> lock(progress_->mutex);
    p.position.x=p.position.y=executionTolerance(tolerance_,progress_->total,progress_->tolerance_cap);
    p.position.z=std::numeric_limits<double>::lowest();
    p.orientation.x=p.orientation.y=p.orientation.z=p.orientation.w=std::numeric_limits<double>::lowest();
    if(yaw_tolerance_>=0) p.orientation=fromJson(Json{{"x",0},{"y",0},{"yaw",yaw_tolerance_}}).orientation;
    v.linear.x=v.linear.y=v.linear.z=v.angular.x=v.angular.y=v.angular.z=std::numeric_limits<double>::lowest();
    return true;
  }
private: std::shared_ptr<Progress> progress_;double tolerance_=0.2,yaw_tolerance_=-1.0;
};

// Humble has one progress checker for all controllers. Preserve its original
// detector for ordinary navigation. Both recorded controllers use ordered
// arc progress, so a short reversing curve need not travel 50cm in 20s.
class EntryProgressChecker : public nav2_core::ProgressChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,const std::string &name) override {
    auto node=parent.lock();
    auto prefix=std::string(node->get_node_base_interface()->get_fully_qualified_name())+"/";
    for(const auto &id : {param(node,name+".entry_controller_id",std::string("RecordedEntryRoute")),
                         param(node,name+".recorded_controller_id",std::string("RecordedRoute"))})
      progresses_.push_back(sharedProgress(prefix+id));
    allowance_=param(node,name+".entry_time_allowance",20.0);
    loader_=std::make_unique<pluginlib::ClassLoader<nav2_core::ProgressChecker>>("nav2_core","nav2_core::ProgressChecker");
    ordinary_=loader_->createSharedInstance("nav2_controller::SimpleProgressChecker");
    ordinary_->initialize(parent,name);reset();
  }
  void reset() override {ordinary_->reset();entry_=false;stamp_=0;baseline_=1e9;last_=Clock::now();}
  bool check(geometry_msgs::msg::PoseStamped &current) override {
    double remaining=1e9,total=1e9;int64_t plan_stamp=0;Clock::time_point updated{};
    for(const auto &progress : progresses_) {
      std::lock_guard<std::mutex> lock(progress->mutex);
      if(progress->updated>updated) {
        remaining=progress->remaining;plan_stamp=progress->stamp;updated=progress->updated;total=progress->total;
      }
    }
    auto now=Clock::now();
    if(plan_stamp && std::chrono::duration<double>(now-updated).count()<0.75) {
      if(!entry_ || plan_stamp!=stamp_) {entry_=true;stamp_=plan_stamp;baseline_=remaining;last_=now;}
      if(baseline_-remaining>=std::min(.02,total*.25)) {baseline_=remaining;last_=now;}
      return std::chrono::duration<double>(now-last_).count()<allowance_;
    }
    if(entry_) {ordinary_->reset();entry_=false;}
    return ordinary_->check(current);
  }
private:
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::ProgressChecker>> loader_;
  nav2_core::ProgressChecker::Ptr ordinary_;std::vector<std::shared_ptr<Progress>> progresses_;
  bool entry_=false;int64_t stamp_=0;double baseline_=1e9,allowance_=20.;Clock::time_point last_{};
};
} // namespace recorded_route_nav2
PLUGINLIB_EXPORT_CLASS(recorded_route_nav2::RecordedGoalChecker,nav2_core::GoalChecker)
PLUGINLIB_EXPORT_CLASS(recorded_route_nav2::EntryProgressChecker,nav2_core::ProgressChecker)
