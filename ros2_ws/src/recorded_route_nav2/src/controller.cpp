#include <chrono>
#include <map>
#include <mutex>
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"
#include "recorded_route_nav2/geometry.hpp"

namespace recorded_route_nav2 {
using Clock=std::chrono::steady_clock;
struct Progress { std::mutex mutex; double remaining=1e9; int64_t stamp=0; Clock::time_point updated{}; };
std::mutex registry_mutex;
std::map<std::string,std::weak_ptr<Progress>> registry;
std::shared_ptr<Progress> sharedProgress(const std::string &key) {
  std::lock_guard<std::mutex> lock(registry_mutex);
  auto value=registry[key].lock();
  if(!value) {value=std::make_shared<Progress>();registry[key]=value;}
  return value;
}
template<class NodeT,class T> T param(const NodeT &node,const std::string &key,const T &fallback) {
  nav2_util::declare_parameter_if_not_declared(node,key,rclcpp::ParameterValue(fallback));
  return node->get_parameter(key).template get_value<T>();
}

class RecordedController : public nav2_core::Controller {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent.lock(); name_=name; tf_=tf;
    progress_=sharedProgress(std::string(node_->get_node_base_interface()->get_fully_qualified_name())+"/"+name);
    deviation_=param(node_,name+".strict_deviation",0.20);
    heading_=param(node_,name+".max_heading_error",0.55);
    max_speed_=param(node_,name+".desired_linear_vel",0.20);
    loader_=std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>("nav2_core","nav2_core::Controller");
    rpp_=loader_->createSharedInstance("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");
    rpp_->configure(parent,name,tf,costmap);
    permit_sub_=node_->create_subscription<std_msgs::msg::String>("/patrol/recorded_motion_permit",10,
      [this](std_msgs::msg::String::ConstSharedPtr msg){
        try {auto j=Json::parse(msg->data);std::lock_guard<std::mutex> l(mutex_);
          permit_=j.value("valid",false);permit_stamp_=j.value("plan_stamp",int64_t(0));
          permitted_speed_=j.value("speed_limit",0.2);permit_time_=Clock::now();
        } catch(const std::exception &) {std::lock_guard<std::mutex> l(mutex_);permit_=false;}
      });
    publisher_=node_->create_publisher<std_msgs::msg::String>("/patrol/recorded_controller_progress",10);
    faults_=node_->create_publisher<std_msgs::msg::String>("/patrol/recorded_controller_fault",10);
  }
  void activate() override {rpp_->activate();publisher_->on_activate();faults_->on_activate();}
  void deactivate() override {rpp_->deactivate();publisher_->on_deactivate();faults_->on_deactivate();}
  void cleanup() override {rpp_->cleanup();permit_sub_.reset();publisher_.reset();}
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(path.poses.size()<2) throw nav2_core::PlannerException("recorded road has fewer than two poses");
    plan_=path;index_=0;arc_.assign(path.poses.size(),0);
    for(size_t i=1;i<path.poses.size();++i) arc_[i]=arc_[i-1]+dist(path.poses[i-1].pose,path.poses[i].pose);
    last_compute_=Clock::now();first_=true;
    const auto &a=path.poses[0].pose; const auto &b=path.poses[1].pose;
    double along=(b.position.x-a.position.x)*std::cos(tf2::getYaw(a.orientation))+
                 (b.position.y-a.position.y)*std::sin(tf2::getYaw(a.orientation));
    direction_=along>=0?1:-1;
    std::lock_guard<std::mutex> p(progress_->mutex);
    progress_->stamp=stamp(plan_.header.stamp);progress_->remaining=arc_.back();progress_->updated=Clock::now();
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &input,
          const geometry_msgs::msg::Twist &velocity,nav2_core::GoalChecker *checker) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now=Clock::now();
    if(!permit_ || permit_stamp_!=stamp(plan_.header.stamp) ||
       std::chrono::duration<double>(now-permit_time_).count()>0.6)
      throw nav2_core::PlannerException("recorded motion permission missing or stale");
    geometry_msgs::msg::PoseStamped current;
    try {current=tf_->transform(input,plan_.header.frame_id,tf2::durationFromSec(0.1));}
    catch(const tf2::TransformException &e) {throw nav2_core::PlannerException(e.what());}
    double dt=std::clamp(std::chrono::duration<double>(now-last_compute_).count(),0.0,0.3);
    double advance=first_?0.20:std::min(0.30,std::abs(velocity.linear.x)*dt*2+0.10);
    size_t best=index_;double error=dist(current.pose,plan_.poses[index_].pose);
    for(size_t i=index_;i<plan_.poses.size() && arc_[i]-arc_[index_]<=advance;++i) {
      double d=dist(current.pose,plan_.poses[i].pose);
      if(d<error) {best=i;error=d;}
    }
    if(error>deviation_) throw nav2_core::PlannerException("outside recorded road; no free replanning allowed");
    if(std::abs(wrap(tf2::getYaw(current.pose.orientation)-tf2::getYaw(plan_.poses[best].pose.orientation)))>heading_)
      throw nav2_core::PlannerException("vehicle heading does not match recorded road");
    index_=best;first_=false;last_compute_=now;
    nav_msgs::msg::Path window;window.header=plan_.header;
    size_t start=index_>0?index_-1:0;
    size_t end=std::min(index_+1,plan_.poses.size()-1);
    while(end+1<plan_.poses.size() && arc_[end]-arc_[index_]<6.0) ++end;
    window.poses.assign(plan_.poses.begin()+start,plan_.poses.begin()+end+1);
    rpp_->setPlan(window);
    rpp_->setSpeedLimit(std::min({max_speed_,permitted_speed_,external_limit_}),false);
    geometry_msgs::msg::TwistStamped command;
    try {command=rpp_->computeVelocityCommands(input,velocity,checker);}
    catch(const nav2_core::PlannerException &error) {
      const std::string text=error.what();
      std_msgs::msg::String fault;
      fault.data=Json({{"plan_stamp",stamp(plan_.header.stamp)},
        {"code",text.find("collision")!=std::string::npos?"COLLISION":"TRACKING_FAILURE"},{"message",text}}).dump();
      faults_->publish(fault);
      throw;
    }
    if(!std::isfinite(command.twist.linear.x) || !std::isfinite(command.twist.angular.z) ||
       command.twist.linear.x*direction_<-0.005)
      throw nav2_core::PlannerException("controller attempted an unrecorded direction change");
    {
      std::lock_guard<std::mutex> p(progress_->mutex);
      progress_->remaining=arc_.back()-arc_[index_];progress_->updated=now;
    }
    std_msgs::msg::String message;
    message.data=Json({{"plan_stamp",stamp(plan_.header.stamp)},{"index",index_},
      {"remaining",arc_.back()-arc_[index_]},{"cross_track",error}}).dump();
    publisher_->publish(message);
    return command;
  }
  void setSpeedLimit(const double &value,const bool &percentage) override {
    std::lock_guard<std::mutex> lock(mutex_);
    external_limit_=value==0?max_speed_:(percentage?max_speed_*value/100:value);
  }
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  nav2_core::Controller::Ptr rpp_;
  std::shared_ptr<Progress> progress_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr permit_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr faults_;
  std::mutex mutex_;nav_msgs::msg::Path plan_;std::vector<double> arc_;
  size_t index_=0;int direction_=1;bool first_=true,permit_=false;
  int64_t permit_stamp_=0;
  double deviation_=0.2,heading_=0.55,max_speed_=0.2,permitted_speed_=0.2,external_limit_=1e6;
  Clock::time_point permit_time_{},last_compute_{};
};

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
    return std::chrono::duration<double>(Clock::now()-progress_->updated).count()<1.0 &&
      progress_->remaining<=tolerance_ && dist(query,goal)<=tolerance_ &&
      (yaw_tolerance_<0 || std::abs(wrap(tf2::getYaw(query.orientation)-tf2::getYaw(goal.orientation)))<=yaw_tolerance_);
  }
  bool getTolerances(Pose &p,geometry_msgs::msg::Twist &v) override {
    p.position.x=tolerance_;p.position.y=tolerance_;
    p.position.z=std::numeric_limits<double>::lowest();
    p.orientation.x=p.orientation.y=p.orientation.z=p.orientation.w=std::numeric_limits<double>::lowest();
    if(yaw_tolerance_>=0) p.orientation=fromJson(Json{{"x",0},{"y",0},{"yaw",yaw_tolerance_}}).orientation;
    v.linear.x=v.linear.y=v.linear.z=v.angular.x=v.angular.y=v.angular.z=std::numeric_limits<double>::lowest();
    return true;
  }
private: std::shared_ptr<Progress> progress_;double tolerance_=0.2,yaw_tolerance_=-1.0;
};

// Humble has one progress checker for all controllers. Preserve its original
// detector for ordinary navigation; low-speed entry uses ordered arc progress
// so a valid short reversing curve is not required to travel 50cm in 20s.
class EntryProgressChecker : public nav2_core::ProgressChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,const std::string &name) override {
    auto node=parent.lock();
    auto id=param(node,name+".entry_controller_id",std::string("RecordedEntryRoute"));
    progress_=sharedProgress(std::string(node->get_node_base_interface()->get_fully_qualified_name())+"/"+id);
    allowance_=param(node,name+".entry_time_allowance",20.0);
    loader_=std::make_unique<pluginlib::ClassLoader<nav2_core::ProgressChecker>>("nav2_core","nav2_core::ProgressChecker");
    ordinary_=loader_->createSharedInstance("nav2_controller::SimpleProgressChecker");
    ordinary_->initialize(parent,name);reset();
  }
  void reset() override {ordinary_->reset();entry_=false;stamp_=0;baseline_=1e9;last_=Clock::now();}
  bool check(geometry_msgs::msg::PoseStamped &current) override {
    double remaining;int64_t plan_stamp;Clock::time_point updated;
    {std::lock_guard<std::mutex> lock(progress_->mutex);
      remaining=progress_->remaining;plan_stamp=progress_->stamp;updated=progress_->updated;}
    auto now=Clock::now();
    if(plan_stamp && std::chrono::duration<double>(now-updated).count()<0.75) {
      if(!entry_ || plan_stamp!=stamp_) {entry_=true;stamp_=plan_stamp;baseline_=remaining;last_=now;}
      if(baseline_-remaining>=.05) {baseline_=remaining;last_=now;}
      return std::chrono::duration<double>(now-last_).count()<allowance_;
    }
    if(entry_) {ordinary_->reset();entry_=false;}
    return ordinary_->check(current);
  }
private:
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::ProgressChecker>> loader_;
  nav2_core::ProgressChecker::Ptr ordinary_;std::shared_ptr<Progress> progress_;
  bool entry_=false;int64_t stamp_=0;double baseline_=1e9,allowance_=20.;Clock::time_point last_{};
};
} // namespace recorded_route_nav2
PLUGINLIB_EXPORT_CLASS(recorded_route_nav2::RecordedController,nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(recorded_route_nav2::RecordedGoalChecker,nav2_core::GoalChecker)
PLUGINLIB_EXPORT_CLASS(recorded_route_nav2::EntryProgressChecker,nav2_core::ProgressChecker)
