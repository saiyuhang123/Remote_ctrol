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
#include "recorded_route_nav2/ordered_pursuit.hpp"
#include "recorded_route_nav2/progress.hpp"

namespace recorded_route_nav2 {
class RecordedController : public nav2_core::Controller {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent.lock(); name_=name; tf_=tf;
    progress_=sharedProgress(std::string(node_->get_node_base_interface()->get_fully_qualified_name())+"/"+name);
    deviation_=param(node_,name+".strict_deviation",0.20);
    heading_=param(node_,name+".max_heading_error",0.55);
    max_speed_=param(node_,name+".desired_linear_vel",0.20);
    radius_=param(node_,name+".minimum_turning_radius",0.69);
    if(!std::isfinite(radius_) || radius_<=0)
      throw std::invalid_argument("minimum_turning_radius must be positive");
    rpp_=std::make_unique<OrderedPursuit>();
    rpp_->configure(parent,name,tf,costmap);
    permit_sub_=node_->create_subscription<std_msgs::msg::String>("/patrol/recorded_motion_permit",10,
      [this](std_msgs::msg::String::ConstSharedPtr msg){
        try {auto j=Json::parse(msg->data);std::lock_guard<std::mutex> l(mutex_);
          permit_=j.value("valid",false);permit_stamp_=j.value("plan_stamp",int64_t(0));
          permitted_speed_=j.value("speed_limit",0.2);
          permitted_direction_=j.value("direction",0);
          tolerance_cap_=j.value("stop_tolerance_cap",0.0);
          motion_length_=j.value("motion_length",0.0);
          ticket_=j.value("ticket_id",std::string());block_=j.value("block",size_t(0));
          permit_=permit_ && (permitted_direction_==1 || permitted_direction_==-1) &&
            std::isfinite(permitted_speed_) && permitted_speed_>0 &&
            std::isfinite(tolerance_cap_) && tolerance_cap_>0 &&
            std::isfinite(motion_length_) && motion_length_>0;
          permit_time_=Clock::now();
        } catch(const std::exception &) {std::lock_guard<std::mutex> l(mutex_);permit_=false;}
      });
    publisher_=node_->create_publisher<std_msgs::msg::String>("/patrol/recorded_controller_progress",10);
    faults_=node_->create_publisher<std_msgs::msg::String>("/patrol/recorded_controller_fault",10);
  }
  void activate() override {rpp_->activate();publisher_->on_activate();faults_->on_activate();}
  void deactivate() override {rpp_->deactivate();publisher_->on_deactivate();faults_->on_deactivate();}
  void cleanup() override {rpp_->cleanup();permit_sub_.reset();publisher_.reset();faults_.reset();}
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(path.poses.size()<2) throw nav2_core::PlannerException("recorded road has fewer than two poses");
    direction_=geometryDirection(path);
    plan_=path;index_=0;arc_=cumulativeArc(path);s_=0;error_=0;
    last_compute_=Clock::now();first_=true;
    std::lock_guard<std::mutex> p(progress_->mutex);
    progress_->stamp=stamp(plan_.header.stamp);progress_->remaining=arc_.back();
    progress_->total=arc_.back();progress_->tolerance_cap=1e9;
    progress_->ready=false;progress_->updated=Clock::now();
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &input,
          const geometry_msgs::msg::Twist &velocity,nav2_core::GoalChecker *checker) override {
    std::lock_guard<std::mutex> lock(mutex_);
    try {return compute(input,velocity,checker);}
    catch(const std::exception &error) {
      const std::string message=error.what();
      std::string code="TRACKING_FAILURE";
      if(message.find("collision")!=std::string::npos) code="COLLISION";
      else if(message.find("direction")!=std::string::npos) code="DIRECTION_MISMATCH";
      else if(message.find("non-finite")!=std::string::npos) code="NONFINITE_COMMAND";
      else if(message.find("endpoint is behind")!=std::string::npos) code="ENDPOINT_UNREACHABLE";
      else if(message.find("permission")!=std::string::npos) code="INVALID_PERMIT";
      std_msgs::msg::String fault;
      fault.data=Json({{"plan_stamp",stamp(plan_.header.stamp)},{"ticket_id",ticket_},
        {"block",block_},{"controller",name_},{"code",code},{"message",message},
        {"direction",direction_},{"authorized_direction",permitted_direction_},
        {"index",index_},{"remaining",arc_.empty()?0:arc_.back()-s_},{"cross_track",error_},
        {"carrot_x",rpp_->carrot_x},{"carrot_y",rpp_->carrot_y},
        {"curvature",rpp_->curvature}}).dump();
      faults_->publish(fault);
      throw nav2_core::PlannerException(message);
    }
  }
  geometry_msgs::msg::TwistStamped compute(const geometry_msgs::msg::PoseStamped &input,
          const geometry_msgs::msg::Twist &velocity,nav2_core::GoalChecker *checker) {
    auto now=Clock::now();
    if(plan_.poses.size()<2 || !permit_ || permit_stamp_!=stamp(plan_.header.stamp) ||
       std::chrono::duration<double>(now-permit_time_).count()>0.6)
      throw nav2_core::PlannerException("recorded motion permission missing or stale");
    if(permitted_direction_!=direction_)
      throw nav2_core::PlannerException("ticket direction contradicts recorded path geometry");
    geometry_msgs::msg::PoseStamped current;
    try {current=tf_->transform(input,plan_.header.frame_id,tf2::durationFromSec(0.1));}
    catch(const tf2::TransformException &e) {throw nav2_core::PlannerException(e.what());}
    if(!std::isfinite(current.pose.position.x) || !std::isfinite(current.pose.position.y) ||
       !std::isfinite(tf2::getYaw(current.pose.orientation)) ||
       !std::isfinite(velocity.linear.x) || !std::isfinite(velocity.angular.z))
      throw nav2_core::PlannerException("non-finite recorded tracking state");
    if(motion_length_+1e-6<arc_.back())
      throw nav2_core::PlannerException("motion permission length shorter than active path");
    double dt=std::clamp(std::chrono::duration<double>(now-last_compute_).count(),0.0,0.3);
    double advance=first_?0.20:std::min(0.30,std::abs(velocity.linear.x)*dt*2+0.10);
    if(first_ && dist(current.pose,plan_.poses.front().pose)>
        std::min(deviation_,std::max(.02,motion_length_*.25)))
      throw nav2_core::PlannerException("not at the start of recorded motion; short block cannot be skipped");
    auto projected=orderedProjection(plan_,arc_,current.pose,s_,advance);
    error_=projected.error;
    if(error_>deviation_) throw nav2_core::PlannerException("outside recorded road; no free replanning allowed");
    if(std::abs(wrap(tf2::getYaw(current.pose.orientation)-tf2::getYaw(projected.pose.orientation)))>heading_)
      throw nav2_core::PlannerException("vehicle heading does not match recorded road");
    s_=projected.s;index_=projected.index;first_=false;last_compute_=now;
    {
      std::lock_guard<std::mutex> p(progress_->mutex);
      progress_->remaining=arc_.back()-s_;progress_->tolerance_cap=tolerance_cap_;
      progress_->total=motion_length_;
      progress_->ready=true;progress_->updated=now;
    }
    std_msgs::msg::String message;
    message.data=Json({{"plan_stamp",stamp(plan_.header.stamp)},{"index",index_},
      {"remaining",arc_.back()-s_},{"arc",s_},{"cross_track",error_},
      {"direction",direction_},{"controller",name_}}).dump();
    publisher_->publish(message);
    // Humble checks the goal AFTER computeVelocityCommands. Do not ask RPP
    // to drive back to an endpoint already inside the arrival tolerance.
    if(checker->isGoalReached(current.pose,plan_.poses.back().pose,velocity)) {
      geometry_msgs::msg::TwistStamped zero;zero.header=input.header;return zero;
    }
    nav_msgs::msg::Path window;window.header=plan_.header;
    geometry_msgs::msg::PoseStamped start;start.header=window.header;start.pose=projected.pose;
    window.poses.push_back(start);
    for(size_t i=index_+1;i<plan_.poses.size();++i) {
      window.poses.push_back(plan_.poses[i]);
      if(arc_[i]-s_>=6.0) break;
    }
    if(window.poses.size()==1) window.poses.push_back(plan_.poses.back());
    rpp_->setPlan(window);
    rpp_->setSpeedLimit(std::min({max_speed_,permitted_speed_,external_limit_}),false);
    Pose tolerance;geometry_msgs::msg::Twist unused;
    if(!checker->getTolerances(tolerance,unused))
      throw nav2_core::PlannerException("recorded goal tolerances unavailable");
    auto command=rpp_->computeDirected(input,velocity,direction_,radius_,arc_.back()-s_,tolerance.position.x);
    if(command.twist.linear.x*direction_<-0.005)
      throw nav2_core::PlannerException("controller attempted an unrecorded direction change");
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
  std::unique_ptr<OrderedPursuit> rpp_;
  std::shared_ptr<Progress> progress_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr permit_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr faults_;
  std::mutex mutex_;nav_msgs::msg::Path plan_;std::vector<double> arc_;
  size_t index_=0,block_=0;int direction_=1,permitted_direction_=0;bool first_=true,permit_=false;
  std::string ticket_;
  int64_t permit_stamp_=0;
  double s_=0,error_=0,radius_=.69,tolerance_cap_=0,motion_length_=0;
  double deviation_=0.2,heading_=0.55,max_speed_=0.2,permitted_speed_=0.2,external_limit_=1e6;
  Clock::time_point permit_time_{},last_compute_{};
};

} // namespace recorded_route_nav2
PLUGINLIB_EXPORT_CLASS(recorded_route_nav2::RecordedController,nav2_core::Controller)
