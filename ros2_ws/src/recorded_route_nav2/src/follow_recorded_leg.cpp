#include <atomic>
#include <chrono>
#include <thread>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "patrol_interfaces/srv/road_command.hpp"
#include "recorded_route_nav2/geometry.hpp"

namespace recorded_route_nav2 {
using Steady=std::chrono::steady_clock;
using Follow=nav2_msgs::action::FollowPath;
using Compute=nav2_msgs::action::ComputePathToPose;
using Road=patrol_interfaces::srv::RoadCommand;
using FollowHandle=rclcpp_action::ClientGoalHandle<Follow>;
using PlanHandle=rclcpp_action::ClientGoalHandle<Compute>;
static double age(Steady::time_point t) {return std::chrono::duration<double>(Steady::now()-t).count();}

class FollowRecordedLeg : public BT::StatefulActionNode {
public:
  FollowRecordedLeg(const std::string &name,const BT::NodeConfiguration &config):BT::StatefulActionNode(name,config) {
    auto parent=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    static std::atomic<unsigned> instance{0};
    rclcpp::NodeOptions options;options.context(parent->get_node_base_interface()->get_context());
    options.use_global_arguments(false);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time",parent->get_parameter("use_sim_time").as_bool())});
    node_=std::make_shared<rclcpp::Node>("recorded_leg_"+std::to_string(++instance),options);
    executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();executor_->add_node(node_);
    tf_=std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    listener_=std::make_shared<tf2_ros::TransformListener>(*tf_,node_,true);
    road_=node_->create_client<Road>("/patrol/road_command");
    follow_=rclcpp_action::create_client<Follow>(node_,"/follow_path");
    planner_=rclcpp_action::create_client<Compute>(node_,"/compute_path_to_pose");
    permit_pub_=node_->create_publisher<std_msgs::msg::String>("/patrol/recorded_motion_permit",10);
    progress_pub_=node_->create_publisher<std_msgs::msg::String>("/patrol/recorded_progress",10);
    odom_sub_=node_->create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){odom_=*msg;odom_time_=Steady::now();});
    local_sub_=node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap",rclcpp::QoS(1).best_effort(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){local_=msg;local_time_=Steady::now();});
    global_sub_=node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",rclcpp::QoS(1).best_effort(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){global_=msg;global_time_=Steady::now();});
    ctrl_sub_=node_->create_subscription<std_msgs::msg::String>("/patrol/recorded_controller_progress",10,
      [this](std_msgs::msg::String::ConstSharedPtr msg){
        try {auto value=Json::parse(msg->data);
          if(value.value("plan_stamp",int64_t(0))==stamp(active_path_.header.stamp)) {
            auto index=value.value("index",size_t(0));
            if(detour_) detour_index_=std::min(index+active_offset_,detour_path_.poses.size()-1);
            else reference_index_=std::min(index+active_offset_,reference_.poses.size()-1);
            ctrl_time_=Steady::now();
          }
        } catch(const std::exception &) {}
      });
    fault_sub_=node_->create_subscription<std_msgs::msg::String>("/patrol/recorded_controller_fault",10,
      [this](std_msgs::msg::String::ConstSharedPtr msg){
        try {auto value=Json::parse(msg->data);
          if(value.value("plan_stamp",int64_t(0))==stamp(active_path_.header.stamp)) controller_fault_=value.value("code",std::string());
        } catch(const std::exception &) {}
      });
  }
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("ticket_id")};}

  BT::NodeStatus onStart() override {
    getInput("ticket_id",ticket_);
    if(ticket_.empty()) return BT::NodeStatus::FAILURE;
    generation_++;phase_=Phase::LOAD;ticket_data_=Json();road_pending_=false;
    stopping_=false;send_pending_=false;follow_done_=false;follow_handle_.reset();
    block_=reference_index_=rejoin_index_=detour_index_=plan_attempt_=0;detour_=false;last_error_.clear();
    started_=Steady::now();last_health_=Steady::time_point{};next_health_=Steady::time_point{};
    complete_time_=Steady::time_point{};wait_started_=Steady::time_point{};
    last_plan_=Steady::time_point{};plan_handle_.reset();plan_done_=false;detour_rejection_.clear();
    requestRoad("LOAD_LEG");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    executor_->spin_some();
    try {
      pollRoad();
      if(phase_==Phase::LOAD) {
        if(age(started_)>4.0) fail("录制路径票据加载超时");
        else if(!road_pending_ && age(next_health_)>=0) requestRoad("LOAD_LEG");
        return BT::NodeStatus::RUNNING;
      }
      if(phase_!=Phase::FAIL && phase_!=Phase::COMPLETE) {
        if(age(last_health_)>1.5) fail("执行票据/操作端健康检查过期");
        else if(!road_pending_ && age(next_health_)>=0) requestRoad("CHECK_LEG");
      }
      if(phase_==Phase::FAIL) {
        permission(false);stopFollow();emitProgress("FAILED");
        return stopped()?BT::NodeStatus::FAILURE:BT::NodeStatus::RUNNING;
      }
      Pose current=currentPose();
      if(phase_==Phase::START) {
        if(!mapsFresh() || age(odom_time_)>1.5) {
          if(age(phase_time_)>5) fail("costmap/里程计未就绪");
          permission(false);return BT::NodeStatus::RUNNING;
        }
        if(block_>=ticket_data_["blocks"].size()) {phase_=Phase::COMPLETE;complete_time_=Steady::now();}
        else {
          if(detour_) {
            if(!pathFree(detour_path_,detour_index_,detour_path_.poses.size()-1,true)) beginWait();
            else startFollow(detour_path_,detour_index_);
          } else if(blockedAhead()) beginWait();
          else startFollow(reference_,reference_index_);
        }
      } else if(phase_==Phase::FOLLOW) {
        if(!mapsFresh() || age(odom_time_)>1.5) fail("运行 costmap/里程计过期");
        else if(controller_fault_=="COLLISION" || blockedAhead()) {stop_reason_=Stop::OBSTACLE;stopFollow();phase_=Phase::STOP;phase_time_=Steady::now();stable_=Steady::time_point{};}
        else if(follow_done_) {
          if(follow_result_!=rclcpp_action::ResultCode::SUCCEEDED) fail("路径跟踪失败，禁止自由重规划");
          else {
            auto &path=detour_?detour_path_:reference_;
            size_t index=detour_?detour_index_:reference_index_;
            bool entry=ticket_data_["blocks"][block_].value("entry",false);
            double tolerance=entry?0.06:((!detour_ && block_+1<ticket_data_["blocks"].size())?0.09:0.21);
            if(pathLength(path,index)>tolerance+0.05 || dist(current,path.poses.back().pose)>tolerance)
              fail("控制器成功但有序路径进度未完成");
            else if(entry && std::abs(wrap(tf2::getYaw(current.orientation)-tf2::getYaw(path.poses.back().pose.orientation)))>0.15)
              fail("自动接入运动段结束时车头未对齐");
            else {stop_reason_=detour_?Stop::REJOIN:Stop::END;phase_=Phase::STOP;phase_time_=Steady::now();stable_=Steady::time_point{};}
          }
        }
      } else if(phase_==Phase::STOP) {
        permission(false);stopFollow();
        if(stopped() && stationary()) {
          if(stop_reason_==Stop::OBSTACLE) beginWait();
          else if(stop_reason_==Stop::REJOIN) {
            const auto &target=reference_.poses.at(rejoin_index_).pose;
            if(dist(current,target)>0.15 || std::abs(wrap(tf2::getYaw(current.orientation)-tf2::getYaw(target.orientation)))>0.25)
              fail("绕障重入位姿不合格");
            else {reference_index_=rejoin_index_;detour_=false;phase_=Phase::START;phase_time_=Steady::now();wait_started_=Steady::time_point{};}
          } else {
            const auto &block=ticket_data_["blocks"][block_];
            // Recheck after the physical stop, before releasing the next
            // gear or the recorded road. A business 20cm stop is not enough.
            Pose target=block.contains("entry_target")?fromJson(block["entry_target"]):reference_.poses.back().pose;
            if(block.value("entry",false) && (dist(current,target)>0.10 ||
               std::abs(wrap(tf2::getYaw(current.orientation)-tf2::getYaw(target.orientation)))>0.20))
              fail("自动接入停车后位姿未对齐，已停止任务");
            else {
              block_++;
              if(block_>=ticket_data_["blocks"].size()) {phase_=Phase::COMPLETE;complete_time_=Steady::now();}
              else {loadBlock();phase_=Phase::START;phase_time_=Steady::now();}
            }
          }
        } else if(age(phase_time_)>10) fail("运动段停止未确认");
      } else if(phase_==Phase::WAIT) {
        permission(false);
        if(age(wait_started_)>ticket_data_.value("wait_timeout",120.0)) fail("障碍等待超时");
        else if(mapsFresh() && stopped()) {
          bool aligned=pointPathDistance(current,reference_,reference_index_,std::min(reference_index_+8,reference_.poses.size()-1))<0.12;
          if(aligned && !referenceBlockedAhead()) {detour_=false;phase_=Phase::START;phase_time_=Steady::now();}
          else if(age(last_plan_)>2.0 && !ticket_data_["blocks"][block_].value("entry",false) &&
                  ticket_data_["blocks"][block_]["direction"].get<int>()>0) requestPlan(current);
        }
      } else if(phase_==Phase::PLAN) {
        permission(false);
        if(plan_done_) {
          plan_done_=false;
          if(plan_result_==rclcpp_action::ResultCode::SUCCEEDED && via_stage_==1) {
            // A lateral waypoint provides bounded left/right alternatives
            // when the unconstrained shortest candidate leaves the band.
            // Both halves remain stationary planning, never motion goals.
            prefix_=candidate_;via_stage_=2;
            if(prefix_.poses.empty()) {phase_=Phase::WAIT;}
            else sendPlan(prefix_.poses.back().pose,reference_.poses[rejoin_index_].pose);
            return BT::NodeStatus::RUNNING;
          }
          if(plan_result_==rclcpp_action::ResultCode::SUCCEEDED && via_stage_==2 && !candidate_.poses.empty()) {
            auto second=candidate_;candidate_=prefix_;
            candidate_.poses.insert(candidate_.poses.end(),second.poses.begin()+1,second.poses.end());
          }
          if(plan_result_==rclcpp_action::ResultCode::SUCCEEDED && validDetour(candidate_)) {
            // Smac lattice samples may be farther apart than the ordered
            // controller search horizon at low speed. Densify the same
            // validated curve so progress never becomes stuck at sample 0.
            detour_path_=densePath(candidate_);detour_=true;detour_index_=0;
            phase_=Phase::START;phase_time_=Steady::now();
          } else {
            if(plan_result_!=rclcpp_action::ResultCode::SUCCEEDED) detour_rejection_="规划器没有返回可用路径";
            RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),5000,"Detour rejected: %s",detour_rejection_.c_str());
            phase_=Phase::WAIT;
          }
        } else if(age(last_plan_)>5) {
          if(plan_handle_) planner_->async_cancel_goal(plan_handle_);
          fail("局部绕障规划超时");
        }
      }
      if(phase_==Phase::COMPLETE) {
        permission(false);emitProgress("COMPLETED");
        return age(complete_time_)>0.25?BT::NodeStatus::SUCCESS:BT::NodeStatus::RUNNING;
      }
      permission(phase_==Phase::FOLLOW && !stopping_ && age(last_health_)<1.5);
      emitProgress(label());
    } catch(const std::exception &error) {fail(error.what());permission(false);stopFollow();}
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
    permission(false);stopFollow();
    if(plan_handle_ && !plan_done_) {
      try {planner_->async_cancel_goal(plan_handle_);} catch(const rclcpp_action::exceptions::UnknownGoalHandleError &) {}
    }
    auto deadline=Steady::now()+std::chrono::seconds(2);
    while(!stopped() && Steady::now()<deadline) {
      executor_->spin_some();permission(false);stopFollow();std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    generation_++;emitProgress(stopped()?"STOPPED":"STOP_UNCONFIRMED");
    if(!stopped()) RCLCPP_ERROR(node_->get_logger(),"FollowPath stop unconfirmed; motion permission revoked");
  }

private:
  enum class Phase {LOAD,START,FOLLOW,STOP,WAIT,PLAN,COMPLETE,FAIL};
  enum class Stop {END,OBSTACLE,REJOIN};
  void requestRoad(const std::string &op) {
    if(!road_->service_is_ready()) {next_health_=Steady::now()+std::chrono::milliseconds(100);return;}
    auto request=std::make_shared<Road::Request>();request->operation=op;
    request->payload_json=Json({{"ticket_id",ticket_}}).dump();
    road_future_=road_->async_send_request(request).share();road_pending_=true;road_operation_=op;road_time_=Steady::now();
  }
  void pollRoad() {
    if(!road_pending_) return;
    if(road_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {
      if(age(road_time_)>2) fail("路径票据服务超时");
      return;
    }
    auto response=road_future_.get();road_pending_=false;
    next_health_=Steady::now()+std::chrono::milliseconds(400);
    if(!response->success) {
      if(phase_==Phase::LOAD && response->error_code=="STALE_TICKET" && age(started_)<3) return;
      fail(response->message);return;
    }
    last_health_=Steady::now();
    if(road_operation_=="LOAD_LEG") {
      ticket_data_=Json::parse(response->data_json);
      if(ticket_data_.at("ticket_id")!=ticket_ || !ticket_data_.contains("blocks") || ticket_data_["blocks"].empty())
        throw std::runtime_error("empty or mismatched recorded leg");
      footprint_.clear();region_.clear();
      for(auto &p:ticket_data_["footprint"]) footprint_.emplace_back(p[0],p[1]);
      for(auto &p:ticket_data_.value("allowed_polygon",Json::array())) region_.emplace_back(p[0],p[1]);
      phase_=Phase::START;phase_time_=Steady::now();block_=0;loadBlock();
      auto resume=ticket_data_.value("resume",Json::object());
      if(!resume.empty()) {
        block_=resume.value("block",size_t(0));
        if(block_>=ticket_data_["blocks"].size()) throw std::runtime_error("invalid resume block");
        loadBlock();reference_index_=std::min(resume.value("reference_index",size_t(0)),reference_.poses.size()-1);
        if(resume.contains("detour_points") && !resume["detour_points"].empty()) {
          detour_path_=makePath(resume["detour_points"]);detour_=true;
          detour_index_=std::min(resume.value("detour_index",size_t(0)),detour_path_.poses.size()-1);
          rejoin_index_=resume.value("rejoin_index",reference_index_);
          if(rejoin_index_>=reference_.poses.size()) throw std::runtime_error("invalid rejoin index");
        }
      }
    }
  }
  nav_msgs::msg::Path makePath(const Json &points) {
    nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp=node_->now();
    for(auto &point:points) {geometry_msgs::msg::PoseStamped pose;pose.header=path.header;pose.pose=fromJson(point);path.poses.push_back(pose);}
    if(path.poses.size()<2) throw std::runtime_error("path has fewer than 2 points");
    return path;
  }
  void loadBlock() {
    reference_=makePath(ticket_data_["blocks"][block_]["samples"]);
    reference_index_=0;detour_index_=0;detour_=false;
  }
  Pose currentPose() {
    auto transform=tf_->lookupTransform("map","base_link",tf2::TimePointZero);
    double seconds=(node_->now()-rclcpp::Time(transform.header.stamp)).seconds();
    if(seconds < -0.5) throw std::runtime_error("map to base TF clock is ahead; age="+std::to_string(seconds));
    if(seconds>1.5) throw std::runtime_error("map to base TF stale; age="+std::to_string(seconds)+"s (limit 1.5s)");
    Pose result;result.position.x=transform.transform.translation.x;result.position.y=transform.transform.translation.y;
    result.orientation=transform.transform.rotation;return result;
  }
  void startFollow(const nav_msgs::msg::Path &source,size_t offset) {
    if(send_pending_ || (follow_handle_ && !follow_done_)) throw std::runtime_error("previous FollowPath still active");
    offset=std::min(offset,source.poses.size()-2);
    Pose current=currentPose();
    if(dist(current,source.poses[offset].pose)>0.20 ||
       std::abs(wrap(tf2::getYaw(current.orientation)-tf2::getYaw(source.poses[offset].pose.orientation)))>0.40)
      throw std::runtime_error("cannot resume off the recorded motion segment");
    if(!follow_->action_server_is_ready()) throw std::runtime_error("FollowPath server not ready");
    active_path_=source;active_path_.poses.assign(source.poses.begin()+offset,source.poses.end());
    active_path_.header.stamp=node_->now();active_offset_=offset;
    follow_done_=false;stopping_=false;follow_handle_.reset();send_pending_=true;
    controller_fault_.clear();
    phase_=Phase::FOLLOW;permission(true);
    Follow::Goal goal;goal.path=active_path_;
    goal.controller_id=ticket_data_["blocks"][block_].value("entry",false)?"RecordedEntryRoute":"RecordedRoute";
    goal.goal_checker_id=ticket_data_["blocks"][block_].value("entry",false)?"recorded_entry_checker":
      ((detour_ || block_+1<ticket_data_["blocks"].size())?"recorded_stop_checker":"recorded_goal_checker");
    const unsigned gen=generation_;
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.goal_response_callback=[this,gen](FollowHandle::SharedPtr handle){
      if(gen!=generation_) {if(handle) follow_->async_cancel_goal(handle);return;}
      send_pending_=false;follow_handle_=handle;
      if(!handle) {follow_done_=true;follow_result_=rclcpp_action::ResultCode::ABORTED;}
      else if(stopping_) follow_->async_cancel_goal(handle);
    };
    options.result_callback=[this,gen](const FollowHandle::WrappedResult &result){
      if(gen==generation_) {follow_done_=true;follow_result_=result.code;}
    };
    follow_->async_send_goal(goal,options);
  }
  void stopFollow() {
    permission(false);
    if(!stopping_) {stopping_=true;if(follow_handle_ && !follow_done_) follow_->async_cancel_goal(follow_handle_);}
  }
  bool stopped() const {return !send_pending_ && (!follow_handle_ || follow_done_);}
  bool stationary() {
    if(age(odom_time_)>1.5 || !std::isfinite(odom_.twist.twist.linear.x) || !std::isfinite(odom_.twist.twist.angular.z) ||
       std::abs(odom_.twist.twist.linear.x)>0.03 || std::abs(odom_.twist.twist.angular.z)>0.05) {
      stable_=Steady::time_point{};return false;
    }
    if(stable_==Steady::time_point{}) stable_=Steady::now();
    return age(stable_)>=0.8;
  }
  void permission(bool valid) {
    if(!permit_pub_) return;
    std_msgs::msg::String value;
    double speed=0.2;
    if(!ticket_data_.is_null() && block_<ticket_data_["blocks"].size()) speed=ticket_data_["blocks"][block_].value("speed",0.2);
    value.data=Json({{"valid",valid},{"plan_stamp",stamp(active_path_.header.stamp)},
                     {"ticket_id",ticket_},{"speed_limit",std::min(detour_?0.10:0.20,speed)}}).dump();
    permit_pub_->publish(value);
  }
  bool mapsFresh() const {return local_ && global_ && age(local_time_)<2.0 && age(global_time_)<3.0;}
  bool freeIn(const Pose &pose,const nav_msgs::msg::OccupancyGrid &grid,bool outside_ok=false) {
    if(grid.info.resolution<=0 || grid.data.size()!=static_cast<size_t>(grid.info.width)*grid.info.height) return false;
    geometry_msgs::msg::PoseStamped input;input.header.frame_id="map";input.pose=pose;
    auto transformed=tf_->transform(input,grid.header.frame_id,tf2::durationFromSec(0.0));
    double yaw=tf2::getYaw(transformed.pose.orientation), gyaw=tf2::getYaw(grid.info.origin.orientation);
    Polygon raster,world;
    double cy=std::cos(yaw),sy=std::sin(yaw),cg=std::cos(gyaw),sg=std::sin(gyaw);
    double body_yaw=tf2::getYaw(pose.orientation),cb=std::cos(body_yaw),sb=std::sin(body_yaw);
    double minx=1e9,maxx=-1e9,miny=1e9,maxy=-1e9;
    for(auto [ox,oy]:footprint_) {
      double x=transformed.pose.position.x+cy*ox-sy*oy;
      double y=transformed.pose.position.y+sy*ox+cy*oy;
      double dx=x-grid.info.origin.position.x,dy=y-grid.info.origin.position.y;
      double gx=(cg*dx+sg*dy)/grid.info.resolution,gy=(-sg*dx+cg*dy)/grid.info.resolution;
      if(gx<0||gy<0||gx>=grid.info.width||gy>=grid.info.height) {
        if(outside_ok) return true; // remaining candidate is checked in the fresh global obstacle map
        free_reason_="超出已观测代价地图，车身顶点栅格="+std::to_string(gx)+","+std::to_string(gy);return false;
      }
      raster.emplace_back(gx,gy);minx=std::min(minx,gx);maxx=std::max(maxx,gx);miny=std::min(miny,gy);maxy=std::max(maxy,gy);
      double mx=pose.position.x+cb*ox-sb*oy,my=pose.position.y+sb*ox+cb*oy;
      if(!inside(mx,my,region_)) {free_reason_="超出允许区域";return false;}
      world.emplace_back(mx,my);
    }
    for(size_t i=0;i<world.size();++i) for(size_t j=0;j<region_.size();++j)
      if(properCross(world[i],world[(i+1)%world.size()],region_[j],region_[(j+1)%region_.size()])) return false;
    for(int y=std::floor(miny);y<=std::floor(maxy);++y) for(int x=std::floor(minx);x<=std::floor(maxx);++x) {
      int cost=grid.data[y*grid.info.width+x];
      if((cost<0||cost>=100) && polygonCell(raster,x,y)) {
        free_reason_="车身碰撞/未知栅格，路径位姿="+std::to_string(pose.position.x)+","+std::to_string(pose.position.y)+
          "，栅格="+std::to_string(x)+","+std::to_string(y)+"，代价="+std::to_string(cost);return false;
      }
    }
    return true;
  }
  bool pathFree(const nav_msgs::msg::Path &path,size_t from,size_t to,bool candidate=false) {
    if(!mapsFresh() || path.poses.empty()) return false;
    to=std::min(to,path.poses.size()-1);
    for(size_t i=from;i<=to;++i) {
      const auto &p=path.poses[i].pose;
      if(!freeIn(p,*global_) || !freeIn(p,*local_,candidate)) return false;
      if(i>from) {
        auto a=path.poses[i-1].pose;double dyaw=wrap(tf2::getYaw(p.orientation)-tf2::getYaw(a.orientation));
        int n=std::max(1,static_cast<int>(std::ceil(std::max(dist(a,p)/0.025,std::abs(dyaw)/0.04))));
        for(int k=1;k<n;++k) {
          double t=static_cast<double>(k)/n;Pose q=a;
          q.position.x+=t*(p.position.x-a.position.x);q.position.y+=t*(p.position.y-a.position.y);
          double heading=tf2::getYaw(a.orientation)+t*dyaw;q.orientation.z=std::sin(heading/2);q.orientation.w=std::cos(heading/2);
          if(!freeIn(q,*global_)||!freeIn(q,*local_,candidate)) return false;
        }
      }
    }
    return true;
  }
  bool referenceBlockedAhead() {
    size_t end=reference_index_;
    while(end+1<reference_.poses.size() && pathLength(reference_,reference_index_,end)<1.0) ++end;
    return !pathFree(reference_,reference_index_,end);
  }
  bool blockedAhead() {
    if(!detour_) return referenceBlockedAhead();
    size_t end=detour_index_;
    while(end+1<detour_path_.poses.size() && pathLength(detour_path_,detour_index_,end)<1.0) ++end;
    return !pathFree(detour_path_,detour_index_,end);
  }
  void beginWait() {
    phase_=Phase::WAIT;
    if(wait_started_==Steady::time_point{}) wait_started_=Steady::now();
    permission(false);
  }
  void requestPlan(const Pose &current) {
    last_plan_=Steady::now();
    if(!planner_->action_server_is_ready()) return;
    size_t limit=reference_index_;
    double max_skip=ticket_data_.value("max_skip",4.0);
    while(limit+1<reference_.poses.size() && pathLength(reference_,reference_index_,limit+1)<=max_skip) ++limit;
    for(auto &barrier:ticket_data_["blocks"][block_]["barriers"]) {
      size_t index=barrier.get<size_t>();
      if(index>reference_index_) limit=std::min(limit,index>2?index-2:reference_index_);
    }
    bool selected=false;
    for(size_t i=limit;i>reference_index_;--i) {
      if(pathLength(reference_,reference_index_,i)<0.60) break;
      if(freeIn(reference_.poses[i].pose,*global_)) {rejoin_index_=i;selected=true;break;}
    }
    if(!selected) return;
    via_stage_=0;
    unsigned alternative=plan_attempt_++%7;
    Pose target=reference_.poses[rejoin_index_].pose;
    if(alternative>=3) {
      // Hybrid-A* raster/smoothing can choose a wider path or cut a body
      // corner despite a feasible parallel bypass. Try exact forward arc
      // templates, subject to the very same complete candidate validation.
      double yaw=tf2::getYaw(current.orientation),c=std::cos(yaw),s=std::sin(yaw);
      for(size_t i=reference_index_;i<=rejoin_index_;++i) {
        const auto &p=reference_.poses[i].pose;
        double lateral=-s*(p.position.x-current.position.x)+c*(p.position.y-current.position.y);
        if(std::abs(lateral)>.08 || std::abs(wrap(tf2::getYaw(p.orientation)-yaw))>.10) return;
      }
      double width=ticket_data_.value("max_offset",0.8)*(alternative<5?.70:.90);
      auto candidate=laneChangeDetour(current,target,ticket_data_.value("minimum_turning_radius",.69),
                                       width*(alternative%2?-1:1));
      if(validDetour(candidate)) {detour_path_=candidate;detour_=true;detour_index_=0;phase_=Phase::START;phase_time_=Steady::now();}
      return;
    }
    if(alternative) {
      size_t blocked=reference_index_;
      while(blocked<rejoin_index_ && freeIn(reference_.poses[blocked].pose,*global_)) ++blocked;
      double anchor=std::min(pathLength(reference_,reference_index_,blocked)+0.90,
                             pathLength(reference_,reference_index_,rejoin_index_)-0.65);
      size_t index=reference_index_;
      while(index<rejoin_index_ && pathLength(reference_,reference_index_,index)<anchor) ++index;
      if(anchor<0.65 || index>=rejoin_index_) return;
      target=reference_.poses[index].pose;double yaw=tf2::getYaw(target.orientation);
      double offset=ticket_data_.value("max_offset",0.8)*0.70*(alternative==1?-1:1);
      target.position.x-=std::sin(yaw)*offset;target.position.y+=std::cos(yaw)*offset;
      if(!freeIn(target,*global_)) return;
      via_stage_=1;
    }
    sendPlan(current,target);
  }
  void sendPlan(const Pose &current,const Pose &target) {
    last_plan_=Steady::now();
    Compute::Goal goal;goal.start.header.frame_id="map";goal.start.header.stamp=node_->now();goal.start.pose=current;
    goal.goal.header=goal.start.header;goal.goal.pose=target;goal.planner_id="RecordedDetour";goal.use_start=true;
    phase_=Phase::PLAN;plan_done_=false;plan_handle_.reset();
    auto gen=generation_;
    rclcpp_action::Client<Compute>::SendGoalOptions options;
    options.goal_response_callback=[this,gen](PlanHandle::SharedPtr handle){
      if(gen!=generation_) {if(handle) planner_->async_cancel_goal(handle);return;}
      plan_handle_=handle;if(!handle) {plan_done_=true;plan_result_=rclcpp_action::ResultCode::ABORTED;}
    };
    options.result_callback=[this,gen](const PlanHandle::WrappedResult &result){
      if(gen!=generation_) return;
      plan_done_=true;plan_result_=result.code;
      if(result.result) candidate_=result.result->path;
    };
    planner_->async_send_goal(goal,options);
  }
  bool validDetour(const nav_msgs::msg::Path &path) {
    auto reject=[this](const std::string &why){detour_rejection_=why;return false;};
    if(path.header.frame_id!="map" || path.poses.size()<2 || pathLength(path)>ticket_data_.value("max_length",10.0)) return reject("路径坐标系/长度不合格");
    if(dist(path.poses.front().pose,currentPose())>0.10 || dist(path.poses.back().pose,reference_.poses[rejoin_index_].pose)>0.10 ||
       std::abs(wrap(tf2::getYaw(path.poses.back().pose.orientation)-tf2::getYaw(reference_.poses[rejoin_index_].pose.orientation)))>0.20) return reject("绕障起点或重入位姿不合格");
    double radius=ticket_data_.value("minimum_turning_radius",0.69),offset=ticket_data_.value("max_offset",0.8);
    for(size_t i=0;i<path.poses.size();++i) {
      const auto &p=path.poses[i].pose;
      if(pastGate(p,reference_.poses[rejoin_index_].pose)) return reject("绕行越过指定重入边界，禁止跨过路口或停靠点");
      if(pointPathDistance(p,reference_,reference_index_,rejoin_index_)>offset) return reject("绕障超出允许偏离距离");
      if(i) {
        const auto &a=path.poses[i-1].pose;double ds=dist(a,p),yaw=tf2::getYaw(a.orientation);
        if(ds>0.20) return reject("绕障采样存在跳变");
        if(ds>0.005 && (p.position.x-a.position.x)*std::cos(yaw)+(p.position.y-a.position.y)*std::sin(yaw)<ds*0.6)
          return reject("绕障包含未授权的倒车或横移");
      }
      // Hybrid-A* headings are quantised. Measure curvature over a physical
      // baseline rather than amplifying raster jitter on 1cm samples.
      size_t end=i;double arc=0;
      while(end+1<path.poses.size() && arc<0.15) {arc+=dist(path.poses[end].pose,path.poses[end+1].pose);++end;}
      if(arc>=0.10 && std::abs(wrap(tf2::getYaw(path.poses[end].pose.orientation)-tf2::getYaw(p.orientation)))/arc>1/radius+0.15)
        return reject("绕障曲率超过底盘能力");
    }
    if(!pathFree(path,0,path.poses.size()-1,true)) return reject("完整车身扫掠校验失败："+free_reason_);
    detour_rejection_.clear();return true;
  }
  void fail(const std::string &reason) {
    if(phase_!=Phase::FAIL) RCLCPP_WARN(node_->get_logger(),"Recorded leg stopped: %s",reason.c_str());
    last_error_=reason;phase_=Phase::FAIL;permission(false);stopFollow();
  }
  std::string label() const {
    switch(phase_) {
      case Phase::FOLLOW:return detour_?"FOLLOWING_DETOUR":"FOLLOWING_REFERENCE";
      case Phase::WAIT:return "WAITING_OBSTACLE";
      case Phase::PLAN:return "PLANNING_DETOUR";
      case Phase::STOP:return stop_reason_==Stop::REJOIN?"REJOINING":"STOPPING";
      case Phase::COMPLETE:return "COMPLETED";
      case Phase::FAIL:return "FAILED";
      default:return "READY";
    }
  }
  void emitProgress(const std::string &state) {
    std::string child_id;const char *hex="0123456789abcdef";
    if(follow_handle_) for(uint8_t byte:follow_handle_->get_goal_id()) {child_id+=hex[byte>>4];child_id+=hex[byte&15];}
    Json value={{"ticket_id",ticket_},{"state",state},{"block",block_},
      {"reference_index",reference_index_},{"detour_index",detour_index_},{"rejoin_index",rejoin_index_},{"error",last_error_},{"waiting_reason",detour_rejection_},
      {"follow_dispatch_pending",send_pending_},{"follow_goal_id",child_id}};
    value["entry"]=!ticket_data_.is_null() && block_<ticket_data_["blocks"].size() && ticket_data_["blocks"][block_].value("entry",false);
    if(detour_) {value["detour_points"]=Json::array();for(const auto &p:detour_path_.poses) value["detour_points"].push_back(toJson(p.pose));}
    std_msgs::msg::String message;message.data=value.dump();progress_pub_->publish(message);
  }

  rclcpp::Node::SharedPtr node_;std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<tf2_ros::Buffer> tf_;std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Client<Road>::SharedPtr road_;rclcpp::Client<Road>::SharedFuture road_future_;
  rclcpp_action::Client<Follow>::SharedPtr follow_;rclcpp_action::Client<Compute>::SharedPtr planner_;
  FollowHandle::SharedPtr follow_handle_;PlanHandle::SharedPtr plan_handle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr permit_pub_,progress_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_sub_,global_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ctrl_sub_,fault_sub_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr local_,global_;nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path reference_,active_path_,detour_path_,candidate_,prefix_;
  Json ticket_data_;std::string ticket_,road_operation_,last_error_,controller_fault_,detour_rejection_,free_reason_;
  Polygon footprint_,region_;
  Phase phase_=Phase::LOAD;Stop stop_reason_=Stop::END;
  bool road_pending_=false,stopping_=false,send_pending_=false,follow_done_=false,plan_done_=false,detour_=false;
  rclcpp_action::ResultCode follow_result_=rclcpp_action::ResultCode::UNKNOWN,plan_result_=rclcpp_action::ResultCode::UNKNOWN;
  unsigned generation_=0,plan_attempt_=0,via_stage_=0;size_t block_=0,reference_index_=0,detour_index_=0,rejoin_index_=0,active_offset_=0;
  Steady::time_point started_{},phase_time_{},road_time_{},last_health_{},next_health_{},odom_time_{},
    local_time_{},global_time_{},last_plan_{},stable_{},complete_time_{},wait_started_{},ctrl_time_{};
};
} // namespace recorded_route_nav2
BT_REGISTER_NODES(factory) {factory.registerNodeType<recorded_route_nav2::FollowRecordedLeg>("FollowRecordedLeg");}
