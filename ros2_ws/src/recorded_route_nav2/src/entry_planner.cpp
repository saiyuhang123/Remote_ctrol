#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "recorded_route_nav2/entry_curve.hpp"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace recorded_route_nav2 {
class RecordedEntryPlanner : public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
                 std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent.lock();costmap_=costmap;
    nav2_util::declare_parameter_if_not_declared(node_,name+".minimum_turning_radius",rclcpp::ParameterValue(.69));
    radius_=node_->get_parameter(name+".minimum_turning_radius").as_double();
    if(!std::isfinite(radius_) || radius_<=0) throw std::runtime_error("invalid entry turning radius");
  }
  void activate() override {}
  void deactivate() override {}
  void cleanup() override {costmap_.reset();node_.reset();}

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal) override {
    if(start.header.frame_id!="map" || goal.header.frame_id!="map")
      throw nav2_core::PlannerException("entry requires map coordinates");
    if(!freePose(start.pose)) throw nav2_core::PlannerException("entry start footprint is blocked or unknown");
    if(!freePose(goal.pose)) throw nav2_core::PlannerException("entry endpoint footprint is blocked or unknown");
    auto direct=entryCurve(start.pose,goal.pose,radius_);
    if(freePath(direct)) {direct.header.stamp=node_->now();return direct;}

    // Continuous SE(2) search checks the same complete vehicle footprint as
    // the executable curve. Quantised raster poses may miss a corner or a
    // short gear cusp, so they are not used as execution waypoints.
    auto space=std::make_shared<EntryStateSpace>(radius_);
    ompl::base::RealVectorBounds bounds(2);
    auto *grid=costmap_->getCostmap();
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
      bounds.setLow(0,grid->getOriginX());bounds.setHigh(0,grid->getOriginX()+grid->getSizeInMetersX());
      bounds.setLow(1,grid->getOriginY());bounds.setHigh(1,grid->getOriginY()+grid->getSizeInMetersY());
    }
    space->setBounds(bounds);
    // DiscreteMotionValidator interpolates along Reeds-Shepp, not straight
    // XY chords; validate at most every 2cm, including reversing travel.
    space->setLongestValidSegmentFraction(std::min(.01,.02/space->getMaximumExtent()));
    ompl::geometric::SimpleSetup setup(space);
    setup.setStateValidityChecker([this](const ompl::base::State *state) {
      auto *p=state->as<ompl::base::SE2StateSpace::StateType>();
      return freePose(fromJson({{"x",p->getX()},{"y",p->getY()},{"yaw",p->getYaw()}}));
    });
    ompl::base::ScopedState<ompl::base::SE2StateSpace> a(space),b(space);
    a->setXY(start.pose.position.x,start.pose.position.y);a->setYaw(tf2::getYaw(start.pose.orientation));
    b->setXY(goal.pose.position.x,goal.pose.position.y);b->setYaw(tf2::getYaw(goal.pose.orientation));
    setup.setStartAndGoalStates(a,b,1e-6);
    auto planner=std::make_shared<ompl::geometric::RRTConnect>(setup.getSpaceInformation());
    planner->setRange(std::max(.5,radius_));setup.setPlanner(planner);
    setup.solve(2.5);
    if(!setup.haveExactSolutionPath()) throw nav2_core::PlannerException("no collision-free Ackermann entry within planning timeout");
    setup.simplifySolution(.15);
    auto &solution=setup.getSolutionPath();
    nav_msgs::msg::Path result;result.header=start.header;
    for(size_t i=1;i<solution.getStateCount();++i) {
      auto *u=solution.getState(i-1)->as<ompl::base::SE2StateSpace::StateType>();
      auto *v=solution.getState(i)->as<ompl::base::SE2StateSpace::StateType>();
      Pose from=fromJson({{"x",u->getX()},{"y",u->getY()},{"yaw",u->getYaw()}});
      Pose to=fromJson({{"x",v->getX()},{"y",v->getY()},{"yaw",v->getYaw()}});
      if(dist(from,to)<1e-6 && std::abs(wrap(tf2::getYaw(from.orientation)-tf2::getYaw(to.orientation)))<1e-6) continue;
      auto part=entryCurve(from,to,radius_);
      if(!freePath(part)) throw nav2_core::PlannerException("entry swept-body validation failed");
      result.poses.insert(result.poses.end(),part.poses.begin()+(result.poses.empty()?0:1),part.poses.end());
      if(result.poses.size()>20000 || pathLength(result)>100)
        throw nav2_core::PlannerException("entry connection exceeds length limit");
    }
    if(result.poses.size()<2) throw nav2_core::PlannerException("empty entry search result");
    result.header.stamp=node_->now();return result;
  }

private:
  bool freePose(const Pose &p) {
    auto *grid=costmap_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
    const auto footprint=costmap_->getRobotFootprint();
    if(footprint.size()<3) return false;
    double yaw=tf2::getYaw(p.orientation),c=std::cos(yaw),s=std::sin(yaw);
    Polygon raster;double minx=1e9,miny=1e9,maxx=-1e9,maxy=-1e9;
    for(const auto &v:footprint) {
      double x=(p.position.x+c*v.x-s*v.y-grid->getOriginX())/grid->getResolution();
      double y=(p.position.y+s*v.x+c*v.y-grid->getOriginY())/grid->getResolution();
      if(x<0 || y<0 || x>=grid->getSizeInCellsX() || y>=grid->getSizeInCellsY()) return false;
      raster.emplace_back(x,y);minx=std::min(minx,x);maxx=std::max(maxx,x);miny=std::min(miny,y);maxy=std::max(maxy,y);
    }
    for(int y=std::floor(miny);y<=std::floor(maxy);++y)
      for(int x=std::floor(minx);x<=std::floor(maxx);++x)
        if(grid->getCost(x,y)>=nav2_costmap_2d::LETHAL_OBSTACLE && polygonCell(raster,x,y)) return false;
    return true;
  }
  bool freePath(const nav_msgs::msg::Path &path) {
    if(path.poses.size()<2) return false;
    for(const auto &p:path.poses) if(!freePose(p.pose)) return false;
    return true;
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  double radius_=.69;
};
} // namespace recorded_route_nav2
PLUGINLIB_EXPORT_CLASS(recorded_route_nav2::RecordedEntryPlanner,nav2_core::GlobalPlanner)
