#pragma once
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include "recorded_route_nav2/geometry.hpp"

namespace recorded_route_nav2 {
class EntryStateSpace : public ompl::base::ReedsSheppStateSpace {
public:
  explicit EntryStateSpace(double radius):ompl::base::ReedsSheppStateSpace(radius) {}
  using ompl::base::ReedsSheppStateSpace::interpolate;
};

// Reeds-Shepp's continuous body yaw and exact gear boundaries are essential.
// Smac's angle-bin poses alone lose both on short multi-point manoeuvres.
inline nav_msgs::msg::Path entryCurve(const Pose &start,const Pose &goal,double radius,double spacing=.025) {
  auto space=std::make_shared<EntryStateSpace>(radius);
  ompl::base::ScopedState<ompl::base::SE2StateSpace> a(space),b(space),q(space);
  a->setXY(start.position.x,start.position.y);a->setYaw(tf2::getYaw(start.orientation));
  b->setXY(goal.position.x,goal.position.y);b->setYaw(tf2::getYaw(goal.orientation));
  auto curve=space->reedsShepp(a.get(),b.get());double total=curve.length()*radius;
  nav_msgs::msg::Path path;path.header.frame_id="map";
  if(!std::isfinite(total) || total<1e-6 || total>100) return path;
  geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose=start;path.poses.push_back(p);
  double before=0;
  for(unsigned segment=0;segment<5;++segment) {
    double arc=std::abs(curve.length_[segment])*radius;
    if(arc<1e-8) continue;
    unsigned n=std::max(1u,static_cast<unsigned>(std::ceil(arc/spacing)));
    for(unsigned i=1;i<=n;++i) {
      space->interpolate(a.get(),curve,(before+arc*i/n)/total,q.get());
      p.pose=fromJson({{"x",q->getX()},{"y",q->getY()},{"yaw",q->getYaw()}});
      path.poses.push_back(p);
    }
    before+=arc;
  }
  path.poses.back().pose=goal;
  return path;
}
} // namespace recorded_route_nav2
