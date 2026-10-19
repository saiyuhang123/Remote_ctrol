#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <nlohmann/json.hpp>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace recorded_route_nav2 {
using Json = nlohmann::json;
using Pose = geometry_msgs::msg::Pose;
inline double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
inline double dist(const Pose &a, const Pose &b) {
  return std::hypot(a.position.x-b.position.x, a.position.y-b.position.y);
}
inline Pose fromJson(const Json &p) {
  Pose q; q.position.x=p.at("x"); q.position.y=p.at("y");
  const double yaw=p.at("yaw"); q.orientation.z=std::sin(yaw/2); q.orientation.w=std::cos(yaw/2);
  return q;
}
inline Json toJson(const Pose &p) {
  return {{"x",p.position.x},{"y",p.position.y},{"yaw",tf2::getYaw(p.orientation)}};
}
inline int64_t stamp(const builtin_interfaces::msg::Time &t) {
  return static_cast<int64_t>(t.sec)*1000000000LL+t.nanosec;
}
inline double pathLength(const nav_msgs::msg::Path &p, size_t from=0, size_t to=std::numeric_limits<size_t>::max()) {
  double value=0;
  for (size_t i=from+1; i<p.poses.size() && i<=to; ++i) value+=dist(p.poses[i-1].pose,p.poses[i].pose);
  return value;
}
inline nav_msgs::msg::Path densePath(const nav_msgs::msg::Path &source,double spacing=0.04) {
  nav_msgs::msg::Path result;result.header=source.header;
  if(source.poses.empty()) return result;
  result.poses.push_back(source.poses.front());
  for(size_t i=1;i<source.poses.size();++i) {
    const auto &a=source.poses[i-1].pose;const auto &b=source.poses[i].pose;
    double yaw=tf2::getYaw(a.orientation),delta=wrap(tf2::getYaw(b.orientation)-yaw);
    int count=std::max(1,static_cast<int>(std::ceil(dist(a,b)/spacing)));
    for(int k=1;k<=count;++k) {
      auto p=source.poses[i];double t=static_cast<double>(k)/count;
      p.pose.position.x=a.position.x+t*(b.position.x-a.position.x);
      p.pose.position.y=a.position.y+t*(b.position.y-a.position.y);
      p.pose.orientation.z=std::sin((yaw+t*delta)/2);p.pose.orientation.w=std::cos((yaw+t*delta)/2);
      result.poses.push_back(p);
    }
  }
  return result;
}
// A forward lane change, parallel bypass, and return. Four exact circular
// arcs keep body heading continuous and respect the Ackermann radius. This
// is only a candidate for a straight reference; collision/band validation
// is mandatory before following it.
inline nav_msgs::msg::Path laneChangeDetour(const Pose &start,const Pose &goal,double radius,double offset) {
  nav_msgs::msg::Path path;path.header.frame_id="map";
  double yaw=tf2::getYaw(start.orientation),c=std::cos(yaw),s=std::sin(yaw);
  double dx=goal.position.x-start.position.x,dy=goal.position.y-start.position.y;
  double forward=c*dx+s*dy,lateral=-s*dx+c*dy,shift=std::abs(offset);
  if(radius<=0 || shift<=0 || shift>=2*radius || std::abs(lateral)>.08 ||
     std::abs(wrap(tf2::getYaw(goal.orientation)-yaw))>.10) return path;
  double alpha=std::acos(1-shift/(2*radius)),reach=2*radius*std::sin(alpha);
  if(forward<2*reach+.05) return path;
  Pose current=start;current.orientation.x=current.orientation.y=0;
  current.orientation.z=std::sin(yaw/2);current.orientation.w=std::cos(yaw/2);
  geometry_msgs::msg::PoseStamped p;p.pose=current;path.poses.push_back(p);
  auto advance=[&](double distance,double curvature) {
    int count=std::max(1,static_cast<int>(std::ceil(distance/.025)));double step=distance/count;
    for(int i=0;i<count;++i) {
      double next=yaw+curvature*step;
      if(std::abs(curvature)<1e-9) {current.position.x+=step*std::cos(yaw);current.position.y+=step*std::sin(yaw);}
      else {current.position.x+=(std::sin(next)-std::sin(yaw))/curvature;current.position.y+=(std::cos(yaw)-std::cos(next))/curvature;}
      yaw=next;current.orientation.z=std::sin(yaw/2);current.orientation.w=std::cos(yaw/2);
      p.pose=current;path.poses.push_back(p);
    }
  };
  double direction=offset>0?1:-1;
  advance(radius*alpha,direction/radius);advance(radius*alpha,-direction/radius);
  advance(forward-2*reach,0);
  advance(radius*alpha,-direction/radius);advance(radius*alpha,direction/radius);
  return path;
}
inline bool pastGate(const Pose &point,const Pose &gate,double tolerance=0.05) {
  double yaw=tf2::getYaw(gate.orientation);
  return (point.position.x-gate.position.x)*std::cos(yaw)+(point.position.y-gate.position.y)*std::sin(yaw)>tolerance;
}
inline bool inside(double x,double y,const Json &polygon) {
  if (polygon.empty()) return true;
  bool value=false;
  for(size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
    double ax=polygon[j][0],ay=polygon[j][1],bx=polygon[i][0],by=polygon[i][1];
    double cross=(x-ax)*(by-ay)-(y-ay)*(bx-ax);
    if(std::abs(cross)<1e-9 && x>=std::min(ax,bx)-1e-9 && x<=std::max(ax,bx)+1e-9 &&
       y>=std::min(ay,by)-1e-9 && y<=std::max(ay,by)+1e-9) return true;
    if((ay>y)!=(by>y) && x<(bx-ax)*(y-ay)/(by-ay)+ax) value=!value;
  }
  return value;
}
using Polygon=std::vector<std::pair<double,double>>;
inline bool inside(double x,double y,const Polygon &polygon) {
  if(polygon.empty()) return true;
  bool result=false;
  for(size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
    auto [ax,ay]=polygon[j];auto [bx,by]=polygon[i];
    if(std::abs((x-ax)*(by-ay)-(y-ay)*(bx-ax))<1e-9 && x>=std::min(ax,bx)-1e-9 && x<=std::max(ax,bx)+1e-9 && y>=std::min(ay,by)-1e-9 && y<=std::max(ay,by)+1e-9) return true;
    if((ay>y)!=(by>y) && x<(bx-ax)*(y-ay)/(by-ay)+ax) result=!result;
  }
  return result;
}
inline double orientation(std::pair<double,double> a,std::pair<double,double> b,std::pair<double,double> c) {
  return (b.first-a.first)*(c.second-a.second)-(b.second-a.second)*(c.first-a.first);
}
inline bool properCross(std::pair<double,double> a,std::pair<double,double> b,std::pair<double,double> c,std::pair<double,double> d) {
  return orientation(a,b,c)*orientation(a,b,d)<-1e-12 && orientation(c,d,a)*orientation(c,d,b)<-1e-12;
}
inline bool polygonCell(const Polygon &p,double x,double y) {
  for(auto [px,py]:p) if(px>=x && px<=x+1 && py>=y && py<=y+1) return true;
  Polygon corners{{x,y},{x+1,y},{x+1,y+1},{x,y+1}};
  for(auto [px,py]:corners) if(inside(px,py,p)) return true;
  for(size_t i=0;i<p.size();++i) for(size_t j=0;j<4;++j)
    if(properCross(p[i],p[(i+1)%p.size()],corners[j],corners[(j+1)%4])) return true;
  return false;
}
inline double pointPathDistance(const Pose &q,const nav_msgs::msg::Path &path,size_t from,size_t to) {
  double best=std::numeric_limits<double>::infinity();
  to=std::min(to,path.poses.size()-1);
  for(size_t i=from;i<to;++i) {
    const auto &a=path.poses[i].pose.position; const auto &b=path.poses[i+1].pose.position;
    double dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
    double t=den>1e-12?std::clamp(((q.position.x-a.x)*dx+(q.position.y-a.y)*dy)/den,0.0,1.0):0;
    best=std::min(best,std::hypot(q.position.x-a.x-t*dx,q.position.y-a.y-t*dy));
  }
  return best;
}
} // namespace recorded_route_nav2
