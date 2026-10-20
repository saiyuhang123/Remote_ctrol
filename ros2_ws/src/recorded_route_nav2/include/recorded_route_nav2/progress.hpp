#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include "nav2_util/node_utils.hpp"

namespace recorded_route_nav2 {
using Clock=std::chrono::steady_clock;
struct Progress {
  std::mutex mutex;
  double remaining=1e9,total=1e9,tolerance_cap=1e9;
  bool ready=false;
  int64_t stamp=0;
  Clock::time_point updated{};
};

// A single registry shared by the controller and checker DSOs.
std::shared_ptr<Progress> sharedProgress(const std::string &key);

template<class NodeT,class T> T param(const NodeT &node,const std::string &key,const T &fallback) {
  nav2_util::declare_parameter_if_not_declared(node,key,rclcpp::ParameterValue(fallback));
  return node->get_parameter(key).template get_value<T>();
}
}  // namespace recorded_route_nav2
