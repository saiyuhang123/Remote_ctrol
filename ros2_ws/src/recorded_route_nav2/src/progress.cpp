#include <map>
#include "recorded_route_nav2/progress.hpp"

namespace recorded_route_nav2 {
std::shared_ptr<Progress> sharedProgress(const std::string &key) {
  static std::mutex mutex;
  static std::map<std::string,std::weak_ptr<Progress>> registry;
  std::lock_guard<std::mutex> lock(mutex);
  auto value=registry[key].lock();
  if(!value) {value=std::make_shared<Progress>();registry[key]=value;}
  return value;
}
}  // namespace recorded_route_nav2
