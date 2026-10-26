#include "adapters/ros_adapter.h"
#include <QTimer>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <map>
#include <thread>
#ifdef DESKTOP_HAS_PATROL_INTERFACES
#include <patrol_interfaces/msg/executor_heartbeat.hpp>
#endif
#ifdef DESKTOP_HAS_YUNLE_MSGS
#include <yunle_msgs/msg/battery.hpp>
#endif
#ifdef DESKTOP_HAS_HK_CAMERA
#include <hk_camera/srv/capture_picture.hpp>
#include <hk_camera/srv/set_ptz_pose.hpp>
#include <hk_camera/srv/get_ptz_pose.hpp>
#include <hk_camera/srv/control_ptz.hpp>
#include <hk_camera/srv/ptz_preset.hpp>
#endif
#ifdef DESKTOP_HAS_OCR_INTERFACES
#include <ocr_interfaces/srv/recognize_text.hpp>
#endif
#ifdef DESKTOP_HAS_SLAM_TOOLBOX
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#endif
#ifdef DESKTOP_HAS_CARTOGRAPHER_ROS_MSGS
#include <cartographer_ros_msgs/srv/write_state.hpp>
#endif

namespace {
using Clock = std::chrono::steady_clock;
qint64 nowMs() { return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count(); }
double yawOf(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2. * (q.w*q.z + q.x*q.y), 1. - 2. * (q.y*q.y + q.z*q.z));
}
bool validPose(const geometry_msgs::msg::Pose &p) {
  const auto &q = p.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(norm) && std::abs(norm - 1.) < .01;
}
QImage toImage(const sensor_msgs::msg::Image &m) {
  if (!m.width || !m.height || m.width > 16384 || m.height > 16384 ||
      m.data.size() < static_cast<size_t>(m.step) * m.height) return {};
  QImage::Format format; unsigned channels;
  if (m.encoding == "bgr8" || m.encoding == "rgb8") { format = QImage::Format_RGB888; channels = 3; }
  else if (m.encoding == "mono8") { format = QImage::Format_Grayscale8; channels = 1; }
  else if (m.encoding == "rgba8") { format = QImage::Format_RGBA8888; channels = 4; }
  else return {};
  if (m.step < m.width * channels) return {};
  QImage value(m.data.data(), m.width, m.height, m.step, format);
  return m.encoding == "bgr8" ? value.rgbSwapped() : value.copy();
}
}

struct RosAdapter::Impl {
  RosAdapter *owner;
  DesktopConfig config;
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialPose;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPreview;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr capture, speech;
  using Navigate = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
  rclcpp_action::Client<Navigate>::SharedPtr navigation;
  std::mutex goalMutex;
  GoalHandle::SharedPtr goalHandle;
  std::atomic_bool navPending{false}, ownedGoalActive{false}, observedNavActive{false}, cancellationKnown{false};
  std::atomic_uint64_t cancelEpoch{0};
  std::atomic_int estopCommand{-1};  // No startup writes; explicit stop/unlock starts feedback.
  std::mutex emergencyMutex;
  qint64 lastStopRequestedNs = 0;
  std::map<std::string, rclcpp::ClientBase::SharedPtr> clients;
  rclcpp::TimerBase::SharedPtr timer;
  std::mutex frameMutex;
  QString frame = "map";
  std::mutex imageMutex;
  QImage latestImage;
  QTimer *imagePump = nullptr;
  RobotSnapshot state;
  qint64 odomTime = 0, stopTime = 0, guardTime = 0, batteryTime = 0, imageTime = 0;
  qint64 patrolHeartbeatTime = 0, patrolLockTime = 0;
  bool navReceived = false;
  std::map<quint64, std::function<void()>> cancelPending;
  quint64 nextRequest = 0;
  Impl(RosAdapter *o, const DesktopConfig &c) : owner(o), config(c) {
    node = std::make_shared<rclcpp::Node>("robot_desktop_state");
    initialPose = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    goalPreview = node->create_publisher<geometry_msgs::msg::PoseStamped>("/desktop_ui/selected_goal", 1);
    velocity = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teleop", 10);
    emergency = node->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
    capture = node->create_publisher<std_msgs::msg::String>("/capture_request", 10);
    speech = node->create_publisher<std_msgs::msg::String>("/tts_say", 10);
    navigation = rclcpp_action::create_client<Navigate>(node, "/navigate_to_pose");
    // Keep clients alive so DDS discovery completes before an operator clicks.
    clients["/map_save"] = node->create_client<std_srvs::srv::Trigger>("/map_save");
    clients["/loop_closure/save_map"] = node->create_client<std_srvs::srv::Trigger>("/loop_closure/save_map");
    clients["/loop_closure/save_bundle"] = node->create_client<std_srvs::srv::Trigger>("/loop_closure/save_bundle");
    for (const auto *command : {"login", "start_stream", "stop_stream"}) {
      const auto name = std::string("/hk_camera/") + command;
      clients[name] = node->create_client<std_srvs::srv::Trigger>(name);
    }
#ifdef DESKTOP_HAS_HK_CAMERA
    clients["/hk_camera/capture"] = node->create_client<hk_camera::srv::CapturePicture>("/hk_camera/capture");
    clients["/hk_camera/set_ptz_pose"] = node->create_client<hk_camera::srv::SetPTZPose>("/hk_camera/set_ptz_pose");
    clients["/hk_camera/get_ptz_pose"] = node->create_client<hk_camera::srv::GetPTZPose>("/hk_camera/get_ptz_pose");
    clients["/hk_camera/control_ptz"] = node->create_client<hk_camera::srv::ControlPTZ>("/hk_camera/control_ptz");
    clients["/hk_camera/preset"] = node->create_client<hk_camera::srv::PTZPreset>("/hk_camera/preset");
#endif
#ifdef DESKTOP_HAS_OCR_INTERFACES
    clients["/ocr/recognize"] = node->create_client<ocr_interfaces::srv::RecognizeText>("/ocr/recognize");
#endif
#ifdef DESKTOP_HAS_SLAM_TOOLBOX
    clients["/slam_toolbox/serialize_map"] = node->create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");
#endif
#ifdef DESKTOP_HAS_CARTOGRAPHER_ROS_MSGS
    clients["/write_state"] = node->create_client<cartographer_ros_msgs::srv::WriteState>("/write_state");
#endif
    imagePump = new QTimer(owner); imagePump->setInterval(100);
    QObject::connect(imagePump, &QTimer::timeout, owner, [this] {
      QImage image;
      { std::lock_guard<std::mutex> lock(imageMutex); image.swap(latestImage); }
      if (!image.isNull()) emit owner->image(image);
    }); imagePump->start();
    tf = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    listener = std::make_unique<tf2_ros::TransformListener>(*tf, node, false);
    subscriptions.push_back(node->create_subscription<nav_msgs::msg::Odometry>(
      config.odomTopic.toStdString(), rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if (!std::isfinite(m->twist.twist.linear.x) || !std::isfinite(m->twist.twist.angular.z)) return;
        state.speed = m->twist.twist.linear.x; state.yawRate = m->twist.twist.angular.z; odomTime = nowMs();
      }));
    subscriptions.push_back(node->create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", rclcpp::QoS(10), [this](std_msgs::msg::Bool::ConstSharedPtr m, const rclcpp::MessageInfo &info) {
        state.estopped = m->data; stopTime = nowMs();
        if (m->data) {
          std::lock_guard<std::mutex> lock(emergencyMutex);
          const auto own = emergency->get_gid();
          const auto &sender = info.get_rmw_message_info().publisher_gid;
          const bool external = !std::equal(sender.data, sender.data + sizeof(sender.data), own.data);
          if (external)
            lastStopRequestedNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
          // Our queued True heartbeat can predate an explicit unlock. Only
          // another publisher can introduce a new stop through this callback.
          if (external && estopCommand.load() == 0) estopCommand.store(1);
        }
      }));
    subscriptions.push_back(node->create_subscription<std_msgs::msg::Bool>(
      "/patrol/guard_ready", rclcpp::QoS(10), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        state.guardReady = m->data; guardTime = nowMs();
      }));
    subscriptions.push_back(node->create_subscription<std_msgs::msg::Bool>(
      "/patrol/stop_lock", rclcpp::QoS(10), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        state.patrolLocked = m->data; patrolLockTime = nowMs();
      }));
#ifdef DESKTOP_HAS_PATROL_INTERFACES
    subscriptions.push_back(node->create_subscription<patrol_interfaces::msg::ExecutorHeartbeat>(
      "/patrol/executor_heartbeat", rclcpp::QoS(10),
      [this](patrol_interfaces::msg::ExecutorHeartbeat::ConstSharedPtr) {
        patrolHeartbeatTime = nowMs();
      }));
#endif
    subscriptions.push_back(node->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr m) {
        bool active = false; navReceived = true;
        for (const auto &s : m->status_list) if (s.status >= 1 && s.status <= 3) active = true;
        observedNavActive.store(active);
      }));
    subscriptions.push_back(node->create_subscription<std_msgs::msg::String>(
      "/capture_done", 10, [this](std_msgs::msg::String::ConstSharedPtr m) { emit owner->message("业务抓拍：" + QString::fromStdString(m->data)); }));
    subscriptions.push_back(node->create_subscription<std_msgs::msg::String>(
      "/tts_status", 10, [this](std_msgs::msg::String::ConstSharedPtr m) { emit owner->message("语音：" + QString::fromStdString(m->data)); }));
    subscriptions.push_back(node->create_subscription<sensor_msgs::msg::Image>(
      config.imageTopic.toStdString(), rclcpp::SensorDataQoS().keep_last(1), [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        if (nowMs() - imageTime < 100) return;
        imageTime = nowMs(); auto image = toImage(*m);
        if (!image.isNull()) { std::lock_guard<std::mutex> lock(imageMutex); latestImage.swap(image); }
      }));
#ifdef DESKTOP_HAS_YUNLE_MSGS
    subscriptions.push_back(node->create_subscription<yunle_msgs::msg::Battery>(
      config.batteryTopic.toStdString(), rclcpp::SensorDataQoS(), [this](yunle_msgs::msg::Battery::ConstSharedPtr m) {
        if (!std::isfinite(m->capacity) || !std::isfinite(m->voltage) || m->capacity < 0 || m->capacity > 100 || m->voltage <= 0) return;
        state.battery = m->capacity; state.voltage = m->voltage; batteryTime = nowMs();
      }));
#endif
    subscriptions.push_back(node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/desktop_ui/goal_request", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        if (m->header.frame_id != "map" || !validPose(m->pose)) { emit owner->message("忽略目标：需要有效的 map 坐标系位姿"); return; }
        emit owner->goalRequested(m->pose.position.x, m->pose.position.y, yawOf(m->pose.orientation));
      }));
    subscriptions.push_back(node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/desktop_ui/initial_pose_request", 10, [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        if (m->header.frame_id != "map" || !validPose(m->pose.pose)) { emit owner->message("忽略重定位：需要有效的 map 坐标系位姿"); return; }
        emit owner->initialPoseRequested(m->pose.pose.position.x, m->pose.pose.position.y, yawOf(m->pose.pose.orientation));
      }));
    timer = node->create_wall_timer(std::chrono::milliseconds(200), [this] { tick(); });
    executor.add_node(node);
    thread = std::thread([this] { try { executor.spin(); } catch (const std::exception &e) { emit owner->message(QString("ROS 状态线程退出：") + e.what()); } });
  }
  ~Impl() {
    imagePump->stop();
    for (auto &entry : cancelPending) entry.second();
    executor.cancel(); if (thread.joinable()) thread.join();
    executor.remove_node(node);
  }
  void tick() {
    { std::lock_guard<std::mutex> lock(frameMutex); state.frame = frame; }
    const auto current = nowMs();
    state.speedValid = odomTime && current - odomTime < config.feedbackTimeoutMs;
    state.estopValid = stopTime && current - stopTime < config.feedbackTimeoutMs;
    state.guardValid = guardTime && current - guardTime < config.feedbackTimeoutMs;
    if (!state.guardValid) state.guardReady = false;
    const bool patrolHeartbeatFresh = patrolHeartbeatTime &&
        current - patrolHeartbeatTime < config.feedbackTimeoutMs;
    const bool patrolLockFresh = patrolLockTime &&
        current - patrolLockTime < config.feedbackTimeoutMs;
    state.patrolInterlockValid = patrolHeartbeatFresh || patrolLockFresh;
    if (!patrolLockFresh) state.patrolLocked = false;
    state.batteryValid = batteryTime && current - batteryTime < 5000;
    const bool navPublisher = node->count_publishers("/navigate_to_pose/_action/status") > 0;
    state.navigationKnown = !navPublisher || navReceived || cancellationKnown.load();
    if (!navPublisher) { observedNavActive.store(false); navReceived = false; }
    state.navigationActive = observedNavActive.load() || navPending.load() || ownedGoalActive.load();
    state.navigationAvailable = navigation->action_server_is_ready();
    if (estopCommand.load() >= 0) {
      std_msgs::msg::Bool msg; msg.data = estopCommand.load() != 0; emergency->publish(msg);
    }
    state.poseValid = false;
    try {
      const auto transform = tf->lookupTransform(state.frame.toStdString(), config.baseFrame.toStdString(), tf2::TimePointZero);
      const auto stamp = rclcpp::Time(transform.header.stamp, node->get_clock()->get_clock_type());
      const double age = (node->now() - stamp).seconds();
      if (age >= -.2 && age * 1000 < config.feedbackTimeoutMs) {
        state.x = transform.transform.translation.x; state.y = transform.transform.translation.y;
        state.yaw = yawOf(transform.transform.rotation);
        state.poseValid = std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.yaw);
      }
    } catch (const tf2::TransformException &) {}
    emit owner->snapshot(state);
  }
  bool motionConflict() {
    const auto ownGid = velocity->get_gid();
    for (const auto &info : node->get_publishers_info_by_topic("/cmd_vel_teleop")) {
      if (!std::equal(info.endpoint_gid().begin(), info.endpoint_gid().end(), ownGid.data)) {
        emit owner->message("本地控制未执行：/cmd_vel_teleop 还有发布者 " + QString::fromStdString(info.node_name()) + "，请先停止远程网关或其他手动控制端");
        return true;
      }
    }
    return false;
  }
  template<class Service, class Callback>
  void request(const QString &operation, const std::string &name, const std::shared_ptr<typename Service::Request> &req, Callback callback) {
    auto &base = clients[name];
    if (!base) base = node->create_client<Service>(name);
    auto client = std::dynamic_pointer_cast<rclcpp::Client<Service>>(base);
    if (!client->service_is_ready()) { emit owner->requestFinished(operation, false, QString::fromStdString(name) + " 服务未就绪"); return; }
    auto done = std::make_shared<std::atomic_bool>(false);
    const auto key = ++nextRequest;
    const auto pending = client->async_send_request(req, [this, client, done, operation, callback, key](typename rclcpp::Client<Service>::SharedFuture response) {
      if (done->exchange(true)) return;
      try { callback(response.get()); } catch (const std::exception &e) { emit owner->requestFinished(operation, false, e.what()); }
      QTimer::singleShot(0, owner, [this, key] { cancelPending.erase(key); });
    });
    const auto id = pending.request_id;
    auto cancel = [client, done, id] { if (!done->exchange(true)) client->remove_pending_request(id); };
    cancelPending.emplace(key, cancel);
    // Map refinement and export can take longer than ordinary device services.
    const int timeoutMs = operation == "save_state" && name.rfind("/loop_closure/save_", 0) == 0
        ? std::max(config.serviceTimeoutMs, 180000) : config.serviceTimeoutMs;
    QTimer::singleShot(timeoutMs, owner, [this, client, done, id, operation, key] {
      if (!done->exchange(true)) { client->remove_pending_request(id); emit owner->requestFinished(operation, false, "服务响应超时，请检查节点日志和输出文件"); }
      cancelPending.erase(key);
    });
  }
};

RosAdapter::RosAdapter(const DesktopConfig &c, QObject *parent) : QObject(parent) { qRegisterMetaType<RobotSnapshot>(); qRegisterMetaType<CameraPose>(); impl_ = std::make_unique<Impl>(this, c); }
RosAdapter::~RosAdapter() = default;
bool RosAdapter::previewGoal(double x, double y, double yaw) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return false;
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "map"; msg.header.stamp = impl_->node->now();
  msg.pose.position.x = x; msg.pose.position.y = y;
  msg.pose.orientation.z = std::sin(yaw / 2.); msg.pose.orientation.w = std::cos(yaw / 2.);
  impl_->goalPreview->publish(msg); return true;
}
bool RosAdapter::publishVelocity(double linear, double angular) {
  if (!std::isfinite(linear) || !std::isfinite(angular)) return false;
  if (std::abs(linear) > 1e-6 || std::abs(angular) > 1e-6) {
    if (impl_->estopCommand.load() != 0 || impl_->navPending.load() || impl_->ownedGoalActive.load() ||
        impl_->observedNavActive.load() || impl_->motionConflict()) return false;
  }
  geometry_msgs::msg::Twist msg;
  msg.linear.x = std::clamp(linear, -impl_->config.maxLinear, impl_->config.maxLinear);
  msg.angular.z = std::clamp(angular, -impl_->config.maxAngular, impl_->config.maxAngular);
  impl_->velocity->publish(msg); return true;
}
bool RosAdapter::setEmergencyStop(bool stopped, qint64 remoteIssuedNs) {
  if (!stopped && (impl_->navPending.load() || impl_->ownedGoalActive.load() ||
                  impl_->observedNavActive.load() || impl_->motionConflict())) return false;
  std::lock_guard<std::mutex> lock(impl_->emergencyMutex);
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  if (!stopped && remoteIssuedNs && (impl_->lastStopRequestedNs > remoteIssuedNs ||
                                     now - remoteIssuedNs > 1000000000LL)) {
    emit message("远程解除急停已过期或其后收到新的急停，保持当前状态"); return false;
  }
  if (stopped) impl_->lastStopRequestedNs = now;
  publishVelocity(0., 0.);
  impl_->estopCommand.store(stopped ? 1 : 0);
  std_msgs::msg::Bool msg; msg.data = stopped; impl_->emergency->publish(msg);
  emit message(stopped ? "已发布本地急停" : "已明确解除本地急停，当前手动速度为零");
  return true;
}
bool RosAdapter::navigate(double x, double y, double yaw) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || impl_->motionConflict()) return false;
  if (impl_->estopCommand.load() != 0) { emit message("请先点击解除急停，再发送导航目标"); return false; }
  if (!impl_->navigation->action_server_is_ready()) { emit message("Nav2 /navigate_to_pose 尚未就绪"); return false; }
  if (impl_->ownedGoalActive.load() || impl_->observedNavActive.load() || impl_->navPending.exchange(true)) {
    emit message("已有导航目标，请先取消并等待结束，再发送新目标"); return false;
  }
  Impl::Navigate::Goal goal;
  goal.pose.header.frame_id = "map"; goal.pose.header.stamp = impl_->node->now();
  goal.pose.pose.position.x = x; goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = std::sin(yaw / 2.); goal.pose.pose.orientation.w = std::cos(yaw / 2.);
  const auto epoch = impl_->cancelEpoch.load();
  rclcpp_action::Client<Impl::Navigate>::SendGoalOptions options;
  options.goal_response_callback = [this, epoch](Impl::GoalHandle::SharedPtr handle) {
    { std::lock_guard<std::mutex> lock(impl_->goalMutex); impl_->goalHandle = handle; }
    impl_->ownedGoalActive.store(static_cast<bool>(handle)); impl_->navPending.store(false);
    if (!handle) { emit navigationEvent("导航目标被拒绝"); return; }
    if (epoch != impl_->cancelEpoch.load()) {
      impl_->navigation->async_cancel_goal(handle); emit navigationEvent("取消迟到的导航目标");
    } else emit navigationEvent("导航目标已接受");
  };
  options.result_callback = [this](const Impl::GoalHandle::WrappedResult &result) {
    impl_->ownedGoalActive.store(false);
    { std::lock_guard<std::mutex> lock(impl_->goalMutex); impl_->goalHandle.reset(); }
    QString text;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: text = "导航到点成功"; break;
      case rclcpp_action::ResultCode::CANCELED: text = "导航已取消"; break;
      case rclcpp_action::ResultCode::ABORTED: text = "导航失败 / 已中止"; break;
      default: text = "导航结果未知";
    }
    emit navigationEvent(text);
  };
  try { impl_->navigation->async_send_goal(goal, options); }
  catch (const std::exception &e) { impl_->navPending.store(false); emit message(QString("导航请求失败：") + e.what()); return false; }
  emit navigationEvent("导航请求已发送，等待 Nav2 接受"); return true;
}
bool RosAdapter::cancelNavigation() {
  ++impl_->cancelEpoch;
  if (!impl_->navigation->action_server_is_ready()) {
    if (impl_->observedNavActive.load() || impl_->navPending.load() || impl_->ownedGoalActive.load()) {
      emit message("Nav2 不可用，无法确认导航已取消"); return false;
    }
    impl_->cancellationKnown.store(true); return true;
  }
  try {
    impl_->navigation->async_cancel_all_goals([this](auto response) {
      if (response->return_code == 0) {
        impl_->cancellationKnown.store(true);
        emit navigationEvent(response->goals_canceling.empty() ? "Nav2 已确认无待取消目标" : "导航取消已接受，等待执行结束");
      } else emit navigationEvent("Nav2 未接受取消请求，请检查导航状态");
    });
  } catch (const std::exception &e) { emit message(QString("取消导航失败：") + e.what()); return false; }
  return true;
}
void RosAdapter::captureBusiness() {
  std_msgs::msg::String msg; msg.data = "desktop_manual"; impl_->capture->publish(msg);
  emit message("业务抓拍请求已发布到 /capture_request");
}
void RosAdapter::speak(const QString &text) {
  if (text.trimmed().isEmpty()) return;
  std_msgs::msg::String msg; msg.data = text.trimmed().toStdString(); impl_->speech->publish(msg);
}
bool RosAdapter::setInitialPose(double x, double y, double yaw) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return false;
  if (impl_->initialPose->get_subscription_count() == 0) {
    emit message("初始位姿未发送：/initialpose 没有定位节点订阅，请等待定位流程启动完成"); return false;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map"; msg.header.stamp = impl_->node->now();
  msg.pose.pose.position.x = x; msg.pose.pose.position.y = y;
  msg.pose.pose.orientation.z = std::sin(yaw / 2.); msg.pose.pose.orientation.w = std::cos(yaw / 2.);
  msg.pose.covariance[0] = .25; msg.pose.covariance[7] = .25; msg.pose.covariance[35] = .07;
  impl_->initialPose->publish(msg); return true;
}
void RosAdapter::setFixedFrame(const QString &frame) { std::lock_guard<std::mutex> lock(impl_->frameMutex); impl_->frame = frame; }
QStringList RosAdapter::capabilities() const {
  QStringList list;
#ifdef DESKTOP_HAS_HK_CAMERA
  list << "camera";
#endif
#ifdef DESKTOP_HAS_OCR_INTERFACES
  list << "ocr";
#endif
#ifdef DESKTOP_HAS_YUNLE_MSGS
  list << "battery";
#endif
  return list;
}
QStringList RosAdapter::conflictingNodes() const {
  const QStringList names{"controller_server", "planner_server", "bt_navigator", "amcl", "slam_toolbox", "cartographer_node", "fast_lio_mapping", "laser_mapping", "map_server", "twist_mux", "chassis_driver", "jd03_odometry", "ekf_filter_node", "robot_state_publisher", "rslidar_sdk_node", "imuDriverNode", "lio_odometry", "global_localization", "transform_fusion", "pointcloud_to_scan", "cloud_crop_box", "robot_patrol", "patrol_guard"};
  QStringList found;
  for (const auto &name : impl_->node->get_node_names()) {
    const auto qname = QString::fromStdString(name);
    if (names.contains(qname.section('/', -1)) || qname.section('/', -1).startsWith("tf_base_link_to_")) found << qname;
  }
  for (const auto &topic : {"/cmd_vel", "/odom", "/scan", "/map", "/Odometry", "/cloud_registered",
                            "/patrol/stop_lock", "/patrol/executor_heartbeat"})
    if (impl_->node->count_publishers(topic) > 0) found << QString::fromLatin1(topic) + "（已有发布者）";
  return found;
}
void RosAdapter::captureCamera() {
#ifdef DESKTOP_HAS_HK_CAMERA
  auto req = std::make_shared<hk_camera::srv::CapturePicture::Request>(); req->quality = 0;
  impl_->request<hk_camera::srv::CapturePicture>("camera", "/hk_camera/capture", req, [this](auto r) {
    const auto image = toImage(r->image);
    if (r->success && !image.isNull()) emit capturedImage(image);
    emit requestFinished("camera", r->success && !image.isNull(), QString::fromStdString(r->message));
  });
#else
  emit requestFinished("camera", false, "构建时未发现 hk_camera 消息包");
#endif
}
void RosAdapter::setPtz(double pan, double tilt, double zoom) {
#ifdef DESKTOP_HAS_HK_CAMERA
  if (!std::isfinite(pan) || !std::isfinite(tilt) || !std::isfinite(zoom) || pan < 0 || pan > 360 || tilt < -180 || tilt > 360 || zoom < 1 || zoom > 999.9) {
    emit requestFinished("ptz", false, "云台角度或变倍超出范围"); return;
  }
  auto req = std::make_shared<hk_camera::srv::SetPTZPose::Request>(); req->pan = pan; req->tilt = tilt; req->zoom = zoom;
  impl_->request<hk_camera::srv::SetPTZPose>("ptz", "/hk_camera/set_ptz_pose", req, [this](auto r) { emit requestFinished("ptz", r->success, QString::fromStdString(r->message)); });
#else
  Q_UNUSED(pan); Q_UNUSED(tilt); Q_UNUSED(zoom);
  emit requestFinished("ptz", false, "构建时未发现 hk_camera 消息包");
#endif
}
void RosAdapter::setPtzPreset(int action, int presetNo) {
#ifdef DESKTOP_HAS_HK_CAMERA
  if (action < 0 || action > 2 || presetNo < 1 || presetNo > 255) {
    emit requestFinished("ptz_preset", false, "预置位参数无效"); return;
  }
  auto req = std::make_shared<hk_camera::srv::PTZPreset::Request>();
  req->action = static_cast<uint8_t>(action);
  req->preset_no = static_cast<uint8_t>(presetNo);
  impl_->request<hk_camera::srv::PTZPreset>("ptz_preset", "/hk_camera/preset", req,
      [this](auto r) {
        emit requestFinished("ptz_preset", r->success, QString::fromStdString(r->message));
      });
#else
  Q_UNUSED(action); Q_UNUSED(presetNo);
  emit requestFinished("ptz_preset", false, "构建时未发现 hk_camera 消息包");
#endif
}

bool RosAdapter::ptzPresetAvailable() const {
#ifdef DESKTOP_HAS_HK_CAMERA
  return impl_->clients.at("/hk_camera/preset")->service_is_ready();
#else
  return false;
#endif
}

bool RosAdapter::cameraAvailable() const {
  const auto client = impl_->clients.at("/hk_camera/login");
  return client->service_is_ready() && impl_->clients.at("/hk_camera/start_stream")->service_is_ready();
}
bool RosAdapter::ptzAvailable() const {
#ifdef DESKTOP_HAS_HK_CAMERA
  return impl_->clients.at("/hk_camera/control_ptz")->service_is_ready() &&
      impl_->clients.at("/hk_camera/get_ptz_pose")->service_is_ready() &&
      impl_->clients.at("/hk_camera/set_ptz_pose")->service_is_ready();
#else
  return false;
#endif
}
bool RosAdapter::ptzNodePresent() const {
  if (ptzAvailable()) return true;
  for (const auto &name : impl_->node->get_node_names())
    if (QString::fromStdString(name).section('/', -1) == "ptz_pelco") return true;
  return false;
}
bool RosAdapter::cameraNodePresent() const {
  if (cameraAvailable() || impl_->node->count_publishers(impl_->config.imageTopic.toStdString()) > 0) return true;
  for (const auto &name : impl_->node->get_node_names())
    if (QString::fromStdString(name).section('/', -1) == "hk_camera_node") return true;
  return false;
}
void RosAdapter::cameraCommand(const QString &command) {
  if (command != "login" && command != "start_stream" && command != "stop_stream") return;
  const auto operation = "camera_" + command;
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  impl_->request<std_srvs::srv::Trigger>(operation, "/hk_camera/" + command.toStdString(), req,
      [this, operation](auto r) { emit requestFinished(operation, r->success, QString::fromStdString(r->message)); });
}
void RosAdapter::readCameraPose() {
#ifdef DESKTOP_HAS_HK_CAMERA
  auto req = std::make_shared<hk_camera::srv::GetPTZPose::Request>();
  impl_->request<hk_camera::srv::GetPTZPose>("ptz_read", "/hk_camera/get_ptz_pose", req, [this](auto r) {
    if (r->success && std::isfinite(r->pan) && std::isfinite(r->tilt) && std::isfinite(r->zoom)) {
      CameraPose pose; pose.pan = r->pan; pose.tilt = r->tilt; pose.zoom = r->zoom;
      pose.limitsValid = r->limits_valid;
      pose.panMin = r->pan_min; pose.panMax = r->pan_max;
      pose.tiltMin = r->tilt_min; pose.tiltMax = r->tilt_max;
      pose.zoomMin = r->zoom_min; pose.zoomMax = r->zoom_max;
      emit cameraPose(pose);
      emit requestFinished("ptz_read", true, QString::fromStdString(r->message));
    } else emit requestFinished("ptz_read", false, QString::fromStdString(r->message));
  });
#else
  emit requestFinished("ptz_read", false, "构建时未发现 hk_camera 消息包");
#endif
}
void RosAdapter::jogCamera(int command, int speed, bool stop) {
#ifdef DESKTOP_HAS_HK_CAMERA
  auto req = std::make_shared<hk_camera::srv::ControlPTZ::Request>();
  req->command = command; req->speed = speed; req->stop = stop;
  const auto operation = stop ? "ptz_stop" : "ptz_jog";
  impl_->request<hk_camera::srv::ControlPTZ>(operation, "/hk_camera/control_ptz", req,
      [this, operation](auto r) { emit requestFinished(operation, r->success, QString::fromStdString(r->message)); });
#else
  Q_UNUSED(command); Q_UNUSED(speed); Q_UNUSED(stop);
  emit requestFinished("ptz_jog", false, "构建时未发现 hk_camera 消息包");
#endif
}
void RosAdapter::recognize() {
#ifdef DESKTOP_HAS_OCR_INTERFACES
  auto req = std::make_shared<ocr_interfaces::srv::RecognizeText::Request>(); req->conf_threshold = 0.;
  impl_->request<ocr_interfaces::srv::RecognizeText>("ocr", "/ocr/recognize", req, [this](auto r) {
    QStringList lines;
    for (const auto &d : r->detections) lines << QString::fromStdString(d.text) + QString("  (%1%)").arg(d.confidence * 100., 0, 'f', 1);
    if (r->success) emit ocrResult(lines.isEmpty() ? "未识别到文本" : lines.join('\n'));
    emit requestFinished("ocr", r->success, QString::fromStdString(r->message) + QString(" / %1 ms").arg(r->processing_time_ms, 0, 'f', 0));
  });
#else
  emit requestFinished("ocr", false, "构建时未发现 ocr_interfaces 消息包");
#endif
}
void RosAdapter::saveState(const QString &backend, const QString &prefix) {
  if (backend == "fastlio") {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    // Snapshot saving is separate from loop acceptance and never replaces the
    // selected navigation map. The backend returns its exact artifact paths.
    impl_->request<std_srvs::srv::Trigger>("save_state", "/loop_closure/save_bundle", req,
        [this](auto r) { emit requestFinished("save_state", r->success, QString::fromStdString(r->message)); });
  }
#ifdef DESKTOP_HAS_SLAM_TOOLBOX
  else if (backend == "slam_toolbox") {
    auto req = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>(); req->filename = prefix.toStdString();
    impl_->request<slam_toolbox::srv::SerializePoseGraph>("save_state", "/slam_toolbox/serialize_map", req, [this](auto r) {
      emit requestFinished("save_state", r->result == 0, r->result == 0 ? "SLAM Toolbox 序列化完成" : "SLAM Toolbox 序列化失败");
    });
  }
#endif
#ifdef DESKTOP_HAS_CARTOGRAPHER_ROS_MSGS
  else if (backend == "cartographer") {
    auto req = std::make_shared<cartographer_ros_msgs::srv::WriteState::Request>(); req->filename = (prefix + ".pbstream").toStdString(); req->include_unfinished_submaps = true;
    impl_->request<cartographer_ros_msgs::srv::WriteState>("save_state", "/write_state", req, [this](auto r) { emit requestFinished("save_state", r->status.code == 0, QString::fromStdString(r->status.message)); });
  }
#endif
  else emit requestFinished("save_state", false, "构建时未发现对应建图服务消息包");
}
