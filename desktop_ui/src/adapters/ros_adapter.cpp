#include "adapters/ros_adapter.h"
#include <QTimer>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <map>
#include <thread>
#ifdef DESKTOP_HAS_YUNLE_MSGS
#include <yunle_msgs/msg/battery.hpp>
#endif
#ifdef DESKTOP_HAS_HK_CAMERA
#include <hk_camera/srv/capture_picture.hpp>
#include <hk_camera/srv/set_ptz_pose.hpp>
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
  std::map<std::string, rclcpp::ClientBase::SharedPtr> clients;
  rclcpp::TimerBase::SharedPtr timer;
  std::mutex frameMutex;
  QString frame = "map";
  std::mutex imageMutex;
  QImage latestImage;
  QTimer *imagePump = nullptr;
  RobotSnapshot state;
  qint64 odomTime = 0, stopTime = 0, batteryTime = 0, imageTime = 0;
  bool navReceived = false;
  std::map<quint64, std::function<void()>> cancelPending;
  quint64 nextRequest = 0;
  Impl(RosAdapter *o, const DesktopConfig &c) : owner(o), config(c) {
    node = std::make_shared<rclcpp::Node>("robot_desktop_state");
    // Keep clients alive so DDS discovery completes before an operator clicks.
    clients["/map_save"] = node->create_client<std_srvs::srv::Trigger>("/map_save");
#ifdef DESKTOP_HAS_HK_CAMERA
    clients["/hk_camera/capture"] = node->create_client<hk_camera::srv::CapturePicture>("/hk_camera/capture");
    clients["/hk_camera/set_ptz_pose"] = node->create_client<hk_camera::srv::SetPTZPose>("/hk_camera/set_ptz_pose");
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
      "/emergency_stop", rclcpp::QoS(10), [this](std_msgs::msg::Bool::ConstSharedPtr m) { state.estopped = m->data; stopTime = nowMs(); }));
    subscriptions.push_back(node->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr m) {
        state.navigationActive = false; navReceived = true;
        for (const auto &s : m->status_list) if (s.status >= 1 && s.status <= 3) state.navigationActive = true;
      }));
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
    state.batteryValid = batteryTime && current - batteryTime < 5000;
    const bool navPublisher = node->count_publishers("/navigate_to_pose/_action/status") > 0;
    state.navigationKnown = !navPublisher || navReceived;
    if (!navPublisher) { state.navigationActive = false; navReceived = false; }
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
    QTimer::singleShot(config.serviceTimeoutMs, owner, [this, client, done, id, operation, key] {
      if (!done->exchange(true)) { client->remove_pending_request(id); emit owner->requestFinished(operation, false, "服务响应超时，请检查节点日志和输出文件"); }
      cancelPending.erase(key);
    });
  }
};

RosAdapter::RosAdapter(const DesktopConfig &c, QObject *parent) : QObject(parent) { qRegisterMetaType<RobotSnapshot>(); impl_ = std::make_unique<Impl>(this, c); }
RosAdapter::~RosAdapter() = default;
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
  const QStringList names{"controller_server", "planner_server", "bt_navigator", "slam_toolbox", "cartographer_node", "fast_lio_mapping", "laser_mapping", "map_server", "twist_mux", "chassis_driver", "jd03_odometry", "ekf_filter_node", "robot_state_publisher", "rslidar_sdk_node", "imuDriverNode", "lio_odometry", "global_localization", "transform_fusion", "pointcloud_to_scan", "cloud_crop_box"};
  QStringList found;
  for (const auto &name : impl_->node->get_node_names()) {
    const auto qname = QString::fromStdString(name);
    if (names.contains(qname.section('/', -1)) || qname.section('/', -1).startsWith("tf_base_link_to_")) found << qname;
  }
  for (const auto &topic : {"/cmd_vel", "/odom", "/scan", "/map", "/Odometry", "/cloud_registered"})
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
  if (!std::isfinite(pan) || !std::isfinite(tilt) || !std::isfinite(zoom) || pan < 0 || pan > 360 || tilt < 0 || tilt > 90 || zoom < 1 || zoom > 40) return;
  auto req = std::make_shared<hk_camera::srv::SetPTZPose::Request>(); req->pan = pan; req->tilt = tilt; req->zoom = zoom;
  impl_->request<hk_camera::srv::SetPTZPose>("ptz", "/hk_camera/set_ptz_pose", req, [this](auto r) { emit requestFinished("ptz", r->success, QString::fromStdString(r->message)); });
#else
  Q_UNUSED(pan); Q_UNUSED(tilt); Q_UNUSED(zoom);
  emit requestFinished("ptz", false, "构建时未发现 hk_camera 消息包");
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
    impl_->request<std_srvs::srv::Trigger>("save_state", "/map_save", req, [this](auto r) { emit requestFinished("save_state", r->success, QString::fromStdString(r->message)); });
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
