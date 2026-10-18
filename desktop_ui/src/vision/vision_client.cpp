#include "vision/vision_client.h"
#include "runtime/process_manager.h"
#include <QElapsedTimer>
#include <QJsonDocument>
#include <QTimer>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#ifdef DESKTOP_HAS_VISION_INTERFACES
#include <vision_interfaces/srv/query_events.hpp>
#include <vision_interfaces/srv/set_recording_context.hpp>
#include <rclcpp/parameter_client.hpp>
#endif

struct VisionClient::Impl {
  VisionClient *owner;
  DesktopConfig config;
  bool offline;
  ProcessManager *recorder, *detector;
  QTimer *timer;
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr face;
  bool waitingFace = false, waitingDetector = false, faceBusy = false, enabledFaceHere = false;
  bool desiredFace = false, desiredDetector = false;
  bool person = true, plate = true;
  QElapsedTimer startup;
  QElapsedTimer recorderAttempt;
  quint64 faceGeneration = 0;
  bool queryBusy = false;
  quint64 queryGeneration = 0;
#ifdef DESKTOP_HAS_VISION_INTERFACES
  rclcpp::Client<vision_interfaces::srv::QueryEvents>::SharedPtr queries;
  rclcpp::Client<vision_interfaces::srv::SetRecordingContext>::SharedPtr context;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters;
#endif
  Impl(VisionClient *o, const DesktopConfig &c, bool isolated)
    : owner(o), config(c), offline(isolated), recorder(new ProcessManager(c, o)), detector(new ProcessManager(c, o)), timer(new QTimer(o)) {
    node = std::make_shared<rclcpp::Node>("desktop_vision_client");
    executor.add_node(node);
    face = node->create_client<std_srvs::srv::SetBool>("/hk_camera/set_face_capture");
#ifdef DESKTOP_HAS_VISION_INTERFACES
    queries = node->create_client<vision_interfaces::srv::QueryEvents>("/vision/query_events");
    context = node->create_client<vision_interfaces::srv::SetRecordingContext>("/vision/set_recording_context");
    parameters = std::make_shared<rclcpp::AsyncParametersClient>(node, "/detector_node");
#endif
    for (const auto &topic : {"/vision/status", "/vision/detector_status", "/hk_camera/face_status"}) {
      subscriptions.push_back(node->create_subscription<std_msgs::msg::String>(topic, 10,
        [this](const std_msgs::msg::String::SharedPtr message) {
          const auto value = QJsonDocument::fromJson(QByteArray::fromStdString(message->data)).object();
          if (!value.isEmpty()) emit owner->status(value.value("component").toString(), value);
        }));
    }
    for (auto *process : {recorder, detector}) {
      QObject::connect(process, &ProcessManager::output, o, [this](const QString &text) { emit owner->message("视觉：" + text); });
      QObject::connect(process, &ProcessManager::finished, o, [this](int code) { emit owner->message(QString("视觉组件退出，退出码 %1").arg(code)); });
    }
    timer->setInterval(50);
    QObject::connect(timer, &QTimer::timeout, o, [this] { tick(); });
    timer->start();
  }
  bool ensureRecorder() {
#ifdef DESKTOP_HAS_VISION_INTERFACES
    if (queries->service_is_ready()) return true;
    if (!recorder->running() && !offline && (!recorderAttempt.isValid() || recorderAttempt.elapsed() > 2000)) {
      recorderAttempt.restart();
      recorder->start({"ros2", "run", "vision_events", "recorder", "--ros-args", "-p", "data_dir:=" + config.projectRoot + "/data/vision"});
    }
#endif
    return false;
  }
  void requestFace(bool enabled) {
    desiredFace = enabled;
    if (faceBusy || !face->service_is_ready()) return;
    faceBusy = true;
    if (enabled) enabledFaceHere = true;  // a timeout may still have enabled SDK reception
    const auto generation = ++faceGeneration;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>(); request->data = enabled;
    const auto pending = face->async_send_request(request, [this, enabled, generation](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      if (generation != faceGeneration) return;
      faceBusy = false;
      const auto result = future.get();
      if (result->success) enabledFaceHere = enabled;
      emit owner->message((result->success ? "人脸抓拍：" : "人脸抓拍失败：") + QString::fromStdString(result->message));
      if (desiredFace != enabled) requestFace(desiredFace);
    });
    QTimer::singleShot(8000, owner, [this, generation, id = pending.request_id] {
      if (generation != faceGeneration || !faceBusy) return;
      face->remove_pending_request(id); faceBusy = false; ++faceGeneration;
      emit owner->message("人脸接收服务请求超时，请检查设备和相机节点");
    });
  }
  void tick() {
    executor.spin_some(std::chrono::milliseconds(3));
    if (detector->running() && !desiredDetector) detector->stop();
    if (!waitingFace && !waitingDetector) return;
    if (startup.elapsed() > 30000) {
      waitingFace = waitingDetector = false;
      emit owner->message("视觉启动超时：请检查相机连接、模块构建和组件日志");
      return;
    }
    if (!ensureRecorder()) return;
    if (waitingFace && face->service_is_ready()) { waitingFace = false; requestFace(true); }
#ifdef DESKTOP_HAS_VISION_INTERFACES
    if (waitingDetector) {
      waitingDetector = false;
      if (parameters->service_is_ready()) {
        parameters->set_parameters_atomically({rclcpp::Parameter("enable_person", person), rclcpp::Parameter("enable_plate", plate)},
          [this](auto future) { if (!future.get().successful) emit owner->message("上位机检测开关修改失败"); });
      } else if (!detector->running()) {
        detector->start({"ros2", "run", "detector_node", "detector", "--ros-args", "-p", "input_mode:=ros",
          "-p", "image_topic:=" + config.imageTopic, "-p", "enable_person:=" + QString(person ? "true" : "false"),
          "-p", "enable_plate:=" + QString(plate ? "true" : "false"), "-p", "staging_dir:=" + config.projectRoot + "/data/vision/staging"});
      }
    }
#endif
  }
  ~Impl() {
    timer->stop();
    delete detector;
    // Only close SDK reception enabled by this client; never kill external nodes.
    if (enabledFaceHere && face->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>(); request->data = false;
      auto result = face->async_send_request(request);
      QElapsedTimer age; age.start();
      while (rclcpp::ok() && age.elapsed() < 1500 && result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        executor.spin_some(); QThread::msleep(10);
      }
      // Give accepted camera events time to drain before closing our recorder.
      QElapsedTimer drain; drain.start();
      while (rclcpp::ok() && drain.elapsed() < 1100) { executor.spin_some(); QThread::msleep(10); }
    }
    delete recorder;
    executor.remove_node(node);
  }
};

VisionClient::VisionClient(const DesktopConfig &config, bool offline, QObject *parent)
  : QObject(parent), impl_(std::make_unique<Impl>(this, config, offline)) {}
VisionClient::~VisionClient() = default;
bool VisionClient::supported() const {
#ifdef DESKTOP_HAS_VISION_INTERFACES
  return true;
#else
  return false;
#endif
}
void VisionClient::startFace() {
  if (impl_->offline || !supported()) return;
  impl_->desiredFace = true; impl_->waitingFace = true; impl_->startup.restart(); impl_->ensureRecorder(); emit cameraRequested();
}
void VisionClient::stopFace() {
  if (impl_->offline) return;
  impl_->waitingFace = false;
  impl_->desiredFace = false;
  if (!impl_->face->service_is_ready()) { emit message("相机未就绪，无法停止人脸接收"); return; }
  impl_->requestFace(false);
}
void VisionClient::startDetector(bool person, bool plate) {
  if (impl_->offline || !supported()) return;
  if (!person && !plate) { emit message("请至少选择行人或车牌检测"); return; }
  impl_->person = person; impl_->plate = plate; impl_->waitingDetector = true;
  impl_->desiredDetector = true;
  impl_->startup.restart(); impl_->ensureRecorder(); emit cameraRequested();
}
void VisionClient::stopDetector() {
  if (impl_->offline) return;
  impl_->waitingDetector = false;
  impl_->desiredDetector = false;
  if (impl_->detector->running()) impl_->detector->stop();
#ifdef DESKTOP_HAS_VISION_INTERFACES
  else if (impl_->parameters->service_is_ready()) {
    impl_->parameters->set_parameters_atomically({rclcpp::Parameter("enable_person", false), rclcpp::Parameter("enable_plate", false)});
  }
#endif
}
void VisionClient::query(const QString &type, const QString &cursor) {
#ifdef DESKTOP_HAS_VISION_INTERFACES
  if (!impl_->ensureRecorder()) { emit message("记录服务正在启动，请稍后刷新"); return; }
  if (impl_->queryBusy) { emit message("视觉记录查询中，请稍候"); return; }
  impl_->queryBusy = true;
  const auto generation = ++impl_->queryGeneration;
  auto request = std::make_shared<vision_interfaces::srv::QueryEvents::Request>();
  request->event_type = type.toStdString(); request->cursor = cursor.toStdString(); request->limit = 30;
  const auto pending = impl_->queries->async_send_request(request, [this, generation](rclcpp::Client<vision_interfaces::srv::QueryEvents>::SharedFuture future) {
    if (generation != impl_->queryGeneration) return;
    impl_->queryBusy = false;
    const auto response = future.get();
    if (!response->success) { emit message(QString::fromStdString(response->message)); return; }
    emit records(QJsonDocument::fromJson(QByteArray::fromStdString(response->events_json)).array(),
                 QString::fromStdString(response->next_cursor), response->total);
  });
  QTimer::singleShot(8000, this, [this, generation, id = pending.request_id] {
    if (generation != impl_->queryGeneration || !impl_->queryBusy) return;
    impl_->queries->remove_pending_request(id); impl_->queryBusy = false; ++impl_->queryGeneration;
    emit message("视觉记录查询超时，请检查记录节点");
  });
#else
  Q_UNUSED(type); Q_UNUSED(cursor);
#endif
}
void VisionClient::setMapLabel(const QString &label) {
#ifdef DESKTOP_HAS_VISION_INTERFACES
  if (impl_->offline) return;
  if (!impl_->context->service_is_ready()) { emit message("记录服务尚未就绪，请稍后关联地图标记"); return; }
  auto request = std::make_shared<vision_interfaces::srv::SetRecordingContext::Request>(); request->map_id = label.trimmed().toStdString();
  impl_->context->async_send_request(request, [this](rclcpp::Client<vision_interfaces::srv::SetRecordingContext>::SharedFuture future) { emit message(QString::fromStdString(future.get()->message)); });
#else
  Q_UNUSED(label);
#endif
}
