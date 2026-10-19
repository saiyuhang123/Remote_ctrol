#include "vision/vision_client.h"
#include "runtime/process_manager.h"
#include <QElapsedTimer>
#include <QJsonDocument>
#include <QMap>
#include <QTimer>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#ifdef DESKTOP_HAS_VISION_INTERFACES
#include <vision_interfaces/srv/query_events.hpp>
#include <vision_interfaces/srv/set_recording_context.hpp>
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
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cameraFace;
  using Parameters = rcl_interfaces::srv::SetParametersAtomically;
  rclcpp::Client<Parameters>::SharedPtr parameters;
  QMap<QString, bool> wantedFeatures, pendingFeatures;
  bool waitingDetector = false, parametersBusy = false, sdkDisableSent = false;
  QElapsedTimer startup;
  QElapsedTimer recorderAttempt;
  quint64 parameterGeneration = 0;
  bool queryBusy = false;
  quint64 queryGeneration = 0;
#ifdef DESKTOP_HAS_VISION_INTERFACES
  rclcpp::Client<vision_interfaces::srv::QueryEvents>::SharedPtr queries;
  rclcpp::Client<vision_interfaces::srv::SetRecordingContext>::SharedPtr context;
#endif
  Impl(VisionClient *o, const DesktopConfig &c, bool isolated)
    : owner(o), config(c), offline(isolated), recorder(new ProcessManager(c, o)), detector(new ProcessManager(c, o)), timer(new QTimer(o)) {
    node = std::make_shared<rclcpp::Node>("desktop_vision_client");
    executor.add_node(node);
    cameraFace = node->create_client<std_srvs::srv::SetBool>("/hk_camera/set_face_capture");
    parameters = node->create_client<Parameters>("/detector_node/set_parameters_atomically");
#ifdef DESKTOP_HAS_VISION_INTERFACES
    queries = node->create_client<vision_interfaces::srv::QueryEvents>("/vision/query_events");
    context = node->create_client<vision_interfaces::srv::SetRecordingContext>("/vision/set_recording_context");
#endif
    for (const auto &topic : {"/vision/status", "/vision/detector_status"}) {
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
  bool detectorPresent() const {
    for (const auto &name : node->get_node_names())
      if (QString::fromStdString(name).section('/', -1) == "detector_node") return true;
    return false;
  }
  void queueFeatures(const QMap<QString, bool> &features) {
    bool enabling = false;
    for (auto it = features.cbegin(); it != features.cend(); ++it) {
      wantedFeatures[it.key()] = pendingFeatures[it.key()] = it.value();
      enabling |= it.value();
    }
    waitingDetector = true; startup.restart();
    if (enabling) { ensureRecorder(); emit owner->cameraRequested(); }
  }
  void disableSdkFaces() {
    if (sdkDisableSent || !cameraFace->service_is_ready()) return;
    sdkDisableSent = true;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>(); request->data = false;
    const auto pending = cameraFace->async_send_request(request, [this](auto future) {
      const auto result = future.get();
      if (!result->success) emit owner->message("上位机检测已选用；关闭旧海康人脸接收失败：" + QString::fromStdString(result->message));
    });
    QTimer::singleShot(8000, owner, [this, id = pending.request_id] { cameraFace->remove_pending_request(id); });
  }
  void requestFeatures() {
    if (parametersBusy || pendingFeatures.isEmpty()) return;
    auto request = std::make_shared<Parameters::Request>();
    for (auto it = pendingFeatures.cbegin(); it != pendingFeatures.cend(); ++it)
      request->parameters.push_back(rclcpp::Parameter(it.key().toStdString(), it.value()).to_parameter_msg());
    pendingFeatures.clear(); parametersBusy = true;
    const auto generation = ++parameterGeneration;
    const auto pending = parameters->async_send_request(request, [this, generation](auto future) {
      if (generation != parameterGeneration) return;
      parametersBusy = false;
      const auto result = future.get()->result;
      emit owner->message(result.successful ? "上位机检测开关已更新，等待检测状态反馈"
        : "上位机检测设置失败：" + QString::fromStdString(result.reason) + "；请确认检测节点已更新");
      waitingDetector = !pendingFeatures.isEmpty();
    });
    QTimer::singleShot(8000, owner, [this, generation, id = pending.request_id] {
      if (generation != parameterGeneration || !parametersBusy) return;
      parameters->remove_pending_request(id); parametersBusy = false; ++parameterGeneration;
      waitingDetector = !pendingFeatures.isEmpty();
      emit owner->message("上位机检测设置超时，请核对检测节点状态后重试");
    });
  }
  void tick() {
    executor.spin_some(std::chrono::milliseconds(3));
    if (!waitingDetector || parametersBusy) return;
    if (startup.elapsed() > 30000) {
      waitingDetector = false; pendingFeatures.clear();
      emit owner->message("视觉启动超时：请检查相机连接、模块构建和组件日志");
      return;
    }
    bool enabling = false;
    for (auto value : pendingFeatures) enabling |= value;
    if (enabling) {
      disableSdkFaces();
      if (!ensureRecorder()) return;
    }
    if (parameters->service_is_ready()) { requestFeatures(); return; }
    if (detector->running() || detectorPresent()) return;
    if (!enabling) { waitingDetector = false; pendingFeatures.clear(); return; }
    auto flag = [this](const char *name) { return QString(name) + ":=" + (wantedFeatures.value(name, false) ? "true" : "false"); };
    detector->start({"ros2", "run", "detector_node", "detector", "--ros-args", "-p", "input_mode:=ros",
      "-p", "image_topic:=" + config.imageTopic, "-p", flag("enable_face"),
      "-p", flag("enable_person"), "-p", flag("enable_plate"), "-p", "staging_dir:=" + config.projectRoot + "/data/vision/staging"});
  }
  ~Impl() {
    timer->stop();
    delete detector;
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
  impl_->queueFeatures({{"enable_face", true}});
}
void VisionClient::stopFace() {
  if (impl_->offline) return;
  impl_->queueFeatures({{"enable_face", false}});
}
void VisionClient::startDetector(bool person, bool plate) {
  if (impl_->offline || !supported()) return;
  if (!person && !plate) { emit message("请至少选择行人或车牌检测"); return; }
  impl_->queueFeatures({{"enable_person", person}, {"enable_plate", plate}});
}
void VisionClient::stopDetector() {
  if (impl_->offline) return;
  impl_->queueFeatures({{"enable_face", false}, {"enable_person", false}, {"enable_plate", false}});
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
