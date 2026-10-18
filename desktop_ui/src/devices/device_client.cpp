#include "devices/device_client.h"
#include "runtime/process_manager.h"
#include <QElapsedTimer>
#include <QJsonDocument>
#include <QThread>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

struct DeviceClient::Impl {
  using Parameters = rcl_interfaces::srv::SetParametersAtomically;
  using Switch = std_srvs::srv::SetBool;
  DeviceClient *owner;
  DesktopConfig config;
  bool offline, authorized = false;
  ProcessManager *gas, *ptz;
  QTimer *timer;
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Client<Switch>::SharedPtr gasSwitch;
  rclcpp::Client<Parameters>::SharedPtr ptzParameters;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions;
  bool gasWanted = false, ptzWanted = false, gasWaiting = false, ptzWaiting = false;
  bool gasBusy = false, ptzBusy = false, ptzTouched = false, blocked = false;
  bool gasTouched = false, lastPtzActive = false;
  quint64 gasGeneration = 0, ptzGeneration = 0;
  qint64 ptzBarrierNs = 0;
  QElapsedTimer gasStartup, ptzStartup, ptzAge;
  int speed = 2;
  double seconds = 20, minimum = -1, maximum = -1;
  QJsonObject lastPtz;

  Impl(DeviceClient *o, const DesktopConfig &c, bool isolated)
    : owner(o), config(c), offline(isolated), gas(new ProcessManager(c, o)),
      ptz(new ProcessManager(c, o)), timer(new QTimer(o)) {
    node = std::make_shared<rclcpp::Node>("desktop_device_client");
    executor.add_node(node);
    gasSwitch = node->create_client<Switch>("/gas_detector/set_enabled");
    ptzParameters = node->create_client<Parameters>("/ptz_patrol/set_parameters_atomically");
    for (const auto &topic : {"/gas_detector/status", "/gas_detector/data", "/ptz_patrol/status"}) {
      const QString name = topic;
      subscriptions.push_back(node->create_subscription<std_msgs::msg::String>(topic, 10,
        [this, name](const std_msgs::msg::String::SharedPtr message) {
          const auto value = QJsonDocument::fromJson(QByteArray::fromStdString(message->data)).object();
          if (value.isEmpty()) return;
          if (name == "/gas_detector/data") { emit owner->gasSample(value); return; }
          if (name == "/ptz_patrol/status") {
            ptzAge.restart(); lastPtz = value; lastPtzActive = value.value("active").toBool(true);
            const bool afterRequest = value.value("status_time_ns").toDouble() >= ptzBarrierNs;
            setBlocked(lastPtzActive || ptzWaiting || ptzBusy || ptzWanted || !afterRequest);
          }
          emit owner->status(value.value("component").toString(), value);
        }));
    }
    for (auto *process : {gas, ptz}) {
      QObject::connect(process, &ProcessManager::output, o, [this](const QString &value) { emit owner->message("设备执勤：" + value); });
    }
    timer->setInterval(50);
    QObject::connect(timer, &QTimer::timeout, o, [this] { tick(); }); timer->start();
  }
  bool present(const std::string &name) const {
    for (const auto &entry : node->get_node_names()) if (entry == name) return true;
    return false;
  }
  void setBlocked(bool value) {
    if (value == blocked) return;
    blocked = value; emit owner->ptzBlocked(value);
  }
  bool freshPtz() const { return ptzAge.isValid() && ptzAge.elapsed() < 2000; }
  void requestGas() {
    if (gasBusy || !gasSwitch->service_is_ready()) return;
    gasWaiting = false; gasBusy = true;
    const bool enabled = gasWanted;
    if (enabled) gasTouched = true;
    const auto generation = ++gasGeneration;
    auto request = std::make_shared<Switch::Request>(); request->data = enabled;
    const auto pending = gasSwitch->async_send_request(request, [this, enabled, generation](rclcpp::Client<Switch>::SharedFuture future) {
      if (generation != gasGeneration) return;
      gasBusy = false;
      const auto result = future.get();
      emit owner->message("气体采集：" + QString::fromStdString(result->message));
      if (result->success && !enabled) { gasTouched = false; if (gas->running()) gas->stop(); }
      if (gasWanted != enabled) requestGas();
    });
    QTimer::singleShot(4000, owner, [this, generation, id = pending.request_id] {
      if (generation != gasGeneration || !gasBusy) return;
      gasSwitch->remove_pending_request(id); gasBusy = false; ++gasGeneration;
      emit owner->message("气体采集请求超时，请核对设备状态");
      if (!gasWanted && gas->running()) gas->stop();
    });
  }
  void requestPtz() {
    if (ptzBusy || !ptzParameters->service_is_ready()) return;
    if (ptzWanted && (!authorized || !freshPtz())) return;
    ptzWaiting = false; ptzBusy = true; setBlocked(true);
    const bool enabled = ptzWanted;
    if (enabled) ptzTouched = true;
    const auto generation = ++ptzGeneration;
    auto request = std::make_shared<Parameters::Request>();
    if (enabled) {
      for (const auto &parameter : {rclcpp::Parameter("pan_speed", speed), rclcpp::Parameter("sweep_seconds", seconds),
             rclcpp::Parameter("pan_min", minimum), rclcpp::Parameter("pan_max", maximum)})
        request->parameters.push_back(parameter.to_parameter_msg());
    }
    request->parameters.push_back(rclcpp::Parameter("enabled", enabled).to_parameter_msg());
    const auto pending = ptzParameters->async_send_request(request, [this, enabled, generation](rclcpp::Client<Parameters>::SharedFuture future) {
      if (generation != ptzGeneration) return;
      ptzBusy = false;
      const auto result = future.get()->result;
      emit owner->message(result.successful ? (enabled ? "已请求开启云台巡航，等待设备反馈" : "已请求停止云台巡航，等待停止确认")
                                         : "云台巡航请求失败：" + QString::fromStdString(result.reason));
      // A pre-request stopped sample cannot authorize manual motion after enable.
      ptzAge.invalidate();
      ptzBarrierNs = node->now().nanoseconds();
      if (ptzWanted != enabled) requestPtz();
    });
    QTimer::singleShot(4000, owner, [this, generation, id = pending.request_id] {
      if (generation != ptzGeneration || !ptzBusy) return;
      ptzParameters->remove_pending_request(id); ptzBusy = false; ++ptzGeneration;
      emit owner->message("云台巡航请求超时，手动控制保持锁定；请点击停止并等待状态确认");
      if (!ptzWanted && ptz->running()) ptz->stop();
    });
  }
  void tick() {
    executor.spin_some(std::chrono::milliseconds(3));
    if (gasWaiting && !gasBusy) {
      if (gasSwitch->service_is_ready()) requestGas();
      else if (gasStartup.elapsed() > 15000) {
        gasWaiting = false; gasWanted = false; if (gas->running()) gas->stop();
        emit owner->message("气体检测节点启动超时；请先构建 gas_detector 包");
      }
    }
    if (ptzWaiting && !ptzBusy) {
      if (ptzParameters->service_is_ready() && (!ptzWanted || freshPtz())) requestPtz();
      else if (ptzStartup.elapsed() > 15000) {
        ptzWaiting = false; ptzWanted = false;
        emit owner->message("云台巡航启动超时，缺少新版本状态反馈；请构建 chassis_nav2 并检查相机");
        if (ptz->running()) ptz->stop();
      }
    }
    // A lost status after active patrol remains blocked until a fresh stop ack.
    if (!freshPtz() && (lastPtzActive || ptzTouched || blocked || present("/ptz_patrol"))) setBlocked(true);
  }
  void shutdown() {
    timer->stop(); gasWanted = ptzWanted = false;
    if (gasTouched) requestGas();
    if (ptzTouched) requestPtz();
    QElapsedTimer drain; drain.start();
    while (drain.elapsed() < 1100 && (gasBusy || ptzBusy || (ptzTouched && blocked))) {
      executor.spin_some(std::chrono::milliseconds(3)); QThread::msleep(10);
    }
    gas->stop(); ptz->stop();
    QObject::disconnect(gas, nullptr, owner, nullptr);
    QObject::disconnect(ptz, nullptr, owner, nullptr);
    delete gas; delete ptz;
    executor.remove_node(node);
  }
};

DeviceClient::DeviceClient(const DesktopConfig &config, bool offline, QObject *parent)
  : QObject(parent), impl_(std::make_unique<Impl>(this, config, offline)) {}
DeviceClient::~DeviceClient() { impl_->shutdown(); }
void DeviceClient::setControlEnabled(bool enabled) {
  const bool wasEnabled = impl_->authorized;
  impl_->authorized = enabled;
  if (!enabled && wasEnabled && (impl_->ptzWanted || impl_->ptzTouched)) stopPtz();
}
void DeviceClient::startGas(const QString &port, int address, int baudrate, double interval) {
  auto &i = *impl_;
  if (i.offline || i.gasWanted || i.gasBusy) return;
  if (port.trimmed().isEmpty()) { emit message("请明确指定气体检测串口"); return; }
  i.gasWanted = i.gasWaiting = true; i.gasStartup.restart();
  if (!i.gasSwitch->service_is_ready() && !i.present("/gas_detector") && !i.gas->running())
    i.gas->start({"ros2", "run", "gas_detector", "gas_monitor", "--ros-args", "-p", "port:=" + port.trimmed(),
                  "-p", "address:=" + QString::number(address), "-p", "baudrate:=" + QString::number(baudrate),
                  "-p", "interval:=" + QString::number(interval, 'f', 1)});
  else if (i.gasSwitch->service_is_ready()) emit message("复用已有气体节点，使用该节点的串口参数");
}
void DeviceClient::stopGas() {
  auto &i = *impl_; if (i.offline) return;
  i.gasWanted = false; i.gasWaiting = i.gas->running(); i.gasStartup.restart();
  if (i.gasSwitch->service_is_ready()) i.requestGas();
  else if (i.gas->running()) i.gas->stop();
}
void DeviceClient::startPtz(int speed, double seconds, double minimum, double maximum) {
  auto &i = *impl_;
  if (i.offline || !i.authorized || i.ptzWanted || i.ptzBusy) return;
  if (i.freshPtz() && i.lastPtzActive) { emit message("云台已经在巡航，请先停止再调整参数"); return; }
  i.speed = speed; i.seconds = seconds; i.minimum = minimum; i.maximum = maximum;
  i.ptzWanted = i.ptzWaiting = true; i.ptzStartup.restart(); i.setBlocked(true);
  emit cameraRequested();
  if (!i.ptzParameters->service_is_ready() && !i.present("/ptz_patrol") && !i.ptz->running())
    i.ptz->start({"ros2", "run", "chassis_nav2", "ptz_patrol.py"});
}
void DeviceClient::stopPtz() {
  auto &i = *impl_; if (i.offline) return;
  i.ptzWanted = false; i.ptzWaiting = true; i.ptzStartup.restart();
  i.requestPtz();
}
