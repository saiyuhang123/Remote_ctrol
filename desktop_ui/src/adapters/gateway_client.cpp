#include "adapters/gateway_client.h"
#include <QJsonDocument>
#include <QUrl>
#include <algorithm>
#include <cmath>

GatewayClient::GatewayClient(double maxLinear, double maxAngular, QObject *parent, int feedbackTimeoutMs)
  : QObject(parent), feedbackTimeoutMs_(feedbackTimeoutMs), maxLinear_(maxLinear), maxAngular_(maxAngular) {
  repeat_.setInterval(50);
  connect(&repeat_, &QTimer::timeout, this, [this] {
    if (!feedbackFresh() || estopped_ || navActive_) { emergencyStop(); return; }
    send({{"type", "cmd_vel"}, {"linear", linear_}, {"angular", angular_}});
  });
  preparation_.setInterval(50);
  connect(&preparation_, &QTimer::timeout, this, [this] {
    if (preparationAge_.elapsed() > 5000) {
      preparation_.stop(); emit message("手动接管超时：请检查急停反馈与导航取消状态"); return;
    }
    if (preparationAge_.elapsed() >= 250 && feedbackFresh() && estopped_ && navKnown_ && !navActive_) {
      preparation_.stop(); manualReady_ = true; emit manualChanged(true);
      emit message("导航已停止。点击“解除急停”后，按住方向按钮行驶");
    }
  });
  connect(&socket_, &QWebSocket::connected, this, [this] { resetControl(); emit connectionChanged(true); });
  connect(&socket_, &QWebSocket::disconnected, this, [this] { resetControl(); fresh_ = false; emit connectionChanged(false); });
  connect(&socket_, &QWebSocket::textMessageReceived, this, [this](const QString &text) {
    QJsonParseError err;
    const auto doc = QJsonDocument::fromJson(text.toUtf8(), &err);
    if (err.error == QJsonParseError::NoError && doc.isObject()) emit received(doc.object());
  });
  connect(&socket_, QOverload<QAbstractSocket::SocketError>::of(&QWebSocket::error), this,
          [this](QAbstractSocket::SocketError) { emit message("网关：" + socket_.errorString()); });
}
bool GatewayClient::connected() const { return socket_.state() == QAbstractSocket::ConnectedState; }
void GatewayClient::connectTo(const QUrl &url) { stopManual(); resetControl(); socket_.abort(); socket_.open(url); }
void GatewayClient::disconnectFrom() { stopManual(); resetControl(); socket_.close(); }
void GatewayClient::resetControl() {
  repeat_.stop(); preparation_.stop(); manualMoving_ = false; manualReady_ = false; enabled_ = false;
  emit manualChanged(false); emit controlChanged(false);
}
void GatewayClient::setControlEnabled(bool enabled) {
  if (!enabled && (manualMoving_ || preparation_.isActive())) emergencyStop();
  if (!enabled) { preparation_.stop(); manualReady_ = false; emit manualChanged(false); }
  enabled_ = enabled && connected(); emit controlChanged(enabled_);
}
void GatewayClient::updateFeedback(bool estopped, bool navActive, bool navKnown, bool fresh) {
  estopped_ = estopped; navActive_ = navActive; navKnown_ = navKnown; fresh_ = fresh;
  feedbackAge_.restart();
  if (manualMoving_ && (!fresh || estopped || navActive)) emergencyStop();
}
bool GatewayClient::feedbackFresh() const { return fresh_ && feedbackAge_.isValid() && feedbackAge_.elapsed() <= feedbackTimeoutMs_; }
bool GatewayClient::send(const QJsonObject &value, bool emergency) {
  if (!connected() || (!emergency && !enabled_)) {
    emit message("操作未发送：请连接网关并启用操作"); return false;
  }
  return socket_.sendTextMessage(QString::fromUtf8(QJsonDocument(value).toJson(QJsonDocument::Compact))) >= 0;
}
bool GatewayClient::emergencyStop() {
  repeat_.stop(); preparation_.stop(); manualMoving_ = false; manualReady_ = false;
  emit manualChanged(false);
  return send({{"type", "estop"}}, true);
}
bool GatewayClient::takeManualControl() {
  if (!enabled_ || !connected() || !feedbackFresh()) { emit message("接管失败：需要网关连接和有效的 ROS 急停反馈"); return false; }
  if (!emergencyStop()) return false;
  preparationAge_.restart(); preparation_.start(); return true;
}
bool GatewayClient::releaseStop() {
  if (!feedbackFresh() || !navKnown_ || navActive_) { emit message("暂不能解除急停：导航状态未知或尚未停止"); return false; }
  return send({{"type", "cmd_vel"}, {"linear", 0.}, {"angular", 0.}});
}
bool GatewayClient::move(double linear, double angular) {
  if (!manualReady_ || !feedbackFresh() || estopped_ || navActive_ || !navKnown_ ||
      !std::isfinite(linear) || !std::isfinite(angular) || std::abs(linear) < 1e-6) {
    emit message("方向指令未发送：请先接管手动、解除急停并确认导航已停止"); return false;
  }
  linear_ = std::clamp(linear, -maxLinear_, maxLinear_);
  angular_ = std::clamp(angular, -maxAngular_, maxAngular_);
  if (!send({{"type", "cmd_vel"}, {"linear", linear_}, {"angular", angular_}})) return false;
  manualMoving_ = true; repeat_.start(); return true;
}
void GatewayClient::stopManual() {
  repeat_.stop();
  if (!manualMoving_) return;
  manualMoving_ = false;
  if (feedbackFresh() && !estopped_) send({{"type", "cmd_vel"}, {"linear", 0.}, {"angular", 0.}});
  else emergencyStop();
}
bool GatewayClient::poseRequest(const QString &type, double x, double y, double yaw) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return false;
  yaw = std::atan2(std::sin(yaw), std::cos(yaw));
  return send({{"type", type}, {"x", x}, {"y", y}, {"yaw", yaw}});
}
bool GatewayClient::navigate(double x, double y, double yaw) {
  if (!feedbackFresh() || estopped_ || manualMoving_) { emit message("目标未发送：需要有效急停反馈、解除急停并结束手动行驶"); return false; }
  manualReady_ = false; emit manualChanged(false);
  return poseRequest("nav_goal", x, y, yaw);
}
bool GatewayClient::initialPose(double x, double y, double yaw) { return poseRequest("initial_pose", x, y, yaw); }
bool GatewayClient::cancelNavigation() { return send({{"type", "nav_cancel"}}); }
bool GatewayClient::capture() { return send({{"type", "capture"}, {"label", "desktop_manual"}}); }
bool GatewayClient::speak(const QString &text) { return !text.trimmed().isEmpty() && send({{"type", "tts"}, {"text", text.trimmed()}}); }
