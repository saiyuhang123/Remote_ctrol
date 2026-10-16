#include "adapters/local_controller.h"
#include "adapters/ros_adapter.h"
#include <algorithm>
#include <cmath>

LocalController::LocalController(RosAdapter *ros, double maxLinear, double maxAngular,
                                 int feedbackTimeoutMs, QObject *parent)
  : QObject(parent), ros_(ros), feedbackTimeoutMs_(feedbackTimeoutMs),
    maxLinear_(maxLinear), maxAngular_(maxAngular) {
  repeat_.setInterval(50);
  connect(&repeat_, &QTimer::timeout, this, [this] {
    if (!feedbackFresh() || estopped_ || navActive_ || !ros_->publishVelocity(linear_, angular_)) emergencyStop();
  });
  preparation_.setInterval(50);
  connect(&preparation_, &QTimer::timeout, this, [this] {
    if (preparationAge_.elapsed() > 5000) {
      preparation_.stop(); unlockWaiting_ = false;
      emit message("操作等待超时：请检查急停反馈及 Nav2 取消结果"); return;
    }
    if (!navKnown_ || navActive_) return;
    if (unlockWaiting_) {
      preparation_.stop(); unlockWaiting_ = false; ros_->setEmergencyStop(false); return;
    }
    if (preparationAge_.elapsed() >= 250 && feedbackFresh() && estopped_) {
      preparation_.stop(); manualReady_ = true; emit manualChanged(true);
      emit message("导航已停止。点击“解除急停”后，按住方向按钮行驶");
    }
  });
}
bool LocalController::feedbackFresh() const {
  return fresh_ && feedbackAge_.isValid() && feedbackAge_.elapsed() <= feedbackTimeoutMs_;
}
void LocalController::setControlEnabled(bool enabled) {
  if (!enabled && (manualMoving_ || preparation_.isActive())) emergencyStop();
  if (!enabled) { preparation_.stop(); manualReady_ = false; emit manualChanged(false); }
  enabled_ = enabled; emit controlChanged(enabled_);
}
void LocalController::updateFeedback(bool estopped, bool navActive, bool navKnown, bool fresh) {
  estopped_ = estopped; navActive_ = navActive; navKnown_ = navKnown; fresh_ = fresh; feedbackAge_.restart();
  if (manualMoving_ && (!fresh || estopped || navActive)) emergencyStop();
}
bool LocalController::emergencyStop() {
  repeat_.stop(); preparation_.stop(); manualMoving_ = false; manualReady_ = false; unlockWaiting_ = false;
  emit manualChanged(false); ros_->cancelNavigation(); return ros_->setEmergencyStop(true);
}
bool LocalController::takeManualControl() {
  if (!enabled_) { emit message("请先启用操作"); return false; }
  if (!emergencyStop()) return false;
  preparationAge_.restart(); preparation_.start(); return true;
}
bool LocalController::releaseStop() {
  if (!enabled_ || navActive_) { emit message("请启用操作，并先取消正在执行的导航"); return false; }
  if (!navKnown_) {
    if (!ros_->cancelNavigation()) return false;
    unlockWaiting_ = true; preparationAge_.restart(); preparation_.start();
    emit message("正在确认 Nav2 无活动目标，再解除急停"); return true;
  }
  return ros_->setEmergencyStop(false);
}
bool LocalController::move(double linear, double angular) {
  if (!enabled_ || !manualReady_ || !feedbackFresh() || estopped_ || navActive_ || !navKnown_ ||
      !std::isfinite(linear) || !std::isfinite(angular) || std::abs(linear) < 1e-6) {
    emit message("请先接管手动、解除急停，并确认导航已停止"); return false;
  }
  linear_ = std::clamp(linear, -maxLinear_, maxLinear_);
  angular_ = std::clamp(angular, -maxAngular_, maxAngular_);
  if (!ros_->publishVelocity(linear_, angular_)) return false;
  manualMoving_ = true; repeat_.start(); return true;
}
void LocalController::stopManual() {
  repeat_.stop();
  if (!manualMoving_) return;
  manualMoving_ = false;
  if (feedbackFresh() && !estopped_) ros_->publishVelocity(0., 0.);
  else emergencyStop();
}
bool LocalController::navigate(double x, double y, double yaw) {
  if (!enabled_ || !feedbackFresh() || estopped_ || manualMoving_) {
    emit message("请启用操作、解除急停，并结束手动行驶后再发送目标"); return false;
  }
  manualReady_ = false; emit manualChanged(false); return ros_->navigate(x, y, yaw);
}
bool LocalController::cancelNavigation() { return enabled_ && ros_->cancelNavigation(); }
