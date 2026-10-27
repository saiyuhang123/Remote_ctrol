#include "ui/camera_panel.h"
#include "adapters/ros_adapter.h"
#include "runtime/process_manager.h"
#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QHideEvent>
#include <QLabel>
#include <QPushButton>
#include <QResizeEvent>
#include <QStandardPaths>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

CameraPanel::CameraPanel(const DesktopConfig &config, RosAdapter *ros, bool offline, QWidget *parent)
  : QWidget(parent), config_(config), ros_(ros), process_(new ProcessManager(config, this)),
    ptzProcess_(new ProcessManager(config, this)), offline_(offline) {
  setObjectName("cameraPanel"); setMinimumWidth(250);
  auto *layout = new QVBoxLayout(this);
  preview_ = new QLabel("海康相机未连接\n点击启动相机，或连接已有节点");
  preview_->setObjectName("cameraPreview"); preview_->setMinimumSize(220, 124);
  preview_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  preview_->setAlignment(Qt::AlignCenter); preview_->setWordWrap(true);
  preview_->setStyleSheet("background:#252b33;color:#dce1e8;border-radius:4px;");
  layout->addWidget(preview_, 1);
  status_ = new QLabel; status_->setObjectName("cameraStatus"); status_->setWordWrap(true); layout->addWidget(status_);
  auto *buttons = new QGridLayout;
  auto makeButton = [&](const QString &label, const char *name, int row, int column) {
    auto *b = new QPushButton(label); b->setObjectName(name); buttons->addWidget(b, row, column); return b;
  };
  start_ = makeButton("启动相机", "cameraStart", 0, 0);
  connect_ = makeButton("连接 / 恢复预览", "cameraConnect", 0, 1);
  stopStream_ = makeButton("停止取流", "cameraStopStream", 1, 0);
  stopNode_ = makeButton("停止本界面相机", "cameraStopNode", 1, 1);
  stopNode_->setToolTip("只停止本界面创建的相机进程，不关闭外部启动的节点");
  stopStream_->setToolTip("停止相机节点的取流，使用该节点的其他预览也会停止");
  layout->addLayout(buttons);
  auto *form = new QFormLayout;
  auto number = [this](const char *name, double min, double max, double value) {
    auto *s = new QDoubleSpinBox(this); s->setObjectName(name); s->setRange(min, max);
    s->setDecimals(1); s->setValue(value); s->setKeyboardTracking(false); return s;
  };
  pan_ = number("cameraPan", 0, 360, 0); tilt_ = number("cameraTilt", -180, 360, 0); zoom_ = number("cameraZoom", 1, 999.9, 1);
  form->addRow("水平 (°)", pan_); form->addRow("俯仰 (°)", tilt_); form->addRow("光学变倍 (×)", zoom_);
  tilt_->setToolTip("按设备返回的坐标和范围设置，支持 485 云台的负角度及 SDK 的 0~360° 坐标");
  auto *deviceDetails = new QWidget;
  auto *deviceLayout = new QVBoxLayout(deviceDetails);
  deviceLayout->setContentsMargins(0, 0, 0, 0);
  deviceLayout->addLayout(form);
  auto *directions = new QGridLayout;
  const QStringList labels{"↑ 上", "← 左", "停止云台", "右 →", "↓ 下", "＋ 放大", "－ 缩小"};
  const int commands[] = {21, 23, 0, 24, 22, 11, 12};
  const int rows[] = {0, 1, 1, 1, 2, 3, 3}, columns[] = {1, 0, 1, 2, 1, 0, 2};
  jogTimer_ = new QTimer(this); jogTimer_->setInterval(150);
  poseReadTimer_ = new QTimer(this); poseReadTimer_->setSingleShot(true);
  connect(poseReadTimer_, &QTimer::timeout, this, [this] {
    if (offline_ || jogCommand_ || !ros_->ptzAvailable()) return;
    if (!pending_.isEmpty()) { poseReadTimer_->start(150); return; }
    pending_ = "ptz_read"; ros_->readCameraPose(); refresh();
  });
  connect(jogTimer_, &QTimer::timeout, this, [this] {
    if (!jogCommand_ || jogInFlight_) return;
    jogInFlight_ = true; ros_->jogCamera(jogCommand_, 3, false);
  });
  for (int i = 0; i < labels.size(); ++i) {
    auto *b = new QPushButton(labels[i]); b->setObjectName("cameraDirection" + QString::number(commands[i]));
    b->setProperty("consoleRole", "direction");
    b->setAutoRepeat(false); b->setToolTip("按住转动或变倍，松开停止"); directions->addWidget(b, rows[i], columns[i]);
    directions_ << b;
    const int command = commands[i];
    if (!command) {
      connect(b, &QPushButton::clicked, this, [this] {
        stopJog();
        if (patrolBlocked_) emit patrolStopRequested();
        else if (!offline_) ros_->jogCamera(0, 3, true);
      });
    } else {
      connect(b, &QPushButton::pressed, this, [this, command] {
        if (offline_ || patrolBlocked_ || !controlEnabled_ || !pending_.isEmpty() || !lastFrame_.isValid() || lastFrame_.elapsed() >= 3000) return;
        stopJog(); poseReadTimer_->stop(); poseRead_ = false; poseTargetPending_ = false;
        jogCommand_ = command; jogInFlight_ = true;
        ros_->jogCamera(command, 3, false); jogTimer_->start(); refresh();
      });
      connect(b, &QPushButton::released, this, &CameraPanel::stopJog);
    }
  }
  layout->addWidget(new QLabel("方向控制：按住转动，松开停止")); layout->addLayout(directions);

  poseStatus_ = new QLabel("先读取云台位置，再修改目标；移动云台需启用操作");
  poseStatus_->setWordWrap(true); poseStatus_->setObjectName("cameraPoseStatus"); deviceLayout->addWidget(poseStatus_);
  auto *actions = new QGridLayout;
  read_ = new QPushButton("读取当前云台"); read_->setObjectName("cameraReadPtz"); actions->addWidget(read_, 0, 0);
  set_ = new QPushButton("设置云台与变倍"); set_->setObjectName("cameraSetPtz"); actions->addWidget(set_, 0, 1);
  deviceLayout->addLayout(actions);
  save_ = new QPushButton("保存当前画面"); save_->setObjectName("cameraSave"); layout->addWidget(save_);
  auto *detailsToggle = new QToolButton; detailsToggle->setObjectName("cameraDetailsToggle");
  detailsToggle->setText("云台位置与设备参数"); detailsToggle->setCheckable(true);
  detailsToggle->setArrowType(Qt::RightArrow); detailsToggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  layout->addWidget(detailsToggle); deviceDetails->hide(); layout->addWidget(deviceDetails);
  connect(detailsToggle, &QToolButton::toggled, this, [deviceDetails, detailsToggle](bool yes) {
    deviceDetails->setVisible(yes); detailsToggle->setArrowType(yes ? Qt::DownArrow : Qt::RightArrow);
  });
  connect(start_, &QPushButton::clicked, this, &CameraPanel::startCamera);
  connect(connect_, &QPushButton::clicked, this, &CameraPanel::connectPreview);
  connect(stopNode_, &QPushButton::clicked, this, [this] { stopJog(); if (patrolBlocked_) emit patrolStopRequested(); waitingForNode_ = false; pending_.clear(); process_->stop(); ptzProcess_->stop(); refresh(); });
  connect(stopStream_, &QPushButton::clicked, this, [this] {
    if (offline_ || !pending_.isEmpty()) return;
    stopJog(); if (patrolBlocked_) emit patrolStopRequested(); pending_ = "camera_stop_stream"; ros_->cameraCommand("stop_stream"); refresh();
  });
  connect(read_, &QPushButton::clicked, this, [this] {
    if (offline_ || !pending_.isEmpty()) return;
    poseReadTimer_->stop(); poseRead_ = false;
    pending_ = "ptz_read"; ros_->readCameraPose(); refresh();
  });
  connect(set_, &QPushButton::clicked, this, [this] {
    if (offline_ || patrolBlocked_ || !controlEnabled_ || !poseRead_ || !pending_.isEmpty()) return;
    targetPan_ = pan_->value(); targetTilt_ = tilt_->value(); targetZoom_ = zoom_->value();
    poseTargetPending_ = false;
    pending_ = "ptz"; ros_->setPtz(pan_->value(), tilt_->value(), zoom_->value()); refresh();
  });
  connect(save_, &QPushButton::clicked, this, &CameraPanel::saveFrame);
  connect(process_, &ProcessManager::output, this, [this](const QString &text) { emit message("相机：" + text); });
  connect(ptzProcess_, &ProcessManager::output, this, [this](const QString &text) { emit message("云台：" + text); });
  connect(ptzProcess_, &ProcessManager::runningChanged, this, [this] { refresh(); });
  connect(process_, &ProcessManager::runningChanged, this, [this] { refresh(); });
  connect(process_, &ProcessManager::finished, this, [this](int code) {
    waitingForNode_ = false; poseRead_ = false;
    detail_ = QString("本界面相机进程已退出，退出码 %1").arg(code);
    emit message(detail_); refresh();
  });
  connect(ros_, &RosAdapter::image, this, [this](const QImage &image) {
    if (!previewActive_ || image.isNull()) return;
    frame_ = image; lastFrame_.restart();
    if (!rateWindow_.isValid()) rateWindow_.start();
    ++rateFrames_;
    if (rateWindow_.elapsed() >= 1000) {
      fps_ = rateFrames_ * 1000. / rateWindow_.restart(); rateFrames_ = 0;
    }
    renderFrame(); refresh();
  });
  connect(ros_, &RosAdapter::cameraPose, this, [this](const CameraPose &pose) {
    auto range = [](QDoubleSpinBox *s, double min, double max, double fallbackMin, double fallbackMax) {
      // Wrapped ranges cannot be represented by a single spinbox interval;
      // the driver validates those before issuing a command.
      if (std::isfinite(min) && std::isfinite(max) && min < max &&
          std::max(min, fallbackMin) < std::min(max, fallbackMax))
        s->setRange(std::max(min, fallbackMin), std::min(max, fallbackMax));
      else s->setRange(fallbackMin, fallbackMax);
    };
    if (pose.limitsValid) {
      range(pan_, pose.panMin, pose.panMax, 0, 360);
      range(tilt_, pose.tiltMin, pose.tiltMax, -180, 360);
      range(zoom_, pose.zoomMin, pose.zoomMax, 1, 999.9);
    } else {
      pan_->setRange(0, 360); tilt_->setRange(-180, 360); zoom_->setRange(1, 999.9);
    }
    pan_->setValue(pose.pan); tilt_->setValue(pose.tilt); zoom_->setValue(pose.zoom);
    absolutePanTilt_ = !(pose.limitsValid && pose.panMin == 0 && pose.panMax == 0 && pose.tiltMin == 0 && pose.tiltMax == 0);
    pan_->setEnabled(absolutePanTilt_); tilt_->setEnabled(absolutePanTilt_);
    set_->setText(absolutePanTilt_ ? "设置云台与变倍" : "设置光学变倍");
    poseRead_ = true;
    poseStatus_->setText(QString("设备上报：水平 %1°，俯仰 %2°，变倍 %3×").arg(pose.pan, 0, 'f', 1).arg(pose.tilt, 0, 'f', 1).arg(pose.zoom, 0, 'f', 1));
    if (!absolutePanTilt_) poseStatus_->setText(QString("光学变倍 %1×（最大 %2×）；水平/俯仰用方向按钮控制，角度反馈不可用").arg(pose.zoom, 0, 'f', 1).arg(zoom_->maximum(), 0, 'f', 1));
    if (poseTargetPending_) {
      const bool arrived = std::abs(pose.zoom - targetZoom_) <= .15 &&
          (!absolutePanTilt_ || (std::abs(std::remainder(pose.pan - targetPan_, 360.)) <= .3 &&
                                 std::abs(pose.tilt - targetTilt_) <= .3));
      if (arrived) poseTargetPending_ = false;
      else if (poseSettle_.elapsed() < 15000) {
        poseStatus_->setText(poseStatus_->text() + "\n设备尚未到位，继续读取实际位置");
        schedulePoseRead();
      } else {
        poseTargetPending_ = false;
        poseStatus_->setText(poseStatus_->text() + "\n15 秒内未确认到位，请检查目标与设备状态");
      }
    }
    refresh();
  });
  connect(ros_, &RosAdapter::requestFinished, this, [this](const QString &op, bool ok, const QString &detail) {
    if (op == "ptz_jog") {
      jogInFlight_ = false;
      if (!ok) { stopJog(); detail_ = "方向控制失败：" + detail; refresh(); }
      return;
    }
    if (op == "ptz_stop") {
      if (!ok) { detail_ = "云台停止失败：" + detail; refresh(); }
      return;
    }
    if (op != pending_ || pending_.isEmpty()) return;
    pending_.clear(); detail_ = (ok ? "" : "失败：") + detail;
    if (op == "camera_login" && ok) {
      pending_ = "camera_start_stream"; ros_->cameraCommand("start_stream");
    } else if (op == "camera_stop_stream" && ok) {
      frame_ = {}; lastFrame_.invalidate(); preview_->clear(); preview_->setText("取流已停止");
    } else if (op == "ptz" && ok) {
      poseRead_ = false;
      poseTargetPending_ = true; poseSettle_.start();
      poseStatus_->setText("目标已接受，等待设备到位后自动读取实际位置");
      schedulePoseRead();
    } else if (op == "ptz_read" && !ok) {
      poseRead_ = false;
      poseStatus_->setText("读取失败：" + detail);
      if (poseTargetPending_ && poseSettle_.elapsed() < 15000) schedulePoseRead();
      else poseTargetPending_ = false;
    }
    refresh();
  });
  auto *timer = new QTimer(this); timer->setInterval(500);
  connect(timer, &QTimer::timeout, this, [this] {
    syncPreviewSubscription();
    if (waitingForNode_) {
      if (ros_->cameraAvailable()) { waitingForNode_ = false; detail_ = "相机节点就绪，等待视频帧"; }
      else if (startup_.elapsed() > 25000) {
        waitingForNode_ = false; detail_ = "启动超过 25 秒，尚未发现相机服务，请检查日志"; emit message(detail_);
      }
    }
    if (!ros_->cameraNodePresent() || !ros_->ptzAvailable()) { poseRead_ = false; stopJog(); }
    if (lastFrame_.isValid() && lastFrame_.elapsed() > 3000) { stopJog(); preview_->clear(); preview_->setText("视频数据已过期\n可点击连接 / 恢复预览"); }
    refresh();
  }); timer->start(); qApp->installEventFilter(this); refresh(); syncPreviewSubscription();
}

CameraPanel::~CameraPanel() {
  qApp->removeEventFilter(this); stopJog();
  previewActive_ = false; ros_->setCameraPreviewActive(false);
  QObject::disconnect(process_, nullptr, this, nullptr);
  delete process_;
  QObject::disconnect(ptzProcess_, nullptr, this, nullptr);
  delete ptzProcess_;
}
void CameraPanel::setControlEnabled(bool enabled) { controlEnabled_ = enabled; if (!enabled) stopJog(); refresh(); }
void CameraPanel::setPatrolBlocked(bool blocked) { patrolBlocked_ = blocked; if (blocked) stopJog(); refresh(); }
void CameraPanel::ensureCamera() {
  if (offline_) return;
  if (!ros_->cameraNodePresent() || ros_->cameraNodeCount() > 1) startCamera();
  else {
    ensurePtz();
    if (pending_.isEmpty() && (!lastFrame_.isValid() || lastFrame_.elapsed() >= 3000)) connectPreview();
  }
}
void CameraPanel::stopInteraction() { stopJog(); }
bool CameraPanel::prepareRemote(QString *detail) {
  if (offline_) { *detail = "离线模式不能启动相机或云台"; return false; }
  if (!ros_->cameraNodePresent() && !QFileInfo::exists(config_.cameraParamsFile)) {
    *detail = "本地相机参数文件不存在，请在 Qt 核对连接参数"; return false;
  }
  if (!ros_->ptzNodePresent() && !QFileInfo::exists(config_.ptzParamsFile)) {
    *detail = "本地 RS485 云台参数文件不存在，请在 Qt 核对连接参数"; return false;
  }
  ensureCamera();
  *detail = "本地已受理相机与云台启动请求，复用已有节点；等待 RS485 云台和网口镜头服务就绪";
  return true;
}
void CameraPanel::showEvent(QShowEvent *event) {
  QWidget::showEvent(event); syncPreviewSubscription();
}
void CameraPanel::hideEvent(QHideEvent *event) {
  stopJog(); QWidget::hideEvent(event); syncPreviewSubscription();
}
void CameraPanel::syncPreviewSubscription() {
  const bool active = !offline_ && isVisible();
  if (active == previewActive_) return;
  previewActive_ = active;
  ros_->setCameraPreviewActive(active);
  frame_ = {}; lastFrame_.invalidate(); rateWindow_.invalidate();
  rateFrames_ = 0; fps_ = 0;
  preview_->clear(); preview_->setText(active ? "等待实时画面…" : "预览已暂停");
  refresh();
}
bool CameraPanel::eventFilter(QObject *object, QEvent *event) {
  if (event->type() == QEvent::WindowDeactivate || event->type() == QEvent::ApplicationDeactivate) stopJog();
  return QWidget::eventFilter(object, event);
}
void CameraPanel::stopJog() {
  jogTimer_->stop();
  if (jogCommand_) {
    jogCommand_ = 0; ros_->jogCamera(0, 3, true); schedulePoseRead(); refresh();
  }
}
void CameraPanel::schedulePoseRead() { if (!offline_) poseReadTimer_->start(1000); }
void CameraPanel::refresh() {
  const bool available = ros_->cameraAvailable(), present = ros_->cameraNodePresent();
  const int instances = ros_->cameraNodeCount();
  const bool ptzAvailable = ros_->ptzAvailable();
  const bool busy = !pending_.isEmpty() || waitingForNode_ || jogCommand_;
  const bool fresh = lastFrame_.isValid() && lastFrame_.elapsed() < 3000;
  start_->setEnabled(!offline_ && !process_->running() && (!present || instances > 1) && !busy);
  start_->setText(instances > 1 ? "重启为单实例" : "启动相机");
  connect_->setEnabled(!offline_ && available && !busy);
  stopStream_->setEnabled(!offline_ && available && !busy);
  stopNode_->setEnabled(!offline_ && process_->running());
  read_->setEnabled(!offline_ && ptzAvailable && !busy);
  set_->setEnabled(!offline_ && !patrolBlocked_ && controlEnabled_ && poseRead_ && ptzAvailable && !busy);
  save_->setEnabled(!offline_ && fresh && !jogCommand_);
  for (auto *b : directions_) {
    const bool stop = b->objectName() == "cameraDirection0";
    b->setEnabled(!offline_ && ptzAvailable && (stop || (!patrolBlocked_ && controlEnabled_ && fresh && (!busy || jogCommand_))));
  }
  const auto provider = process_->running() ? "本界面管理" : instances > 1 ? QString("外部节点×%1").arg(instances) : present ? "外部节点" : "未启动";
  QString text = QString("相机：%1 · %2").arg(provider, available ? "服务就绪" : "服务未就绪");
  if (fresh) text += QString("\n%1×%2 · 预览 %3 fps").arg(frame_.width()).arg(frame_.height()).arg(fps_, 0, 'f', 1);
  else text += "\n尚无实时画面";
  if (busy) text += waitingForNode_ ? "\n正在启动相机…" : "\n请求处理中…";
  else if (!detail_.isEmpty()) text += "\n" + detail_;
  if (offline_) text += "\n离线验证模式";
  else if (available && !ptzAvailable) text += "\n云台服务未就绪，请连接 / 恢复预览并检查云台日志";
  if (patrolBlocked_) text += "\n云台巡航活动或待停止确认；手动云台已锁定";
  status_->setText(text);
}
void CameraPanel::startCamera() {
  if (offline_ || process_->running() || !pending_.isEmpty()) return;
  const int instances = ros_->cameraNodeCount();
  if (ros_->cameraNodePresent() && instances <= 1) { emit message("已发现单个相机节点，直接复用并连接预览"); refresh(); return; }
  if (instances > 1) emit message(QString("检测到 %1 个海康节点，将结束旧实例并重启为单实例").arg(instances));
  if (!QFileInfo::exists(config_.cameraParamsFile)) { emit message("相机参数文件不存在：" + config_.cameraParamsFile); return; }
  // Reuse the driver's private parameter file; credentials never enter desktop
  // JSON, command line, or a gateway. Monitoring does not enable face capture.
  waitingForNode_ = true; startup_.start(); poseRead_ = false;
  if (!process_->start({"ros2", "run", "hk_camera", "hk_camera_node", "--ros-args",
      "--params-file", config_.cameraParamsFile, "-p", "auto_login:=true", "-p", "auto_stream:=true",
      "-p", "enable_face_capture:=false"})) waitingForNode_ = false;
  ensurePtz();
  refresh();
}
void CameraPanel::ensurePtz() {
  if (offline_ || ptzProcess_->running() || ros_->ptzNodePresent()) return;
  if (!QFileInfo::exists(config_.ptzParamsFile)) { emit message("云台参数文件不存在：" + config_.ptzParamsFile); return; }
  // Public PTZ services combine RS485 angles with the camera's optical lens.
  if (!ptzProcess_->running())
    ptzProcess_->start({"ros2", "run", "ptz_pelco", "ptz_pelco_node", "--ros-args",
        "--params-file", config_.ptzParamsFile});
}
void CameraPanel::connectPreview() {
  if (offline_ || !pending_.isEmpty()) return;
  ensurePtz();
  pending_ = "camera_login"; ros_->cameraCommand("login"); refresh();
}
void CameraPanel::renderFrame() {
  if (!previewActive_ || !isVisible()) return;
  if (!frame_.isNull() && lastFrame_.isValid() && lastFrame_.elapsed() < 3000)
    preview_->setPixmap(QPixmap::fromImage(frame_).scaled(preview_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
void CameraPanel::resizeEvent(QResizeEvent *event) { QWidget::resizeEvent(event); renderFrame(); }
void CameraPanel::saveFrame() {
  if (offline_ || frame_.isNull() || !lastFrame_.isValid() || lastFrame_.elapsed() >= 3000) return;
  const QImage snapshot = frame_;
  const auto directory = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation) + "/robot_camera";
  QDir().mkpath(directory);
  const auto path = QFileDialog::getSaveFileName(this, "保存当前相机画面",
      directory + "/" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz") + ".png", "图像 (*.png *.jpg)");
  if (!path.isEmpty()) emit message(snapshot.save(path) ? "相机画面已保存：" + path : "相机画面保存失败：" + path);
}
