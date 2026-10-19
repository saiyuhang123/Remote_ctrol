#include "ui/mainwindow.h"
#include "ui/camera_panel.h"
#include "devices/device_client.h"
#include "devices/device_panel.h"
#include "vision/vision_panel.h"
#include "ui/console_theme.h"
#include "ui/map_editor_dialog.h"
#include "patrol/patrol_client.h"
#include "patrol/patrol_page.h"
#include "ui_mainwindow.h"
#include "adapters/local_controller.h"
#include "runtime/map_saver.h"
#include "runtime/process_manager.h"
#include "rviz/rviz_host.h"
#include "ui/widgets/dashboard.h"
#include "DockAreaWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QElapsedTimer>
#include <QDesktopServices>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSignalBlocker>
#include <QScreen>
#include <QScrollArea>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <cmath>

namespace {
constexpr double pi = 3.14159265358979323846;
QString pathIdentity(const QString &path) {
  if (path.trimmed().isEmpty()) return {};
  const QFileInfo info(path);
  const QString canonical = info.canonicalFilePath();
  return canonical.isEmpty() ? info.absoluteFilePath() : canonical;
}

}

MainWindow::MainWindow(const DesktopConfig &config, bool offline, QWidget *parent)
  : QMainWindow(parent), config_(config), offline_(offline) {
  Q_INIT_RESOURCE(images); Q_INIT_RESOURCE(media);
  ros_ = new RosAdapter(config, this);
  controller_ = new LocalController(ros_, config.maxLinear, config.maxAngular, config.feedbackTimeoutMs, this);
  process_ = new ProcessManager(config, this);
  saver_ = new MapSaver(config, ros_, this);
  patrolClient_ = new PatrolClient(this);
  connect(patrolClient_, &PatrolClient::mapsChanged, this, [this] {
    const QString mapId = patrolClient_->activeMapId();
    if (!mapId.isEmpty()) patrolClient_->refreshHomePoint(mapId);
    updateControls();
  });
  connect(patrolClient_, &PatrolClient::homePointChanged, this, &MainWindow::updateControls);
  closeSafetyTimer_ = new QTimer(this);
  closeSafetyTimer_->setInterval(200);
  connect(closeSafetyTimer_, &QTimer::timeout, this, &MainWindow::pollCloseAfterSafetyStop);
  setupUi();
  patrolClient_->refreshMaps();
  connect(controller_, &LocalController::message, this, &MainWindow::log);
  connect(ros_, &RosAdapter::message, this, &MainWindow::log);
  connect(saver_, &MapSaver::message, this, &MainWindow::log);
  connect(saver_, &MapSaver::saved, this, [this](bool ok, const QString &text) {
    const auto message = (ok ? "保存成功：" : "保存未完成：") + text;
    log(message); showOperationResult("save_map", ok, message);
  });
  connect(saver_, &MapSaver::busyChanged, this, [this] { updateControls(); });
  connect(process_, &ProcessManager::output, this, &MainWindow::log);
  connect(process_, &ProcessManager::runningChanged, this, [this](bool running) {
    if (!running) sessionPatrolInterlock_ = false;
    updateControls();
  });
  connect(process_, &ProcessManager::finished, this, [this](int code) {
    sessionPatrolInterlock_ = false;
    log("本界面启动的会话已退出，退出码：" + QString::number(code));
    updateControls();
  });
  connect(controller_, &LocalController::controlChanged, this, [this](bool yes) { QSignalBlocker block(enableControl_); enableControl_->setChecked(yes); updateControls(); });
  connect(controller_, &LocalController::manualChanged, this, [this](bool ready) { take_->setText(ready ? "手动已接管" : "手动接管（先取消导航）"); updateControls(); });
  connect(patrolClient_, &PatrolClient::statusChanged, this, [this] {
    syncPatrolState();
    updateControls();
  });
  connect(ros_, &RosAdapter::navigationEvent, this, [this](const QString &text) { lastNavigationEvent_ = text; refreshStatusSummary(); log(text); });
  connect(ros_, &RosAdapter::snapshot, this, &MainWindow::updateSnapshot);
  connect(ros_, &RosAdapter::goalRequested, this, &MainWindow::sendGoal);
  connect(ros_, &RosAdapter::initialPoseRequested, this, &MainWindow::sendInitialPose);
  connect(ros_, &RosAdapter::capturedImage, this, [this](const QImage &image) { capturedPreview_ = image; picture_->setPixmap(QPixmap::fromImage(image).scaled(picture_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)); });
  connect(ros_, &RosAdapter::ocrResult, ocr_, &QLabel::setText);
  connect(ros_, &RosAdapter::requestFinished, this, [this](const QString &op, bool ok, const QString &detail) {
    if (op != "save_state" && !(ok && (op == "ptz_jog" || op == "ptz_stop"))) {
      log(op + (ok ? " 成功：" : " 失败：") + detail);
      showOperationResult(op, ok, detail);
    }
  });
  connect(qApp, &QGuiApplication::applicationStateChanged, this, [this](Qt::ApplicationState s) { if (s != Qt::ApplicationActive && controller_) controller_->stopManual(); });
  log(offline_ ? "离线验证模式：控制、业务写入和进程启动均禁用" : "观察模式。操作前请启用操作；网页与桌面端请勿同时操控");
  applyProfile();
  updateControls();
}
MainWindow::~MainWindow() {
  saveConsoleLayout();
  layoutReady_ = false;
  if (patrolWindow_) {
    delete patrolWindow_;
    patrolWindow_ = nullptr;
    patrolPage_ = nullptr;
  }
  // Children are destroyed by QObject after MainWindow's own members have
  // already been destroyed. Tear the editor down while `this` is still a
  // complete object, and disconnect its destroyed callback first.
  if (mapEditor_) {
    QObject::disconnect(mapEditor_, nullptr, this, nullptr);
    delete mapEditor_;
    mapEditor_ = nullptr;
  }
  controller_->stopManual();
  if (process_->running()) controller_->emergencyStop();
  QObject::disconnect(process_, nullptr, this, nullptr);
  delete process_;
  process_ = nullptr;
  delete visionPanel_;
  visionPanel_ = nullptr;
  delete deviceWindow_;
  deviceWindow_ = nullptr; devicePanel_ = nullptr;
  delete deviceClient_;
  deviceClient_ = nullptr;
  delete cameraPanel_;
  cameraPanel_ = nullptr;
  delete controller_;
  controller_ = nullptr;
  delete rviz_;
  rviz_ = nullptr;
  delete ros_;
  ros_ = nullptr;
  delete ui_;
}
const Profile &MainWindow::profile() const { return desktopProfiles().at(profiles_->currentIndex()); }
bool MainWindow::allowed() const { return !offline_ && enableControl_->isChecked(); }
void MainWindow::setProfileIndex(int index) { profiles_->setCurrentIndex(index); }
void MainWindow::applyProfile() {
  if (process_->running() || saver_->busy()) { log("请先结束当前会话后再切换模式"); return; }
  const int profileIndex = profiles_->currentIndex();
  const bool profileChanged = profileIndex != lastProfileIndex_;
  controller_->stopManual();
  if (rviz_ && !rviz_->loadProfile(config_.resourceDir + "/config/rviz/" + profile().rvizFile)) log("RViz 显示配置不存在");
  ros_->setFixedFrame(profile().fixedFrame);
  mapPrefix_->setText(config_.projectRoot + "/maps/" + profile().backend + "/map");
  if (profileChanged) navigationYaml_->clear();
  lastProfileIndex_ = profileIndex;
  if (mapEditor_) refreshEditorNavigationFeedback();
  state_.poseValid = false;
  log("已加载显示：" + profile().name + "；此操作不切换后台算法"); updateControls();
}
void MainWindow::updateControls() {
  const bool ok = allowed(); const bool running = process_->running(); const bool busy = saver_->busy();
  enableControl_->setEnabled(!offline_);
  cameraPanel_->setControlEnabled(ok);
  devicePanel_->setControlEnabled(ok);
  deviceClient_->setControlEnabled(ok);
  const bool motion = ok && controller_->controlEnabled();
  const bool patrolRunning = patrolClient_ && patrolClient_->hasActiveOrPendingRun();
  for (auto *b : writeButtons_) { const auto cap = b->property("capability").toString(); b->setEnabled(ok && (!b->property("motion").toBool() || motion) && (cap.isEmpty() || ros_->capabilities().contains(cap))); }
  take_->setEnabled(motion); unlock_->setEnabled(motion && !state_.navigationActive && !patrolRunning && (!state_.estopValid || state_.estopped));
  const bool manual = motion && controller_->manualReady() && !patrolRunning && state_.estopValid && !state_.estopped && state_.navigationKnown && !state_.navigationActive;
  for (auto *b : directionButtons_) b->setEnabled(manual);
  sendGoalButton_->setEnabled(motion && profile().navigation && state_.poseValid && state_.frame == "map" &&
                              state_.navigationAvailable && state_.estopValid && !state_.estopped &&
                              !patrolRunning && !sessionPatrolInterlock_ && !state_.patrolInterlockValid);
  if (returnHomeButton_) {
    const auto home = patrolClient_ ? patrolClient_->homePoint() : PatrolHomeInfo();
    const QString activeMap = patrolClient_ ? patrolClient_->activeMapId() : QString();
    const auto patrol = patrolClient_ ? patrolClient_->status() : PatrolStatusInfo();
    const bool returnRunning = patrol.active && patrol.runId.startsWith(QStringLiteral("return_"));
    returnHomeButton_->setEnabled(
        !offline_ && allowed() && profile().navigation && !home.mapId.isEmpty() &&
        home.exists && home.mapId == activeMap && !returnRunning &&
        state_.poseValid && state_.frame == QStringLiteral("map") &&
        state_.navigationAvailable && state_.estopValid && !state_.estopped &&
        state_.guardValid && state_.guardReady && !(controller_ && controller_->manualReady()));
  }
  manage_->setEnabled(!offline_ && !running && !busy);
  start_->setEnabled(!offline_ && manage_->isChecked() && !running && !busy);
  stop_->setEnabled(!offline_ && running && !busy);
  save_->setEnabled(ok && profile().mapping && !busy);
  profiles_->setEnabled(!running && !busy); mapPrefix_->setReadOnly(running || busy);
  if (navigationYaml_) navigationYaml_->setReadOnly(running || busy);
  if (mapEditorAction_) mapEditorAction_->setEnabled(!busy);
  if (patrolAction_) patrolAction_->setEnabled(patrolClient_ && patrolClient_->available());
  sensors_->setEnabled(!running); chassis_->setEnabled(!running); bridge_->setEnabled(!running);
  refreshOdometrySource();
  if (patrolInterlock_) patrolInterlock_->setEnabled(!offline_ && !running && profile().navigation && !profile().mapping);
  refreshStatusSummary();
}
void MainWindow::syncPatrolState() {
  if (!controller_) return;
  controller_->setPatrolActive(patrolRunActive());
  refreshStatusSummary();
}

bool MainWindow::cancelPatrolRun(const QString &reason) {
  if (!patrolClient_ || !patrolClient_->available()) return false;
  if (!patrolClient_->hasActiveOrPendingRun()) return false;
  const auto status = patrolClient_->status();
  if (status.runId.startsWith(QStringLiteral("return_"))) {
    patrolClient_->cancelReturnHome();
  } else {
    patrolClient_->cancelRunRoute();
    if (!status.runId.isEmpty()) patrolClient_->controlRun(3, status.runId);
  }
  log(reason + "：已向本地任务发送取消请求，等待停车确认");
  return true;
}

void MainWindow::returnHome() {
  if (!patrolClient_ || !patrolClient_->available()) {
    log(QStringLiteral("一键返回不可用：本地巡航接口未构建")); return;
  }
  if (offline_ || !allowed()) { log(QStringLiteral("一键返回：请先启用操作")); return; }
  if (!profile().navigation) { log(QStringLiteral("一键返回：请先切换到定位导航模式")); return; }
  if (controller_ && controller_->manualReady()) {
    log(QStringLiteral("一键返回：请先结束手动接管并确认车辆停稳")); return;
  }
  if (!state_.poseValid || state_.frame != QStringLiteral("map") ||
      !state_.navigationAvailable || !state_.estopValid || state_.estopped ||
      !state_.guardValid || !state_.guardReady) {
    log(QStringLiteral("一键返回：定位、Nav2、急停或巡航联锁未就绪")); return;
  }
  const QString mapId = patrolClient_->activeMapId();
  const auto home = patrolClient_->homePoint();
  if (mapId.isEmpty() || !home.exists || home.mapId != mapId) {
    log(QStringLiteral("一键返回：当前活跃地图没有有效的待命返航点")); return;
  }
  quint64 mapRevision = 0;
  for (const auto &item : patrolClient_->maps())
    if (item.mapId == mapId) mapRevision = item.revision;
  if (!mapRevision) { log(QStringLiteral("一键返回：活跃地图版本未知，请刷新地图")); return; }
  const auto status = patrolClient_->status();
  if (status.active && status.runId.startsWith(QStringLiteral("return_"))) {
    log(QStringLiteral("一键返回：返航任务已在执行")); return;
  }
  patrolClient_->returnHome(mapId, mapRevision);
  log(QStringLiteral("一键返回：已请求返回当前地图待命点"));
}

void MainWindow::updateSnapshot(const RobotSnapshot &s) {
  state_ = s;
  controller_->setMotionState(s.speedValid, s.speed, s.yawRate);
  controller_->updateFeedback(s.estopped, s.navigationActive, s.navigationKnown, s.estopValid);
  if (!s.navigationAvailable) lastNavigationEvent_.clear();
  battery_->setValue(s.batteryValid ? qRound(s.battery) : 0);
  battery_->setFormat(s.batteryValid ? "%p%" : "—");
  power_->setText(s.batteryValid ? QString::number(s.voltage, 'f', 1) + " V" : "— V");
  battery_->setToolTip(s.batteryValid ? "真实底盘电池反馈" : "电池反馈未接入或已过期");
  updateControls();
}
void MainWindow::sendGoal(double x, double y, double yaw) {
  if (sessionPatrolInterlock_ || state_.patrolInterlockValid) {
    log("目标未发送：当前后端启用了路线巡航联锁，单点 Nav2 目标会绕过巡航执行器所有权；请使用路线巡航，或关闭联锁重启后再发送");
    return;
  }
  if (patrolClient_ && patrolClient_->hasActiveOrPendingRun()) {
    log("目标未发送：本地巡航任务或启动请求仍活动，请先取消整条巡航任务");
    return;
  }
  if (!offline_ && profile().navigation && ros_->previewGoal(x, y, yaw)) {
    goalX_->setValue(x); goalY_->setValue(y); goalYaw_->setValue(yaw*180./pi);
  }
  if (!allowed() || !controller_->controlEnabled() || !profile().navigation || !state_.poseValid || state_.frame != "map") { log("目标未发送：请启用操作、选择导航模式并等待 map 定位有效"); return; }
  if (controller_->navigate(x, y, yaw)) log(QString("导航请求已发送：(%1, %2), %3°").arg(x).arg(y).arg(yaw*180./pi));
}
void MainWindow::sendInitialPose(double x, double y, double yaw) {
  if (patrolClient_ && patrolClient_->hasActiveOrPendingRun()) {
    log("初始位姿未发送：请先取消本地巡航任务");
    return;
  }
  if (!allowed()) { log("初始位姿未发送：请勾选主窗口顶部的“启用操作”"); return; }
  if (!profile().navigation) { log("初始位姿未发送：当前界面为“" + profile().name + "”，请切换到定位导航模式"); return; }
  if (profile().backend == "cartographer") { log("当前 Cartographer 纯定位入口不支持 /initialpose，请按原流程在建图起点启动"); return; }
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) { log("初始位姿未发送：坐标或朝向无效"); return; }
  if (state_.navigationActive) { log("初始位姿未发送：仍有活动导航目标，请先取消导航"); return; }
  // Initial localization only publishes a pose. It must not depend on odometry
  // speed, stationary windows, map TF, or Nav2 having finished activation.
  if (ros_->setInitialPose(x, y, yaw))
    log("初始位姿已发布到 /initialpose，等待定位结果；此操作不发送导航目标");
}
void MainWindow::startSession() {
  if (offline_ || !manage_->isChecked() || process_->running()) return;
  const auto prefix = mapPrefix_->text().trimmed();
  if (!QFileInfo(prefix).isAbsolute() || prefix.isEmpty()) { log("地图路径需要为绝对路径，且不带扩展名"); return; }
  const QString navigationYaml = navigationYaml_->text().trimmed();
  if (!navigationYaml.isEmpty()) {
    const QFileInfo file(navigationYaml);
    if (!file.isAbsolute() || !file.isFile()) { log("导航栅格 YAML 需要是存在的绝对路径文件：" + navigationYaml); return; }
  }
  if (profile().backend == "fastlio" && profile().navigation) {
    const QFileInfo pcd(prefix + ".pcd");
    if (!pcd.isFile() || pcd.size() == 0) { log("无法启动 FAST-LIO 定位导航：PCD 地图不存在或为空：" + pcd.filePath() + "；请先完成建图保存，或选择配套的有效地图"); return; }
  }
  const auto conflicts = ros_->conflictingNodes();
  if (!conflicts.isEmpty()) { log("已有机器人管线，请使用连接模式；如需重启，先从原启动入口停止：" + conflicts.join(", ")); return; }
  const bool enablePatrol = patrolInterlock_ && patrolInterlock_->isChecked();
  const QStringList args = ProcessManager::launchArguments(config_, profile(), prefix, sensors_->isChecked(), chassis_->isChecked(), bridge_->isChecked(), odom_->currentText(), navigationYaml, enablePatrol);
  log("启动本机会话：" + args.join(' '));
  if (enablePatrol) log("已启用路线巡航联锁：本次 launch 将同时启动 patrol_node、patrol_guard 和巡逻 mux 锁；单点 Nav2 目标将被禁用");
  const bool started = process_->start(args);
  if (started) sessionPatrolInterlock_ = enablePatrol;
  updateControls();
  if (profile().backend == "fastlio" && profile().navigation) log("FAST-LIO 定位需先点选车辆当前位置与朝向；ICP 成功后才会建立 map TF，Nav2 随后继续激活");
  if (profile().id == "slam_navigation") log("AMCL 已启动：请启用操作并点选车辆实际位置与朝向；收到激光后建立 map TF，Nav2 随后继续激活");
  log("车辆操作使用本地 ROS2；首次行驶前需明确点击解除急停");
}
bool MainWindow::mapEditorSourceMatchesCurrent() const {
  return !mapEditorProfileId_.isEmpty() &&
         profile().id == mapEditorProfileId_ &&
         mapPrefix_->text().trimmed() == mapEditorSourcePrefix_;
}

void MainWindow::refreshEditorNavigationFeedback() {
  if (!mapEditor_ || !navigationYaml_ || !mapPrefix_) return;
  mapEditor_->updateNavigationSelection(navigationYaml_->text().trimmed(),
                                        mapEditorSourceMatchesCurrent());
}

void MainWindow::openMapEditor() {
  if (mapEditor_) {
    mapEditor_->raise();
    mapEditor_->activateWindow();
    return;
  }
  QString initial = navigationYaml_->text().trimmed();
  if (initial.isEmpty()) initial = mapPrefix_->text().trimmed() + ".yaml";
  auto *editor = new MapEditorDialog(initial, this);
  editor->setAttribute(Qt::WA_DeleteOnClose);
  mapEditor_ = editor;
  mapEditorProfileId_ = profile().id;
  mapEditorSourcePrefix_ = mapPrefix_->text().trimmed();
  mapEditorInitialSourceYaml_ = initial;
  refreshEditorNavigationFeedback();
  connect(editor, &MapEditorDialog::mapSaved, this, [this](const QString &yamlPath, const QString &sourceYamlPath) {
    const bool sameDocumentSource = !mapEditorInitialSourceYaml_.trimmed().isEmpty() &&
                                     !sourceYamlPath.trimmed().isEmpty() &&
                                     pathIdentity(sourceYamlPath) == pathIdentity(mapEditorInitialSourceYaml_);
    const bool sameSource = sameDocumentSource &&
                            !mapEditorProfileId_.isEmpty() &&
                            profile().id == mapEditorProfileId_ &&
                            mapPrefix_->text().trimmed() == mapEditorSourcePrefix_;
    if (!sameSource) {
      refreshEditorNavigationFeedback();
      log(QStringLiteral("地图编辑来源模式或原地图前缀已变化，未自动填入导航栅格 YAML；"
                         "请重新确认编辑图与当前定位文件对应后再手动选择：%1（来源：%2）")
              .arg(yamlPath, sourceYamlPath));
      return;
    }
    navigationYaml_->setText(yamlPath);
    refreshEditorNavigationFeedback();
    log("地图编辑已保存：" + yamlPath + "（来源：" + sourceYamlPath + "）；已填入导航栅格 YAML，下次启动导航使用");
  });
  connect(editor, &QObject::destroyed, this, [this] {
    mapEditor_ = nullptr;
    mapEditorProfileId_.clear();
    mapEditorSourcePrefix_.clear();
    mapEditorInitialSourceYaml_.clear();
  });
  editor->show();
  editor->raise();
  editor->activateWindow();
}

void MainWindow::openPatrolPage() {
  if (patrolWindow_) {
    patrolWindow_->raise();
    patrolWindow_->activateWindow();
    return;
  }
  if (!patrolClient_ || !patrolClient_->available()) {
    log("本地路线巡航不可用：请先构建 patrol_interfaces 和 robot_patrol");
    return;
  }
  QString navYaml = navigationYaml_->text().trimmed();
  const QString prefix = mapPrefix_->text().trimmed();
  if (navYaml.isEmpty()) navYaml = prefix + ".yaml";
  QString localizationFile;
  if (profile().backend == "fastlio") localizationFile = prefix + ".pcd";
  else if (profile().backend == "cartographer") localizationFile = prefix + ".pbstream";
  auto *dialog = new QDialog(this);
  dialog->setObjectName("patrolWindow");
  dialog->setWindowTitle("路线巡航");
  dialog->setAttribute(Qt::WA_DeleteOnClose);
  const QSize available = screen() ? screen()->availableGeometry().size() : QSize(1280, 840);
  dialog->resize(qMin(1180, available.width() - 40), qMin(760, available.height() - 40));
  auto *layout = new QVBoxLayout(dialog);
  layout->setContentsMargins(0, 0, 0, 0);
  auto basePermission = [this](QString *reason) {
    QStringList blockers;
    if (offline_) blockers << QStringLiteral("当前为离线模式，不能执行巡航");
    if (!profile().navigation) blockers << QStringLiteral("当前不是定位导航模式，请先切换模式并启动对应导航");
    if (!allowed()) blockers << QStringLiteral("尚未启用操作，请勾选主窗口顶部“启用操作”");
    if (controller_ && controller_->manualReady())
      blockers << QStringLiteral("仍在手动接管，请停稳后关闭再打开“启用操作”，退出手动就绪");
    if (!state_.poseValid || state_.frame != QStringLiteral("map"))
      blockers << QStringLiteral("尚无有效的 map 定位，请先完成定位并确认车辆位置");
    if (!state_.navigationAvailable) blockers << QStringLiteral("Nav2 尚未就绪，请等待导航启动完成或检查日志");
    if (!state_.estopValid)
      blockers << QStringLiteral("急停反馈缺失或过期，请在主窗口“手动”页明确解除急停并检查反馈");
    else if (state_.estopped)
      blockers << QStringLiteral("急停保持中，请在主窗口“手动”页点击“解除急停（保持静止）”");
    if (!state_.guardValid)
      blockers << QStringLiteral("巡航联锁心跳缺失或过期，请检查路线巡航联锁是否已启用及后台是否正常");
    else if (!state_.guardReady)
      blockers << QStringLiteral("巡航联锁尚未就绪，请检查心跳或按故障原因复位联锁");
    if (reason) *reason = blockers.join('\n');
    return blockers.isEmpty();
  };
  auto startPermission = [this, basePermission](QString *reason) {
    if (!basePermission(reason)) return false;
    if (patrolClient_ && patrolClient_->hasActiveOrPendingRun()) {
      if (reason) *reason = QStringLiteral("已有巡航任务或启动请求正在处理");
      return false;
    }
    return true;
  };
  auto resumePermission = [basePermission](QString *reason) {
    // Resuming an already-paused run must not be rejected merely because
    // that same run is still active and owns the single execution slot.
    return basePermission(reason);
  };
  patrolPage_ = new PatrolPage(patrolClient_, navYaml, profile().backend,
                               localizationFile, profile().name, offline_, startPermission, dialog);
  patrolPage_->setResumePermission(resumePermission);
  layout->addWidget(patrolPage_);
  patrolWindow_ = dialog;
  connect(dialog, &QObject::destroyed, this, [this] {
    patrolWindow_ = nullptr;
    patrolPage_ = nullptr;
  });
  dialog->show();
  dialog->raise();
  dialog->activateWindow();
}

void MainWindow::log(const QString &text) {
  if (logs_) logs_->appendPlainText(QDateTime::currentDateTime().toString("HH:mm:ss ") + text.trimmed());
  if (messageSummary_ && operationErrorKey_.isEmpty()) {
    messageSummary_->setText(text.trimmed()); messageSummary_->setToolTip(text.trimmed());
  }
}
bool MainWindow::patrolRunActive() const {
  return patrolClient_ && patrolClient_->hasActiveOrPendingRun();
}
bool MainWindow::vehicleStopConfirmed() const {
  return state_.speedValid && std::abs(state_.speed) <= 0.03 &&
         std::abs(state_.yawRate) <= 0.05;
}
void MainWindow::pollCloseAfterSafetyStop() {
  if (!closeAfterStop_) return;
  if (patrolRunActive() || state_.navigationActive) return;
  if (vehicleStopConfirmed() || closeSafetyAge_.elapsed() >= 10000) {
    closeAfterStop_ = false;
    closeSafetyTimer_->stop();
    close();
  }
}
void MainWindow::closeEvent(QCloseEvent *event) {
  if (process_->running() || saver_->busy()) { log("请先完成地图保存，并停止本界面启动的会话后退出"); event->ignore(); return; }
  if (mapEditor_ && !mapEditor_->maybeSave()) { event->ignore(); return; }
  controller_->stopManual();
  const bool needsSafetyWait = patrolRunActive() || state_.navigationActive ||
                               (state_.speedValid && !vehicleStopConfirmed());
  if (needsSafetyWait || closeAfterStop_) {
    if (!closeAfterStop_) {
      closeAfterStop_ = true;
      cancelPatrolRun(QStringLiteral("关闭桌面控制端"));
      controller_->emergencyStop();
      if (patrolClient_) patrolClient_->setOperatorAuthorized(false);
      closeSafetyAge_.start();
      closeSafetyTimer_->start();
      log("正在取消巡航并等待车辆停稳，完成后窗口会自动关闭");
    }
    if (patrolRunActive() || state_.navigationActive) { event->ignore(); return; }
    if (!vehicleStopConfirmed() && closeSafetyAge_.elapsed() < 10000) {
      event->ignore(); return;
    }
    closeAfterStop_ = false;
    closeSafetyTimer_->stop();
  }
  event->accept();
}
void MainWindow::changeEvent(QEvent *event) { if (event->type() == QEvent::ActivationChange && !isActiveWindow() && controller_) controller_->stopManual(); QMainWindow::changeEvent(event); }
bool MainWindow::saveScreenshot(const QString &path) {
  return screen() && screen()->grabWindow(winId()).save(path);
}
