#include "ui/mainwindow.h"
#include "ui/camera_panel.h"
#include "ui/map_editor_dialog.h"
#include "ui_mainwindow.h"
#include "adapters/business_client.h"
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
#include <QJsonDocument>
#include <QJsonObject>
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
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <cmath>

namespace {
constexpr double pi = 3.14159265358979323846;
QDoubleSpinBox *number(QWidget *parent, double min, double max, double value, int decimals = 2) {
  auto *s = new QDoubleSpinBox(parent); s->setRange(min, max); s->setDecimals(decimals); s->setValue(value); s->setKeyboardTracking(false); return s;
}
QPushButton *button(const QString &text, QLayout *layout) { auto *b = new QPushButton(text); layout->addWidget(b); return b; }
QString pathIdentity(const QString &path) {
  if (path.trimmed().isEmpty()) return {};
  const QFileInfo info(path);
  const QString canonical = info.canonicalFilePath();
  return canonical.isEmpty() ? info.absoluteFilePath() : canonical;
}
QLabel *imageLabel(const QString &text) { auto *l = new QLabel(text); l->setAlignment(Qt::AlignCenter); l->setMinimumSize(220, 140); l->setWordWrap(true); l->setStyleSheet("background:#252b33;color:#dce1e8;border-radius:4px;"); return l; }
}

MainWindow::MainWindow(const DesktopConfig &config, bool offline, QWidget *parent)
  : QMainWindow(parent), config_(config), offline_(offline) {
  Q_INIT_RESOURCE(images); Q_INIT_RESOURCE(media);
  business_ = new BusinessClient(this);
  ros_ = new RosAdapter(config, this);
  controller_ = new LocalController(ros_, config.maxLinear, config.maxAngular, config.feedbackTimeoutMs, this);
  process_ = new ProcessManager(config, this);
  saver_ = new MapSaver(config, ros_, this);
  setupUi();
  connect(controller_, &LocalController::message, this, &MainWindow::log);
  connect(business_, &BusinessClient::message, this, &MainWindow::log);
  connect(ros_, &RosAdapter::message, this, &MainWindow::log);
  connect(saver_, &MapSaver::message, this, &MainWindow::log);
  connect(saver_, &MapSaver::saved, this, [this](bool ok, const QString &text) { log((ok ? "保存成功：" : "保存未完成：") + text); });
  connect(saver_, &MapSaver::busyChanged, this, [this] { updateControls(); });
  connect(process_, &ProcessManager::output, this, &MainWindow::log);
  connect(process_, &ProcessManager::runningChanged, this, [this] { updateControls(); });
  connect(process_, &ProcessManager::finished, this, [this](int code) { log("本界面启动的会话已退出，退出码：" + QString::number(code)); });
  connect(controller_, &LocalController::controlChanged, this, [this](bool yes) { QSignalBlocker block(enableControl_); enableControl_->setChecked(yes); updateControls(); });
  connect(controller_, &LocalController::manualChanged, this, [this](bool ready) { take_->setText(ready ? "手动已接管" : "手动接管（先取消导航）"); updateControls(); });
  connect(ros_, &RosAdapter::navigationEvent, this, [this](const QString &text) { lastNavigationEvent_ = text; navState_->setText("本地 Nav2：" + text); log(text); });
  connect(ros_, &RosAdapter::snapshot, this, &MainWindow::updateSnapshot);
  connect(ros_, &RosAdapter::goalRequested, this, &MainWindow::sendGoal);
  connect(ros_, &RosAdapter::initialPoseRequested, this, &MainWindow::sendInitialPose);
  connect(ros_, &RosAdapter::capturedImage, this, [this](const QImage &image) { picture_->setPixmap(QPixmap::fromImage(image).scaled(picture_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)); });
  connect(ros_, &RosAdapter::ocrResult, ocr_, &QLabel::setText);
  connect(ros_, &RosAdapter::requestFinished, this, [this](const QString &op, bool ok, const QString &detail) { if (op != "save_state" && !(ok && (op == "ptz_jog" || op == "ptz_stop"))) log(op + (ok ? " 成功：" : " 失败：") + detail); });
  connect(business_, &BusinessClient::points, this, &MainWindow::showPoints);
  connect(qApp, &QGuiApplication::applicationStateChanged, this, [this](Qt::ApplicationState s) { if (s != Qt::ApplicationActive && controller_) controller_->stopManual(); });
  log(offline_ ? "离线验证模式：控制、业务写入和进程启动均禁用" : "观察模式。操作前请启用操作；网页与桌面端请勿同时操控");
  applyProfile();
  updateControls();
}
MainWindow::~MainWindow() {
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
void MainWindow::setupUi() {
  ui_ = new Ui::MainWindow; ui_->setupUi(this);
  setWindowTitle("巡检机器人 · ROS2 桌面控制台"); resize(1440, 960);
  ui_->menuView->setTitle("视图");
  ads::CDockManager::setConfigFlag(ads::CDockManager::OpaqueSplitterResize, true);
  ads::CDockManager::setConfigFlag(ads::CDockManager::FocusHighlighting, true);
  ads::CDockManager::setConfigFlag(ads::CDockManager::DockAreaHasUndockButton, false);
  docks_ = new ads::CDockManager(this);
  auto *tools = addToolBar("导航工具"); tools->setMovable(false);
  profiles_ = new QComboBox; profiles_->setObjectName("profileSelector");
  int selected = 0;
  for (int i = 0; i < desktopProfiles().size(); ++i) { profiles_->addItem(desktopProfiles()[i].name); if (desktopProfiles()[i].id == config_.defaultProfile) selected = i; }
  profiles_->setCurrentIndex(selected); tools->addWidget(profiles_);
  auto *load = tools->addAction("加载显示"); connect(load, &QAction::triggered, this, &MainWindow::applyProfile);
  mapEditorAction_ = tools->addAction(QIcon(":/images/edit.svg"), "地图编辑");
  mapEditorAction_->setObjectName("mapEditorAction");
  connect(mapEditorAction_, &QAction::triggered, this, &MainWindow::openMapEditor);
  auto *panels = tools->addAction("显示项 / 视角"); panels->setCheckable(true);
  connect(panels, &QAction::toggled, this, [this](bool visible) { if (rviz_) rviz_->setPanelsVisible(visible); });
  auto *initial = tools->addAction(QIcon(":/images/reloc2.svg"), "点选初始位姿"); connect(initial, &QAction::triggered, this, [this] { rviz_->selectTool("rviz_default_plugins/SetInitialPose"); });
  auto *goal = tools->addAction(QIcon(":/images/goal.svg"), "点选目标"); connect(goal, &QAction::triggered, this, [this] { rviz_->selectTool("rviz_default_plugins/SetGoal"); });
  tools->addSeparator();
  enableControl_ = new QCheckBox("启用操作"); enableControl_->setObjectName("enableControl"); tools->addWidget(enableControl_);
  connect(enableControl_, &QCheckBox::toggled, this, [this](bool yes) { controller_->setControlEnabled(yes); updateControls(); });
  auto *cancel = new QPushButton("取消导航"); cancel->setProperty("motion", true); tools->addWidget(cancel); writeButtons_ << cancel; connect(cancel, &QPushButton::clicked, controller_, &LocalController::cancelNavigation);
  auto *emergency = new QPushButton("急停"); emergency->setObjectName("emergencyStop"); emergency->setStyleSheet("background:#c93434;color:white;font-weight:bold;padding:8px 18px;"); tools->addWidget(emergency);
  emergency->setEnabled(!offline_); connect(emergency, &QPushButton::clicked, controller_, &LocalController::emergencyStop);

  rviz_ = new RvizHost(config_.resourceDir + "/config/rviz/" + profile().rvizFile);
  auto *central = new ads::CDockWidget("地图 / 雷达 / 点云"); central->setWidget(rviz_); auto *centralArea = docks_->setCentralWidget(central);
  centralArea->setAllowedAreas(ads::DockWidgetArea::OuterDockAreas);
  auto addDock = [this](const QString &name, QWidget *widget, ads::DockWidgetArea area, ads::CDockAreaWidget *relative = nullptr) {
    auto *dock = new ads::CDockWidget(name); dock->setWidget(widget); ui_->menuView->addAction(dock->toggleViewAction()); return docks_->addDockWidget(area, dock, relative);
  };
  auto *control = new QWidget; auto *left = new QVBoxLayout(control); control->setMinimumWidth(300);
  auto *connections = new QGroupBox("本地 ROS2 / 可选业务服务"); auto *connectionForm = new QFormLayout(connections);
  auto *local = new QLabel("导航、初始位姿、速度与急停直接连接本机 ROS2，无需网关"); local->setWordWrap(true); connectionForm->addRow(local);
  backendUrl_ = new QLineEdit(config_.backendUrl); connectionForm->addRow("业务后端", backendUrl_);
  auto *connectButton = new QPushButton("连接本地业务服务"); connectionForm->addRow(connectButton);
  connect(connectButton, &QPushButton::clicked, this, [this] { business_->setBaseUrl(backendUrl_->text()); business_->refreshPoints(); });
  connectButton->setEnabled(!offline_);
  left->addWidget(connections);
  auto *nav = new QGroupBox("目标位姿 / map 坐标系"); auto *navForm = new QFormLayout(nav);
  goalX_ = number(nav, -100000, 100000, 0, 3); goalY_ = number(nav, -100000, 100000, 0, 3); goalYaw_ = number(nav, -180, 180, 0, 1);
  goalX_->setObjectName("goalX"); goalY_->setObjectName("goalY"); goalYaw_->setObjectName("goalYaw");
  navForm->addRow("X (m)", goalX_); navForm->addRow("Y (m)", goalY_); navForm->addRow("朝向 (°)", goalYaw_);
  auto *send = new QPushButton("发送导航目标"); send->setObjectName("sendGoal"); navForm->addRow(send); writeButtons_ << send;
  send->setProperty("motion", true);
  connect(send, &QPushButton::clicked, this, [this] { sendGoal(goalX_->value(), goalY_->value(), goalYaw_->value()*pi/180.); });
  auto *relocate = new QPushButton("设置初始位姿"); navForm->addRow(relocate); writeButtons_ << relocate;
  relocate->setObjectName("setInitialPose");
  connect(relocate, &QPushButton::clicked, this, [this] { sendInitialPose(goalX_->value(), goalY_->value(), goalYaw_->value()*pi/180.); });
  left->addWidget(nav);
  auto *manual = new QGroupBox("手动行驶 · 按住运动，松开停止"); auto *manualLayout = new QVBoxLayout(manual);
  take_ = button("手动接管（先取消导航）", manualLayout); take_->setObjectName("takeManual"); connect(take_, &QPushButton::clicked, controller_, &LocalController::takeManualControl);
  unlock_ = button("解除急停（保持静止）", manualLayout); unlock_->setObjectName("releaseStop"); connect(unlock_, &QPushButton::clicked, controller_, &LocalController::releaseStop);
  auto *limits = new QFormLayout; speed_ = number(manual, .05, config_.maxLinear, .15); speed_->setSingleStep(.05); turn_ = number(manual, .05, config_.maxAngular, .25); turn_->setSingleStep(.05);
  limits->addRow("速度 (m/s)", speed_); limits->addRow("角速度 (rad/s)", turn_); manualLayout->addLayout(limits);
  auto *grid = new QGridLayout;
  const QStringList labels{"前进左转", "前进", "前进右转", "倒车左打轮", "倒车", "倒车右打轮"};
  for (int i = 0; i < labels.size(); ++i) {
    auto *b = new QPushButton(labels[i]); b->setAutoRepeat(false); b->setObjectName("manualDirection" + QString::number(i)); grid->addWidget(b, i/3, i%3); directionButtons_ << b;
    connect(b, &QPushButton::pressed, this, [this, i] { const double sign = i < 3 ? 1. : -1.; const double steer = i%3 == 0 ? 1. : i%3 == 2 ? -1. : 0.; controller_->move(sign*speed_->value(), sign*steer*turn_->value()); });
    connect(b, &QPushButton::released, controller_, &LocalController::stopManual);
  }
  manualLayout->addLayout(grid); left->addWidget(manual);
  auto *runtime = new QGroupBox("本机会话（默认仅连接已有系统）"); auto *run = new QVBoxLayout(runtime);
  manage_ = new QCheckBox("由界面启动与管理所选流程"); run->addWidget(manage_);
  mapPrefix_ = new QLineEdit; mapPrefix_->setObjectName("mapPrefix"); mapPrefix_->setPlaceholderText("地图绝对路径，不带扩展名"); run->addWidget(mapPrefix_);
  navigationYaml_ = new QLineEdit; navigationYaml_->setObjectName("navigationYaml"); navigationYaml_->setPlaceholderText("可选：编辑后的导航栅格 YAML 绝对路径；留空使用上方前缀 .yaml"); run->addWidget(navigationYaml_);
  sensors_ = new QCheckBox("启动雷达 / IMU"); sensors_->setChecked(true); chassis_ = new QCheckBox("启动底盘驱动"); bridge_ = new QCheckBox("启用底盘控制桥"); run->addWidget(sensors_); run->addWidget(chassis_); run->addWidget(bridge_);
  odom_ = new QComboBox; odom_->addItems({"lio", "wheel", "wheel_imu"}); run->addWidget(odom_);
  start_ = button("启动所选流程（不另起 RViz）", run); connect(start_, &QPushButton::clicked, this, &MainWindow::startSession);
  save_ = button("保存当前建图产物", run); connect(save_, &QPushButton::clicked, this, [this] { if (allowed()) saver_->save(profile(), mapPrefix_->text().trimmed()); });
  save_->setToolTip("保存建图状态及导航 YAML/PGM；FAST-LIO 保存 PCD 后自动进行二维切片");
  stop_ = button("停止本界面启动的流程", run); connect(stop_, &QPushButton::clicked, this, [this] { if (saver_->busy()) return; controller_->emergencyStop(); process_->stop(); });
  connect(manage_, &QCheckBox::toggled, this, [this] { updateControls(); });
  left->addWidget(runtime); left->addStretch();
  auto *controlArea = addDock("控制", control, ads::LeftDockWidgetArea);

  cameraPanel_ = new CameraPanel(config_, ros_, offline_);
  connect(cameraPanel_, &CameraPanel::message, this, &MainWindow::log);
  auto *cameraScroll = new QScrollArea; cameraScroll->setWidgetResizable(true);
  cameraScroll->setMinimumWidth(320);
  cameraScroll->setFrameShape(QFrame::NoFrame); cameraScroll->setWidget(cameraPanel_);
  auto *cameraArea = addDock("相机监控", cameraScroll, ads::RightDockWidgetArea);
  auto *pictureWidget = new QWidget; auto *pictureLayout = new QVBoxLayout(pictureWidget);
  picture_ = imageLabel("尚未抓拍"); pictureLayout->addWidget(picture_, 1);
  ocr_ = new QLabel("仪表识别：尚未识别"); ocr_->setWordWrap(true); ocr_->setTextInteractionFlags(Qt::TextSelectableByMouse); pictureLayout->addWidget(ocr_);
  auto *capture = button("相机抓拍（预览）", pictureLayout); writeButtons_ << capture; capture->setProperty("capability", "camera"); connect(capture, &QPushButton::clicked, this, [this] { if (allowed()) ros_->captureCamera(); });
  auto *businessCapture = button("业务抓拍（现有记录链路）", pictureLayout); writeButtons_ << businessCapture; connect(businessCapture, &QPushButton::clicked, ros_, &RosAdapter::captureBusiness);
  auto *ocrButton = button("识别当前仪表画面", pictureLayout); writeButtons_ << ocrButton; ocrButton->setProperty("capability", "ocr"); connect(ocrButton, &QPushButton::clicked, this, [this] { if (allowed()) ros_->recognize(); });
  addDock("抓拍 / 数字仪表识别", pictureWidget, ads::BottomDockWidgetArea, cameraArea);

  auto *statusWidget = new QWidget; auto *statusLayout = new QVBoxLayout(statusWidget); statusLayout->setSpacing(3);
  speedStack_ = new QStackedWidget; auto *unavailable = new QLabel("速度反馈未接入 / 已过期"); unavailable->setAlignment(Qt::AlignCenter); speedStack_->addWidget(unavailable);
  dashboard_ = new DashBoard(speedStack_); speedStack_->addWidget(dashboard_); speedStack_->setMinimumSize(260, 100); statusLayout->addWidget(speedStack_);
  velocity_ = new QLabel("实际速度：-- m/s   实际角速度：-- rad/s"); statusLayout->addWidget(velocity_);
  pose_ = new QLabel("位姿未接入"); pose_->setObjectName("poseStatus"); pose_->setWordWrap(true); statusLayout->addWidget(pose_);
  stopState_ = new QLabel("急停状态未知"); statusLayout->addWidget(stopState_);
  navState_ = new QLabel("本地 Nav2：等待状态"); statusLayout->addWidget(navState_);
  auto *batteryRow = new QHBoxLayout; battery_ = new QProgressBar; battery_->setRange(0, 100); battery_->setValue(0); battery_->setFormat("未接入"); power_ = new QLabel("-- V"); batteryRow->addWidget(battery_); batteryRow->addWidget(power_); statusLayout->addLayout(batteryRow);
  auto *statusArea = addDock("运行状态 / 速度仪表", statusWidget, ads::BottomDockWidgetArea, centralArea);
  auto *pointsWidget = new QWidget; auto *pointLayout = new QVBoxLayout(pointsWidget);
  auto *mapHint = new QLabel("点位来自现有业务后端，请确认属于当前地图"); mapHint->setWordWrap(true); pointLayout->addWidget(mapHint);
  points_ = new QTableWidget(0, 5); points_->setHorizontalHeaderLabels({"ID", "名称", "X", "Y", "朝向°"}); points_->setSelectionBehavior(QAbstractItemView::SelectRows); points_->setSelectionMode(QAbstractItemView::SingleSelection); points_->setEditTriggers(QAbstractItemView::NoEditTriggers); points_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents); pointLayout->addWidget(points_);
  connect(points_, &QTableWidget::itemSelectionChanged, this, [this] { int row = points_->currentRow(); if (row < 0 || !points_->item(row, 0)) return; auto o = points_->item(row, 0)->data(Qt::UserRole).toJsonObject(); goalX_->setValue(o.value("x").toDouble()); goalY_->setValue(o.value("y").toDouble()); goalYaw_->setValue(o.value("yaw").toDouble()*180./pi); });
  auto *pointButtons = new QHBoxLayout;
  auto *refresh = button("刷新点位", pointButtons); refresh->setEnabled(!offline_); connect(refresh, &QPushButton::clicked, this, [this] { business_->setBaseUrl(backendUrl_->text()); business_->refreshPoints(); });
  auto *savePoint = button("保存当前位置", pointButtons); writeButtons_ << savePoint;
  connect(savePoint, &QPushButton::clicked, this, [this] { if (!allowed() || !state_.poseValid || state_.frame != "map") { log("保存点位需要有效的 map 位姿"); return; } bool ok = false; auto name = QInputDialog::getText(this, "保存点位", "点位名称", QLineEdit::Normal, "", &ok); if (ok && !name.trimmed().isEmpty()) business_->addPoint(name.trimmed(), state_.x, state_.y, state_.yaw); });
  auto *remove = button("删除所选", pointButtons); writeButtons_ << remove; connect(remove, &QPushButton::clicked, this, [this] { int row = points_->currentRow(); if (allowed() && row >= 0 && points_->item(row, 0)) business_->deletePoint(points_->item(row, 0)->text().toInt()); });
  pointLayout->addLayout(pointButtons);
  auto *plans = button("打开路线 / 巡检计划管理", pointLayout); plans->setEnabled(!offline_); connect(plans, &QPushButton::clicked, this, [this] { QDesktopServices::openUrl(QUrl(backendUrl_->text() + "/plans.html")); });
  addDock("任务点位", pointsWidget, ads::RightDockWidgetArea, statusArea);
  logs_ = new QPlainTextEdit; logs_->setReadOnly(true); logs_->setMaximumBlockCount(500); logs_->setMinimumHeight(100); auto *logArea = addDock("消息与会话日志", logs_, ads::BottomDockWidgetArea);
  QTimer::singleShot(0, this, [this, centralArea, controlArea, statusArea, logArea] {
    docks_->setSplitterSizes(logArea, {780, 130});
    docks_->setSplitterSizes(controlArea, {340, 750, 330});
    docks_->setSplitterSizes(centralArea, {500, 240});
    docks_->setSplitterSizes(statusArea, {300, 540});
  });
  connection_ = new QLabel("本地 ROS2 接口"); statusBar()->addPermanentWidget(connection_);
  connect(profiles_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] { applyProfile(); });
}
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
  state_.poseValid = false;
  log("已加载显示：" + profile().name + "；此操作不切换后台算法"); updateControls();
}
void MainWindow::updateControls() {
  const bool ok = allowed(); const bool running = process_->running(); const bool busy = saver_->busy();
  enableControl_->setEnabled(!offline_);
  cameraPanel_->setControlEnabled(ok);
  const bool motion = ok && controller_->controlEnabled();
  for (auto *b : writeButtons_) { const auto cap = b->property("capability").toString(); b->setEnabled(ok && (!b->property("motion").toBool() || motion) && (cap.isEmpty() || ros_->capabilities().contains(cap))); }
  take_->setEnabled(motion); unlock_->setEnabled(motion && !state_.navigationActive && (!state_.estopValid || state_.estopped));
  const bool manual = motion && controller_->manualReady() && state_.estopValid && !state_.estopped && state_.navigationKnown && !state_.navigationActive;
  for (auto *b : directionButtons_) b->setEnabled(manual);
  manage_->setEnabled(!offline_ && !running && !busy);
  start_->setEnabled(!offline_ && manage_->isChecked() && !running && !busy);
  stop_->setEnabled(!offline_ && running && !busy);
  save_->setEnabled(ok && profile().mapping && !busy);
  profiles_->setEnabled(!running && !busy); mapPrefix_->setReadOnly(running || busy);
  if (navigationYaml_) navigationYaml_->setReadOnly(running || busy);
  if (mapEditorAction_) mapEditorAction_->setEnabled(!busy);
  sensors_->setEnabled(!running); chassis_->setEnabled(!running); bridge_->setEnabled(!running); odom_->setEnabled(!running);
}
void MainWindow::updateSnapshot(const RobotSnapshot &s) {
  state_ = s; controller_->updateFeedback(s.estopped, s.navigationActive, s.navigationKnown, s.estopValid);
  pose_->setText(s.poseValid ? QString("%1: x=%2 m  y=%3 m  朝向=%4°").arg(s.frame).arg(s.x, 0, 'f', 3).arg(s.y, 0, 'f', 3).arg(s.yaw*180./pi, 0, 'f', 1) : s.frame + " 位姿未接入 / 已过期");
  speedStack_->setCurrentIndex(s.speedValid ? 1 : 0);
  if (s.speedValid) dashboard_->set_speed(qRound(std::abs(s.speed)*100.));
  velocity_->setText(s.speedValid ? QString("实际 v=%1 m/s   ω=%2 rad/s").arg(s.speed, 0, 'f', 3).arg(s.yawRate, 0, 'f', 3) : "实际速度：-- m/s   实际角速度：-- rad/s");
  if (!s.navigationAvailable) { lastNavigationEvent_.clear(); navState_->setText("本地 Nav2：未启动 / 未就绪"); }
  else if (s.navigationActive) navState_->setText("本地 Nav2：目标执行中 / 取消中");
  else navState_->setText("本地 Nav2：" + (lastNavigationEvent_.isEmpty() ? "已就绪" : lastNavigationEvent_));
  stopState_->setText(!s.estopValid ? "急停状态未知 / 已过期" : s.estopped ? "急停保持中" : "急停已解除");
  battery_->setValue(s.batteryValid ? qRound(s.battery) : 0); battery_->setFormat(s.batteryValid ? "%p%" : "未接入 / 已过期"); power_->setText(s.batteryValid ? QString::number(s.voltage, 'f', 1) + " V" : "-- V");
  updateControls();
}
void MainWindow::sendGoal(double x, double y, double yaw) {
  if (!offline_ && profile().navigation && ros_->previewGoal(x, y, yaw)) {
    goalX_->setValue(x); goalY_->setValue(y); goalYaw_->setValue(yaw*180./pi);
  }
  if (!allowed() || !controller_->controlEnabled() || !profile().navigation || !state_.poseValid || state_.frame != "map") { log("目标未发送：请启用操作、选择导航模式并等待 map 定位有效"); return; }
  if (controller_->navigate(x, y, yaw)) log(QString("导航请求已发送：(%1, %2), %3°").arg(x).arg(y).arg(yaw*180./pi));
}
void MainWindow::sendInitialPose(double x, double y, double yaw) {
  if (!allowed() || !profile().navigation) { log("初始位姿未发送：请启用操作并选择定位导航模式"); return; }
  if (profile().backend == "cartographer") { log("当前 Cartographer 纯定位入口不支持 /initialpose，请按原流程在建图起点启动"); return; }
  if (state_.navigationActive || (state_.speedValid && (std::abs(state_.speed) > .01 || std::abs(state_.yawRate) > .02))) { log("请先停止车辆并取消导航，再设置初始位姿"); return; }
  if (ros_->setInitialPose(x, y, yaw)) log("初始位姿已发布到 /initialpose，等待定位结果；此操作不发送导航目标");
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
  const QStringList args = ProcessManager::launchArguments(config_, profile(), prefix, sensors_->isChecked(), chassis_->isChecked(), bridge_->isChecked(), odom_->currentText(), navigationYaml);
  log("启动本机会话：" + args.join(' ')); process_->start(args);
  if (profile().backend == "fastlio" && profile().navigation) log("FAST-LIO 定位需先点选车辆当前位置与朝向；ICP 成功后才会建立 map TF，Nav2 随后继续激活");
  if (profile().id == "slam_navigation") log("AMCL 已启动：请启用操作并点选车辆实际位置与朝向；收到激光后建立 map TF，Nav2 随后继续激活");
  log("车辆操作使用本地 ROS2；首次行驶前需明确点击解除急停");
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
  connect(editor, &MapEditorDialog::mapSaved, this, [this](const QString &yamlPath, const QString &sourceYamlPath) {
    const bool sameDocumentSource = !mapEditorInitialSourceYaml_.trimmed().isEmpty() &&
                                     !sourceYamlPath.trimmed().isEmpty() &&
                                     pathIdentity(sourceYamlPath) == pathIdentity(mapEditorInitialSourceYaml_);
    const bool sameSource = sameDocumentSource &&
                            !mapEditorProfileId_.isEmpty() &&
                            profile().id == mapEditorProfileId_ &&
                            mapPrefix_->text().trimmed() == mapEditorSourcePrefix_;
    if (!sameSource) {
      log(QStringLiteral("地图编辑来源模式或原地图前缀已变化，未自动填入导航栅格 YAML；"
                         "请重新确认编辑图与当前定位文件对应后再手动选择：%1（来源：%2）")
              .arg(yamlPath, sourceYamlPath));
      return;
    }
    navigationYaml_->setText(yamlPath);
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

void MainWindow::showPoints(const QJsonArray &items) {
  points_->setRowCount(0);
  for (const auto &value : items) {
    const auto o = value.toObject(); int row = points_->rowCount(); points_->insertRow(row);
    const QStringList values{QString::number(o.value("id").toInt()), o.value("name").toString(), QString::number(o.value("x").toDouble(), 'f', 3), QString::number(o.value("y").toDouble(), 'f', 3), QString::number(o.value("yaw").toDouble()*180./pi, 'f', 1)};
    for (int col = 0; col < values.size(); ++col) points_->setItem(row, col, new QTableWidgetItem(values[col]));
    points_->item(row, 0)->setData(Qt::UserRole, o);
  }
}
void MainWindow::log(const QString &text) { if (logs_) logs_->appendPlainText(QDateTime::currentDateTime().toString("HH:mm:ss ") + text.trimmed()); statusBar()->showMessage(text.trimmed().left(180), 7000); }
void MainWindow::closeEvent(QCloseEvent *event) {
  if (process_->running() || saver_->busy()) { log("请先完成地图保存，并停止本界面启动的会话后退出"); event->ignore(); return; }
  if (mapEditor_ && !mapEditor_->maybeSave()) { event->ignore(); return; }
  controller_->stopManual(); event->accept();
}
void MainWindow::changeEvent(QEvent *event) { if (event->type() == QEvent::ActivationChange && !isActiveWindow() && controller_) controller_->stopManual(); QMainWindow::changeEvent(event); }
bool MainWindow::saveScreenshot(const QString &path) {
  return screen() && screen()->grabWindow(winId()).save(path);
}
