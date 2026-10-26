#include "ui/mainwindow.h"
#include "ui/console_theme.h"
#include "ui/camera_panel.h"
#include "devices/device_client.h"
#include "devices/device_panel.h"
#include "devices/gas_readout.h"
#include <QDialog>
#include "vision/vision_panel.h"
#include "ui_mainwindow.h"
#include "adapters/local_controller.h"
#include "patrol/patrol_client.h"
#include "runtime/map_saver.h"
#include "runtime/process_manager.h"
#include "rviz/rviz_host.h"
#include "DockAreaWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDesktopServices>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QMenuBar>
#include <QLineEdit>
#include <QMenu>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QPainter>
#include <QResizeEvent>
#include <QScreen>
#include <QScrollArea>
#include <QSettings>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <cmath>

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr int layoutVersion = 1;
constexpr int compactWidth = 1180;

class ElidedLabel : public QLabel {
public:
  using QLabel::QLabel;
protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    painter.setPen(palette().color(QPalette::WindowText));
    painter.drawText(contentsRect(), Qt::AlignVCenter | Qt::AlignLeft,
                     fontMetrics().elidedText(text(), Qt::ElideRight, contentsRect().width()));
  }
};

QDoubleSpinBox *number(QWidget *parent, const char *name, double min, double max, double value, int decimals = 2) {
  auto *input = new QDoubleSpinBox(parent);
  input->setObjectName(name); input->setRange(min, max); input->setDecimals(decimals);
  input->setValue(value); input->setKeyboardTracking(false);
  return input;
}
QPushButton *button(const QString &text, QLayout *layout, const char *name = nullptr, bool primary = false) {
  auto *result = new QPushButton(text);
  if (name) result->setObjectName(name);
  if (primary) result->setProperty("consoleRole", "primary");
  layout->addWidget(result); return result;
}
QLabel *hint(const QString &text, QLayout *layout = nullptr) {
  auto *label = new QLabel(text); label->setWordWrap(true); label->setProperty("muted", true);
  label->setTextFormat(Qt::PlainText);
  if (layout) layout->addWidget(label);
  return label;
}
QScrollArea *scrollPane(QWidget *content) {
  auto *area = new QScrollArea;
  area->setWidgetResizable(true); area->setFrameShape(QFrame::NoFrame);
  area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff); area->setWidget(content);
  return area;
}
QVBoxLayout *pageLayout(QWidget *page) {
  auto *layout = new QVBoxLayout(page); layout->setContentsMargins(12, 12, 12, 12); layout->setSpacing(10);
  return layout;
}
QSettings layoutSettings() {
  return QSettings(QSettings::IniFormat, QSettings::UserScope, "RemoteCtrol", "desktop_console_v1");
}
}

void MainWindow::setupUi() {
  Q_INIT_RESOURCE(console_icons);
  ui_ = new Ui::MainWindow; ui_->setupUi(this);
  setWindowTitle("巡检机器人 · 本地控制台");
  const auto available = screen()->availableGeometry();
  resize(qMin(1600, available.width() - 24), qMin(1000, available.height() - 36));
  setStyleSheet(consoleStyleSheet());
  ui_->menuView->setTitle("视图");
  statusBar()->hide();

  // A single expanding header widget keeps safety controls out of toolbar overflow.
  auto *headerBar = addToolBar("控制台"); headerBar->setMovable(false); headerBar->setFloatable(false);
  headerBar->setObjectName("consoleHeaderBar"); headerBar->setStyleSheet("QToolBar#consoleHeaderBar { border: none; padding: 0px; }");
  auto *header = new QWidget; header->setObjectName("consoleHeader");
  header->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  auto *headerRow = new QHBoxLayout(header); headerRow->setContentsMargins(16, 7, 16, 7); headerRow->setSpacing(12);
  auto *logo = new QLabel("R"); logo->setAlignment(Qt::AlignCenter); logo->setFixedSize(34, 34);
  logo->setStyleSheet("background:#2563eb; color:white; border-radius:8px; font-size:22px; font-weight:600;");
  headerRow->addWidget(logo);
  auto *identity = new QVBoxLayout; identity->setSpacing(1);
  auto *title = new QLabel("巡检机器人控制台"); title->setObjectName("consoleTitle"); identity->addWidget(title);
  connection_ = new QLabel; connection_->setObjectName("consoleSource"); identity->addWidget(connection_);
  headerRow->addLayout(identity); headerRow->addStretch();
  modeBadge_ = new QLabel; modeBadge_->setObjectName("consoleMode"); headerRow->addWidget(modeBadge_);
  enableControl_ = new QCheckBox("启用操作"); enableControl_->setObjectName("enableControl"); headerRow->addWidget(enableControl_);
  connect(enableControl_, &QCheckBox::toggled, this, [this](bool yes) {
    patrolClient_->setOperatorAuthorized(yes);
    if (!yes) cancelPatrolRun(QStringLiteral("关闭操作许可"));
    controller_->setControlEnabled(yes); updateControls();
  });
  cancelButton_ = button("取消导航", headerRow, "cancelNavigation");
  cancelButton_->setProperty("motion", true); writeButtons_ << cancelButton_;
  connect(cancelButton_, &QPushButton::clicked, this, [this] {
    if (!cancelPatrolRun(QStringLiteral("取消导航"))) controller_->cancelNavigation();
  });
  returnHomeButton_ = button("一键返回", headerRow, "returnHome");
  returnHomeButton_->setToolTip("取消当前路线并返回当前地图已保存的待命点");
  connect(returnHomeButton_, &QPushButton::clicked, this, &MainWindow::returnHome);
  auto *emergency = button("急停", headerRow, "emergencyStop"); emergency->setEnabled(!offline_);
  connect(emergency, &QPushButton::clicked, controller_, &LocalController::emergencyStop);
  headerBar->addWidget(header);
  addToolBarBreak();
  auto *tools = addToolBar("地图工具"); tools->setObjectName("consoleTools");
  tools->setMovable(false); tools->setFloatable(false); tools->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  tools->addWidget(new QLabel("显示方案  "));
  profiles_ = new QComboBox; profiles_->setObjectName("profileSelector"); profiles_->setMinimumWidth(236);
  for (const auto &p : desktopProfiles()) profiles_->addItem(p.name);
  for (int i = 0; i < desktopProfiles().size(); ++i)
    if (desktopProfiles()[i].id == config_.defaultProfile) profiles_->setCurrentIndex(i);
  tools->addWidget(profiles_);
  auto *load = tools->addAction("加载显示"); connect(load, &QAction::triggered, this, &MainWindow::applyProfile);
  tools->addSeparator();
  auto *goal = tools->addAction(QIcon(":/console/target.svg"), "点选目标"); goal->setObjectName("consoleGoalTool");
  connect(goal, &QAction::triggered, this, [this] { controlTabs_->setCurrentIndex(1); rviz_->selectTool("rviz_default_plugins/SetGoal"); });
  auto *initial = tools->addAction(QIcon(":/console/locate.svg"), "初始位姿");
  connect(initial, &QAction::triggered, this, [this] { controlTabs_->setCurrentIndex(1); rviz_->selectTool("rviz_default_plugins/SetInitialPose"); });
  auto *interaction = tools->addAction("交互");
  connect(interaction, &QAction::triggered, this, [this] { rviz_->selectTool("rviz_default_plugins/Interact"); });
  auto *more = new QToolButton; more->setText("更多工具"); more->setPopupMode(QToolButton::InstantPopup);
  auto *toolMenu = new QMenu(more);
  for (const auto &entry : {qMakePair(QStringLiteral("移动视角"), QStringLiteral("MoveCamera")),
                           qMakePair(QStringLiteral("选择"), QStringLiteral("Select")),
                           qMakePair(QStringLiteral("聚焦视角"), QStringLiteral("FocusCamera")),
                           qMakePair(QStringLiteral("测距"), QStringLiteral("Measure"))}) {
    auto *action = toolMenu->addAction(entry.first);
    connect(action, &QAction::triggered, this, [this, id = entry.second] { rviz_->selectTool("rviz_default_plugins/" + id); });
  }
  more->setMenu(toolMenu); tools->addWidget(more);
  auto *panels = tools->addAction(QIcon(":/console/layers.svg"), "显示项 / 视角"); panels->setCheckable(true);
  connect(panels, &QAction::toggled, this, [this](bool visible) { rviz_->setPanelsVisible(visible); });
  auto *spacer = new QWidget; spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred); tools->addWidget(spacer);
  mapEditorAction_ = tools->addAction(QIcon(":/console/edit.svg"), "地图编辑"); mapEditorAction_->setObjectName("mapEditorAction");
  connect(mapEditorAction_, &QAction::triggered, this, &MainWindow::openMapEditor);
  patrolAction_ = tools->addAction(QIcon(":/console/route.svg"), "路线巡航"); patrolAction_->setObjectName("patrolAction");
  patrolAction_->setToolTip("打开独立的本地点位、路线与多点巡航页面");
  connect(patrolAction_, &QAction::triggered, this, &MainWindow::openPatrolPage);
  auto *moduleMenu = ui_->menubar->addMenu("工具"); moduleMenu->addAction(mapEditorAction_); moduleMenu->addAction(patrolAction_);
  auto *devices = tools->addAction("设备执勤"); devices->setObjectName("deviceDutyAction");
  devices->setToolTip("独立启动气体检测和云台往返巡航"); moduleMenu->addAction(devices);
  connect(devices, &QAction::triggered, this, &MainWindow::openDeviceWindow);

  auto *shell = new QWidget; shell->setObjectName("consoleShell"); setCentralWidget(shell);
  auto *shellLayout = new QVBoxLayout(shell); shellLayout->setContentsMargins(10, 10, 10, 8); shellLayout->setSpacing(8);
  ads::CDockManager::setConfigFlag(ads::CDockManager::OpaqueSplitterResize, true);
  ads::CDockManager::setConfigFlag(ads::CDockManager::FocusHighlighting, false);
  ads::CDockManager::setConfigFlag(ads::CDockManager::DockAreaHasUndockButton, false);
  docks_ = new ads::CDockManager(shell); docks_->setObjectName("consoleDocks");
  docks_->setStyleSheet(docks_->styleSheet() + consoleDockStyleSheet()); shellLayout->addWidget(docks_, 1);
  rviz_ = new RvizHost(config_.resourceDir + "/config/rviz/" + profile().rvizFile);
  mapDock_ = new ads::CDockWidget("地图 / 雷达 / 点云"); mapDock_->setObjectName("consoleMapDock");
  mapDock_->setWidget(rviz_); auto *center = docks_->setCentralWidget(mapDock_);
  center->setAllowedAreas(ads::DockWidgetArea::OuterDockAreas);
  auto addDock = [this, center](const QString &name, const char *id, QWidget *widget, ads::DockWidgetArea area) {
    auto *dock = new ads::CDockWidget(name); dock->setObjectName(id);
    dock->setWidget(widget, ads::CDockWidget::ForceNoScrollArea);
    dock->setFeature(ads::CDockWidget::DockWidgetFloatable, false);
    dock->setMinimumSizeHintMode(ads::CDockWidget::MinimumSizeHintFromContentMinimumSize);
    ui_->menuView->addAction(dock->toggleViewAction()); docks_->addDockWidget(area, dock, center); return dock;
  };

  auto *control = new QWidget; control->setObjectName("consoleControls"); control->setMinimumWidth(264);
  auto *left = new QVBoxLayout(control); left->setContentsMargins(0, 0, 0, 0);
  controlTabs_ = new QTabWidget; controlTabs_->setObjectName("controlWorkspaceTabs"); left->addWidget(controlTabs_);
  auto *runtime = new QWidget; auto *run = pageLayout(runtime);
  sessionHint_ = hint("本地 ROS2 直连 · 无需网关", run); sessionHint_->setObjectName("consoleSessionHint");
  manage_ = new QCheckBox("由界面启动与管理流程"); manage_->setObjectName("manageSession"); run->addWidget(manage_);
  auto *maps = new QGroupBox("地图文件"); auto *mapLayout = new QVBoxLayout(maps); mapLayout->setSpacing(8);
  hint("地图前缀（不带扩展名）", mapLayout);
  auto *prefixRow = new QHBoxLayout;
  mapPrefix_ = new QLineEdit; mapPrefix_->setObjectName("mapPrefix"); prefixRow->addWidget(mapPrefix_, 1);
  auto *browseMap = button("…", prefixRow, "consoleBrowseMap"); browseMap->setFixedWidth(30); mapLayout->addLayout(prefixRow);
  hint("导航栅格 YAML（可选）", mapLayout);
  auto *yamlRow = new QHBoxLayout;
  navigationYaml_ = new QLineEdit; navigationYaml_->setObjectName("navigationYaml"); navigationYaml_->setPlaceholderText("留空使用地图前缀 .yaml"); yamlRow->addWidget(navigationYaml_, 1);
  auto *browseYaml = button("…", yamlRow, "consoleBrowseYaml"); browseYaml->setFixedWidth(30); mapLayout->addLayout(yamlRow);
  run->addWidget(maps);
  connect(browseMap, &QPushButton::clicked, this, [this] {
    if (mapPrefix_->isReadOnly()) return;
    const auto path = QFileDialog::getOpenFileName(this, "选择已有地图以填写前缀", mapPrefix_->text(), "地图 (*.yaml *.pcd *.pbstream *.posegraph)");
    if (!path.isEmpty()) { QFileInfo info(path); mapPrefix_->setText(info.absolutePath() + "/" + info.completeBaseName()); }
  });
  connect(browseYaml, &QPushButton::clicked, this, [this] {
    if (navigationYaml_->isReadOnly()) return;
    const auto path = QFileDialog::getOpenFileName(this, "选择导航栅格", navigationYaml_->text(), "导航栅格 (*.yaml)");
    if (!path.isEmpty()) navigationYaml_->setText(path);
  });
  connect(mapPrefix_, &QLineEdit::textChanged, this, [this](const QString &value) { mapPrefix_->setToolTip(value); refreshEditorNavigationFeedback(); });
  connect(navigationYaml_, &QLineEdit::textChanged, this, [this](const QString &value) { navigationYaml_->setToolTip(value); refreshEditorNavigationFeedback(); });
  auto *options = new QGroupBox("启动选项"); auto *optionLayout = new QVBoxLayout(options); optionLayout->setSpacing(4);
  sensors_ = new QCheckBox("雷达 / IMU"); sensors_->setChecked(true);
  chassis_ = new QCheckBox("底盘驱动"); bridge_ = new QCheckBox("底盘控制桥");
  optionLayout->addWidget(sensors_); optionLayout->addWidget(chassis_); optionLayout->addWidget(bridge_);
  patrolInterlock_ = new QCheckBox("路线巡航联锁"); patrolInterlock_->setObjectName("enablePatrolInterlock");
  patrolInterlock_->setToolTip("同时启动巡航执行器和安全联锁；开启后请通过路线巡航下发目标，单点导航入口禁用"); optionLayout->addWidget(patrolInterlock_);
  auto *advancedToggle = new QToolButton; advancedToggle->setObjectName("runtimeAdvancedToggle"); advancedToggle->setText("高级选项"); advancedToggle->setCheckable(true); advancedToggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon); advancedToggle->setArrowType(Qt::RightArrow);
  auto *advanced = new QWidget; auto *advancedForm = new QFormLayout(advanced); advancedForm->setContentsMargins(0, 0, 0, 0);
  odom_ = new QComboBox; odom_->setObjectName("odometrySource"); odom_->addItems({"lio", "wheel", "wheel_imu"}); advancedForm->addRow("里程计来源", odom_);
  odom_->setItemData(0, "FAST-LIO 雷达 + IMU 运动估计", Qt::ToolTipRole);
  odom_->setItemData(1, "根据底盘反馈的速度、转角推算运动", Qt::ToolTipRole);
  odom_->setItemData(2, "底盘运动数据 + IMU 融合", Qt::ToolTipRole);
  odomHint_ = hint({}); odomHint_->setObjectName("odometrySourceHint"); advancedForm->addRow(odomHint_);
  connect(odom_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] {
    if (profile().backend == "fastlio" && profile().navigation && !profile().mapping) {
      fastlioNavigationOdomSource_ = odom_->currentText();
      refreshOdometrySource();
    }
  });
  advanced->hide(); optionLayout->addWidget(advancedToggle); optionLayout->addWidget(advanced);
  connect(advancedToggle, &QToolButton::toggled, this, [advanced, advancedToggle](bool yes) { advanced->setVisible(yes); advancedToggle->setArrowType(yes ? Qt::DownArrow : Qt::RightArrow); });
  run->addWidget(options);
  auto *runtimePage = new QWidget; auto *runtimePageLayout = new QVBoxLayout(runtimePage);
  runtimePageLayout->setContentsMargins(0, 0, 0, 0); runtimePageLayout->setSpacing(0);
  run->addStretch(); runtimePageLayout->addWidget(scrollPane(runtime), 1);
  auto *runtimeFooter = new QWidget; auto *footerLayout = pageLayout(runtimeFooter);
  runtimeFooterLayout_ = footerLayout;
  start_ = button("启动所选流程", footerLayout, "startSession", true);
  connect(start_, &QPushButton::clicked, this, &MainWindow::startSession);
  auto *sessionActions = new QHBoxLayout;
  save_ = button("保存地图", sessionActions, "saveMap"); stop_ = button("停止流程", sessionActions, "stopSession"); footerLayout->addLayout(sessionActions);
  save_->setToolTip("FAST-LIO 保存独立地图包并自动生成切片；单程也可保存，现有导航地图继续保留");
  connect(save_, &QPushButton::clicked, this, [this] { if (allowed()) saver_->save(profile(), mapPrefix_->text().trimmed()); });
  stop_->setToolTip("只停止本界面启动的流程，外部节点不受影响");
  connect(stop_, &QPushButton::clicked, this, [this] { if (saver_->busy()) return; cancelPatrolRun(QStringLiteral("停止本界面流程")); controller_->emergencyStop(); process_->stop(); });
  mapSaveStatus_ = hint("保存后在这里查看地图状态；单程建图也可保存。", footerLayout);
  mapSaveStatus_->setObjectName("mapSaveStatus"); mapSaveStatus_->setWordWrap(true);
  auto *savedActions = new QHBoxLayout;
  openSavedMap_ = button("打开保存目录", savedActions, "openSavedMap");
  loadSavedMap_ = button("载入本次地图", savedActions, "loadSavedMap");
  footerLayout->addLayout(savedActions);
  connect(openSavedMap_, &QPushButton::clicked, this, [this] {
    if (!saver_->savedPrefix().isEmpty())
      QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(saver_->savedPrefix()).absolutePath()));
  });
  connect(loadSavedMap_, &QPushButton::clicked, this, [this] {
    if (!allowed() || process_->running() || saver_->busy() || !saver_->canLoadSavedMap() || patrolRunActive()) return;
    profiles_->setCurrentIndex(0);
    mapPrefix_->setText(saver_->savedPrefix()); navigationYaml_->setText(saver_->savedPrefix() + ".yaml");
    log("已选择本次 PCD 和配套切片；启动定位导航后，请重新设置实际位置与朝向并核对点云。此操作不启动行驶。");
    updateControls();
  });
  connect(manage_, &QCheckBox::toggled, this, [this] { updateControls(); });
  runtimePageLayout->addWidget(runtimeFooter); controlTabs_->addTab(runtimePage, "运行");

  auto *nav = new QWidget; auto *navLayout = pageLayout(nav);
  hint("在地图上拖出目标朝向，或输入坐标。", navLayout);
  auto *target = new QGroupBox("目标位姿 · map"); auto *form = new QFormLayout(target);
  goalX_ = number(nav, "goalX", -100000, 100000, 0, 3); goalY_ = number(nav, "goalY", -100000, 100000, 0, 3); goalYaw_ = number(nav, "goalYaw", -180, 180, 0, 1);
  form->addRow("X (m)", goalX_); form->addRow("Y (m)", goalY_); form->addRow("朝向 (°)", goalYaw_); navLayout->addWidget(target);
  sendGoalButton_ = button("发送导航目标", navLayout, "sendGoal", true); sendGoalButton_->setProperty("motion", true); writeButtons_ << sendGoalButton_;
  connect(sendGoalButton_, &QPushButton::clicked, this, [this] { sendGoal(goalX_->value(), goalY_->value(), goalYaw_->value() * pi / 180.); });
  auto *relocate = button("设置初始位姿", navLayout, "setInitialPose"); writeButtons_ << relocate;
  connect(relocate, &QPushButton::clicked, this, [this] { sendInitialPose(goalX_->value(), goalY_->value(), goalYaw_->value() * pi / 180.); });
  navigationHint_ = hint("请先启用操作", navLayout); navigationHint_->setObjectName("consoleNavigationHint");
  auto *openRoute = button("打开路线巡航", navLayout); connect(openRoute, &QPushButton::clicked, this, &MainWindow::openPatrolPage);
  navLayout->addStretch(); controlTabs_->addTab(scrollPane(nav), "导航");

  manualPage_ = new QWidget; auto *manual = pageLayout(manualPage_); manualPage_->installEventFilter(this);
  hint("按住方向按钮行驶，松开或离开此页停止。", manual);
  take_ = button("手动接管（先取消导航）", manual, "takeManual", true);
  connect(take_, &QPushButton::clicked, this, [this] {
    if (cancelPatrolRun(QStringLiteral("手动接管"))) log("已请求取消巡航；等待旧目标终止和车辆停稳后才允许手动行驶");
    controller_->takeManualControl();
  });
  unlock_ = button("解除急停（保持静止）", manual, "releaseStop"); connect(unlock_, &QPushButton::clicked, controller_, &LocalController::releaseStop);
  auto *limits = new QFormLayout;
  speed_ = number(manualPage_, "manualSpeed", .05, config_.maxLinear, .15); speed_->setSingleStep(.05);
  turn_ = number(manualPage_, "manualTurn", .05, config_.maxAngular, .25); turn_->setSingleStep(.05);
  limits->addRow("速度 (m/s)", speed_); limits->addRow("角速度 (rad/s)", turn_); manual->addLayout(limits);
  auto *directions = new QGridLayout; directions->setSpacing(6);
  const QStringList labels{"前进左转", "前进", "前进右转", "倒车左打轮", "倒车", "倒车右打轮"};
  for (int i = 0; i < labels.size(); ++i) {
    auto *b = new QPushButton(labels[i]); b->setObjectName("manualDirection" + QString::number(i)); b->setProperty("consoleRole", "direction"); b->setAutoRepeat(false);
    directions->addWidget(b, i / 3, i % 3); directionButtons_ << b;
    connect(b, &QPushButton::pressed, this, [this, i] { const double sign = i < 3 ? 1. : -1.; const double steer = i % 3 == 0 ? 1. : i % 3 == 2 ? -1. : 0.; controller_->move(sign * speed_->value(), sign * steer * turn_->value()); });
    connect(b, &QPushButton::released, controller_, &LocalController::stopManual);
  }
  manual->addLayout(directions); manual->addStretch(); controlTabs_->addTab(scrollPane(manualPage_), "手动");
  connect(controlTabs_, &QTabWidget::currentChanged, this, [this](int index) { if (index != 2) controller_->stopManual(); });
  controlDock_ = addDock("操作工作区", "consoleControlDock", control, ads::LeftDockWidgetArea);
  connect(controlDock_, &ads::CDockWidget::visibilityChanged, this, [this](bool visible) { if (!visible && controller_) controller_->stopManual(); });

  auto *monitor = new QWidget; monitor->setObjectName("consoleMonitor"); monitor->setMinimumWidth(294);
  auto *rightLayout = new QVBoxLayout(monitor); rightLayout->setContentsMargins(0, 0, 0, 0);
  rightTabs_ = new QTabWidget; rightTabs_->setObjectName("rightWorkspaceTabs"); rightLayout->addWidget(rightTabs_);
  cameraPanel_ = new CameraPanel(config_, ros_, offline_); connect(cameraPanel_, &CameraPanel::message, this, &MainWindow::log);
  deviceClient_ = new DeviceClient(config_, offline_, this);
  auto *deviceDialog = new QDialog(this); deviceWindow_ = deviceDialog;
  deviceDialog->setObjectName("deviceDutyWindow"); deviceDialog->setWindowTitle("设备执勤 · 气体检测与云台巡航");
  auto *deviceLayout = new QVBoxLayout(deviceDialog);
  deviceLayout->setContentsMargins(0, 0, 0, 0);
  devicePanel_ = new DevicePanel(deviceClient_, offline_); deviceLayout->addWidget(devicePanel_);
  deviceDialog->resize(640, 720);
  connect(deviceClient_, &DeviceClient::message, this, &MainWindow::log);
  connect(deviceClient_, &DeviceClient::cameraRequested, cameraPanel_, &CameraPanel::ensureCamera);
  connect(deviceClient_, &DeviceClient::ptzBlocked, cameraPanel_, &CameraPanel::setPatrolBlocked);
  connect(cameraPanel_, &CameraPanel::patrolStopRequested, deviceClient_, &DeviceClient::stopPtz);
  rightTabs_->addTab(scrollPane(cameraPanel_), "相机");
  auto *capturePage = new QWidget; auto *captureLayout = pageLayout(capturePage);
  picture_ = new QLabel("尚未抓拍\n连接相机后可抓拍预览"); picture_->setObjectName("consoleCapturePreview");
  picture_->setAlignment(Qt::AlignCenter); picture_->setWordWrap(true); picture_->setMinimumSize(220, 150);
  picture_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  picture_->installEventFilter(this);
  picture_->setStyleSheet("background:#202832; color:#dce1e8; border-radius:6px;"); captureLayout->addWidget(picture_, 1);
  ocr_ = new QLabel("仪表识别：尚未识别"); ocr_->setWordWrap(true); ocr_->setTextFormat(Qt::PlainText); ocr_->setTextInteractionFlags(Qt::TextSelectableByMouse); captureLayout->addWidget(ocr_);
  auto *capture = button("相机抓拍 · 预览", captureLayout, "consoleCapture", true); writeButtons_ << capture; capture->setProperty("capability", "camera");
  connect(capture, &QPushButton::clicked, this, [this] { if (allowed()) ros_->captureCamera(); });
  auto *businessCapture = button("业务抓拍 · 保存记录", captureLayout, "consoleBusinessCapture"); writeButtons_ << businessCapture;
  connect(businessCapture, &QPushButton::clicked, ros_, &RosAdapter::captureBusiness);
  auto *ocrButton = button("识别当前仪表画面", captureLayout, "consoleRecognize"); writeButtons_ << ocrButton; ocrButton->setProperty("capability", "ocr");
  connect(ocrButton, &QPushButton::clicked, this, [this] { if (allowed()) ros_->recognize(); });
  auto *visionGroup = new QGroupBox("独立视觉执勤"); auto *visionLayout = new QVBoxLayout(visionGroup);
  visionPanel_ = new VisionPanel(config_, offline_); visionLayout->addWidget(visionPanel_); captureLayout->insertWidget(0, visionGroup);
  connect(visionPanel_, &VisionPanel::cameraRequested, cameraPanel_, &CameraPanel::ensureCamera);
  connect(visionPanel_, &VisionPanel::message, this, &MainWindow::log);
  rightTabs_->addTab(scrollPane(capturePage), "抓拍识别");
  auto *webPage = new QWidget; auto *webLayout = pageLayout(webPage);
  auto *webInfo = new QGroupBox("网页端", webPage); auto *webInfoLayout = new QVBoxLayout(webInfo);
  auto *webHint = new QLabel("网页端为只读监控与任务入口；地图、点位、路线、返航点仍由本地桌面维护。", webInfo);
  webHint->setWordWrap(true); webInfoLayout->addWidget(webHint);
  auto *webAddress = new QLabel("浏览器访问：http://<机器人IP>:8888/", webInfo);
  webAddress->setTextInteractionFlags(Qt::TextSelectableByMouse); webInfoLayout->addWidget(webAddress);
  auto *webButtons = new QHBoxLayout;
  const QString webBase = config_.backendUrl;
  auto *monitorWeb = button("监控台", webButtons); monitorWeb->setEnabled(!offline_);
  connect(monitorWeb, &QPushButton::clicked, this, [webBase] { QDesktopServices::openUrl(QUrl(webBase + "/monitor.html")); });
  auto *tasksWeb = button("巡检任务", webButtons); tasksWeb->setEnabled(!offline_);
  connect(tasksWeb, &QPushButton::clicked, this, [webBase] { QDesktopServices::openUrl(QUrl(webBase + "/tasks.html")); });
  auto *recordsWeb = button("巡检记录", webButtons); recordsWeb->setEnabled(!offline_);
  connect(recordsWeb, &QPushButton::clicked, this, [webBase] { QDesktopServices::openUrl(QUrl(webBase + "/records.html")); });
  webInfoLayout->addLayout(webButtons);
  webLayout->addWidget(webInfo);
  webLayout->addStretch();
  rightTabs_->addTab(scrollPane(webPage), "网页端");

  monitorDock_ = addDock("相机与业务", "consoleMonitorDock", monitor, ads::RightDockWidgetArea);
  connect(rightTabs_, &QTabWidget::currentChanged, this, [this](int index) { if (index != 0 && cameraPanel_) cameraPanel_->stopInteraction(); });
  connect(monitorDock_, &ads::CDockWidget::visibilityChanged, this, [this](bool visible) { if (!visible && cameraPanel_) cameraPanel_->stopInteraction(); });

  auto *status = new QWidget; status->setObjectName("consoleStatusStrip");
  auto *statusLayout = new QVBoxLayout(status); statusLayout->setContentsMargins(14, 8, 14, 8); statusLayout->setSpacing(4);
  auto *metrics = new QHBoxLayout; metrics->setSpacing(18);
  auto card = [metrics](const QString &name, int stretch) {
    auto *widget = new QWidget; auto *layout = new QVBoxLayout(widget); layout->setContentsMargins(0, 0, 0, 0); layout->setSpacing(4);
    hint(name, layout); metrics->addWidget(widget, stretch); return layout;
  };
  auto *speedCard = card("实际线速度", 1); speedValue_ = new QLabel("— m/s"); speedValue_->setObjectName("consoleSpeed"); speedValue_->setProperty("metric", true); speedCard->addWidget(speedValue_);
  auto *localizationCard = card("定位 / 角速度", 1); localizationState_ = new QLabel("位姿未知"); localizationState_->setObjectName("consoleLocalization"); localizationCard->addWidget(localizationState_);
  velocity_ = hint("ω — rad/s", localizationCard);
  auto *navCard = card("导航 / 路线巡航", 3); navState_ = new QLabel("等待状态"); navState_->setObjectName("consoleNavigationStatus"); navState_->setWordWrap(true); navState_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred); navCard->addWidget(navState_);
  auto *batteryCard = card("电池", 1); battery_ = new QProgressBar; battery_->setObjectName("consoleBattery"); battery_->setRange(0, 100); battery_->setValue(0); battery_->setFormat("—"); battery_->setMaximumWidth(130); batteryCard->addWidget(battery_);
  power_ = hint("— V", batteryCard);
  auto *gasReadout = new GasReadout(deviceClient_, offline_);
  metrics->addWidget(gasReadout, 2);
  connect(gasReadout, &GasReadout::openRequested, this, &MainWindow::openDeviceWindow);
  auto *safetyCard = card("急停 / 巡航联锁", 2); stopState_ = new QLabel("急停未知"); stopState_->setObjectName("consoleStopStatus"); safetyCard->addWidget(stopState_);
  guardState_ = hint("巡航联锁未启用", safetyCard); guardState_->setObjectName("consoleGuardStatus");
  auto *details = new QToolButton; details->setText("详情"); details->setObjectName("consoleStatusDetails"); details->setCheckable(true); metrics->addWidget(details);
  statusLayout->addLayout(metrics);
  statusDetails_ = new QWidget; auto *detailLayout = new QVBoxLayout(statusDetails_); detailLayout->setContentsMargins(0, 5, 0, 0);
  pose_ = hint("位姿未接入", detailLayout); pose_->setObjectName("poseStatus"); statusDetails_->hide(); statusLayout->addWidget(statusDetails_);
  connect(details, &QToolButton::toggled, this, [this](bool yes) { statusDetails_->setVisible(yes); adaptConsoleDensity(); }); shellLayout->addWidget(status);

  auto *logPanel = new QWidget; logPanel->setObjectName("consoleLogPanel"); auto *logLayout = new QVBoxLayout(logPanel); logLayout->setContentsMargins(10, 2, 8, 2); logLayout->setSpacing(0);
  auto *summaryRow = new QHBoxLayout;
  messageSummary_ = new ElidedLabel("本地控制台就绪"); messageSummary_->setObjectName("consoleMessageSummary"); messageSummary_->setTextFormat(Qt::PlainText);
  messageSummary_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred); summaryRow->addWidget(messageSummary_, 1);
  auto *dismiss = new QToolButton; dismiss->setText("清除提示"); summaryRow->addWidget(dismiss);
  connect(dismiss, &QToolButton::clicked, this, [this] { operationErrorKey_.clear(); messageSummary_->setText("消息与会话日志"); setConsoleTone(messageSummary_, "muted"); });
  logToggle_ = new QToolButton; logToggle_->setObjectName("consoleLogToggle"); logToggle_->setText("展开日志"); logToggle_->setCheckable(true); summaryRow->addWidget(logToggle_); logLayout->addLayout(summaryRow);
  logs_ = new QPlainTextEdit; logs_->setReadOnly(true); logs_->setMaximumBlockCount(500); logs_->setMinimumHeight(80); logs_->setMaximumHeight(150); logs_->hide(); logLayout->addWidget(logs_);
  connect(logToggle_, &QToolButton::toggled, this, [this](bool yes) { logs_->setVisible(yes); logToggle_->setText(yes ? "收起日志" : "展开日志"); }); shellLayout->addWidget(logPanel);
  auto *logAction = ui_->menuView->addAction("展开日志"); logAction->setCheckable(true);
  connect(logAction, &QAction::toggled, logToggle_, &QToolButton::setChecked); connect(logToggle_, &QToolButton::toggled, logAction, &QAction::setChecked);
  auto *reset = ui_->menuView->addAction("恢复默认布局"); reset->setObjectName("consoleResetLayout"); connect(reset, &QAction::triggered, this, &MainWindow::resetConsoleLayout);
  connect(profiles_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::applyProfile);
  QTimer::singleShot(0, this, &MainWindow::restoreConsoleLayout);
}

void MainWindow::refreshOdometrySource() {
  const bool fastlio = profile().backend == "fastlio";
  const bool selectable = fastlio && profile().navigation && !profile().mapping;
  const QString source = selectable ? fastlioNavigationOdomSource_
                                    : (fastlio ? QString("lio") : QString("wheel_imu"));
  // Fixed modes must not overwrite the user's FAST-LIO navigation choice.
  const QSignalBlocker block(odom_);
  odom_->setCurrentText(source);
  odom_->setEnabled(selectable && !process_->running() && !saver_->busy());
  QString description;
  if (!selectable) {
    description = fastlio ? "当前建图流程固定使用雷达 + IMU，无需切换。"
                          : "当前流程固定使用底盘 + IMU 融合，无需切换。";
  } else {
    if (source == "lio") description = "使用 FAST-LIO 的雷达 + IMU 运动估计。";
    else if (source == "wheel") description = "根据底盘反馈的速度、转角推算运动。";
    else description = "融合底盘运动数据与 IMU，需要两者反馈正常。";
    if (process_->running()) description += "流程运行中，停止后可修改。";
  }
  odomHint_->setText(description);
  odom_->setToolTip(description + "\n仅影响由本界面启动的流程；连接已有系统时不切换后台里程计。" +
                    (selectable ? QString("\n三种选项均保留 FAST-LIO + ICP 地图定位。") : QString()));
}

void MainWindow::sizeConsolePanes() {
  if (!mapDock_ || !mapDock_->dockAreaWidget()) return;
  const int width = docks_->width();
  docks_->setSplitterSizes(mapDock_->dockAreaWidget(), {272, qMax(240, width - 584), 304});
}

void MainWindow::restoreConsoleLayout() {
  sizeConsolePanes(); defaultDockState_ = docks_->saveState(layoutVersion);
  if (!offline_) {
    auto settings = layoutSettings();
    if (settings.value("version").toInt() == layoutVersion) {
      restoreGeometry(settings.value("geometry").toByteArray());
      const auto available = screen()->availableGeometry();
      resize(qMin(width(), available.width()), qMin(height(), available.height()));
      move(qBound(available.left(), x(), qMax(available.left(), available.right() - width() + 1)),
           qBound(available.top(), y(), qMax(available.top(), available.bottom() - height() + 1)));
      if (!docks_->restoreState(settings.value("docks").toByteArray(), layoutVersion)) docks_->restoreState(defaultDockState_, layoutVersion);
      controlTabs_->setCurrentIndex(qBound(0, settings.value("leftTab", 0).toInt(), 2));
      rightTabs_->setCurrentIndex(qBound(0, settings.value("rightTab", 0).toInt(), 2));
    }
  }
  compactLayout_ = width() < compactWidth;
  rightBeforeCompact_ = !monitorDock_->isClosed();
  if (!offline_) rightBeforeCompact_ = layoutSettings().value("monitorVisible", rightBeforeCompact_).toBool();
  monitorDock_->toggleView(!compactLayout_ && rightBeforeCompact_);
  layoutReady_ = true;
  adaptConsoleDensity();
}

void MainWindow::adaptConsoleDensity() {
  const bool dense = height() < 650;
  const auto direction = dense ? QBoxLayout::LeftToRight : QBoxLayout::TopToBottom;
  if (runtimeFooterLayout_->direction() != direction) runtimeFooterLayout_->setDirection(direction);
  runtimeFooterLayout_->setContentsMargins(12, dense ? 6 : 12, 12, dense ? 6 : 12);
  start_->setText(dense ? "启动" : "启动所选流程");
  start_->setToolTip("启动所选流程，不另起 RViz；需选择由界面管理");
  save_->setText(dense ? "存图" : "保存地图");
  stop_->setText(dense ? "停止" : "停止流程");
  sessionHint_->setVisible(!dense);
  velocity_->setVisible(!dense || statusDetails_->isVisible());
  power_->setVisible(!dense || statusDetails_->isVisible());
}

void MainWindow::saveConsoleLayout() {
  if (offline_ || !layoutReady_) return;
  auto settings = layoutSettings(); settings.setValue("version", layoutVersion);
  settings.setValue("geometry", saveGeometry()); settings.setValue("docks", docks_->saveState(layoutVersion));
  settings.setValue("monitorVisible", compactLayout_ ? rightBeforeCompact_ : !monitorDock_->isClosed());
  settings.setValue("leftTab", controlTabs_->currentIndex()); settings.setValue("rightTab", rightTabs_->currentIndex());
}

void MainWindow::resetConsoleLayout() {
  controller_->stopManual(); cameraPanel_->stopInteraction();
  docks_->restoreState(defaultDockState_, layoutVersion);
  controlDock_->toggleView(true); monitorDock_->toggleView(width() >= compactWidth);
  controlTabs_->setCurrentIndex(0); rightTabs_->setCurrentIndex(0); logToggle_->setChecked(false);
  auto *details = findChild<QToolButton *>("consoleStatusDetails"); if (details) details->setChecked(false);
  compactLayout_ = width() < compactWidth; rightBeforeCompact_ = true;
  QTimer::singleShot(0, this, &MainWindow::sizeConsolePanes);
}

void MainWindow::resizeEvent(QResizeEvent *event) {
  QMainWindow::resizeEvent(event);
  if (!layoutReady_) return;
  adaptConsoleDensity();
  const bool compact = width() < compactWidth;
  if (compact == compactLayout_) return;
  controller_->stopManual(); cameraPanel_->stopInteraction();
  if (compact) { rightBeforeCompact_ = !monitorDock_->isClosed(); monitorDock_->toggleView(false); }
  else if (rightBeforeCompact_) monitorDock_->toggleView(true);
  compactLayout_ = compact;
}

bool MainWindow::eventFilter(QObject *object, QEvent *event) {
  if (object == manualPage_ && event->type() == QEvent::Hide && controller_) controller_->stopManual();
  if (object == picture_ && event->type() == QEvent::Resize && !capturedPreview_.isNull())
    picture_->setPixmap(QPixmap::fromImage(capturedPreview_).scaled(picture_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  return QMainWindow::eventFilter(object, event);
}

void MainWindow::refreshStatusSummary() {
  if (!modeBadge_) return;
  modeBadge_->setText(offline_ ? "离线预览" : allowed() ? "操作已启用" : "观察模式");
  connection_->setText(offline_ ? "隔离环境 · 不连接实车" : process_->running() ? "本地 ROS2 · 本界面管理会话" : "本地 ROS2 · 连接已有系统");
  sessionHint_->setText(process_->running() ? "本界面会话运行中；停止只影响本次流程。" : "本地 ROS2 直连，无需网关。外部会话的算法来源需自行确认。");
  const bool speedValid = state_.speedValid && std::isfinite(state_.speed) && std::isfinite(state_.yawRate);
  speedValue_->setText(speedValid ? QString::number(state_.speed, 'f', 2) + " m/s" : "— m/s");
  velocity_->setText(speedValid ? QString("ω %1 rad/s").arg(state_.yawRate, 0, 'f', 2) : "ω — rad/s");
  speedValue_->setToolTip(speedValid ? "真实里程计反馈；倒车为负值" : "速度反馈未接入或已过期");
  setConsoleTone(speedValue_, speedValid ? "normal" : "muted");
  localizationState_->setText(state_.poseValid ? state_.frame + " 位姿有效" : "位姿未接入 / 过期");
  setConsoleTone(localizationState_, state_.poseValid ? "good" : "muted");
  pose_->setText(state_.poseValid ? QString("%1: x=%2 m  y=%3 m  朝向=%4°").arg(state_.frame).arg(state_.x, 0, 'f', 3).arg(state_.y, 0, 'f', 3).arg(state_.yaw * 180. / pi, 0, 'f', 1) : state_.frame + " 位姿未接入 / 已过期");
  localizationState_->setToolTip(pose_->text());
  if (patrolRunActive()) {
    const auto patrol = patrolClient_->status();
    navState_->setText(QString("巡航 · %1  圈 %2/%3  点 %4/%5").arg(PatrolClient::stateName(patrol.state)).arg(patrol.loopIndex).arg(patrol.totalLoops == 0 ? QStringLiteral("∞") : QString::number(patrol.totalLoops)).arg(patrol.stepIndex).arg(patrol.stepCount));
    navState_->setToolTip(patrol.message);
  } else {
    navState_->setText(!state_.navigationAvailable ? "Nav2 未启动 / 未就绪" : state_.navigationActive ? "Nav2 目标执行中 / 取消中" : "Nav2 · " + (lastNavigationEvent_.isEmpty() ? QStringLiteral("已就绪") : lastNavigationEvent_));
    navState_->setToolTip(navState_->text());
  }
  cancelButton_->setText(patrolRunActive() ? "取消巡航" : "取消导航");
  stopState_->setText(!state_.estopValid ? "急停状态未知 / 过期" : state_.estopped ? "急停保持中" : "急停已解除");
  setConsoleTone(stopState_, !state_.estopValid ? "muted" : state_.estopped ? "danger" : "good");
  const bool patrolEnabled = sessionPatrolInterlock_ || state_.patrolInterlockValid;
  guardState_->setText(!patrolEnabled ? "未检测到巡航联锁" : !state_.guardValid ? "巡航联锁反馈过期" : state_.patrolLocked ? "巡航速度锁定" : !state_.guardReady ? "巡航联锁未就绪" : "巡航联锁就绪");
  setConsoleTone(guardState_, !patrolEnabled || !state_.guardValid ? "muted" : state_.patrolLocked || !state_.guardReady ? "warning" : "good");
  QString reason;
  if (offline_) reason = "离线预览不发送车辆指令";
  else if (patrolEnabled) reason = "巡航联锁已启用，请通过路线巡航下发目标";
  else if (patrolRunActive()) reason = "请先取消整条巡航任务";
  else if (!allowed()) reason = "请先启用操作；观察模式仍可点选目标预览";
  else if (!profile().navigation) reason = "当前为建图显示方案，请选择定位导航";
  else if (!state_.poseValid || state_.frame != "map") reason = "等待有效 map 位姿；可先设置初始位姿";
  else if (!state_.navigationAvailable) reason = "等待 Nav2 就绪";
  else if (!state_.estopValid || state_.estopped) reason = "发送导航前请确认车辆停稳并解除急停";
  else reason = "地图点选或输入坐标后，点击发送导航目标";
  navigationHint_->setText(reason); sendGoalButton_->setToolTip(reason);
  save_->setToolTip(profile().mapping ? "保存建图状态及导航 YAML/PGM；需先启用操作" : "当前为定位导航模式，保存地图在建图模式可用");
}

void MainWindow::showOperationResult(const QString &operation, bool success, const QString &text) {
  if (!success) operationErrorKey_ = operation;
  else if (!operationErrorKey_.isEmpty() && operationErrorKey_ != operation) return;
  else operationErrorKey_.clear();
  messageSummary_->setText(text); messageSummary_->setToolTip(text);
  setConsoleTone(messageSummary_, success ? "muted" : "danger");
}

void MainWindow::openDeviceWindow() {
  const auto available = screen()->availableGeometry().size();
  deviceWindow_->resize(qMin(640, available.width() - 40), qMin(760, available.height() - 60));
  deviceWindow_->show(); deviceWindow_->raise(); deviceWindow_->activateWindow();
}
