#include "patrol/patrol_page.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;

QTableWidget *table(const QStringList &headers, QWidget *parent) {
  auto *view = new QTableWidget(0, headers.size(), parent);
  view->setHorizontalHeaderLabels(headers);
  view->setSelectionBehavior(QAbstractItemView::SelectRows);
  view->setSelectionMode(QAbstractItemView::SingleSelection);
  view->setEditTriggers(QAbstractItemView::NoEditTriggers);
  view->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  view->verticalHeader()->setVisible(false);
  return view;
}

QPushButton *pageButton(const QString &text, QWidget *parent, const QString &name) {
  auto *button = new QPushButton(text, parent);
  button->setObjectName(name);
  return button;
}

QTableWidgetItem *readOnlyItem(const QString &text) {
  auto *item = new QTableWidgetItem(text);
  item->setFlags(item->flags() & ~Qt::ItemIsEditable);
  return item;
}
}  // namespace

PatrolPage::PatrolPage(PatrolClient *client, const QString &initialMapYaml,
                       const QString &backend, const QString &localizationFile,
                       const QString &profileName, bool offline,
                       std::function<bool(QString *)> startPermission, QWidget *parent)
    : QWidget(parent), client_(client), mapYaml_(initialMapYaml), backend_(backend),
      localizationFile_(localizationFile), profileName_(profileName), offline_(offline),
      startPermission_(std::move(startPermission)) {
  setObjectName(QStringLiteral("patrolPage"));
  buildUi();
  connect(client_, &PatrolClient::mapsChanged, this, &PatrolPage::onMapsChanged);
  connect(client_, &PatrolClient::pointsChanged, this, &PatrolPage::onPointsChanged);
  connect(client_, &PatrolClient::routesChanged, this, &PatrolPage::onRoutesChanged);
  connect(client_, &PatrolClient::runsChanged, this, &PatrolPage::onRunsChanged);
  connect(client_, &PatrolClient::statusChanged, this, &PatrolPage::onStatusChanged);
  connect(client_, &PatrolClient::operationFinished, this, &PatrolPage::showOperation);
  connect(client_, &PatrolClient::errorOccurred, this, &PatrolPage::showError);
  client_->refreshMaps();
  auto *permissionTimer = new QTimer(this);
  permissionTimer->setInterval(500);
  connect(permissionTimer, &QTimer::timeout, this, &PatrolPage::updateActionStates);
  permissionTimer->start();
  updateActionStates();
}

void PatrolPage::setResumePermission(std::function<bool(QString *)> permission) {
  resumePermission_ = std::move(permission);
  updateActionStates();
}

void PatrolPage::buildUi() {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(8);

  auto *mapGroup = new QGroupBox(QStringLiteral("本地巡航地图绑定"), this);
  auto *mapLayout = new QVBoxLayout(mapGroup);
  auto *mapRow = new QHBoxLayout;
  serviceLabel_ = new QLabel(client_->available()
                                 ? (offline_ ? QStringLiteral("本地巡航服务：离线模式（仅编辑）")
                                             : QStringLiteral("本地巡航服务：接口已构建"))
                                 : QStringLiteral("本地巡航服务：不可用"), mapGroup);
  serviceLabel_->setObjectName(QStringLiteral("patrolServiceStatus"));
  mapRow->addWidget(serviceLabel_);
  mapRow->addWidget(new QLabel(QStringLiteral("当前模式：") + profileName_, mapGroup));
  mapRow->addStretch();
  refreshAllButton_ = pageButton(QStringLiteral("刷新"), mapGroup, QStringLiteral("patrolRefreshAll"));
  mapRow->addWidget(refreshAllButton_);
  mapLayout->addLayout(mapRow);

  auto *yamlRow = new QHBoxLayout;
  yamlRow->addWidget(new QLabel(QStringLiteral("导航 YAML"), mapGroup));
  mapYamlEdit_ = new QLineEdit(mapYaml_, mapGroup);
  mapYamlEdit_->setObjectName(QStringLiteral("patrolMapYaml"));
  mapYamlEdit_->setMinimumWidth(360);
  yamlRow->addWidget(mapYamlEdit_, 1);
  registerMapButton_ = pageButton(QStringLiteral("登记当前地图"), mapGroup,
                                  QStringLiteral("patrolRegisterMap"));
  yamlRow->addWidget(registerMapButton_);
  mapLayout->addLayout(yamlRow);

  auto *activeRow = new QHBoxLayout;
  activeRow->addWidget(new QLabel(QStringLiteral("已登记地图"), mapGroup));
  mapsCombo_ = new QComboBox(mapGroup);
  mapsCombo_->setObjectName(QStringLiteral("patrolMaps"));
  mapsCombo_->setMinimumWidth(260);
  activeRow->addWidget(mapsCombo_, 1);
  setActiveButton_ = pageButton(QStringLiteral("设为活跃地图"), mapGroup,
                                QStringLiteral("patrolSetActiveMap"));
  activeRow->addWidget(setActiveButton_);
  activeMapLabel_ = new QLabel(QStringLiteral("活跃地图：未设置"), mapGroup);
  activeMapLabel_->setObjectName(QStringLiteral("patrolActiveMap"));
  activeRow->addWidget(activeMapLabel_);
  mapLayout->addLayout(activeRow);

  messageLabel_ = new QLabel(QStringLiteral("路线巡航只调用本地 ROS2 服务，不依赖 HTTP 后端或网关"), mapGroup);
  messageLabel_->setObjectName(QStringLiteral("patrolMessage"));
  messageLabel_->setWordWrap(true);
  mapLayout->addWidget(messageLabel_);
  root->addWidget(mapGroup);

  tabs_ = new QTabWidget(this);
  tabs_->setObjectName(QStringLiteral("patrolTabs"));

  // Points tab.
  auto *pointsTab = new QWidget(tabs_);
  auto *pointsLayout = new QHBoxLayout(pointsTab);
  pointsTable_ = table({QStringLiteral("名称"), QStringLiteral("X"), QStringLiteral("Y"),
                        QStringLiteral("朝向°"), QStringLiteral("修订")}, pointsTab);
  pointsTable_->setObjectName(QStringLiteral("patrolPointsTable"));
  pointsLayout->addWidget(pointsTable_, 1);
  auto *pointForm = new QGroupBox(QStringLiteral("点位"), pointsTab);
  auto *pointFormLayout = new QVBoxLayout(pointForm);
  auto *pointFields = new QFormLayout;
  pointNameEdit_ = new QLineEdit(pointForm);
  pointNameEdit_->setObjectName(QStringLiteral("patrolPointName"));
  pointNameEdit_->setPlaceholderText(QStringLiteral("点位名称"));
  pointX_ = new QDoubleSpinBox(pointForm); pointX_->setRange(-100000, 100000); pointX_->setDecimals(3);
  pointY_ = new QDoubleSpinBox(pointForm); pointY_->setRange(-100000, 100000); pointY_->setDecimals(3);
  pointYaw_ = new QDoubleSpinBox(pointForm); pointYaw_->setRange(-180, 180); pointYaw_->setDecimals(2);
  pointFields->addRow(QStringLiteral("名称"), pointNameEdit_);
  pointFields->addRow(QStringLiteral("X (m)"), pointX_);
  pointFields->addRow(QStringLiteral("Y (m)"), pointY_);
  pointFields->addRow(QStringLiteral("朝向 (°)"), pointYaw_);
  pointFormLayout->addLayout(pointFields);
  saveCurrentPointButton_ = pageButton(QStringLiteral("保存当前位置为导航点"), pointForm,
                                       QStringLiteral("patrolSaveCurrentPoint"));
  upsertPointButton_ = pageButton(QStringLiteral("新增 / 更新点位"), pointForm,
                                  QStringLiteral("patrolUpsertPoint"));
  deletePointButton_ = pageButton(QStringLiteral("删除所选点位"), pointForm,
                                  QStringLiteral("patrolDeletePoint"));
  pointFormLayout->addWidget(saveCurrentPointButton_);
  pointFormLayout->addWidget(upsertPointButton_);
  pointFormLayout->addWidget(deletePointButton_);
  auto *pointHint = new QLabel(QStringLiteral("“保存当前位置”由 robot_patrol 读取停稳后的 map 位姿；"
                                               "手动新增只用于演示或外部导入。"), pointForm);
  pointHint->setWordWrap(true);
  pointFormLayout->addWidget(pointHint);
  pointFormLayout->addStretch();
  pointsLayout->addWidget(pointForm, 0);
  tabs_->addTab(pointsTab, QStringLiteral("点位"));

  // Routes tab.
  auto *routesTab = new QWidget(tabs_);
  auto *routesLayout = new QHBoxLayout(routesTab);
  auto *routeListPanel = new QWidget(routesTab);
  auto *routeListLayout = new QVBoxLayout(routeListPanel);
  routeListLayout->setContentsMargins(0, 0, 0, 0);
  routesTable_ = table({QStringLiteral("路线"), QStringLiteral("步骤"), QStringLiteral("修订")},
                       routeListPanel);
  routesTable_->setObjectName(QStringLiteral("patrolRoutesTable"));
  routeListLayout->addWidget(routesTable_, 1);
  auto *routeListButtons = new QHBoxLayout;
  newRouteButton_ = pageButton(QStringLiteral("新建路线"), routeListPanel, QStringLiteral("patrolNewRoute"));
  deleteRouteButton_ = pageButton(QStringLiteral("删除路线"), routeListPanel, QStringLiteral("patrolDeleteRoute"));
  routeListButtons->addWidget(newRouteButton_);
  routeListButtons->addWidget(deleteRouteButton_);
  routeListLayout->addLayout(routeListButtons);
  routesLayout->addWidget(routeListPanel, 1);

  auto *routeEditor = new QWidget(routesTab);
  auto *routeEditorLayout = new QVBoxLayout(routeEditor);
  routeEditorLayout->setContentsMargins(0, 0, 0, 0);
  auto *routeNameRow = new QHBoxLayout;
  routeNameRow->addWidget(new QLabel(QStringLiteral("路线名称"), routeEditor));
  routeNameEdit_ = new QLineEdit(routeEditor);
  routeNameEdit_->setObjectName(QStringLiteral("patrolRouteName"));
  routeNameRow->addWidget(routeNameEdit_, 1);
  routeEditorLayout->addLayout(routeNameRow);
  auto *addStepRow = new QHBoxLayout;
  pointCombo_ = new QComboBox(routeEditor);
  pointCombo_->setObjectName(QStringLiteral("patrolPointCombo"));
  pointCombo_->setMinimumWidth(180);
  addStepDwell_ = new QDoubleSpinBox(routeEditor);
  addStepDwell_->setObjectName(QStringLiteral("patrolAddStepDwell"));
  addStepDwell_->setRange(0, 86400); addStepDwell_->setDecimals(1);
  addStepDwell_->setSuffix(QStringLiteral(" s"));
  addStepButton_ = pageButton(QStringLiteral("添加点位"), routeEditor, QStringLiteral("patrolAddStep"));
  addStepRow->addWidget(new QLabel(QStringLiteral("加入路线"), routeEditor));
  addStepRow->addWidget(pointCombo_, 1);
  addStepRow->addWidget(addStepDwell_);
  addStepRow->addWidget(addStepButton_);
  routeEditorLayout->addLayout(addStepRow);
  stepsTable_ = table({QStringLiteral("点位"), QStringLiteral("X"), QStringLiteral("Y"),
                       QStringLiteral("停留(s)")}, routeEditor);
  stepsTable_->setObjectName(QStringLiteral("patrolStepsTable"));
  routeEditorLayout->addWidget(stepsTable_, 1);
  auto *stepButtons = new QHBoxLayout;
  stepDwell_ = new QDoubleSpinBox(routeEditor);
  stepDwell_->setObjectName(QStringLiteral("patrolStepDwell"));
  stepDwell_->setRange(0, 86400); stepDwell_->setDecimals(1);
  stepDwell_->setSuffix(QStringLiteral(" s"));
  updateDwellButton_ = pageButton(QStringLiteral("更新停留"), routeEditor, QStringLiteral("patrolUpdateDwell"));
  moveUpButton_ = pageButton(QStringLiteral("上移"), routeEditor, QStringLiteral("patrolStepUp"));
  moveDownButton_ = pageButton(QStringLiteral("下移"), routeEditor, QStringLiteral("patrolStepDown"));
  removeStepButton_ = pageButton(QStringLiteral("移除步骤"), routeEditor, QStringLiteral("patrolRemoveStep"));
  stepButtons->addWidget(new QLabel(QStringLiteral("停留"), routeEditor));
  stepButtons->addWidget(stepDwell_);
  stepButtons->addWidget(updateDwellButton_);
  stepButtons->addWidget(moveUpButton_);
  stepButtons->addWidget(moveDownButton_);
  stepButtons->addWidget(removeStepButton_);
  stepButtons->addStretch();
  routeEditorLayout->addLayout(stepButtons);
  auto *saveRouteRow = new QHBoxLayout;
  saveRouteButton_ = pageButton(QStringLiteral("保存路线"), routeEditor, QStringLiteral("patrolSaveRoute"));
  saveRouteButton_->setProperty("primary", true);
  saveRouteRow->addWidget(saveRouteButton_);
  saveRouteRow->addStretch();
  routeEditorLayout->addLayout(saveRouteRow);

  auto *runGroup = new QGroupBox(QStringLiteral("执行"), routeEditor);
  auto *runLayout = new QVBoxLayout(runGroup);
  auto *runRow = new QHBoxLayout;
  runRow->addWidget(new QLabel(QStringLiteral("循环"), runGroup));
  loopCount_ = new QSpinBox(runGroup);
  loopCount_->setObjectName(QStringLiteral("patrolLoopCount"));
  loopCount_->setRange(0, 100000);
  loopCount_->setValue(1);
  loopCount_->setSpecialValueText(QStringLiteral("持续循环"));
  runRow->addWidget(loopCount_);
  startButton_ = pageButton(QStringLiteral("开始"), runGroup, QStringLiteral("patrolStart"));
  pauseButton_ = pageButton(QStringLiteral("暂停"), runGroup, QStringLiteral("patrolPause"));
  resumeButton_ = pageButton(QStringLiteral("继续"), runGroup, QStringLiteral("patrolResume"));
  cancelButton_ = pageButton(QStringLiteral("取消"), runGroup, QStringLiteral("patrolCancel"));
  runRow->addWidget(startButton_);
  runRow->addWidget(pauseButton_);
  runRow->addWidget(resumeButton_);
  runRow->addWidget(cancelButton_);
  runRow->addStretch();
  runLayout->addLayout(runRow);
  statusLabel_ = new QLabel(QStringLiteral("状态：空闲"), runGroup);
  statusLabel_->setObjectName(QStringLiteral("patrolStatusLabel"));
  progressLabel_ = new QLabel(QStringLiteral("尚未执行路线"), runGroup);
  progressLabel_->setObjectName(QStringLiteral("patrolProgressLabel"));
  statusLabel_->setWordWrap(true);
  progressLabel_->setWordWrap(true);
  runLayout->addWidget(statusLabel_);
  runLayout->addWidget(progressLabel_);
  routeEditorLayout->addWidget(runGroup);
  routesLayout->addWidget(routeEditor, 2);
  tabs_->addTab(routesTab, QStringLiteral("路线 / 执行"));

  // Runs tab.
  auto *runsTab = new QWidget(tabs_);
  auto *runsLayout = new QVBoxLayout(runsTab);
  auto *runsTop = new QHBoxLayout;
  runsTop->addWidget(new QLabel(QStringLiteral("最近任务记录"), runsTab));
  runsTop->addStretch();
  refreshRunsButton_ = pageButton(QStringLiteral("刷新记录"), runsTab, QStringLiteral("patrolRefreshRuns"));
  runsTop->addWidget(refreshRunsButton_);
  runsLayout->addLayout(runsTop);
  runsTable_ = table({QStringLiteral("Run"), QStringLiteral("路线"), QStringLiteral("状态"),
                      QStringLiteral("圈数"), QStringLiteral("步骤"), QStringLiteral("结果"),
                      QStringLiteral("说明")}, runsTab);
  runsTable_->setObjectName(QStringLiteral("patrolRunsTable"));
  runsLayout->addWidget(runsTable_, 1);
  tabs_->addTab(runsTab, QStringLiteral("执行记录"));
  root->addWidget(tabs_, 1);

  connect(registerMapButton_, &QPushButton::clicked, this, &PatrolPage::registerCurrentMap);
  connect(setActiveButton_, &QPushButton::clicked, this, &PatrolPage::setActiveMap);
  connect(refreshAllButton_, &QPushButton::clicked, this, &PatrolPage::refreshAll);
  connect(pointsTable_, &QTableWidget::itemSelectionChanged, this, &PatrolPage::updatePointFormFromSelection);
  connect(saveCurrentPointButton_, &QPushButton::clicked, this, &PatrolPage::saveCurrentPoint);
  connect(upsertPointButton_, &QPushButton::clicked, this, &PatrolPage::upsertSelectedPoint);
  connect(deletePointButton_, &QPushButton::clicked, this, &PatrolPage::deleteSelectedPoint);
  connect(routesTable_, &QTableWidget::itemSelectionChanged, this, [this] {
    const int row = routesTable_->currentRow();
    if (row < 0 || !routesTable_->item(row, 0)) return;
    const QString routeId = routesTable_->item(row, 0)->data(Qt::UserRole).toString();
    if (routeDirty_ && routeId != currentRouteId_) {
      setMessage(QStringLiteral("当前路线有未保存修改；请先保存或新建路线"), true);
      return;
    }
    const auto route = routeById(routeId);
    if (!route.routeId.isEmpty()) loadRouteIntoEditor(route);
  });
  connect(newRouteButton_, &QPushButton::clicked, this, &PatrolPage::newRoute);
  connect(deleteRouteButton_, &QPushButton::clicked, this, &PatrolPage::deleteSelectedRoute);
  connect(addStepButton_, &QPushButton::clicked, this, &PatrolPage::addStepFromPoint);
  connect(removeStepButton_, &QPushButton::clicked, this, &PatrolPage::removeSelectedStep);
  connect(moveUpButton_, &QPushButton::clicked, this, &PatrolPage::moveStepUp);
  connect(moveDownButton_, &QPushButton::clicked, this, &PatrolPage::moveStepDown);
  connect(updateDwellButton_, &QPushButton::clicked, this, &PatrolPage::updateStepDwell);
  connect(saveRouteButton_, &QPushButton::clicked, this, &PatrolPage::saveRoute);
  connect(startButton_, &QPushButton::clicked, this, &PatrolPage::startRun);
  connect(pauseButton_, &QPushButton::clicked, this, &PatrolPage::pauseRun);
  connect(resumeButton_, &QPushButton::clicked, this, &PatrolPage::resumeRun);
  connect(cancelButton_, &QPushButton::clicked, this, &PatrolPage::cancelRun);
  connect(refreshRunsButton_, &QPushButton::clicked, this, [this] {
    client_->refreshRuns(activeMapId_);
  });
  connect(routeNameEdit_, &QLineEdit::textChanged, this, [this] {
    if (!loadingRoute_) { routeDirty_ = true; updateActionStates(); }
  });
  connect(stepsTable_, &QTableWidget::itemSelectionChanged, this, [this] {
    const int row = stepsTable_->currentRow();
    if (row >= 0 && row < currentSteps_.size()) stepDwell_->setValue(currentSteps_[row].dwellSeconds);
  });
  if (!client_->available()) {
    tabs_->setEnabled(false);
    mapYamlEdit_->setEnabled(false);
    registerMapButton_->setEnabled(false);
    setActiveButton_->setEnabled(false);
  }
}

void PatrolPage::registerCurrentMap() {
  const QString yaml = mapYamlEdit_->text().trimmed();
  if (yaml.isEmpty()) { setMessage(QStringLiteral("请填写导航地图 YAML"), true); return; }
  client_->registerMap(yaml, backend_, localizationFile_, profileName_);
}

void PatrolPage::setActiveMap() {
  const int index = mapsCombo_->currentIndex();
  if (index < 0) { setMessage(QStringLiteral("请选择已登记地图"), true); return; }
  const QString mapId = mapsCombo_->itemData(index).toString();
  quint64 revision = 0;
  for (const auto &item : client_->maps()) if (item.mapId == mapId) revision = item.revision;
  if (!mapId.isEmpty()) client_->setActiveMap(mapId, revision);
}

void PatrolPage::refreshAll() {
  client_->refreshMaps();
  if (!activeMapId_.isEmpty()) {
    client_->refreshPoints(activeMapId_);
    client_->refreshRoutes(activeMapId_);
    client_->refreshRuns(activeMapId_);
  }
  setMessage(QStringLiteral("已请求刷新本地巡航数据"), false);
}

void PatrolPage::onMapsChanged() {
  const QString previousActive = activeMapId_;
  activeMapId_ = client_->activeMapId();
  populateMaps();
  if (activeMapId_ != previousActive) {
    currentRouteId_.clear();
    currentRouteRevision_ = 0;
    currentSteps_.clear();
    clearRouteEditor();
  }
  if (!activeMapId_.isEmpty()) {
    client_->refreshPoints(activeMapId_);
    client_->refreshRoutes(activeMapId_);
    client_->refreshRuns(activeMapId_);
  } else {
    pointsTable_->setRowCount(0);
    routesTable_->setRowCount(0);
    runsTable_->setRowCount(0);
    pointCombo_->clear();
  }
  updateActionStates();
}

void PatrolPage::populateMaps() {
  const QString selectedId = mapsCombo_->currentData().toString();
  mapsCombo_->clear();
  for (const auto &item : client_->maps()) {
    mapsCombo_->addItem(item.name + QStringLiteral("  [") + item.backend + QStringLiteral("]"),
                        item.mapId);
  }
  const QString desired = activeMapId_.isEmpty() ? selectedId : activeMapId_;
  const int index = mapsCombo_->findData(desired);
  if (index >= 0) mapsCombo_->setCurrentIndex(index);
  const auto map = activeMap();
  activeMapLabel_->setText(map.mapId.isEmpty()
                               ? QStringLiteral("活跃地图：未设置")
                               : QStringLiteral("活跃地图：%1 rev=%2").arg(map.name).arg(map.revision));
}

void PatrolPage::onPointsChanged() {
  populatePoints();
  updateActionStates();
}

void PatrolPage::onRoutesChanged() {
  populateRoutes();
  if (!currentRouteId_.isEmpty()) {
    const auto route = routeById(currentRouteId_);
    if (!route.routeId.isEmpty() && !routeDirty_) loadRouteIntoEditor(route);
  }
  updateActionStates();
}

void PatrolPage::onRunsChanged() { populateRuns(); }

void PatrolPage::populatePoints() {
  const int selectedRow = pointsTable_->currentRow();
  pointsTable_->setRowCount(0);
  const auto list = client_->points();
  for (const auto &point : list) {
    const int row = pointsTable_->rowCount();
    pointsTable_->insertRow(row);
    auto *name = readOnlyItem(point.name);
    name->setData(Qt::UserRole, point.pointId);
    name->setData(Qt::UserRole + 1, QVariant::fromValue(point.revision));
    pointsTable_->setItem(row, 0, name);
    pointsTable_->setItem(row, 1, readOnlyItem(QString::number(point.x, 'f', 3)));
    pointsTable_->setItem(row, 2, readOnlyItem(QString::number(point.y, 'f', 3)));
    pointsTable_->setItem(row, 3, readOnlyItem(QString::number(point.yaw * 180.0 / kPi, 'f', 2)));
    pointsTable_->setItem(row, 4, readOnlyItem(QString::number(point.revision)));
  }
  if (selectedRow >= 0 && selectedRow < pointsTable_->rowCount()) pointsTable_->selectRow(selectedRow);
  populateAvailablePoints();
}

void PatrolPage::populateAvailablePoints() {
  const QString selected = pointCombo_->currentData().toString();
  pointCombo_->clear();
  for (const auto &point : client_->points()) pointCombo_->addItem(point.name, point.pointId);
  const int index = pointCombo_->findData(selected);
  if (index >= 0) pointCombo_->setCurrentIndex(index);
}

void PatrolPage::populateRoutes() {
  const int selectedRow = routesTable_->currentRow();
  routesTable_->setRowCount(0);
  for (const auto &route : client_->routes()) {
    const int row = routesTable_->rowCount();
    routesTable_->insertRow(row);
    auto *name = readOnlyItem(route.name);
    name->setData(Qt::UserRole, route.routeId);
    name->setData(Qt::UserRole + 1, QVariant::fromValue(route.revision));
    routesTable_->setItem(row, 0, name);
    routesTable_->setItem(row, 1, readOnlyItem(QString::number(route.steps.size())));
    routesTable_->setItem(row, 2, readOnlyItem(QString::number(route.revision)));
  }
  for (int row = 0; row < routesTable_->rowCount(); ++row) {
    if (routesTable_->item(row, 0)->data(Qt::UserRole).toString() == currentRouteId_) {
      routesTable_->selectRow(row);
      break;
    }
  }
  if (routesTable_->currentRow() < 0 && selectedRow >= 0 && selectedRow < routesTable_->rowCount())
    routesTable_->selectRow(selectedRow);
}

void PatrolPage::populateRuns() {
  runsTable_->setRowCount(0);
  for (const auto &run : client_->runs()) {
    const int row = runsTable_->rowCount();
    runsTable_->insertRow(row);
    runsTable_->setItem(row, 0, readOnlyItem(run.runId));
    runsTable_->setItem(row, 1, readOnlyItem(run.routeId));
    runsTable_->setItem(row, 2, readOnlyItem(PatrolClient::stateName(run.state)));
    runsTable_->setItem(row, 3, readOnlyItem(QString::number(run.completedLoops)));
    runsTable_->setItem(row, 4, readOnlyItem(QString::number(run.completedSteps)));
    runsTable_->setItem(row, 5, readOnlyItem(run.success ? QStringLiteral("成功") : QStringLiteral("未成功")));
    runsTable_->setItem(row, 6, readOnlyItem(run.message));
  }
}

void PatrolPage::loadRouteIntoEditor(const PatrolRouteInfo &route) {
  loadingRoute_ = true;
  currentRouteId_ = route.routeId;
  currentRouteRevision_ = route.revision;
  routeNameEdit_->setText(route.name);
  currentSteps_ = route.steps;
  syncStepTable();
  loadingRoute_ = false;
  routeDirty_ = false;
  updateActionStates();
}

void PatrolPage::clearRouteEditor() {
  loadingRoute_ = true;
  currentRouteId_.clear();
  currentRouteRevision_ = 0;
  currentSteps_.clear();
  routeNameEdit_->setText(QStringLiteral("新路线"));
  syncStepTable();
  loadingRoute_ = false;
  routeDirty_ = false;
  updateActionStates();
}

void PatrolPage::syncStepTable() {
  stepsTable_->setRowCount(0);
  for (const auto &step : currentSteps_) {
    const int row = stepsTable_->rowCount();
    stepsTable_->insertRow(row);
    stepsTable_->setItem(row, 0, readOnlyItem(step.pointName.isEmpty() ? step.pointId : step.pointName));
    stepsTable_->setItem(row, 1, readOnlyItem(QString::number(step.x, 'f', 3)));
    stepsTable_->setItem(row, 2, readOnlyItem(QString::number(step.y, 'f', 3)));
    stepsTable_->setItem(row, 3, readOnlyItem(QString::number(step.dwellSeconds, 'f', 1)));
  }
}

void PatrolPage::updatePointFormFromSelection() {
  const int row = pointsTable_->currentRow();
  if (row < 0 || !pointsTable_->item(row, 0)) return;
  const QString pointId = pointsTable_->item(row, 0)->data(Qt::UserRole).toString();
  for (const auto &point : client_->points()) {
    if (point.pointId != pointId) continue;
    pointNameEdit_->setText(point.name);
    pointX_->setValue(point.x);
    pointY_->setValue(point.y);
    pointYaw_->setValue(point.yaw * 180.0 / kPi);
    return;
  }
}

void PatrolPage::saveCurrentPoint() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先登记并设置活跃地图"), true); return; }
  QString name = pointNameEdit_->text().trimmed();
  if (name.isEmpty()) name = QStringLiteral("当前点");
  client_->saveCurrentPoint(activeMapId_, name);
}

void PatrolPage::upsertSelectedPoint() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先设置活跃地图"), true); return; }
  const int row = pointsTable_->currentRow();
  const QString pointId = row >= 0 && pointsTable_->item(row, 0)
                              ? pointsTable_->item(row, 0)->data(Qt::UserRole).toString()
                              : QString();
  const quint64 revision = row >= 0 && pointsTable_->item(row, 0)
                               ? pointsTable_->item(row, 0)->data(Qt::UserRole + 1).toULongLong()
                               : 0;
  const QString name = pointNameEdit_->text().trimmed();
  if (name.isEmpty()) { setMessage(QStringLiteral("请填写点位名称"), true); return; }
  client_->upsertPoint(pointId, activeMapId_, name, pointX_->value(), pointY_->value(),
                       pointYaw_->value() * kPi / 180.0, revision);
}

void PatrolPage::deleteSelectedPoint() {
  const int row = pointsTable_->currentRow();
  if (row < 0 || !pointsTable_->item(row, 0)) { setMessage(QStringLiteral("请选择要删除的点位"), true); return; }
  client_->deletePoint(pointsTable_->item(row, 0)->data(Qt::UserRole).toString(),
                       pointsTable_->item(row, 0)->data(Qt::UserRole + 1).toULongLong());
}

void PatrolPage::addStepFromPoint() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先设置活跃地图"), true); return; }
  const QString pointId = pointCombo_->currentData().toString();
  if (pointId.isEmpty()) { setMessage(QStringLiteral("没有可加入路线的点位"), true); return; }
  for (const auto &point : client_->points()) {
    if (point.pointId != pointId) continue;
    PatrolRouteStepInfo step;
    step.pointId = point.pointId; step.pointName = point.name;
    step.x = point.x; step.y = point.y; step.yaw = point.yaw;
    step.dwellSeconds = addStepDwell_->value();
    currentSteps_.push_back(step);
    routeDirty_ = true;
    syncStepTable();
    updateActionStates();
    return;
  }
}

void PatrolPage::removeSelectedStep() {
  const int row = stepsTable_->currentRow();
  if (row < 0 || row >= currentSteps_.size()) return;
  currentSteps_.removeAt(row);
  routeDirty_ = true;
  syncStepTable();
  updateActionStates();
}

void PatrolPage::moveStepUp() {
  const int row = stepsTable_->currentRow();
  if (row <= 0 || row >= currentSteps_.size()) return;
  currentSteps_.swapItemsAt(row, row - 1);
  routeDirty_ = true;
  syncStepTable();
  stepsTable_->selectRow(row - 1);
  updateActionStates();
}

void PatrolPage::moveStepDown() {
  const int row = stepsTable_->currentRow();
  if (row < 0 || row + 1 >= currentSteps_.size()) return;
  currentSteps_.swapItemsAt(row, row + 1);
  routeDirty_ = true;
  syncStepTable();
  stepsTable_->selectRow(row + 1);
  updateActionStates();
}

void PatrolPage::updateStepDwell() {
  const int row = stepsTable_->currentRow();
  if (row < 0 || row >= currentSteps_.size()) { setMessage(QStringLiteral("请选择路线步骤"), true); return; }
  currentSteps_[row].dwellSeconds = stepDwell_->value();
  routeDirty_ = true;
  syncStepTable();
  stepsTable_->selectRow(row);
  updateActionStates();
}

void PatrolPage::newRoute() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先设置活跃地图"), true); return; }
  clearRouteEditor();
  routesTable_->clearSelection();
  setMessage(QStringLiteral("已新建路线草稿，请添加点位后保存"), false);
}

void PatrolPage::saveRoute() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先设置活跃地图"), true); return; }
  const QString name = routeNameEdit_->text().trimmed();
  if (name.isEmpty()) { setMessage(QStringLiteral("请填写路线名称"), true); return; }
  if (currentSteps_.isEmpty()) { setMessage(QStringLiteral("路线至少需要一个点位"), true); return; }
  client_->upsertRoute(currentRouteId_, activeMapId_, name, currentSteps_, currentRouteRevision_);
}

void PatrolPage::deleteSelectedRoute() {
  const int row = routesTable_->currentRow();
  if (row < 0 || !routesTable_->item(row, 0)) { setMessage(QStringLiteral("请选择要删除的路线"), true); return; }
  client_->deleteRoute(routesTable_->item(row, 0)->data(Qt::UserRole).toString(),
                       routesTable_->item(row, 0)->data(Qt::UserRole + 1).toULongLong());
}

void PatrolPage::startRun() {
  if (activeMapId_.isEmpty() || currentRouteId_.isEmpty()) {
    setMessage(QStringLiteral("请选择已保存路线并设置活跃地图"), true);
    return;
  }
  if (routeDirty_) { setMessage(QStringLiteral("路线有未保存修改，请先保存"), true); return; }
  if (startPermission_) {
    QString reason;
    if (!startPermission_(&reason)) {
      setMessage(reason.isEmpty() ? QStringLiteral("当前不允许启动巡航") : reason, true);
      return;
    }
  }
  const auto map = activeMap();
  client_->runRoute(currentRouteId_, activeMapId_, currentRouteRevision_, map.revision,
                    static_cast<quint32>(loopCount_->value()));
}

void PatrolPage::pauseRun() {
  const auto status = client_->status();
  if (status.runId.isEmpty()) { setMessage(QStringLiteral("当前没有可暂停的巡航任务"), true); return; }
  client_->controlRun(1, status.runId);
}

void PatrolPage::resumeRun() {
  const auto status = client_->status();
  if (status.runId.isEmpty()) { setMessage(QStringLiteral("当前没有可继续的巡航任务"), true); return; }
  const auto &permission = resumePermission_ ? resumePermission_ : startPermission_;
  if (permission) {
    QString reason;
    if (!permission(&reason)) {
      setMessage(reason.isEmpty() ? QStringLiteral("当前不允许继续巡航") : reason, true);
      return;
    }
  }
  client_->controlRun(2, status.runId);
}

void PatrolPage::cancelRun() {
  const auto status = client_->status();
  if (status.runId.isEmpty()) { setMessage(QStringLiteral("当前没有可取消的巡航任务"), true); return; }
  client_->controlRun(3, status.runId);
}

void PatrolPage::onStatusChanged() {
  const auto status = client_->status();
  statusLabel_->setText(QStringLiteral("状态：%1%2")
                            .arg(PatrolClient::stateName(status.state),
                                 status.message.isEmpty() ? QString() : QStringLiteral(" · ") + status.message));
  progressLabel_->setText(QStringLiteral("Run：%1 · 路线：%2 · 圈 %3/%4 · 点 %5/%6 · 停留 %7 s")
                              .arg(status.runId.isEmpty() ? QStringLiteral("—") : status.runId,
                                   status.routeId.isEmpty() ? QStringLiteral("—") : status.routeId)
                              .arg(status.loopIndex).arg(status.totalLoops == 0 ? QStringLiteral("∞")
                                                                              : QString::number(status.totalLoops))
                              .arg(status.stepIndex).arg(status.stepCount)
                              .arg(QString::number(status.dwellRemaining, 'f', 1)));
  updateActionStates();
}

void PatrolPage::showOperation(const QString &operation, bool success, const QString &message) {
  setMessage(operation + (success ? QStringLiteral("：") : QStringLiteral("失败：")) + message, !success);
  if (success && operation.contains(QStringLiteral("保存路线"))) {
    routeDirty_ = false;
    client_->refreshRoutes(activeMapId_);
  }
  if (operation.contains(QStringLiteral("巡航任务"))) client_->refreshRuns(activeMapId_);
  updateActionStates();
}

void PatrolPage::showError(const QString &message) { setMessage(message, true); }

void PatrolPage::setMessage(const QString &message, bool error) {
  messageLabel_->setText(message);
  messageLabel_->setStyleSheet(error ? QStringLiteral("color:#b42318;font-weight:600;")
                                     : QStringLiteral("color:#475467;"));
}

QString PatrolPage::activeMapName() const { return activeMap().name; }

PatrolMapInfo PatrolPage::activeMap() const {
  for (const auto &item : client_->maps()) if (item.mapId == activeMapId_) return item;
  return {};
}

PatrolRouteInfo PatrolPage::routeById(const QString &routeId) const {
  for (const auto &route : client_->routes()) if (route.routeId == routeId) return route;
  return {};
}

void PatrolPage::updateActionStates() {
  const bool available = client_ && client_->available();
  const bool hasMap = !activeMapId_.isEmpty();
  const auto status = client_ ? client_->status() : PatrolStatusInfo();
  const bool activeRun = status.active;
  const bool paused = status.state == 7;
  registerMapButton_->setEnabled(available);
  setActiveButton_->setEnabled(available && mapsCombo_->currentIndex() >= 0);
  refreshAllButton_->setEnabled(available);
  saveCurrentPointButton_->setEnabled(available && hasMap);
  upsertPointButton_->setEnabled(available && hasMap);
  deletePointButton_->setEnabled(available && hasMap && pointsTable_->currentRow() >= 0);
  addStepButton_->setEnabled(available && hasMap);
  updateDwellButton_->setEnabled(available && stepsTable_->currentRow() >= 0);
  removeStepButton_->setEnabled(available && stepsTable_->currentRow() >= 0);
  moveUpButton_->setEnabled(available && stepsTable_->currentRow() > 0);
  moveDownButton_->setEnabled(available && stepsTable_->currentRow() >= 0 &&
                              stepsTable_->currentRow() + 1 < currentSteps_.size());
  newRouteButton_->setEnabled(available && hasMap);
  saveRouteButton_->setEnabled(available && hasMap && !currentSteps_.isEmpty());
  deleteRouteButton_->setEnabled(available && !currentRouteId_.isEmpty());
  QString startPermissionReason;
  const bool startPermitted = !startPermission_ ||
                              startPermission_(&startPermissionReason);
  const QString startPermissionTip = startPermitted ? QString() : startPermissionReason;
  startButton_->setEnabled(!offline_ && available && hasMap && !currentRouteId_.isEmpty() &&
                           !routeDirty_ && !activeRun && startPermitted);
  startButton_->setToolTip(startPermissionTip);

  const auto &resumePermission = resumePermission_ ? resumePermission_ : startPermission_;
  QString resumePermissionReason;
  const bool resumePermitted = !resumePermission ||
                               resumePermission(&resumePermissionReason);
  const QString resumePermissionTip = resumePermitted ? QString() : resumePermissionReason;
  pauseButton_->setEnabled(!offline_ && available && activeRun && !paused);
  resumeButton_->setEnabled(!offline_ && available && activeRun && paused &&
                            resumePermitted);
  resumeButton_->setToolTip(resumePermissionTip);
  cancelButton_->setEnabled(!offline_ && available && activeRun);
  pointCombo_->setEnabled(available && hasMap && pointCombo_->count() > 0);
  tabs_->setEnabled(available);
}
