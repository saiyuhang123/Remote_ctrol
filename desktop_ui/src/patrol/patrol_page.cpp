#include "patrol/patrol_page.h"
#include "patrol/inspection_target_page.h"
#include "patrol/report_viewer.h"
#include "patrol/patrol_ui_style.h"
#include "adapters/ros_adapter.h"

#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QBrush>
#include <QPen>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QStringList>
#include <QSplitter>
#include <QTableWidget>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QCloseEvent>
#include <QSignalBlocker>
#include <QPlainTextEdit>
#include <QDialog>
#include <QScrollArea>
#include <QDateTime>
#include <QDir>
#include <QUuid>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;

QTableWidget *table(const QStringList &headers, QWidget *parent) {
  auto *view = new QTableWidget(0, headers.size(), parent);
  view->setHorizontalHeaderLabels(headers);
  patrolui::configureTable(view);
  auto *empty = patrolui::hint(
      headers.front() == QStringLiteral("名称") ? QStringLiteral("暂无导航点\n在右侧填写名称，保存第一个导航点。")
      : headers.front() == QStringLiteral("路线") ? QStringLiteral("暂无路线\n点击“新建路线”开始编排。")
      : headers.front() == QStringLiteral("点位") ? QStringLiteral("尚未添加点位\n从上方选择导航点，按顺序加入路线。")
                                               : QStringLiteral("暂无执行记录\n执行巡航后可在这里查看结果。"), view->viewport());
  empty->setAlignment(Qt::AlignCenter);
  empty->setObjectName("patrolTableEmpty");
  empty->setAttribute(Qt::WA_TransparentForMouseEvents);
  auto *emptyLayout = new QVBoxLayout(view->viewport());
  emptyLayout->setContentsMargins(10, 10, 10, 10);
  emptyLayout->addWidget(empty);
  return view;
}

void updateEmptyTable(QTableWidget *view) {
  if (auto *empty = view->findChild<QLabel *>("patrolTableEmpty"))
    empty->setVisible(view->rowCount() == 0);
}

QPushButton *pageButton(const QString &text, QWidget *parent, const QString &name) {
  auto *button = new QPushButton(text, parent);
  button->setObjectName(name);
  button->setAutoDefault(false);
  return button;
}

QTableWidgetItem *readOnlyItem(const QString &text) {
  auto *item = new QTableWidgetItem(text);
  item->setFlags(item->flags() & ~Qt::ItemIsEditable);
  item->setToolTip(text);
  return item;
}
}  // namespace

PatrolPage::PatrolPage(PatrolClient *client, const QString &initialMapYaml,
                       const QString &backend, const QString &localizationFile,
                       const QString &profileName, bool offline,
                       std::function<bool(QString *)> startPermission, QWidget *parent,
                       RosAdapter *ros)
    : QWidget(parent), client_(client), ros_(ros), mapYaml_(initialMapYaml), backend_(backend),
      localizationFile_(localizationFile), profileName_(profileName), offline_(offline),
      startPermission_(std::move(startPermission)) {
  Q_INIT_RESOURCE(console_icons);
  setObjectName(QStringLiteral("patrolPage"));
  setAttribute(Qt::WA_StyledBackground);
  setStyleSheet(patrolui::styleSheet());
  buildUi();
  connect(client_, &PatrolClient::mapsChanged, this, &PatrolPage::onMapsChanged);
  connect(client_, &PatrolClient::pointsChanged, this, &PatrolPage::onPointsChanged);
  connect(client_, &PatrolClient::pointShotsChanged, this, [this](const QString &pointId) {
    if (pointId == selectedPointId() && !client_->pointShotsBusy()) populatePointShots();
    updateActionStates();
  });
  connect(client_, &PatrolClient::routesChanged, this, &PatrolPage::onRoutesChanged);
  connect(client_, &PatrolClient::runsChanged, this, &PatrolPage::onRunsChanged);
  connect(client_, &PatrolClient::inspectionResultsChanged, this,
          &PatrolPage::onInspectionResultsChanged);
  connect(client_, &PatrolClient::homePointChanged, this, &PatrolPage::onHomePointChanged);
  connect(client_, &PatrolClient::statusChanged, this, &PatrolPage::onStatusChanged);
  connect(client_, &PatrolClient::operationFinished, this, &PatrolPage::showOperation);
  connect(client_, &PatrolClient::errorOccurred, this, &PatrolPage::showError);
  if (ros_) {
    connect(ros_, &RosAdapter::requestFinished, this,
            [this](const QString &op, bool ok, const QString &detail) {
              if (op == QStringLiteral("ptz_preset")) {
                setMessage((ok ? QStringLiteral("云台预置位操作：")
                               : QStringLiteral("云台预置位操作失败：")) + detail, !ok);
              }
            });
  }
  client_->refreshMaps();
  auto *permissionTimer = new QTimer(this);
  permissionTimer->setInterval(500);
  connect(permissionTimer, &QTimer::timeout, this, &PatrolPage::updateActionStates);
  permissionTimer->start();
  updateActionStates();
}

void PatrolPage::setResumePermission(std::function<bool(QString *)> permission) {
  resumePermission_ = std::move(permission);
  inspectionTarget_->setReturnPermission(resumePermission_);
  updateActionStates();
}

void PatrolPage::showInspectionTarget() { tabs_->setCurrentWidget(inspectionTarget_); }

void PatrolPage::setReportProjectRoot(const QString &projectRoot) {
  reportProjectRoot_ = projectRoot;
}

void PatrolPage::showSelectedReport() {
  const int row = runsTable_->currentRow();
  if (row < 0 || !runsTable_->item(row, 0)) {
    setMessage(QStringLiteral("请先选择一条任务记录"), true);
    return;
  }
  QString root = reportProjectRoot_;
  if (root.isEmpty()) root = qEnvironmentVariable("ROBOT_PROJECT_ROOT");
  if (root.isEmpty()) root = QDir(DESKTOP_UI_SOURCE_DIR).absoluteFilePath("..");
  auto *dialog = new PatrolReportDialog(runsTable_->item(row, 0)->text(), root, this);
  dialog->show();
}

void PatrolPage::previewSelectedInspection() {
  const int row = inspectionResultsTable_->currentRow();
  if (row < 0 || !inspectionResultsTable_->item(row, 3)) {
    inspectionPhoto_->setPhoto({});
    return;
  }
  const auto path = inspectionResultsTable_->item(row, 3)->data(Qt::UserRole).toString();
  const auto caption = inspectionResultsTable_->item(row, 1)->text() + " · " + inspectionResultsTable_->item(row, 4)->text();
  inspectionPhoto_->setPhoto(path, caption);
}

void PatrolPage::buildUi() {
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(16, 14, 16, 10);
  root->setSpacing(10);

  auto *header = patrolui::card(this);
  auto *headerLayout = new QVBoxLayout(header);
  headerLayout->setContentsMargins(16, 12, 16, 12);
  headerLayout->setSpacing(10);
  auto *titleRow = new QHBoxLayout;
  auto *title = new QLabel("巡检任务", header);
  title->setObjectName("patrolTitle");
  titleRow->addWidget(title);
  auto *profileLabel = patrolui::hint(profileName_, header);
  profileLabel->setWordWrap(false);
  titleRow->addWidget(profileLabel);
  titleRow->addStretch();
  serviceLabel_ = new QLabel(header);
  serviceLabel_->setObjectName("patrolServiceStatus");
  serviceLabel_->setProperty("patrolBadge", true);
  titleRow->addWidget(serviceLabel_);
  refreshAllButton_ = pageButton("刷新", header, "patrolRefreshAll");
  titleRow->addWidget(refreshAllButton_);
  auto *settingsToggle = pageButton("地图与返航设置", header, "patrolMapSettingsToggle");
  settingsToggle->setCheckable(true);
  titleRow->addWidget(settingsToggle);
  headerLayout->addLayout(titleRow);

  auto *summary = new QHBoxLayout;
  activeMapLabel_ = new QLabel("活跃地图：未设置", header);
  activeMapLabel_->setObjectName("patrolActiveMap");
  activeMapLabel_->setProperty("patrolBadge", true);
  activeMapLabel_->setWordWrap(true);
  activeMapLabel_->setMinimumWidth(220);
  summary->addWidget(activeMapLabel_);
  homePointLabel_ = patrolui::hint("待命返航点：未设置", header);
  homePointLabel_->setObjectName("patrolHomePoint");
  summary->addWidget(homePointLabel_, 1);
  auto *targetButton = pageButton("一键到达", header, "patrolInspectPoint");
  summary->addWidget(targetButton);
  connect(targetButton, &QPushButton::clicked, this, &PatrolPage::showInspectionTarget);
  returnHomeButton_ = pageButton("一键返回", header, "patrolReturnHome");
  summary->addWidget(returnHomeButton_);
  headerLayout->addLayout(summary);

  auto *mapSettings = new QWidget(header);
  mapSettings->setObjectName("patrolMapSettings");
  auto *mapLayout = new QVBoxLayout(mapSettings);
  mapLayout->setContentsMargins(0, 4, 0, 0);
  mapLayout->setSpacing(8);
  auto *yamlRow = new QHBoxLayout;
  yamlRow->addWidget(new QLabel("导航地图", mapSettings));
  mapYamlEdit_ = new QLineEdit(mapYaml_, mapSettings);
  mapYamlEdit_->setObjectName("patrolMapYaml");
  mapYamlEdit_->setPlaceholderText("选择与当前定位对应的地图 YAML 文件");
  mapYamlEdit_->setMinimumWidth(180);
  yamlRow->addWidget(mapYamlEdit_, 1);
  registerMapButton_ = pageButton("登记当前地图", mapSettings, "patrolRegisterMap");
  yamlRow->addWidget(registerMapButton_);
  mapLayout->addLayout(yamlRow);
  auto *activeRow = new QHBoxLayout;
  activeRow->addWidget(new QLabel("已登记地图", mapSettings));
  mapsCombo_ = new QComboBox(mapSettings);
  mapsCombo_->setObjectName("patrolMaps");
  mapsCombo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  mapsCombo_->setMinimumContentsLength(16);
  activeRow->addWidget(mapsCombo_, 1);
  setActiveButton_ = pageButton("设为活跃地图", mapSettings, "patrolSetActiveMap");
  activeRow->addWidget(setActiveButton_);
  saveHomeButton_ = pageButton("当前位置设为返航点", mapSettings, "patrolSaveHome");
  clearHomeButton_ = pageButton("清除返航点", mapSettings, "patrolClearHome");
  clearHomeButton_->setProperty("patrolDanger", true);
  activeRow->addWidget(saveHomeButton_);
  activeRow->addWidget(clearHomeButton_);
  mapLayout->addLayout(activeRow);
  headerLayout->addWidget(mapSettings);
  mapSettings->hide();
  connect(settingsToggle, &QPushButton::toggled, mapSettings, &QWidget::setVisible);
  root->addWidget(header);

  tabs_ = new QTabWidget(this);
  tabs_->setObjectName("patrolTabs");
  tabs_->setDocumentMode(true);

  // Point capture and manual coordinates share a compact side card.
  auto *pointsTab = new QWidget(tabs_);
  auto *pointsLayout = new QHBoxLayout(pointsTab);
  pointsLayout->setContentsMargins(0, 10, 0, 0);
  pointsLayout->setSpacing(12);
  auto *pointList = patrolui::card(pointsTab);
  auto *pointListLayout = new QVBoxLayout(pointList);
  pointListLayout->setContentsMargins(14, 14, 14, 12);
  auto *pointsTop = new QHBoxLayout;
  pointsTop->addWidget(patrolui::heading("导航点", pointList));
  pointsCountLabel_ = patrolui::hint("0 个点位", pointList);
  pointsTop->addStretch();
  pointsTop->addWidget(pointsCountLabel_);
  pointListLayout->addLayout(pointsTop);
  pointsTable_ = table({"名称", "X (m)", "Y (m)", "朝向 (°)", "巡检", "版本"}, pointList);
  pointsTable_->setObjectName("patrolPointsTable");
  pointListLayout->addWidget(pointsTable_, 1);
  pointListLayout->addWidget(patrolui::hint("选择点位查看或修改坐标。坐标均相对于当前地图。", pointList));
  pointsLayout->addWidget(pointList, 1);

  auto *pointForm = patrolui::card(pointsTab);
  pointForm->setMinimumWidth(294);
  pointForm->setMaximumWidth(350);
  auto *pointFormLayout = new QVBoxLayout(pointForm);
  pointFormLayout->setContentsMargins(16, 16, 16, 16);
  pointFormLayout->setSpacing(10);
  pointFormLayout->addWidget(patrolui::heading("录入导航点", pointForm));
  pointFormLayout->addWidget(patrolui::hint("将车辆开到目标位置并停稳，填写名称后保存当前位置。", pointForm));
  pointNameEdit_ = new QLineEdit(pointForm);
  pointNameEdit_->setObjectName("patrolPointName");
  pointNameEdit_->setPlaceholderText("例如：设备间入口");
  pointFormLayout->addWidget(new QLabel("点位名称", pointForm));
  pointFormLayout->addWidget(pointNameEdit_);
  auto *inspectionGroup = new QGroupBox("到点巡检动作", pointForm);
  pointInspectionGroup_ = inspectionGroup;
  auto *inspectionLayout = new QVBoxLayout(inspectionGroup);
  inspectionLayout->setSpacing(7);
  pointInspectionEnabled_ = new QCheckBox("启用到点巡检", inspectionGroup);
  pointInspectionEnabled_->setObjectName("patrolPointInspectionEnabled");
  inspectionLayout->addWidget(pointInspectionEnabled_);
  auto *shotsGroup = new QGroupBox("拍摄视角（1～4×）", inspectionGroup);
  auto *shotsLayout = new QVBoxLayout(shotsGroup);
  shotsLayout->setSpacing(7);
  shotsLayout->addWidget(patrolui::hint(
      "先保存并选择点位，停车后在相机页调好构图，再记录。一个视角即可，最多两个；到点后补偿朝向、拍照并识别数字仪表。", shotsGroup));
  pointShotsCombo_ = new QComboBox(shotsGroup);
  pointShotsCombo_->setObjectName("patrolPointShots");
  shotsLayout->addWidget(pointShotsCombo_);
  pointShotDetails_ = patrolui::hint("请选择已保存点位", shotsGroup);
  pointShotDetails_->setObjectName("patrolPointShotDetails");
  shotsLayout->addWidget(pointShotDetails_);
  auto *shotForm = new QFormLayout;
  pointShotName_ = new QLineEdit(shotsGroup);
  pointShotName_->setObjectName("patrolPointShotName");
  pointShotName_->setMaxLength(120);
  pointShotName_->setPlaceholderText("例如：仪表正面");
  shotForm->addRow("视角名称", pointShotName_);
  pointShotSettle_ = new QDoubleSpinBox(shotsGroup);
  pointShotSettle_->setObjectName("patrolPointShotSettle");
  pointShotSettle_->setRange(0., 10.); pointShotSettle_->setDecimals(1);
  pointShotSettle_->setSingleStep(.5); pointShotSettle_->setValue(1.5);
  pointShotSettle_->setSuffix(" 秒");
  pointShotSettle_->setToolTip("巡检时角度和倍率到位后，额外等待镜头对焦的时间");
  shotForm->addRow("对焦等待", pointShotSettle_);
  shotsLayout->addLayout(shotForm);
  pointRecordShot_ = pageButton("记录当前拍摄视角", shotsGroup, "patrolRecordPointShot");
  pointRecordShot_->setProperty("primary", true);
  shotsLayout->addWidget(pointRecordShot_);
  auto *shotButtons = new QHBoxLayout;
  pointDeleteShot_ = pageButton("删除所选", shotsGroup, "patrolDeletePointShot");
  pointClearShots_ = pageButton("清空视角", shotsGroup, "patrolClearPointShots");
  pointRefreshShots_ = pageButton("刷新", shotsGroup, "patrolRefreshPointShots");
  shotButtons->addWidget(pointDeleteShot_); shotButtons->addWidget(pointClearShots_);
  shotButtons->addWidget(pointRefreshShots_);
  shotsLayout->addLayout(shotButtons);
  inspectionLayout->addWidget(shotsGroup);
  auto *legacyToggle = pageButton("设备预置位（未记录视角时使用）", inspectionGroup, "patrolLegacyInspectionToggle");
  legacyToggle->setCheckable(true);
  inspectionLayout->addWidget(legacyToggle);
  auto *legacy = new QWidget(inspectionGroup);
  auto *legacyLayout = new QVBoxLayout(legacy);
  legacyLayout->setContentsMargins(0, 0, 0, 0);
  inspectionLayout->addWidget(legacy);
  legacy->hide();
  connect(legacyToggle, &QPushButton::toggled, legacy, &QWidget::setVisible);
  auto *inspectionForm = new QFormLayout;
  inspectionForm->setSpacing(7);
  pointPtzPreset_ = new QSpinBox(inspectionGroup);
  pointPtzPreset_->setObjectName("patrolPointPtzPreset");
  pointPtzPreset_->setRange(1, 89);
  pointPtzPreset_->setValue(1);
  pointPtzPreset_->setToolTip(
      "巡检业务预置位 1～89；90～99 留给云台巡航。请先在右侧“相机”页把云台调到目标视角");
  inspectionForm->addRow("云台预置位", pointPtzPreset_);
  legacyLayout->addLayout(inspectionForm);
  auto *presetButtons = new QHBoxLayout;
  pointSavePtzPreset_ = pageButton("当前云台存为此预置位", inspectionGroup,
                                   "patrolSavePtzPreset");
  pointGotoPtzPreset_ = pageButton("转到该预置位", inspectionGroup,
                                   "patrolGotoPtzPreset");
  presetButtons->addWidget(pointSavePtzPreset_);
  presetButtons->addWidget(pointGotoPtzPreset_);
  legacyLayout->addLayout(presetButtons);
  legacyLayout->addWidget(patrolui::hint(
      "未记录拍摄视角时，使用设备预置位抓拍并识别。已记录视角时，优先按角度补偿拍照并逐张识别。", inspectionGroup));
  inspectionLayout->addWidget(new QLabel("数字仪表识别提示词（留空使用默认规则）", inspectionGroup));
  pointRecognitionPrompt_ = new QPlainTextEdit(inspectionGroup);
  pointRecognitionPrompt_->setObjectName("patrolPointRecognitionPrompt");
  pointRecognitionPrompt_->setPlaceholderText(
      "留空自动读取数字仪表，保留小数点、正负号和单位，看不清写无法辨认。"
      "也可填写本点位专用提示词。");
  pointRecognitionPrompt_->setMaximumHeight(70);
  inspectionLayout->addWidget(pointRecognitionPrompt_);
  inspectionLayout->addWidget(patrolui::hint(
      "到点后自动确认云台稳定、拍照，再使用 DeepSeek 逐张读取数字仪表。照片和识别结果可在执行记录中查看。", inspectionGroup));
  pointFormLayout->addWidget(inspectionGroup);
  saveCurrentPointButton_ = pageButton("保存当前位置为巡检点", pointForm, "patrolSaveCurrentPoint");
  saveCurrentPointButton_->setProperty("primary", true);
  pointFormLayout->addWidget(saveCurrentPointButton_);
  auto *coordinates = new QGroupBox("手动坐标 / 所选点位", pointForm);
  auto *pointFields = new QFormLayout(coordinates);
  pointFields->setSpacing(10);
  pointX_ = new QDoubleSpinBox(coordinates); pointX_->setRange(-100000, 100000); pointX_->setDecimals(3);
  pointY_ = new QDoubleSpinBox(coordinates); pointY_->setRange(-100000, 100000); pointY_->setDecimals(3);
  pointYaw_ = new QDoubleSpinBox(coordinates); pointYaw_->setRange(-180, 180); pointYaw_->setDecimals(2);
  pointFields->addRow("X (m)", pointX_);
  pointFields->addRow("Y (m)", pointY_);
  pointFields->addRow("朝向 (°)", pointYaw_);
  upsertPointButton_ = pageButton("新增 / 更新点位", coordinates, "patrolUpsertPoint");
  pointFields->addRow(upsertPointButton_);
  pointFormLayout->addWidget(coordinates);
  pointFormLayout->addStretch();
  deletePointButton_ = pageButton("删除所选点位", pointForm, "patrolDeletePoint");
  deletePointButton_->setProperty("patrolDanger", true);
  pointFormLayout->addWidget(deletePointButton_);
  auto *pointScroll = new QScrollArea(pointsTab);
  pointScroll->setObjectName("patrolPointEditorScroll");
  pointScroll->setWidgetResizable(true);
  pointScroll->setMinimumHeight(100);
  pointScroll->setMinimumWidth(310);
  pointScroll->setMaximumWidth(366);
  pointScroll->setWidget(pointForm);
  pointsLayout->addWidget(pointScroll);
  tabs_->addTab(pointsTab, "点位管理");

  auto *routesTab = new QWidget(tabs_);
  auto *routesLayout = new QHBoxLayout(routesTab);
  routesLayout->setContentsMargins(0, 10, 0, 0);
  routesLayout->setSpacing(12);
  auto *routeListPanel = patrolui::card(routesTab);
  routeListPanel->setMinimumWidth(248);
  routeListPanel->setMaximumWidth(340);
  auto *routeListLayout = new QVBoxLayout(routeListPanel);
  routeListLayout->setContentsMargins(14, 14, 14, 14);
  auto *routesTop = new QHBoxLayout;
  routesTop->addWidget(patrolui::heading("巡检路线", routeListPanel));
  routesTop->addStretch();
  routesCountLabel_ = patrolui::hint("0 条", routeListPanel);
  routesTop->addWidget(routesCountLabel_);
  routeListLayout->addLayout(routesTop);
  routesTable_ = table({"路线", "点位数", "版本"}, routeListPanel);
  routesTable_->setObjectName("patrolRoutesTable");
  routeListLayout->addWidget(routesTable_, 1);
  auto *routeListButtons = new QHBoxLayout;
  newRouteButton_ = pageButton("新建路线", routeListPanel, "patrolNewRoute");
  deleteRouteButton_ = pageButton("删除", routeListPanel, "patrolDeleteRoute");
  deleteRouteButton_->setProperty("patrolDanger", true);
  routeListButtons->addWidget(newRouteButton_, 1);
  routeListButtons->addWidget(deleteRouteButton_);
  routeListLayout->addLayout(routeListButtons);
  routesLayout->addWidget(routeListPanel, 1);

  auto *routeRight = new QWidget(routesTab);
  auto *routeRightLayout = new QVBoxLayout(routeRight);
  routeRightLayout->setContentsMargins(0, 0, 0, 0);
  routeRightLayout->setSpacing(10);
  auto *editorCard = patrolui::card(routeRight);
  auto *editorCardLayout = new QVBoxLayout(editorCard);
  editorCardLayout->setContentsMargins(14, 12, 14, 12);
  auto *editorHeader = new QHBoxLayout;
  editorHeader->addWidget(patrolui::heading("路线编排", editorCard));
  routeDraftLabel_ = new QLabel("新路线", editorCard);
  routeDraftLabel_->setObjectName("patrolRouteDraftState");
  routeDraftLabel_->setProperty("patrolBadge", true);
  editorHeader->addWidget(routeDraftLabel_);
  editorHeader->addStretch();
  saveRouteButton_ = pageButton("保存路线", editorCard, "patrolSaveRoute");
  saveRouteButton_->setProperty("primary", true);
  editorHeader->addWidget(saveRouteButton_);
  editorCardLayout->addLayout(editorHeader);

  auto *routeEditor = new QWidget(editorCard);
  auto *routeEditorLayout = new QVBoxLayout(routeEditor);
  routeEditorLayout->setContentsMargins(0, 0, 0, 0);
  routeEditorLayout->setSpacing(8);
  auto *routeNameRow = new QHBoxLayout;
  routeNameRow->addWidget(new QLabel("路线名称", routeEditor));
  routeNameEdit_ = new QLineEdit(routeEditor);
  routeNameEdit_->setObjectName("patrolRouteName");
  routeNameEdit_->setPlaceholderText("输入巡检路线名称");
  routeNameRow->addWidget(routeNameEdit_, 1);
  navigationMode_ = new QComboBox(routeEditor);
  navigationMode_->setObjectName("patrolNavigationMode");
  navigationMode_->addItem("自由导航", "FREE_NAV");
  navigationMode_->setToolTip("当前版本只启用自由导航：按点位顺序由 Nav2 导航到达");
  routeNameRow->addWidget(navigationMode_);
  routeEditorLayout->addLayout(routeNameRow);
  auto *modeRow = new QHBoxLayout;
  routeNetwork_ = new QComboBox(routeEditor);
  routeNetwork_->setObjectName("patrolRouteNetwork");
  routeNetwork_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  routeNetwork_->setMinimumContentsLength(12);
  previewRecorded_ = pageButton("预览实际路线", routeEditor, "patrolPreviewRecorded");
  modeRow->addWidget(routeNetwork_, 1);
  modeRow->addWidget(previewRecorded_);
  routeNetwork_->hide();
  previewRecorded_->hide();
  routeEditorLayout->addLayout(modeRow);
  auto *addStepRow = new QHBoxLayout;
  pointCombo_ = new QComboBox(routeEditor);
  pointCombo_->setObjectName("patrolPointCombo");
  pointCombo_->setMinimumWidth(140);
  addStepDwell_ = new QDoubleSpinBox(routeEditor);
  addStepDwell_->setObjectName("patrolAddStepDwell");
  addStepDwell_->setRange(0, 86400); addStepDwell_->setDecimals(1);
  addStepDwell_->setSuffix(" s");
  addStepDwell_->setToolTip("该点位到达后的停留时间");
  addStepButton_ = pageButton("添加点位", routeEditor, "patrolAddStep");
  addStepRow->addWidget(new QLabel("加入路线", routeEditor));
  addStepRow->addWidget(pointCombo_, 1);
  addStepRow->addWidget(new QLabel("停留", routeEditor));
  addStepRow->addWidget(addStepDwell_);
  addStepRow->addWidget(addStepButton_);
  routeEditorLayout->addLayout(addStepRow);
  stepsTable_ = table({"点位", "X (m)", "Y (m)", "停留 (s)", "巡检动作"}, routeEditor);
  stepsTable_->setObjectName("patrolStepsTable");
  stepsTable_->setMinimumHeight(118);
  routeEditorLayout->addWidget(stepsTable_, 1);
  auto *stepButtons = new QHBoxLayout;
  stepDwell_ = new QDoubleSpinBox(routeEditor);
  stepDwell_->setObjectName("patrolStepDwell");
  stepDwell_->setRange(0, 86400); stepDwell_->setDecimals(1); stepDwell_->setSuffix(" s");
  updateDwellButton_ = pageButton("更新停留", routeEditor, "patrolUpdateDwell");
  moveUpButton_ = pageButton("上移", routeEditor, "patrolStepUp");
  moveDownButton_ = pageButton("下移", routeEditor, "patrolStepDown");
  removeStepButton_ = pageButton("移除", routeEditor, "patrolRemoveStep");
  removeStepButton_->setProperty("patrolDanger", true);
  stepButtons->addWidget(stepDwell_);
  stepButtons->addWidget(updateDwellButton_);
  stepButtons->addStretch();
  stepButtons->addWidget(moveUpButton_);
  stepButtons->addWidget(moveDownButton_);
  stepButtons->addWidget(removeStepButton_);
  routeEditorLayout->addLayout(stepButtons);
  auto *routeScroll = new QScrollArea(editorCard);
  routeScroll->setObjectName("patrolRouteEditorScroll");
  routeScroll->setWidgetResizable(true);
  routeScroll->setMinimumHeight(100);
  routeScroll->setWidget(routeEditor);
  editorCardLayout->addWidget(routeScroll, 1);
  routeRightLayout->addWidget(editorCard, 1);

  auto *previewCard = patrolui::card(routeRight);
  auto *previewLayout = new QVBoxLayout(previewCard);
  previewLayout->setContentsMargins(14, 10, 14, 10);
  previewLayout->setSpacing(6);
  previewLayout->addWidget(patrolui::heading("点位预览（按路线顺序）", previewCard));
  routePreviewScene_ = new QGraphicsScene(previewCard);
  routePreview_ = new QGraphicsView(routePreviewScene_, previewCard);
  routePreview_->setObjectName("patrolRoutePreview");
  routePreview_->setMinimumHeight(130);
  routePreview_->setMaximumHeight(190);
  routePreview_->setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
  routePreview_->setBackgroundBrush(QColor("#f8fafc"));
  routePreview_->setFrameShape(QFrame::StyledPanel);
  previewLayout->addWidget(routePreview_);
  routeRightLayout->addWidget(previewCard, 0);

  // Execution stays outside the editor's scroll area on shorter displays.
  auto *runGroup = patrolui::card(routeRight);
  auto *runLayout = new QVBoxLayout(runGroup);
  runLayout->setContentsMargins(14, 12, 14, 12);
  runLayout->setSpacing(8);
  auto *runHeader = new QHBoxLayout;
  runHeader->addWidget(patrolui::heading("任务执行", runGroup));
  runHeader->addStretch();
  statusLabel_ = new QLabel("状态：空闲", runGroup);
  statusLabel_->setObjectName("patrolStatusLabel");
  statusLabel_->setProperty("patrolBadge", true);
  statusLabel_->setWordWrap(true);
  statusLabel_->setMaximumWidth(440);
  runHeader->addWidget(statusLabel_);
  runLayout->addLayout(runHeader);
  startReadinessLabel_ = new QLabel(runGroup);
  startReadinessLabel_->setObjectName("patrolStartReadiness");
  startReadinessLabel_->setWordWrap(true);
  startReadinessLabel_->setTextFormat(Qt::PlainText);
  startReadinessLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  startReadinessLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  startReadinessLabel_->setMargin(8);
  runLayout->addWidget(startReadinessLabel_);
  auto *runRow = new QHBoxLayout;
  runRow->addWidget(new QLabel("执行次数", runGroup));
  loopCount_ = new QSpinBox(runGroup);
  loopCount_->setObjectName("patrolLoopCount");
  loopCount_->setRange(0, 100000); loopCount_->setValue(1);
  loopCount_->setSpecialValueText("持续循环");
  loopCount_->setToolTip("0 表示持续循环，其他数字表示执行圈数");
  runRow->addWidget(loopCount_);
  runRow->addStretch();
  startButton_ = pageButton("开始", runGroup, "patrolStart");
  startButton_->setProperty("primary", true);
  pauseButton_ = pageButton("暂停", runGroup, "patrolPause");
  resumeButton_ = pageButton("继续", runGroup, "patrolResume");
  cancelButton_ = pageButton("取消任务", runGroup, "patrolCancel");
  cancelButton_->setProperty("patrolDanger", true);
  resetInterlockButton_ = pageButton("重启巡航联锁", runGroup, "patrolResetInterlock");
  resetInterlockButton_->setToolTip("取消旧 Nav2 目标并复位巡航联锁；要求车辆已停稳、没有活动任务");
  runRow->addWidget(startButton_); runRow->addWidget(pauseButton_);
  runRow->addWidget(resumeButton_); runRow->addWidget(cancelButton_);
  runRow->addWidget(resetInterlockButton_);
  runLayout->addLayout(runRow);
  progressLabel_ = patrolui::hint("尚未执行路线", runGroup);
  progressLabel_->setObjectName("patrolProgressLabel");
  runLayout->addWidget(progressLabel_);
  auto *roadProgress = patrolui::hint(QString(), runGroup);
  roadProgress->setObjectName("patrolRoadProgress");
  roadProgress->hide();
  runLayout->addWidget(roadProgress);
  routeRightLayout->addWidget(runGroup);
  routesLayout->addWidget(routeRight, 3);
  tabs_->addTab(routesTab, "路线与执行");

  auto *runsTab = new QWidget(tabs_);
  auto *runsOuter = new QVBoxLayout(runsTab);
  runsOuter->setContentsMargins(0, 10, 0, 0);
  auto *runsCard = patrolui::card(runsTab);
  auto *runsLayout = new QVBoxLayout(runsCard);
  runsLayout->setContentsMargins(14, 14, 14, 14);
  auto *runsTop = new QHBoxLayout;
  runsTop->addWidget(patrolui::heading("最近任务记录", runsCard));
  runsTop->addStretch();
  viewReportButton_ = pageButton("查看图文报告 / 导出", runsCard, "patrolViewReport");
  viewReportButton_->setEnabled(false);
  runsTop->addWidget(viewReportButton_);
  refreshRunsButton_ = pageButton("刷新记录", runsCard, "patrolRefreshRuns");
  runsTop->addWidget(refreshRunsButton_);
  auto *roadEvents = pageButton("查看道路事件", runsCard, "patrolRoadEvents");
  runsTop->addWidget(roadEvents);
  runsLayout->addLayout(runsTop);
  runsTable_ = table({"任务编号", "路线", "状态", "圈数", "点位数", "结果", "说明"}, runsCard);
  runsTable_->setObjectName("patrolRunsTable");
  runsTable_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
  runsTable_->setColumnWidth(0, 180);
  runsTable_->horizontalHeader()->setSectionResizeMode(6, QHeaderView::Stretch);
  runsLayout->addWidget(runsTable_, 1);
  runsLayout->addWidget(patrolui::hint("选中已结束任务，点击“查看图文报告 / 导出”可查看全部照片与视觉记录，并保存 PDF、HTML 或 JSON。", runsCard));
  auto *inspectionHeader = new QHBoxLayout;
  inspectionHeader->addWidget(patrolui::heading("巡检识别结果", runsCard));
  inspectionHeader->addStretch();
  refreshInspectionResultsButton_ = pageButton(
      "刷新识别结果", runsCard, "patrolRefreshInspectionResults");
  inspectionHeader->addWidget(refreshInspectionResultsButton_);
  runsLayout->addLayout(inspectionHeader);
  inspectionResultsTable_ = table(
      {"时间", "点位", "识别结果", "图片", "状态"}, runsCard);
  inspectionResultsTable_->setObjectName("patrolInspectionResultsTable");
  inspectionResultsTable_->horizontalHeader()->setSectionResizeMode(
      2, QHeaderView::Stretch);
  inspectionResultsTable_->setColumnWidth(0, 130);
  inspectionResultsTable_->setColumnWidth(1, 130);
  inspectionResultsTable_->setColumnWidth(3, 220);
  inspectionResultsTable_->setColumnWidth(4, 150);
  auto *inspectionSplit = new QSplitter(Qt::Horizontal, runsCard);
  inspectionPhoto_ = new InspectionPhotoView(inspectionSplit);
  inspectionPhoto_->setObjectName("patrolInspectionPhotoPreview");
  inspectionSplit->addWidget(inspectionResultsTable_);
  inspectionSplit->addWidget(inspectionPhoto_);
  inspectionSplit->setSizes({780, 320});
  runsLayout->addWidget(inspectionSplit, 1);
  runsOuter->addWidget(runsCard);
  connect(roadEvents, &QPushButton::clicked, this, [this] {
    const int row = runsTable_->currentRow();
    if (row < 0 || !runsTable_->item(row, 0)) { setMessage("请先选择一条执行记录", true); return; }
    client_->roadCommand("EVENTS", {{"run_id", runsTable_->item(row, 0)->text()}});
  });
  tabs_->addTab(runsTab, "执行记录");
  inspectionTarget_ = new InspectionTargetPage(client_, offline_, startPermission_, tabs_);
  tabs_->addTab(inspectionTarget_, "一键到达与巡检");
  // Fixed-route authoring/recorded-road UI is disabled in this build.
  root->addWidget(tabs_, 1);

  messageLabel_ = patrolui::hint("临时任务使用“一键到达与巡检”；多点任务在“路线与执行”启动。一键返回会结束当前任务，返回当前地图待命点。", this);
  messageLabel_->setObjectName("patrolMessage");
  messageLabel_->setTextFormat(Qt::PlainText);
  messageLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  root->addWidget(messageLabel_);

  connect(registerMapButton_, &QPushButton::clicked, this, &PatrolPage::registerCurrentMap);
  connect(setActiveButton_, &QPushButton::clicked, this, &PatrolPage::setActiveMap);
  connect(refreshAllButton_, &QPushButton::clicked, this, &PatrolPage::refreshAll);
  connect(saveHomeButton_, &QPushButton::clicked, this, &PatrolPage::saveHomePoint);
  connect(clearHomeButton_, &QPushButton::clicked, this, &PatrolPage::clearHomePoint);
  connect(returnHomeButton_, &QPushButton::clicked, this, &PatrolPage::returnHome);
  connect(pointsTable_, &QTableWidget::itemSelectionChanged, this, &PatrolPage::updatePointFormFromSelection);
  connect(saveCurrentPointButton_, &QPushButton::clicked, this, &PatrolPage::saveCurrentPoint);
  connect(upsertPointButton_, &QPushButton::clicked, this, &PatrolPage::upsertSelectedPoint);
  connect(pointInspectionEnabled_, &QCheckBox::toggled, this,
          [this](bool) { updateActionStates(); });
  connect(pointSavePtzPreset_, &QPushButton::clicked, this, [this] {
    if (!ros_ || offline_) {
      setMessage(QStringLiteral("相机 ROS 适配器不可用，无法设置云台预置位"), true);
      return;
    }
    const int preset = pointPtzPreset_->value();
    ros_->setPtzPreset(0, preset);
    setMessage(QStringLiteral("正在把当前云台姿态保存为预置位 %1…").arg(preset), false);
  });
  connect(pointGotoPtzPreset_, &QPushButton::clicked, this, [this] {
    if (!ros_ || offline_) {
      setMessage(QStringLiteral("相机 ROS 适配器不可用，无法调用云台预置位"), true);
      return;
    }
    const int preset = pointPtzPreset_->value();
    ros_->setPtzPreset(1, preset);
    setMessage(QStringLiteral("正在转到云台预置位 %1…").arg(preset), false);
  });
  connect(deletePointButton_, &QPushButton::clicked, this, &PatrolPage::deleteSelectedPoint);
  connect(pointRecordShot_, &QPushButton::clicked, this, &PatrolPage::recordSelectedShot);
  connect(pointDeleteShot_, &QPushButton::clicked, this, &PatrolPage::deleteSelectedShot);
  connect(pointClearShots_, &QPushButton::clicked, this, [this] {
    const auto info = client_->pointShots(selectedPointId());
    if (pointClearShots_->isEnabled() && info.loaded)
      client_->clearPointShots(info.point.pointId, info.point.mapId, info.mapRevision, info.point.revision);
  });
  connect(pointRefreshShots_, &QPushButton::clicked, this, &PatrolPage::loadPointShots);
  connect(pointShotsCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] {
    selectedShotId_ = pointShotsCombo_->currentData().toString();
    updateShotForm();
  });
  connect(pointShotName_, &QLineEdit::textChanged, this, &PatrolPage::updateActionStates);
  connect(routesTable_, &QTableWidget::itemSelectionChanged, this, [this] {
    const int row = routesTable_->currentRow();
    if (row < 0 || !routesTable_->item(row, 0)) return;
    const QString routeId = routesTable_->item(row, 0)->data(Qt::UserRole).toString();
    if (routeDirty_ && routeId != currentRouteId_) {
      if(!confirmDraftDiscard()) {
        const QSignalBlocker block(routesTable_);
        for(int i=0;i<routesTable_->rowCount();++i)
          if(routesTable_->item(i,0)->data(Qt::UserRole).toString()==currentRouteId_) routesTable_->selectRow(i);
        return;
      }
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
  connect(resetInterlockButton_, &QPushButton::clicked, this, [this] {
    if (QMessageBox::question(
            this, QStringLiteral("重启巡航联锁"),
            QStringLiteral("将取消旧 Nav2 目标并复位巡航联锁。\n"
                           "要求车辆已停稳、没有活动任务。继续？")) != QMessageBox::Yes)
      return;
    client_->resetInterlock();
  });
  connect(refreshRunsButton_, &QPushButton::clicked, this, [this] {
    client_->refreshRuns(activeMapId_);
  });
  connect(viewReportButton_, &QPushButton::clicked, this, &PatrolPage::showSelectedReport);
  connect(inspectionResultsTable_, &QTableWidget::itemSelectionChanged, this, &PatrolPage::previewSelectedInspection);
  connect(inspectionResultsTable_, &QTableWidget::cellDoubleClicked, this,
          [this](int row, int) {
            if (!inspectionResultsTable_->item(row, 3)) return;
            const QString path = inspectionResultsTable_->item(row, 3)
                                     ->data(Qt::UserRole).toString();
            if (!path.isEmpty()) InspectionPhotoView::showPhoto(path, "到点巡检照片", this);
          });
  connect(refreshInspectionResultsButton_, &QPushButton::clicked, this, [this] {
    const int row = runsTable_->currentRow();
    const QString runId = row >= 0 && runsTable_->item(row, 0)
                              ? runsTable_->item(row, 0)->text() : QString();
    client_->refreshInspectionResults(activeMapId_, runId, 200);
  });
  connect(runsTable_, &QTableWidget::itemSelectionChanged, this, [this] {
    const int row = runsTable_->currentRow();
    const QString runId = row >= 0 && runsTable_->item(row, 0)
                              ? runsTable_->item(row, 0)->text() : QString();
    viewReportButton_->setEnabled(!runId.isEmpty());
    inspectionResultsTable_->setRowCount(0);
    updateEmptyTable(inspectionResultsTable_);
    inspectionPhoto_->setPhoto({});
    client_->refreshInspectionResults(activeMapId_, runId, 200);
  });
  connect(routeNameEdit_, &QLineEdit::textChanged, this, [this] {
    if (!loadingRoute_) { routeDirty_ = true; updateActionStates(); }
  });
  connect(navigationMode_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{
    if(!loadingRoute_) {routeDirty_=true;planDigest_.clear();} updateActionStates();
  });
  connect(routeNetwork_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{
    if(!loadingRoute_) {routeDirty_=true;planDigest_.clear();} updateActionStates();
  });
  connect(loopCount_,QOverload<int>::of(&QSpinBox::valueChanged),this,[this]{planDigest_.clear();startAfterPlan_=false;updateActionStates();});
  connect(previewRecorded_,&QPushButton::clicked,this,[this]{
    if(routeDirty_ || currentRouteId_.isEmpty()) {setMessage("请先保存路线再预览",true);return;}
    startAfterPlan_=false;planDigest_.clear();client_->roadCommand("PLAN",{{"route_id",currentRouteId_},{"loop_count",loopCount_->value()}});
  });
  connect(client_,&PatrolClient::roadNetworksChanged,this,[this]{
    const auto oldId=routeNetwork_->currentData();
    const auto oldRevision=routeNetwork_->currentData(Qt::UserRole+1).toULongLong();
    const bool previousLoading=loadingRoute_;loadingRoute_=true;
    routeNetwork_->clear();
    for(auto value:client_->roadNetworks()) {auto n=value.toObject();if(!n["published"].toBool()) continue;
      routeNetwork_->addItem(n["name"].toString()+QString(" · v%1").arg(n["revision"].toInt()),n["network_id"].toString());
      routeNetwork_->setItemData(routeNetwork_->count()-1,n["revision"].toVariant(),Qt::UserRole+1);
    }
    int index=routeNetwork_->findData(oldId);if(index>=0) routeNetwork_->setCurrentIndex(index);
    loadingRoute_=previousLoading;
    if(navigationMode_->currentData()=="RECORDED_NETWORK" &&
       (routeNetwork_->currentData()!=oldId || routeNetwork_->currentData(Qt::UserRole+1).toULongLong()!=oldRevision)) {
      routeDirty_=true;planDigest_.clear();
    }
    updateActionStates();
  });
  connect(client_,&PatrolClient::roadReply,this,[this](const QString &op,bool ok,const QString &message,const QJsonObject &data){
    const QMap<QString,QString> phases{{"READY","准备跟踪"},{"FOLLOWING_REFERENCE","沿录制道路行驶"},
      {"FOLLOWING_DETOUR","低速绕障"},{"WAITING_OBSTACLE","停车等待障碍解除或可行绕行路径"},
      {"PLANNING_DETOUR","停车规划受限绕行"},{"STOPPING","确认停稳 / 换向"},{"REJOINING","回到原道路，确认重入位姿"},
      {"COMPLETED","本段道路完成"},{"FAILED","道路执行失败"},{"STOPPED","已停止"},{"STOP_UNCONFIRMED","停止未确认"}};
    if(op=="STATUS" && ok) {
      const bool draft=data["state"].toString("IDLE")!="IDLE";
      if(draft!=recordingDraft_) {recordingDraft_=draft;if(draft) planDigest_.clear();updateActionStates();}
      const auto progress=data["progress"].toObject();const QString state=progress["state"].toString();
      QString line;
      if(client_->status().active && !state.isEmpty()) {
        line=QString("道路执行：%1 · 运动段 %2 · 参考采样 %3")
          .arg(phases.value(state,state)).arg(progress["block"].toInt()+1).arg(progress["reference_index"].toInt());
        if(progress["entry"].toBool()) line="自动接入路径 · "+line;
        const auto reason=progress["error"].toString().isEmpty()?progress["waiting_reason"].toString():progress["error"].toString();
        if(!reason.isEmpty()) line+=" · "+reason;
      }
      auto *roadProgress = findChild<QLabel*>("patrolRoadProgress");
      roadProgress->setText(line);
      roadProgress->setVisible(!line.isEmpty());
      return;
    }
    if(op=="EVENTS") {
      if(!ok) {setMessage(message,true);return;}
      QStringList lines;
      for(auto item:data["events"].toArray()) {auto event=item.toObject();
        QString line=QString("%1 · 段 %2 · 参考采样 %3").arg(phases.value(event["state"].toString(),event["state"].toString()))
          .arg(event["block"].toInt()+1).arg(event["reference_index"].toInt());
        if(event.contains("timestamp_ns")) line=QDateTime::fromMSecsSinceEpoch(event["timestamp_ns"].toVariant().toLongLong()/1000000).toString("yyyy-MM-dd hh:mm:ss")+" · "+line;
        if(!event["error"].toString().isEmpty()) line+=" · "+event["error"].toString();
        if(!event["waiting_reason"].toString().isEmpty()) line+=" · "+event["waiting_reason"].toString();
        lines<<line;
      }
      auto *dialog=new QDialog(this);dialog->setAttribute(Qt::WA_DeleteOnClose);dialog->setWindowTitle("道路执行事件");dialog->resize(820,480);
      auto *layout=new QVBoxLayout(dialog);auto *text=new QPlainTextEdit(dialog);text->setReadOnly(true);
      text->setPlainText(lines.isEmpty()?"此任务没有录制道路执行事件。":lines.join('\n'));layout->addWidget(text);dialog->show();return;
    }
    if(op!="PLAN") return;
    if(!ok) {startAfterPlan_=false;planDigest_.clear();setMessage(message,true);updateActionStates();return;}
    if(data["route_id"].toString()!=currentRouteId_ || data["route_revision"].toVariant().toULongLong()!=currentRouteRevision_ ||
       data["loop_count"].toInt()!=loopCount_->value() || routeDirty_) return;
    planDigest_=data["digest"].toString();
    QStringList parts;
    for(auto leg:data["legs"].toArray()) {auto l=leg.toObject();parts<<QString("%1 段录制道路").arg(l["edge_ids"].toArray().size());}
    setMessage(data["traversal"]=="SHUTTLE" ? "路径已验证：非闭环按趟往返，返程沿原轨迹倒车。"
               : "已验证实际行驶路线："+parts.join(" → ")+"。绿色轨迹可在“录制路径”页查看。",false);
    const auto entry=data["entry"].toObject();
    if(entry["required"].toBool()) {
      setMessage(QString("将自动接入 %1（约 %2 米，%3 次前进/倒车换向），对齐后沿录制路径巡检。接入不计巡检趟数。")
        .arg(entry["target_name"].toString()).arg(entry["length"].toDouble(),0,'f',1)
        .arg(entry["direction_changes"].toInt()),false);
    }
    const bool requestedStart=startAfterPlan_;startAfterPlan_=false;
    updateActionStates();
    if(requestedStart && startButton_->isEnabled()) dispatchRun();
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
    saveHomeButton_->setEnabled(false);
    clearHomeButton_->setEnabled(false);
    returnHomeButton_->setEnabled(false);
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
  if(mapId!=activeMapId_ && !confirmDraftDiscard()) return;
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
    client_->refreshInspectionResults(activeMapId_, QString(), 200);
    client_->refreshHomePoint(activeMapId_);
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
    client_->refreshInspectionResults(activeMapId_, QString(), 200);
    client_->refreshHomePoint(activeMapId_);
  } else {
    pointsTable_->setRowCount(0);
    routesTable_->setRowCount(0);
    runsTable_->setRowCount(0);
    inspectionResultsTable_->setRowCount(0);
    updateEmptyTable(pointsTable_);
    updateEmptyTable(routesTable_);
    updateEmptyTable(runsTable_);
    pointsCountLabel_->setText(QStringLiteral("0 个点位"));
    routesCountLabel_->setText(QStringLiteral("0 条"));
    pointCombo_->clear();
    homePointLabel_->setText(QStringLiteral("待命返航点：未设置"));
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
                               : QStringLiteral("当前地图：%1").arg(map.name));
  activeMapLabel_->setToolTip(QStringLiteral("地图版本：%1\n%2").arg(map.revision).arg(map.navigationYaml));
}

void PatrolPage::onPointsChanged() {
  populatePoints();
  updateActionStates();
}

void PatrolPage::onRoutesChanged() {
  populateRoutes();
  if(!pendingPathRouteId_.isEmpty()) {
    const auto path=routeById(pendingPathRouteId_);
    if(!path.routeId.isEmpty()) {
      pendingPathRouteId_.clear();loadRouteIntoEditor(path);
      for(int row=0;row<routesTable_->rowCount();++row)
        if(routesTable_->item(row,0)->data(Qt::UserRole).toString()==path.routeId) routesTable_->selectRow(row);
      tabs_->setCurrentIndex(1);
      setMessage("巡检路径已就绪，停车后选择执行次数并点击“开始”。系统自动接入可达端点，再沿录制路径巡检。",false);
    }
  }
  if (!currentRouteId_.isEmpty()) {
    const auto route = routeById(currentRouteId_);
    if (!route.routeId.isEmpty() && !routeDirty_) loadRouteIntoEditor(route);
  }
  updateActionStates();
}

void PatrolPage::onRunsChanged() { populateRuns(); }

void PatrolPage::onInspectionResultsChanged() { populateInspectionResults(); }

void PatrolPage::onHomePointChanged() {
  const auto home = client_->homePoint();
  if (home.mapId != activeMapId_ || !home.exists) {
    homePointLabel_->setText(QStringLiteral("待命返航点：未设置或已失效"));
  } else {
    homePointLabel_->setText(QStringLiteral("返航点：X %1 m · Y %2 m · 朝向 %3°")
                                 .arg(home.x, 0, 'f', 2).arg(home.y, 0, 'f', 2)
                                 .arg(home.yaw * 180.0 / kPi, 0, 'f', 1));
    homePointLabel_->setToolTip(QStringLiteral("返航点地图版本：%1").arg(home.mapRevision));
  }
  updateActionStates();
}

void PatrolPage::refreshHomePoint() {
  if (!activeMapId_.isEmpty()) client_->refreshHomePoint(activeMapId_);
}

void PatrolPage::saveHomePoint() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先设置活跃地图"), true); return; }
  QString reason;
  if (startPermission_ && !startPermission_(&reason)) {
    setMessage(reason.isEmpty() ? QStringLiteral("当前不允许保存返航点") : reason, true);
    return;
  }
  client_->saveCurrentPoseAsHome(activeMapId_, activeMap().revision);
}

void PatrolPage::clearHomePoint() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先设置活跃地图"), true); return; }
  client_->clearHomePoint(activeMapId_, activeMap().revision);
}

void PatrolPage::returnHome() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先设置活跃地图"), true); return; }
  const auto home = client_->homePoint();
  if (!home.exists || home.mapId != activeMapId_ || home.mapRevision != activeMap().revision) {
    setMessage(QStringLiteral("当前地图没有有效待命返航点"), true);
    return;
  }
  const auto &permission = resumePermission_ ? resumePermission_ : startPermission_;
  QString reason;
  if (permission && !permission(&reason)) {
    setMessage(reason.isEmpty() ? QStringLiteral("当前不允许返航") : reason, true);
    return;
  }
  client_->returnHome(activeMapId_, activeMap().revision);
}

void PatrolPage::populatePoints() {
  const QString selected = selectedPointId();
  const QSignalBlocker block(pointsTable_);
  pointsTable_->setRowCount(0);
  const auto list = client_->points();
  pointsCountLabel_->setText(QStringLiteral("%1 个点位").arg(list.size()));
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
    const QString action = point.inspectionEnabled
        ? QStringLiteral("云台抓拍巡检")
        : QStringLiteral("仅导航");
    pointsTable_->setItem(row, 4, readOnlyItem(action));
    pointsTable_->setItem(row, 5, readOnlyItem(QString::number(point.revision)));
  }
  for (int row = 0; row < pointsTable_->rowCount(); ++row)
    if (pointsTable_->item(row, 0)->data(Qt::UserRole).toString() == selected) pointsTable_->selectRow(row);
  updateEmptyTable(pointsTable_);
  populateAvailablePoints();
  updatePointFormFromSelection();
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
  int visibleRoutes=0;
  for (const auto &route : client_->routes()) {
    if (route.navigationMode=="RECORDED_NETWORK") continue;
    ++visibleRoutes;
    const int row = routesTable_->rowCount();
    routesTable_->insertRow(row);
    auto *name = readOnlyItem(route.name);
    name->setData(Qt::UserRole, route.routeId);
    name->setData(Qt::UserRole + 1, QVariant::fromValue(route.revision));
    routesTable_->setItem(row, 0, name);
    routesTable_->setItem(row, 1, readOnlyItem(QString::number(route.steps.size())));
    routesTable_->setItem(row, 2, readOnlyItem(QString::number(route.revision)));
  }
  routesCountLabel_->setText(QStringLiteral("%1 条").arg(visibleRoutes));
  for (int row = 0; row < routesTable_->rowCount(); ++row) {
    if (routesTable_->item(row, 0)->data(Qt::UserRole).toString() == currentRouteId_) {
      routesTable_->selectRow(row);
      break;
    }
  }
  if (routesTable_->currentRow() < 0 && selectedRow >= 0 && selectedRow < routesTable_->rowCount())
    routesTable_->selectRow(selectedRow);
  updateEmptyTable(routesTable_);
}

void PatrolPage::populateRuns() {
  const int selectedRow = runsTable_->currentRow();
  const auto selectedId = selectedRow >= 0 && runsTable_->item(selectedRow, 0)
      ? runsTable_->item(selectedRow, 0)->text() : QString();
  const QSignalBlocker blocker(runsTable_);
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
  updateEmptyTable(runsTable_);
  for (int row = 0; row < runsTable_->rowCount(); ++row)
    if (runsTable_->item(row, 0)->text() == selectedId) { runsTable_->selectRow(row); break; }
  if (runsTable_->currentRow() < 0 && runsTable_->rowCount()) runsTable_->selectRow(0);
  viewReportButton_->setEnabled(runsTable_->currentRow() >= 0);
  inspectionResultsTable_->setRowCount(0);
  updateEmptyTable(inspectionResultsTable_);
  inspectionPhoto_->setPhoto({});
  if (runsTable_->currentRow() >= 0)
    client_->refreshInspectionResults(activeMapId_, runsTable_->item(runsTable_->currentRow(), 0)->text(), 200);
}

void PatrolPage::populateInspectionResults() {
  const int selectedRow = inspectionResultsTable_->currentRow();
  const int runRow = runsTable_->currentRow();
  const auto runId = runRow >= 0 && runsTable_->item(runRow, 0) ? runsTable_->item(runRow, 0)->text() : QString();
  const QSignalBlocker blocker(inspectionResultsTable_);
  inspectionResultsTable_->setRowCount(0);
  for (const auto &item : client_->inspectionResults()) {
    if (!activeMapId_.isEmpty() && item.mapId != activeMapId_) continue;
    if (!runId.isEmpty() && item.runId != runId) continue;
    const int row = inspectionResultsTable_->rowCount();
    inspectionResultsTable_->insertRow(row);
    inspectionResultsTable_->setItem(row, 0, readOnlyItem(item.startedAtText));
    auto *point = readOnlyItem(item.pointName.isEmpty() ? item.pointId : item.pointName);
    point->setToolTip(QStringLiteral("圈 %1 · 第 %2 点 · 云台预置位 %3")
                          .arg(item.loopIndex).arg(item.stepIndex)
                          .arg(item.ptzPreset));
    inspectionResultsTable_->setItem(row, 1, point);
    auto *text = readOnlyItem(item.success ? item.text : item.message);
    text->setToolTip(item.prompt.isEmpty()
                         ? text->text()
                         : QStringLiteral("提示词：%1\n\n%2").arg(item.prompt, text->text()));
    inspectionResultsTable_->setItem(row, 2, text);
    auto *image = readOnlyItem(item.imagePath);
    image->setData(Qt::UserRole, item.imagePath);
    image->setToolTip(item.imagePath);
    inspectionResultsTable_->setItem(row, 3, image);
    inspectionResultsTable_->setItem(
        row, 4, readOnlyItem(item.success
                                 ? QStringLiteral("成功 · %1").arg(item.model)
                                 : QStringLiteral("失败 · %1").arg(item.errorCode)));
  }
  updateEmptyTable(inspectionResultsTable_);
  if (selectedRow >= 0 && selectedRow < inspectionResultsTable_->rowCount())
    inspectionResultsTable_->selectRow(selectedRow);
  else if (inspectionResultsTable_->rowCount()) inspectionResultsTable_->selectRow(0);
  previewSelectedInspection();
}

void PatrolPage::loadRouteIntoEditor(const PatrolRouteInfo &route) {
  startAfterPlan_=false;
  loadingRoute_ = true;
  currentRouteId_ = route.routeId;
  currentRouteRevision_ = route.revision;
  planDigest_.clear();
  navigationMode_->setCurrentIndex(0);
  int networkIndex=routeNetwork_->findData(route.networkId);
  if(networkIndex<0 && !route.networkId.isEmpty()) {
    routeNetwork_->addItem("已保存路网（待刷新）",route.networkId);networkIndex=routeNetwork_->count()-1;
    routeNetwork_->setItemData(networkIndex,route.networkRevision,Qt::UserRole+1);
  }
  if(networkIndex>=0) routeNetwork_->setCurrentIndex(networkIndex);
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
  planDigest_.clear();navigationMode_->setCurrentIndex(0);
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
    const QString action = step.inspectionEnabled
        ? QStringLiteral("云台抓拍巡检")
        : QStringLiteral("仅导航");
    stepsTable_->setItem(row, 4, readOnlyItem(action));
  }
  updateEmptyTable(stepsTable_);
  updateRoutePreview();
}

void PatrolPage::updateRoutePreview() {
  if (!routePreview_ || !routePreviewScene_) return;
  routePreviewScene_->clear();
  if (currentSteps_.isEmpty()) {
    auto *empty = routePreviewScene_->addSimpleText(QStringLiteral("尚未添加点位"));
    empty->setBrush(QColor("#94a3b8"));
    const QRectF rect(-100, -60, 200, 120);
    routePreviewScene_->setSceneRect(rect);
    routePreview_->fitInView(rect, Qt::KeepAspectRatio);
    QTimer::singleShot(0, routePreview_, [view=routePreview_, rect] {
      view->fitInView(rect, Qt::KeepAspectRatio);
    });
    return;
  }
  const QPen linePen(QColor("#94a3b8"), 2.0);
  QPointF previous;
  for (int i = 0; i < currentSteps_.size(); ++i) {
    const auto &step = currentSteps_[i];
    const QPointF point(step.x * 100.0, -step.y * 100.0);
    if (i > 0) routePreviewScene_->addLine(QLineF(previous, point), linePen);
    const bool inspection = step.inspectionEnabled;
    routePreviewScene_->addEllipse(
        point.x() - 9.0, point.y() - 9.0, 18.0, 18.0,
        QPen(QColor(inspection ? "#059669" : "#2563eb"), 2.0),
        QBrush(QColor(inspection ? "#ecfdf5" : "#ffffff")));
    auto *number = routePreviewScene_->addSimpleText(QString::number(i + 1));
    number->setBrush(QColor(inspection ? "#047857" : "#1d4ed8"));
    number->setPos(point.x() - 4.0, point.y() - 8.0);
    const QString label = (step.pointName.isEmpty() ? step.pointId : step.pointName) +
                          (inspection ? QStringLiteral(" · 云台巡检") : QString());
    auto *name = routePreviewScene_->addSimpleText(label);
    name->setBrush(QColor(inspection ? "#047857" : "#334155"));
    name->setPos(point.x() + 12.0, point.y() - 18.0);
    previous = point;
  }
  QRectF bounds = routePreviewScene_->itemsBoundingRect();
  if (bounds.width() < 160.0) bounds.adjust(-80.0, 0.0, 80.0, 0.0);
  if (bounds.height() < 80.0) bounds.adjust(0.0, -40.0, 0.0, 40.0);
  const QRectF rect = bounds.adjusted(-24.0, -20.0, 24.0, 20.0);
  routePreviewScene_->setSceneRect(rect);
  routePreview_->fitInView(rect, Qt::KeepAspectRatio);
  QTimer::singleShot(0, routePreview_, [view=routePreview_, rect] {
    view->fitInView(rect, Qt::KeepAspectRatio);
  });
}

void PatrolPage::updatePointFormFromSelection() {
  const int row = pointsTable_->currentRow();
  if (row < 0 || !pointsTable_->item(row, 0)) { loadPointShots(); return; }
  const QString pointId = pointsTable_->item(row, 0)->data(Qt::UserRole).toString();
  for (const auto &point : client_->points()) {
    if (point.pointId != pointId) continue;
    pointNameEdit_->setText(point.name);
    pointX_->setValue(point.x);
    pointY_->setValue(point.y);
    pointYaw_->setValue(point.yaw * 180.0 / kPi);
    pointInspectionEnabled_->setChecked(point.inspectionEnabled);
    pointPtzPreset_->setValue(point.ptzPreset > 0 ? point.ptzPreset : 1);
    pointRecognitionPrompt_->setPlainText(point.recognitionPrompt);
    loadPointShots();
    updateActionStates();
    return;
  }
}

QString PatrolPage::selectedPointId() const {
  const int row = pointsTable_->currentRow();
  return row >= 0 && pointsTable_->item(row, 0)
      ? pointsTable_->item(row, 0)->data(Qt::UserRole).toString() : QString();
}

void PatrolPage::loadPointShots() {
  const QString pointId = selectedPointId();
  if (pointId != shotPointId_) {
    shotPointId_ = pointId;
    selectedShotId_.clear();
    const QSignalBlocker block(pointShotsCombo_);
    pointShotsCombo_->clear();
    pointShotName_->clear();
  }
  if (pointId.isEmpty() || offline_ || !client_->pointShotsReady()) {
    pointShotDetails_->setText(pointId.isEmpty() ? QStringLiteral("请选择已保存点位")
        : QStringLiteral("视角服务未就绪，请启动更新后的巡检会话"));
    updateActionStates();
    return;
  }
  client_->refreshPointShots(pointId);
}

void PatrolPage::populatePointShots() {
  const auto info = client_->pointShots(selectedPointId());
  if (!info.loaded) {
    pointShotDetails_->setText(QStringLiteral("正在读取已保存视角…"));
    updateActionStates();
    return;
  }
  const QSignalBlocker block(pointShotsCombo_);
  pointShotsCombo_->clear();
  for (const auto &value : info.shots) {
    const auto shot = value.toObject();
    pointShotsCombo_->addItem(shot["name"].toString(), shot["shot_id"].toString());
  }
  if (info.shots.size() < 2) pointShotsCombo_->addItem(QStringLiteral("＋ 添加拍摄视角"), QString());
  const int index = pointShotsCombo_->findData(selectedShotId_);
  pointShotsCombo_->setCurrentIndex(index >= 0 ? index : 0);
  selectedShotId_ = pointShotsCombo_->currentData().toString();
  updateShotForm();
}

void PatrolPage::updateShotForm() {
  const auto info = client_->pointShots(selectedPointId());
  if (!info.loaded) { updateActionStates(); return; }
  QJsonObject selected;
  bool stale = false;
  for (const auto &value : info.shots) {
    const auto shot = value.toObject();
    stale = stale || shot["map_revision"].toVariant().toULongLong() != info.mapRevision;
    if (shot["shot_id"].toString() == selectedShotId_) selected = shot;
  }
  if (selected.isEmpty()) {
    pointShotName_->setText(QStringLiteral("视角%1").arg(info.shots.size()+1));
    pointShotSettle_->setValue(1.5);
    pointShotDetails_->setText(QStringLiteral("已保存 %1 / 2 个视角。调好相机后点击记录；只需一个时无需添加第二个。")
                                  .arg(info.shots.size()));
  } else {
    pointShotName_->setText(selected["name"].toString());
    pointShotSettle_->setValue(selected["settle_seconds"].toDouble(1.5));
    pointShotDetails_->setText(QStringLiteral("已保存 %1 / 2 个视角\n水平 %2° · 俯仰 %3° · %4×\n标定车头朝向 %5°")
        .arg(info.shots.size()).arg(selected["pan_cal"].toDouble(), 0, 'f', 2)
        .arg(selected["tilt"].toDouble(), 0, 'f', 2).arg(selected["zoom"].toDouble(), 0, 'f', 2)
        .arg(selected["yaw_cal"].toDouble()*180./kPi, 0, 'f', 2));
  }
  if (stale) pointShotDetails_->setText(pointShotDetails_->text() + QStringLiteral("\n地图已更新，请清空旧视角后重新记录。"));
  if (info.devicePanSign == 0)
    pointShotDetails_->setText(pointShotDetails_->text() + QStringLiteral("\n设备方向待确认：完成一次性设备配置后即可记录，所有点位自动沿用。"));
  else {
    for (const auto &value : info.shots)
      if (value.toObject()["pan_sign"].toInt() != info.devicePanSign) {
        pointShotDetails_->setText(pointShotDetails_->text() + QStringLiteral("\n设备方向配置已更新，请清空旧视角后重新记录。"));
        break;
      }
  }
  updateActionStates();
}

void PatrolPage::recordSelectedShot() {
  if (!pointRecordShot_->isEnabled()) return;
  const auto info = client_->pointShots(selectedPointId());
  if (!info.loaded) return;
  const QString name = pointShotName_->text().trimmed();
  const double settle = pointShotSettle_->value();
  if (selectedShotId_.isEmpty()) selectedShotId_ = "view_" + QUuid::createUuid().toString(QUuid::Id128);
  setMessage(QStringLiteral("正在读取实际角度、倍率与车头朝向，请保持车辆和云台静止…"));
  client_->recordPointShot(info.point.pointId, info.point.mapId, info.mapRevision,
                           info.point.revision, selectedShotId_, name, settle);
}

void PatrolPage::deleteSelectedShot() {
  if (!pointDeleteShot_->isEnabled()) return;
  const auto info = client_->pointShots(selectedPointId());
  if (info.loaded && !selectedShotId_.isEmpty())
    client_->removePointShot(info.point.pointId, info.point.mapId, info.mapRevision,
                            info.point.revision, selectedShotId_);
}

void PatrolPage::saveCurrentPoint() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先登记并设置活跃地图"), true); return; }
  QString name = pointNameEdit_->text().trimmed();
  if (name.isEmpty()) name = QStringLiteral("当前点");
  client_->saveCurrentPoint(
      activeMapId_, name, pointInspectionEnabled_->isChecked(),
      pointPtzPreset_->value(), pointRecognitionPrompt_->toPlainText().trimmed());
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
                       pointYaw_->value() * kPi / 180.0, revision,
                       pointInspectionEnabled_->isChecked(), pointPtzPreset_->value(),
                       pointRecognitionPrompt_->toPlainText().trimmed());
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
    step.inspectionEnabled = point.inspectionEnabled;
    step.ptzPreset = point.ptzPreset;
    step.recognitionPrompt = point.recognitionPrompt;
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
  if(!confirmDraftDiscard()) return;
  clearRouteEditor();
  routesTable_->clearSelection();
  setMessage(QStringLiteral("已新建路线草稿，请添加点位后保存"), false);
}

bool PatrolPage::confirmDraftDiscard() {
  if(!routeDirty_) return true;
  const auto choice=QMessageBox::question(this,"未保存路线","当前路线有未保存修改。保存后留在本页，或放弃修改继续？",
                                         QMessageBox::Save|QMessageBox::Discard|QMessageBox::Cancel,QMessageBox::Cancel);
  if(choice==QMessageBox::Save) {saveRoute();return false;}
  if(choice==QMessageBox::Discard) {routeDirty_=false;planDigest_.clear();return true;}
  return false;
}

void PatrolPage::closeEvent(QCloseEvent *event) {
  startAfterPlan_=false;planDigest_.clear();
  if(!confirmDraftDiscard()) {event->ignore();return;}
  // Fixed-route recording page is intentionally hidden in this build.
  QWidget::closeEvent(event);
}

void PatrolPage::saveRoute() {
  if (activeMapId_.isEmpty()) { setMessage(QStringLiteral("请先设置活跃地图"), true); return; }
  const QString name = routeNameEdit_->text().trimmed();
  if (name.isEmpty()) { setMessage(QStringLiteral("请填写路线名称"), true); return; }
  if (currentSteps_.isEmpty()) { setMessage(QStringLiteral("路线至少需要一个点位"), true); return; }
  const bool fixed=navigationMode_->currentData()=="RECORDED_NETWORK";
  if(fixed && routeNetwork_->currentData().toString().isEmpty()) {setMessage("请先在录制路网页发布路网",true);return;}
  client_->upsertRoute(currentRouteId_, activeMapId_, name, currentSteps_, currentRouteRevision_,
                      navigationMode_->currentData().toString(),fixed?routeNetwork_->currentData().toString():QString(),
                      fixed?routeNetwork_->currentData(Qt::UserRole+1).toULongLong():0);
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
  updateActionStates();
  if(!startButton_->isEnabled()) {setMessage(startButton_->toolTip(),true);return;}
  if (startPermission_) {
    QString reason;
    if (!startPermission_(&reason)) {
      setMessage(reason.isEmpty() ? QStringLiteral("当前不允许启动巡航") : reason, true);
      return;
    }
  }
  if(simpleRecordedPath()) {
    // Every explicit start plans from the current pose afresh.
    planDigest_.clear();
    startAfterPlan_=true;const auto generation=++startPlanGeneration_;
    setMessage("正在检查路径并规划自动接入，请保持车辆静止……",false);
    client_->roadCommand("PLAN",{{"route_id",currentRouteId_},{"loop_count",loopCount_->value()}});
    updateActionStates();
    QTimer::singleShot(20000,this,[this,generation]{
      if(startAfterPlan_&&generation==startPlanGeneration_) {
        startAfterPlan_=false;setMessage("路径检查超时，请确认后台状态后重新开始。",true);updateActionStates();
      }
    });
    return;
  }
  dispatchRun();
}

void PatrolPage::dispatchRun() {
  updateActionStates();
  if(!startButton_->isEnabled() || (navigationMode_->currentData()=="RECORDED_NETWORK" && planDigest_.isEmpty())) return;
  if(startPermission_) {
    QString reason;
    if(!startPermission_(&reason)) {setMessage(reason,true);return;}
  }
  const auto map = activeMap();
  client_->runRoute(currentRouteId_, activeMapId_, currentRouteRevision_, map.revision,
                    static_cast<quint32>(loopCount_->value()),planDigest_);
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
  if(startAfterPlan_) {startAfterPlan_=false;planDigest_.clear();updateActionStates();setMessage("已取消启动路径检查",false);return;}
  const auto status = client_->status();
  client_->cancelActiveTask();
  if (status.runId.startsWith(QStringLiteral("return_"))) {
    return;
  }
  if (status.runId.isEmpty()) { setMessage(QStringLiteral("当前没有可取消的巡航任务"), true); return; }
  client_->controlRun(3, status.runId);
}

void PatrolPage::onStatusChanged() {
  const auto status = client_->status();
  statusLabel_->setText(QStringLiteral("状态：%1%2")
                            .arg(PatrolClient::stateName(status.state),
                                 status.message.isEmpty() ? QString() : QStringLiteral(" · ") + status.message));
  progressLabel_->setText(status.runId.isEmpty() ? QStringLiteral("尚未执行路线")
                         : QStringLiteral("进度：第 %1 / %2 圈 · 第 %3 / %4 个点位 · 剩余停留 %5 s")
                              .arg(status.loopIndex).arg(status.totalLoops == 0 ? QStringLiteral("∞")
                                                                              : QString::number(status.totalLoops))
                              .arg(status.stepIndex).arg(status.stepCount)
                              .arg(QString::number(status.dwellRemaining, 'f', 1)));
  progressLabel_->setToolTip(QStringLiteral("任务：%1\n路线：%2").arg(status.runId, status.routeId));
  updateActionStates();
}

void PatrolPage::showOperation(const QString &operation, bool success, const QString &message) {
  setMessage(operation + (success ? QStringLiteral("：") : QStringLiteral("失败：")) + message, !success);
  if (operation == QStringLiteral("读取拍摄视角") && !success)
    pointShotDetails_->setText(QStringLiteral("读取视角失败，请点击“刷新”重试。"));
  if (success && operation.contains(QStringLiteral("保存路线"))) {
    routeDirty_ = false;
    client_->refreshRoutes(activeMapId_);
  }
  if (operation.contains(QStringLiteral("巡航任务"))) {
    client_->refreshRuns(activeMapId_);
    client_->refreshInspectionResults(activeMapId_, QString(), 200);
  }
  updateActionStates();
}

void PatrolPage::showError(const QString &message) { setMessage(message, true); }

void PatrolPage::setMessage(const QString &message, bool error) {
  messageLabel_->setText(message);
  patrolui::tone(messageLabel_, error ? "danger" : "info");
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

bool PatrolPage::simpleRecordedPath() const {
  if(navigationMode_->currentData()!="RECORDED_NETWORK" || currentRouteId_.isEmpty()) return false;
  for(auto value:client_->roadNetworks()) {
    const auto net=value.toObject();
    if(net["network_id"]!=routeNetwork_->currentData().toString()) continue;
    const auto doc=net["document"].toObject(),meta=doc["inspection_path"].toObject();
    if(meta["route_id"]!=currentRouteId_) return false;
    const auto edges=doc["edges"].toObject();const auto ids=meta["edge_ids"].toArray();
    QStringList nodes;
    for(auto id:ids) {
      const auto edge=edges[id.toString()].toObject();
      if(nodes.isEmpty() && !meta["closed"].toBool()) nodes<<edge["from"].toString();
      nodes<<edge["to"].toString();
    }
    if(nodes.size()!=currentSteps_.size()) return false;
    for(int i=0;i<nodes.size();++i)
      if(meta["point_ids"].toObject()[nodes[i]].toString()!=currentSteps_[i].pointId) return false;
    return true;
  }
  return false;
}

void PatrolPage::updateActionStates() {
  const bool available = client_ && client_->available();
  const bool hasMap = !activeMapId_.isEmpty();
  const auto status = client_ ? client_->status() : PatrolStatusInfo();
  const bool activeRun = status.active;
  const bool routeRun = activeRun && !status.runId.startsWith(QStringLiteral("return_"));
  const bool paused = status.state == 7 && routeRun;
  serviceLabel_->setText(offline_ ? QStringLiteral("离线编辑")
                        : !available ? QStringLiteral("巡航接口不可用")
                        : client_->routeServerReady() ? QStringLiteral("巡航服务已连接")
                                                    : QStringLiteral("巡航服务未就绪"));
  patrolui::tone(serviceLabel_, !offline_ && available && client_->routeServerReady() ? "good" : "warning");
  if (routeDraftLabel_) {
    routeDraftLabel_->setText(routeDirty_ ? QStringLiteral("未保存修改")
                           : currentRouteId_.isEmpty() ? QStringLiteral("新路线") : QStringLiteral("已保存"));
    patrolui::tone(routeDraftLabel_, routeDirty_ ? "warning" : "info");
  }
  registerMapButton_->setEnabled(available);
  setActiveButton_->setEnabled(available && mapsCombo_->currentIndex() >= 0);
  refreshAllButton_->setEnabled(available);
  const bool shotBusy = client_ && client_->pointShotsBusy();
  saveCurrentPointButton_->setEnabled(available && hasMap && !shotBusy);
  upsertPointButton_->setEnabled(available && hasMap && !shotBusy);
  const bool inspectionChecked = pointInspectionEnabled_->isChecked();
  pointInspectionEnabled_->setEnabled(available && hasMap && !shotBusy);
  pointPtzPreset_->setEnabled(inspectionChecked);
  pointRecognitionPrompt_->setEnabled(inspectionChecked);
  const bool cameraReady = !offline_ && ros_ && ros_->cameraAvailable() &&
                            ros_->ptzPresetAvailable();
  pointSavePtzPreset_->setEnabled(inspectionChecked && cameraReady && !activeRun && !shotBusy);
  pointGotoPtzPreset_->setEnabled(inspectionChecked && cameraReady && !activeRun && !shotBusy);
  const QString presetTip = cameraReady
      ? QStringLiteral("预置位由相机设备保存；配置拍摄视角后优先执行角度补偿拍照")
      : QStringLiteral("相机服务未就绪；先在右侧“相机”页启动相机");
  pointSavePtzPreset_->setToolTip(presetTip);
  pointGotoPtzPreset_->setToolTip(presetTip);
  deletePointButton_->setEnabled(available && hasMap && pointsTable_->currentRow() >= 0 && !shotBusy);
  const auto shotInfo = client_ ? client_->pointShots(selectedPointId()) : PatrolPointShotsInfo();
  const bool shotsReady = !offline_ && available && hasMap && !selectedPointId().isEmpty() &&
      client_->pointShotsReady() && shotInfo.loaded && shotInfo.point.mapId == activeMapId_;
  const bool editShots = shotsReady && !activeRun && !client_->hasActiveOrPendingRun() && !shotBusy;
  bool staleShots = false;
  for (const auto &value : shotInfo.shots)
    staleShots = staleShots || value.toObject()["map_revision"].toVariant().toULongLong() != shotInfo.mapRevision ||
        (shotInfo.devicePanSign != 0 && value.toObject()["pan_sign"].toInt() != shotInfo.devicePanSign);
  pointShotsCombo_->setEnabled(shotsReady && !shotBusy);
  pointShotName_->setEnabled(editShots && !staleShots);
  pointShotSettle_->setEnabled(editShots && !staleShots);
  pointRecordShot_->setEnabled(editShots && !staleShots && pointShotsCombo_->currentIndex() >= 0 &&
      (shotInfo.devicePanSign == -1 || shotInfo.devicePanSign == 1) && !pointShotName_->text().trimmed().isEmpty());
  pointRecordShot_->setText(shotBusy ? QStringLiteral("正在处理视角…")
      : pointShotsCombo_->currentData().toString().isEmpty() ? QStringLiteral("记录当前拍摄视角")
                                                         : QStringLiteral("重新记录所选视角"));
  pointRecordShot_->setToolTip(!shotsReady ? QStringLiteral("先选择已保存点位；视角服务需就绪，读取完成后可记录")
      : activeRun || client_->hasActiveOrPendingRun() ? QStringLiteral("请先结束巡检或返航")
      : staleShots ? QStringLiteral("地图或设备配置已更新，请清空旧视角后重新记录")
      : shotInfo.devicePanSign == 0 ? QStringLiteral("设备方向待确认，完成一次性设备配置后所有点位自动沿用")
      : QStringLiteral("只读取并保存实际位置，不移动云台；成功后自动启用该点巡检"));
  pointDeleteShot_->setEnabled(editShots && !selectedShotId_.isEmpty() && (!staleShots || shotInfo.shots.size() == 1));
  pointClearShots_->setEnabled(editShots && !shotInfo.shots.isEmpty());
  pointRefreshShots_->setEnabled(!offline_ && available && !selectedPointId().isEmpty() &&
                                  client_->pointShotsReady() && !shotBusy);
  if (refreshInspectionResultsButton_)
    refreshInspectionResultsButton_->setEnabled(available);
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
  QStringList startBlockers;
  if (shotBusy) startBlockers << QStringLiteral("正在记录或修改拍摄视角，请等待完成后启动巡检");
  if(recordingDraft_) startBlockers << QStringLiteral("录制草稿尚未处理，请先保存道路或放弃草稿");
  if (offline_) startBlockers << QStringLiteral("当前为离线模式，不能执行巡航");
  if (!available) {
    startBlockers << QStringLiteral("巡航接口未构建，请先构建巡航模块");
  } else if (!offline_ && !client_->routeServerReady()) {
    startBlockers << QStringLiteral("巡航后台未就绪，请在启动导航前勾选“路线巡航联锁”，再启动流程");
  }
  if (!hasMap) startBlockers << QStringLiteral("未设置活跃地图，请登记地图并点击“设为活跃地图”");
  if (currentRouteId_.isEmpty())
    startBlockers << QStringLiteral("未选择已保存路线，请选择路线，或新建后点击“保存路线”");
  if (routeDirty_) startBlockers << QStringLiteral("路线有未保存修改，请点击“保存路线”");
  const bool fixed=navigationMode_->currentData()=="RECORDED_NETWORK";
  routeNetwork_->setVisible(fixed);previewRecorded_->setVisible(fixed);
  previewRecorded_->setEnabled(!offline_ && !recordingDraft_ && !routeDirty_ && !activeRun && !currentRouteId_.isEmpty() && client_->roadServerReady());
  const bool simple=simpleRecordedPath();
  loopCount_->setSuffix(simple ? " 趟/圈" : "");
  loopCount_->setToolTip(simple?"非闭环：1趟到另一端，2趟往返，3趟再去；可从任一端点开始，返程沿原轨迹倒车。闭环每次完整走一圈。0为持续执行。":"0 表示持续循环，其他数字表示执行圈数");
  if(fixed && planDigest_.isEmpty() && !simple) startBlockers << "固定路线尚未验证，请点击“预览实际路线”；缺路时需补录";
  if(startAfterPlan_) startBlockers << "正在检查路径，请稍候；可点击取消任务终止启动";
  // The recorded-road page is hidden in this build, so its stale draft must
  // not block ordinary point-to-point navigation.
  if (activeRun) {
    startBlockers << (paused ? QStringLiteral("当前任务已暂停，请使用“继续”，或取消后再开始新任务")
                            : QStringLiteral("已有巡航或返航任务正在执行，请先取消并等待任务结束"));
  } else if (client_ && client_->hasActiveOrPendingRun()) {
    startBlockers << QStringLiteral("巡航或返航请求正在处理，请等待结果");
  }
  if (!startPermitted) {
    startBlockers.append(startPermissionReason.isEmpty()
                            ? QStringList{QStringLiteral("当前操作条件未满足，暂不能开始巡航")}
                            : startPermissionReason.split('\n', Qt::SkipEmptyParts));
  }
  startBlockers.removeDuplicates();
  const bool canStart = startBlockers.isEmpty();
  const QString startDisabledTip = startBlockers.join('\n');
  startButton_->setEnabled(canStart);
  startButton_->setToolTip(startDisabledTip);
  resetInterlockButton_->setEnabled(!offline_ && client_->available());
  startReadinessLabel_->setText(
      canStart ? QStringLiteral("启动条件：已就绪，可以点击“开始”")
               : QStringLiteral("启动条件：尚未就绪（%1 项）\n%2")
                     .arg(startBlockers.size()).arg(startBlockers.join(QStringLiteral("；"))));
  const QVariant previousReady = startReadinessLabel_->property("ready");
  if (!previousReady.isValid() || previousReady.toBool() != canStart) {
    startReadinessLabel_->setProperty("ready", canStart);
    startReadinessLabel_->setStyleSheet(
        canStart ? QStringLiteral("QLabel#patrolStartReadiness { color:#166534; background:#f0fdf4; border:1px solid #bbf7d0; border-radius:5px; }")
                 : QStringLiteral("QLabel#patrolStartReadiness { color:#92400e; background:#fffbeb; border:1px solid #fde68a; border-radius:5px; }"));
  }

  const auto &resumePermission = resumePermission_ ? resumePermission_ : startPermission_;
  QString resumePermissionReason;
  const bool resumePermitted = !resumePermission ||
                               resumePermission(&resumePermissionReason);
  const QString resumePermissionTip = resumePermitted ? QString() : resumePermissionReason;
  pauseButton_->setEnabled(!offline_ && available && routeRun && !paused);
  resumeButton_->setEnabled(!offline_ && available && routeRun && paused &&
                            resumePermitted);
  resumeButton_->setToolTip(resumePermissionTip);
  cancelButton_->setEnabled(!offline_ && available && (client_->hasActiveOrPendingRun()||startAfterPlan_));
  const auto home = client_->homePoint();
  const bool validHome = hasMap && home.exists && home.mapId == activeMapId_ && home.mapRevision == activeMap().revision;
  const bool returnRunning = client_->returnActiveOrPending();
  const auto &returnPermission = resumePermission_ ? resumePermission_ : startPermission_;
  QString returnReason;
  const bool returnPermitted = !returnPermission || returnPermission(&returnReason);
  saveHomeButton_->setEnabled(!offline_ && available && hasMap && !activeRun && startPermitted);
  saveHomeButton_->setToolTip(startPermissionTip);
  clearHomeButton_->setEnabled(!offline_ && available && validHome && !activeRun);
  returnHomeButton_->setEnabled(!offline_ && available && validHome && !returnRunning && returnPermitted && client_->returnServerReady());
  returnHomeButton_->setToolTip(!validHome ? QStringLiteral("先在地图与返航设置中保存当前地图的待命点")
      : returnPermitted ? QStringLiteral("结束当前路线或到点巡检，确认停止后返回待命点") : returnReason);
  pointCombo_->setEnabled(available && hasMap && pointCombo_->count() > 0);
  tabs_->setEnabled(available);
}
