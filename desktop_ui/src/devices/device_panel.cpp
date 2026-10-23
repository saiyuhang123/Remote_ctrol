#include "devices/device_panel.h"
#include "devices/device_client.h"
#include "devices/device_panel_style.h"
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QScrollArea>
#include <cmath>
#include <QTimer>
#include <QVBoxLayout>

struct DevicePanel::View : QObject {
  using QObject::QObject;
  QLineEdit *port;
  QSpinBox *address, *baudrate, *speed;
  QDoubleSpinBox *interval, *seconds, *minimum, *maximum;
  QCheckBox *bounds;
  QPushButton *gasStart, *gasStop, *ptzStart, *ptzStop;
  QLabel *gasStatus, *gasValue, *gasDetails, *ptzStatus;
  QLabel *gasSummary, *gasBadge, *gasAlarm, *gasEstimate, *gasFreshness;
  QLabel *ptzSummary, *ptzBadge, *ptzActivity, *ptzReadiness;
  QElapsedTimer gasAge, sampleAge, ptzAge;
  QJsonObject gas, sample, ptz;
  bool gasPending = false, ptzBlocked = false;
};

DevicePanel::DevicePanel(DeviceClient *client, bool offline, QWidget *parent)
  : QWidget(parent), client_(client), offline_(offline), view_(new View(this)) {
  Q_INIT_RESOURCE(console_icons);
  setObjectName("devicePanel");
  setAttribute(Qt::WA_StyledBackground);
  setStyleSheet(deviceui::styleSheet());
  auto &v = *view_;
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(16, 14, 16, 12);
  layout->setSpacing(10);
  auto *titleRow = new QHBoxLayout;
  auto *title = deviceui::label("设备执勤", this);
  title->setObjectName("deviceTitle");
  titleRow->addWidget(title);
  titleRow->addStretch();
  auto *mode = deviceui::label(offline_ ? "离线预览" : "独立设备", this);
  mode->setProperty("deviceBadge", true);
  titleRow->addWidget(mode);
  layout->addLayout(titleRow);
  layout->addWidget(deviceui::label("气体检测与云台巡航可独立运行，也可在车辆静止时使用。", this, true));

  auto *overview = new QHBoxLayout;
  overview->setSpacing(10);
  auto summaryCard = [&](const QString &name, QLabel *&value, QLabel *&badge) {
    auto *card = deviceui::card(this);
    auto *box = new QVBoxLayout(card);
    box->setContentsMargins(12, 10, 12, 10);
    box->setSpacing(6);
    auto *head = new QHBoxLayout;
    head->addWidget(deviceui::label(name, card, true));
    head->addStretch();
    badge = deviceui::label("未连接", card);
    badge->setProperty("deviceBadge", true);
    head->addWidget(badge);
    box->addLayout(head);
    value = deviceui::label("—", card);
    value->setProperty("deviceMetric", true);
    value->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    box->addWidget(value);
    overview->addWidget(card, 1);
  };
  summaryCard("气体检测", v.gasSummary, v.gasBadge);
  summaryCard("云台巡航", v.ptzSummary, v.ptzBadge);
  v.gasSummary->setObjectName("deviceGasSummary");
  v.ptzSummary->setObjectName("devicePtzSummary");
  layout->addLayout(overview);

  auto *tabs = new QTabWidget(this);
  tabs->setObjectName("deviceTabs");
  tabs->setDocumentMode(true);
  layout->addWidget(tabs, 1);
  auto makeTab = [&](const QString &name, const QString &objectName, QVBoxLayout *&bodyLayout) {
    auto *tab = new QWidget(tabs);
    auto *box = new QVBoxLayout(tab);
    box->setContentsMargins(0, 8, 0, 0);
    box->setSpacing(10);
    auto *scroll = new QScrollArea(tab);
    scroll->setObjectName(objectName);
    scroll->setWidgetResizable(true);
    scroll->setMinimumHeight(80);
    auto *body = new QWidget(scroll);
    bodyLayout = new QVBoxLayout(body);
    bodyLayout->setContentsMargins(0, 0, 0, 0);
    bodyLayout->setSpacing(10);
    scroll->setWidget(body);
    box->addWidget(scroll, 1);
    tabs->addTab(tab, name);
    return box;
  };
  auto button = [](const QString &text, const char *name, QWidget *parent) {
    auto *result = new QPushButton(text, parent);
    result->setObjectName(name);
    result->setAutoDefault(false);
    return result;
  };
  QVBoxLayout *gasBody;
  auto *gasTabLayout = makeTab("气体检测", "gasContentScroll", gasBody);
  auto *gas = deviceui::card(this);
  auto *g = new QVBoxLayout(gas);
  g->setContentsMargins(16, 14, 16, 14);
  g->setSpacing(8);
  auto *gasTop = new QHBoxLayout;
  auto *gasHeading = deviceui::label("实时检测", gas);
  gasHeading->setProperty("deviceHeading", true);
  gasTop->addWidget(gasHeading); gasTop->addStretch();
  v.gasAlarm = deviceui::label("等待采样", gas);
  v.gasAlarm->setObjectName("gasAlarm");
  v.gasAlarm->setProperty("deviceBadge", true);
  gasTop->addWidget(v.gasAlarm);
  g->addLayout(gasTop);
  g->addWidget(deviceui::label("原始寄存器数值 · GQ-OTK9633", gas, true));
  v.gasValue = deviceui::label("—", gas);
  v.gasValue->setObjectName("gasValue");
  v.gasValue->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  g->addWidget(v.gasValue);
  v.gasFreshness = deviceui::label("无新鲜采样", gas, true);
  v.gasFreshness->setObjectName("gasFreshness");
  g->addWidget(v.gasFreshness);
  v.gasEstimate = deviceui::label("估算显示：—", gas, true);
  v.gasEstimate->setObjectName("gasEstimate");
  g->addWidget(v.gasEstimate);
  v.gasStatus = deviceui::label(QString(), gas);
  v.gasStatus->setObjectName("gasStatus");
  g->addWidget(v.gasStatus);
  v.gasDetails = deviceui::label(QString(), gas, true);
  v.gasDetails->setObjectName("gasDetails");
  g->addWidget(v.gasDetails);
  gasBody->addWidget(gas);

  auto *connection = deviceui::card(this);
  auto *connectionLayout = new QVBoxLayout(connection);
  connectionLayout->setContentsMargins(12, 10, 12, 10);
  auto *settings = button("连接参数", "gasSettingsToggle", connection);
  settings->setCheckable(true);
  settings->setIcon(QIcon(":/console/chevron-down.svg"));
  settings->setToolTip("采集停止时设置；复用外部节点时显示并使用外部节点的实际配置。");
  connectionLayout->addWidget(settings);
  auto *settingsBody = new QWidget(connection);
  settingsBody->setObjectName("gasSettings");
  auto *form = new QFormLayout(settingsBody);
  form->setContentsMargins(0, 8, 0, 0); form->setSpacing(10);
  v.port = new QLineEdit("/dev/gq_otk9633", settingsBody); v.port->setObjectName("gasPort");
  v.port->setToolTip("明确指定气体仪固定串口，避免占用雷达/IMU 串口。复用外部节点时采用其现有配置。");
  form->addRow("固定串口", v.port);
  v.address = new QSpinBox(settingsBody); v.address->setObjectName("gasAddress");
  v.address->setRange(1, 254); v.address->setValue(1); form->addRow("设备地址", v.address);
  v.baudrate = new QSpinBox(settingsBody); v.baudrate->setObjectName("gasBaudrate");
  v.baudrate->setRange(300, 4000000); v.baudrate->setValue(2400); form->addRow("波特率 / 8N1", v.baudrate);
  v.interval = new QDoubleSpinBox(settingsBody); v.interval->setObjectName("gasInterval");
  v.interval->setRange(1.5, 60); v.interval->setValue(2); v.interval->setSuffix(" s"); form->addRow("完整采样间隔", v.interval);
  connectionLayout->addWidget(settingsBody);
  settingsBody->hide();
  connect(settings, &QPushButton::toggled, settingsBody, &QWidget::setVisible);
  connect(settings, &QPushButton::toggled, this, [settings](bool expanded) {
    settings->setText(expanded ? "收起连接参数" : "连接参数");
    settings->setIcon(QIcon(expanded ? ":/console/chevron-up.svg" : ":/console/chevron-down.svg"));
  });
  gasBody->addWidget(connection);
  gasBody->addWidget(deviceui::label("小数缩放需与仪表核对，报警状态以设备上报为准。", this, true));
  gasBody->addStretch();
  auto *gasActions = new QHBoxLayout;
  v.gasStart = button("启动气体检测", "gasStart", this);
  v.gasStart->setProperty("devicePrimary", true);
  v.gasStop = button("停止气体检测", "gasStop", this);
  v.gasStop->setProperty("deviceStop", true);
  gasActions->addWidget(v.gasStart, 1); gasActions->addWidget(v.gasStop, 1);
  gasTabLayout->addLayout(gasActions);

  QVBoxLayout *ptzBody;
  auto *ptzTabLayout = makeTab("云台巡航", "ptzContentScroll", ptzBody);
  auto *ptz = deviceui::card(this);
  auto *p = new QVBoxLayout(ptz);
  p->setContentsMargins(16, 14, 16, 14); p->setSpacing(10);
  auto *ptzHeading = deviceui::label("往返巡航", ptz);
  ptzHeading->setProperty("deviceHeading", true); p->addWidget(ptzHeading);
  v.ptzActivity = deviceui::label("状态未知", ptz);
  v.ptzActivity->setObjectName("ptzActivity"); p->addWidget(v.ptzActivity);
  v.ptzStatus = deviceui::label(QString(), ptz);
  v.ptzStatus->setObjectName("ptzPatrolStatus"); p->addWidget(v.ptzStatus);
  ptzBody->addWidget(ptz);
  auto *ptzParameters = deviceui::card(this);
  auto *parametersLayout = new QVBoxLayout(ptzParameters);
  parametersLayout->setContentsMargins(16, 14, 16, 14);
  auto *parametersHeading = deviceui::label("扫掠参数", ptzParameters);
  parametersHeading->setProperty("deviceHeading", true); parametersLayout->addWidget(parametersHeading);
  auto *ptzForm = new QFormLayout; ptzForm->setSpacing(10);
  v.speed = new QSpinBox(ptzParameters); v.speed->setObjectName("ptzSpeed");
  v.speed->setRange(1, 7); v.speed->setValue(2); ptzForm->addRow("速度档位", v.speed);
  v.seconds = new QDoubleSpinBox(ptzParameters); v.seconds->setObjectName("ptzSweepSeconds");
  v.seconds->setRange(1, 600); v.seconds->setValue(20); v.seconds->setSuffix(" s"); ptzForm->addRow("单侧扫掠时间", v.seconds);
  parametersLayout->addLayout(ptzForm);
  v.bounds = new QCheckBox("使用水平角度边界", ptzParameters); v.bounds->setObjectName("ptzBounds");
  parametersLayout->addWidget(v.bounds);
  auto *angleFields = new QWidget(ptzParameters); angleFields->setObjectName("ptzAngleFields");
  auto *angles = new QFormLayout(angleFields); angles->setContentsMargins(0, 0, 0, 0); angles->setSpacing(10);
  v.minimum = new QDoubleSpinBox(angleFields); v.minimum->setObjectName("ptzPanMinimum");
  v.minimum->setRange(0, 360); v.minimum->setSuffix("°");
  v.maximum = new QDoubleSpinBox(angleFields); v.maximum->setObjectName("ptzPanMaximum");
  v.maximum->setRange(0, 360); v.maximum->setValue(180); v.maximum->setSuffix("°");
  angles->addRow("最小水平角", v.minimum); angles->addRow("最大水平角", v.maximum);
  parametersLayout->addWidget(angleFields); angleFields->hide();
  connect(v.bounds, &QCheckBox::toggled, angleFields, &QWidget::setVisible);
  parametersLayout->addWidget(deviceui::label("角度反馈不可用时按时间换向；角度边界是换向参考，不是硬限位。", ptzParameters, true));
  ptzBody->addWidget(ptzParameters); ptzBody->addStretch();
  v.ptzReadiness = deviceui::label(QString(), this);
  v.ptzReadiness->setObjectName("ptzReadiness");
  v.ptzReadiness->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  ptzTabLayout->addWidget(v.ptzReadiness);
  auto *ptzActions = new QHBoxLayout;
  v.ptzStart = button("启动云台巡航", "ptzPatrolStart", this);
  v.ptzStart->setProperty("devicePrimary", true);
  v.ptzStop = button("停止云台巡航", "ptzPatrolStop", this);
  v.ptzStop->setProperty("deviceStop", true);
  ptzActions->addWidget(v.ptzStart, 1); ptzActions->addWidget(v.ptzStop, 1);
  ptzTabLayout->addLayout(ptzActions);
  layout->addWidget(deviceui::label("关闭此窗口继续执勤；退出主程序会停止本界面开启的功能。", this, true));
  connect(v.gasStart, &QPushButton::clicked, this, [this] {
    view_->gasPending = true; client_->startGas(view_->port->text(), view_->address->value(), view_->baudrate->value(), view_->interval->value()); refresh();
  });
  connect(v.gasStop, &QPushButton::clicked, this, [this] { view_->gasPending = false; client_->stopGas(); refresh(); });
  connect(v.ptzStart, &QPushButton::clicked, this, [this] {
    client_->startPtz(view_->speed->value(), view_->seconds->value(), view_->bounds->isChecked() ? view_->minimum->value() : -1,
                     view_->bounds->isChecked() ? view_->maximum->value() : -1);
  });
  connect(v.ptzStop, &QPushButton::clicked, client_, &DeviceClient::stopPtz);
  connect(v.bounds, &QCheckBox::toggled, this, [this] { refresh(); });
  connect(v.minimum, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] { refresh(); });
  connect(v.maximum, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this] { refresh(); });
  connect(client_, &DeviceClient::status, this, [this](const QString &component, const QJsonObject &value) {
    if (component == "gas") {
      view_->gas = value; view_->gasAge.restart();
      if (!value.value("enabled").toBool()) view_->gasPending = false;
      if (!value.value("enabled").toBool() || !value.value("connected").toBool()) {
        view_->sample = {}; view_->sampleAge.invalidate();
      }
    }
    if (component == "ptz") { view_->ptz = value; view_->ptzAge.restart(); }
    refresh();
  });
  connect(client_, &DeviceClient::gasSample, this, [this](const QJsonObject &value) { view_->sample = value; view_->sampleAge.restart(); refresh(); });
  connect(client_, &DeviceClient::ptzBlocked, this, [this](bool value) { view_->ptzBlocked = value; refresh(); });
  auto *timer = new QTimer(this); connect(timer, &QTimer::timeout, this, &DevicePanel::refresh); timer->start(500); refresh();
}
void DevicePanel::setControlEnabled(bool enabled) { authorized_ = enabled; refresh(); }
void DevicePanel::refresh() {
  auto &v = *view_;
  const bool gasFresh = v.gasAge.isValid() && v.gasAge.elapsed() < 2500;
  const bool enabled = v.gas.value("enabled").toBool();
  const bool connected = v.gas.value("connected").toBool();
  const auto gasError = v.gas.value("last_error").toString();
  const bool gasActive = v.gasPending || (gasFresh && (enabled || connected));
  v.gasStart->setEnabled(!offline_ && !gasActive);
  v.gasStop->setEnabled(!offline_);
  v.gasStart->setToolTip(offline_ ? "离线预览，不能启动设备" : gasActive ? "正在采集或等待串口关闭" : "开始读取气体检测仪");
  for (QWidget *widget : QList<QWidget *>{v.port, v.address, v.baudrate, v.interval})
    widget->setEnabled(!offline_ && !gasActive);

  QString gasState = offline_ ? "离线预览" : !gasFresh ? (v.gasAge.isValid() ? "状态过期" : "未连接")
      : !enabled ? (connected ? "正在停止" : "已停止")
      : !gasError.isEmpty() ? "设备异常" : !connected ? "正在连接" : "采集中";
  const char *gasTone = offline_ ? "muted" : gasFresh && enabled && connected && gasError.isEmpty() ? "good"
      : gasFresh && !gasError.isEmpty() ? "danger" : "muted";
  QString gasText = !gasFresh ? "未连接 / 状态已过期" :
      !enabled ? (connected ? "正在停止，等待串口关闭" : "采集已停止") :
      v.gas.value("state").toString() == "sampling" ? "正在采集" : "正在连接 / 重连";
  if (offline_) gasText = "离线预览，启动和停止操作已禁用";
  else if (gasFresh) {
    gasText += QString("\n实际串口 %1 · 地址 %2 · %3 baud")
        .arg(v.gas.value("port").toString()).arg(v.gas.value("address").toInt()).arg(v.gas.value("baudrate").toInt());
    if (!gasError.isEmpty()) gasText += "\n" + gasError;
  }
  v.gasStatus->setText(gasText);
  deviceui::tone(v.gasStatus, gasFresh && !gasError.isEmpty() ? "danger" : "muted");
  const double interval = v.gas.value("interval").toDouble(2);
  const double maxAge = std::isfinite(interval) ? qMax(6000., interval * 3000.) : 6000.;
  const bool sampleFresh = !offline_ && gasFresh && enabled && connected && gasError.isEmpty() &&
      v.sampleAge.isValid() && v.sampleAge.elapsed() < maxAge &&
      v.sample.value("raw_value").isDouble() && std::isfinite(v.sample.value("raw_value").toDouble());
  if (!sampleFresh) {
    v.gasValue->setText("—");
    v.gasValue->setToolTip({});
    v.gasSummary->setText("—");
    v.gasFreshness->setText("无新鲜采样 · 设备单位：—");
    v.gasAlarm->setText("无数据");
    v.gasEstimate->setText("估算显示：—");
    v.gasDetails->setText("量程 / 低报 / 高报原始值：— / — / —");
    deviceui::tone(v.gasValue, "muted");
    deviceui::tone(v.gasAlarm, "muted");
    if (qstrcmp(gasTone, "good") == 0) { gasState = "等待采样"; gasTone = "warning"; }
  } else {
    const auto &sample = v.sample;
    const QString unit = sample.value("unit").toString("未知单位");
    const QString raw = QString::number(sample.value("raw_value").toDouble(), 'g', 10);
    const int state = sample.value("state_code").toInt(-1);
    const QString alarm = state == 0 ? "正常" : state == 1 ? "低报" : state == 2 ? "高报" : "状态未知";
    const char *tone = state == 1 || state == 2 ? "danger" : state == 0 ? "good" : "warning";
    v.gasValue->setText(raw);
    v.gasValue->setToolTip("原始寄存器值，设备上报单位为 " + unit + "；小数缩放见下方估算显示。");
    v.gasAlarm->setText(alarm);
    v.gasSummary->setText(raw + "（原始）· " + unit);
    v.gasFreshness->setText(QString("设备单位：%1 · 最近采样 %2 秒前").arg(unit).arg(v.sampleAge.elapsed()/1000.,0,'f',1));
    deviceui::tone(v.gasValue, tone); deviceui::tone(v.gasAlarm, tone);
    gasTone = tone; gasState = state == 0 ? "采集中" : alarm;
    const auto scaled = sample.value("value_scaled");
    if (scaled.isDouble() && std::isfinite(scaled.toDouble())) {
      const bool verified = sample.value("decimal_scaling_verified").toBool();
      const int decimals = qBound(0, sample.value("decimals").toInt(1), 3);
      v.gasEstimate->setText(QString("%1 %2 %3（%4）")
          .arg(verified ? "显示值" : "≈", QString::number(scaled.toDouble(),'f',decimals), unit,
               verified ? "缩放已核对" : "估算，缩放待核对"));
    } else v.gasEstimate->setText("估算显示：—");
    auto rawField = [&sample](const char *name) {
      const auto value = sample.value(name);
      return value.isDouble() && std::isfinite(value.toDouble()) ? QString::number(value.toDouble(),'g',10) : QString("—");
    };
    v.gasDetails->setText(QString("量程 / 低报 / 高报原始值：%1 / %2 / %3")
        .arg(rawField("range_raw"),rawField("low_alarm_raw"),rawField("high_alarm_raw")));
  }
  v.gasBadge->setText(gasState); deviceui::tone(v.gasBadge, gasTone);
  deviceui::tone(v.gasSummary, gasTone);

  const bool ptzFresh = v.ptzAge.isValid() && v.ptzAge.elapsed() < 2000;
  const bool validBounds = !v.bounds->isChecked() || v.minimum->value() < v.maximum->value();
  v.ptzStart->setEnabled(!offline_ && authorized_ && !v.ptzBlocked && validBounds);
  v.ptzStop->setEnabled(!offline_);
  v.speed->setEnabled(!v.ptzBlocked); v.seconds->setEnabled(!v.ptzBlocked); v.bounds->setEnabled(!v.ptzBlocked);
  v.minimum->setEnabled(!v.ptzBlocked && v.bounds->isChecked()); v.maximum->setEnabled(!v.ptzBlocked && v.bounds->isChecked());
  QString ptzState = offline_ ? "离线预览" : !ptzFresh ? "状态未知"
      : v.ptz.value("state").toString() == "inspection_stopping" ? "巡检停转中"
      : v.ptz.value("inspection_held").toBool() ? "巡检拍照中"
      : v.ptz.value("state").toString() == "stopping" ? "正在停止"
      : !v.ptz.value("last_error").toString().isEmpty() ? "设备异常"
      : v.ptz.value("state").toString() == "stopped" ? "已停止"
      : v.ptz.value("moving").toBool() ? "往返扫掠中" : "等待相机";
  const char *ptzTone = offline_ ? "muted" : ptzFresh && !v.ptz.value("last_error").toString().isEmpty() ? "danger"
      : v.ptzBlocked && !ptzFresh ? "warning"
      : ptzFresh && v.ptz.value("moving").toBool() ? "info" : "muted";
  QString ptzText = !ptzFresh ? "尚无新鲜云台巡航状态" :
      v.ptz.value("state").toString() == "inspection_stopping" ? "已暂停自转，等待云台停止确认" :
      v.ptz.value("inspection_held").toBool() ? "到点抓拍 / 停留期间暂停自转，离点后恢复原先设置" :
      v.ptz.value("state").toString() == "stopping" ? "正在停止，等待设备确认" :
      !v.ptz.value("last_error").toString().isEmpty() ? "云台巡航异常" :
      v.ptz.value("state").toString() == "stopped" ? "巡航已停止，可使用手动云台" :
      v.ptz.value("moving").toBool() ? "正在往返扫掠" : "等待相机 / 处理请求";
  if (ptzFresh) {
    const QString actual = QString("速度 %1 档 · 单侧 %2 s").arg(v.ptz.value("pan_speed").toInt()).arg(v.ptz.value("sweep_seconds").toDouble());
    ptzText += "\n实际配置：" + actual;
    v.ptzSummary->setText(actual);
    if (!v.ptz.value("last_error").toString().isEmpty()) ptzText += "\n" + v.ptz.value("last_error").toString();
  } else v.ptzSummary->setText("—");
  if (v.ptzBlocked) ptzText += "\n手动云台已锁定；需收到巡航停止反馈后恢复";
  if (!authorized_) ptzText += "\n开始旋转前请勾选主界面“启用操作”";
  v.ptzStatus->setText(ptzText);
  v.ptzActivity->setText(ptzState); v.ptzBadge->setText(ptzState);
  deviceui::tone(v.ptzActivity, ptzTone); deviceui::tone(v.ptzBadge, ptzTone);
  QStringList blockers;
  if (offline_) blockers << "离线预览，无法启动设备";
  if (!authorized_) blockers << "请勾选主界面“启用操作”";
  if (v.ptzBlocked) blockers << (!ptzFresh ? "状态失联，等待新鲜停止反馈" : "巡航已占用或正在停止，请先等待停止确认");
  if (!validBounds) blockers << "最小水平角必须小于最大水平角";
  const bool ready = blockers.isEmpty();
  v.ptzReadiness->setText(ready ? "启动条件已就绪；巡航运行时会锁定手动云台。" : blockers.join("；"));
  deviceui::tone(v.ptzReadiness, ready ? "good" : "warning");
  v.ptzStart->setToolTip(blockers.join('\n'));
}
