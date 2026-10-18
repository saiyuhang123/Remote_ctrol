#include "devices/device_panel.h"
#include "devices/device_client.h"
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
  QElapsedTimer gasAge, sampleAge, ptzAge;
  QJsonObject gas, sample, ptz;
  bool gasPending = false, ptzBlocked = false;
};

DevicePanel::DevicePanel(DeviceClient *client, bool offline, QWidget *parent)
  : QWidget(parent), client_(client), offline_(offline), view_(new View(this)) {
  setObjectName("devicePanel"); auto &v = *view_;
  auto *layout = new QVBoxLayout(this);
  auto *hint = new QLabel("独立于建图、导航和路线任务。关闭此窗口后继续执勤；退出主程序会停止本界面开启的功能。");
  hint->setWordWrap(true); layout->addWidget(hint);
  auto *gas = new QGroupBox("气体检测 · GQ-OTK9633"); auto *g = new QVBoxLayout(gas);
  auto *form = new QFormLayout;
  v.port = new QLineEdit("/dev/gq_otk9633"); v.port->setObjectName("gasPort");
  v.port->setToolTip("明确指定气体仪固定串口，避免占用雷达/IMU 串口。复用外部节点时采用其现有配置。");
  form->addRow("固定串口", v.port);
  v.address = new QSpinBox; v.address->setRange(1, 254); v.address->setValue(1); form->addRow("设备地址", v.address);
  v.baudrate = new QSpinBox; v.baudrate->setRange(300, 4000000); v.baudrate->setValue(2400); form->addRow("波特率 / 8N1", v.baudrate);
  v.interval = new QDoubleSpinBox; v.interval->setRange(1.5, 60); v.interval->setValue(2); v.interval->setSuffix(" s"); form->addRow("完整采样间隔", v.interval);
  g->addLayout(form);
  auto *gasButtons = new QHBoxLayout;
  v.gasStart = new QPushButton("启动气体检测"); v.gasStart->setObjectName("gasStart");
  v.gasStop = new QPushButton("停止气体检测"); v.gasStop->setObjectName("gasStop");
  gasButtons->addWidget(v.gasStart); gasButtons->addWidget(v.gasStop); g->addLayout(gasButtons);
  v.gasValue = new QLabel("当前检测值：—"); v.gasValue->setObjectName("gasValue");
  v.gasStatus = new QLabel; v.gasStatus->setObjectName("gasStatus");
  v.gasDetails = new QLabel("缩放值待与仪表核对；报警以设备上报状态为准。");
  for (auto *label : {v.gasValue, v.gasStatus, v.gasDetails}) { label->setWordWrap(true); g->addWidget(label); }
  layout->addWidget(gas);
  auto *ptz = new QGroupBox("云台往返巡航"); auto *p = new QVBoxLayout(ptz);
  auto *ptzForm = new QFormLayout;
  v.speed = new QSpinBox; v.speed->setRange(1, 7); v.speed->setValue(2); ptzForm->addRow("速度档位", v.speed);
  v.seconds = new QDoubleSpinBox; v.seconds->setRange(1, 600); v.seconds->setValue(20); v.seconds->setSuffix(" s"); ptzForm->addRow("单侧扫掠时间", v.seconds);
  v.bounds = new QCheckBox("同时使用水平角度边界"); v.bounds->setObjectName("ptzBounds"); ptzForm->addRow(v.bounds);
  v.minimum = new QDoubleSpinBox; v.minimum->setRange(0, 360); v.minimum->setSuffix("°");
  v.maximum = new QDoubleSpinBox; v.maximum->setRange(0, 360); v.maximum->setValue(180); v.maximum->setSuffix("°");
  ptzForm->addRow("最小水平角", v.minimum); ptzForm->addRow("最大水平角", v.maximum); p->addLayout(ptzForm);
  auto *ptzButtons = new QHBoxLayout;
  v.ptzStart = new QPushButton("启动云台巡航"); v.ptzStart->setObjectName("ptzPatrolStart");
  v.ptzStop = new QPushButton("停止云台巡航"); v.ptzStop->setObjectName("ptzPatrolStop");
  ptzButtons->addWidget(v.ptzStart); ptzButtons->addWidget(v.ptzStop); p->addLayout(ptzButtons);
  v.ptzStatus = new QLabel; v.ptzStatus->setObjectName("ptzPatrolStatus"); v.ptzStatus->setWordWrap(true); p->addWidget(v.ptzStatus);
  auto *ptzHint = new QLabel("旋转需勾选主界面“启用操作”；巡航期间锁定手动云台。角度反馈不可用时按时间换向；角度边界不作为硬限位。");
  ptzHint->setWordWrap(true); p->addWidget(ptzHint); layout->addWidget(ptz); layout->addStretch();
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
    if (component == "gas") { view_->gas = value; view_->gasAge.restart(); if (!value.value("enabled").toBool()) view_->gasPending = false; }
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
  const bool gasActive = v.gasPending || (gasFresh && (v.gas.value("enabled").toBool() || v.gas.value("connected").toBool()));
  v.gasStart->setEnabled(!offline_ && !gasActive); v.gasStop->setEnabled(!offline_);
  for (QWidget *widget : QList<QWidget *>{v.port, v.address, v.baudrate, v.interval}) widget->setEnabled(!offline_ && !gasActive);
  const QStringList states{"正常", "低报", "高报"};
  QString gasText = !gasFresh ? "未连接 / 状态已过期" :
    !v.gas.value("enabled").toBool() ? (v.gas.value("connected").toBool() ? "正在停止，等待串口关闭" : "采集已停止") :
    v.gas.value("state").toString() == "sampling" ? "正在采集" : "正在连接 / 重连";
  if (gasFresh) gasText += QString("\n实际串口 %1 · 地址 %2 · %3 baud").arg(v.gas.value("port").toString()).arg(v.gas.value("address").toInt()).arg(v.gas.value("baudrate").toInt());
  if (gasFresh && !v.gas.value("last_error").toString().isEmpty()) gasText += "\n" + v.gas.value("last_error").toString();
  v.gasStatus->setText(gasText);
  const bool sampleFresh = gasFresh && v.gas.value("enabled").toBool() && v.sampleAge.isValid() &&
      v.sampleAge.elapsed() < qMax(6000., v.gas.value("interval").toDouble(2) * 3000.);
  if (!sampleFresh) {
    v.gasValue->setText("当前检测值：—（无新鲜采样）");
    v.gasValue->setStyleSheet({});
  } else {
    const auto &s = v.sample; const int state = s.value("state_code").toInt(-1);
    const QString stateText = state >= 0 && state < states.size() ? states[state] : "未知状态";
    v.gasValue->setText(QString("原始值 %1 · 单位 %2 · 设备状态：%3").arg(s.value("raw_value").toInt()).arg(s.value("unit").toString()).arg(stateText));
    v.gasValue->setStyleSheet(state == 1 || state == 2 ? "color:#b42318;font-weight:600;" : "");
    v.gasDetails->setText(QString("估算显示值 %1 %2（缩放尚未核对）\n量程 / 低报 / 高报原始值：%3 / %4 / %5\n本界面只读，不修改报警阈值或执行标定。").arg(s.value("value_scaled").isDouble() ? QString::number(s.value("value_scaled").toDouble()) : "—")
      .arg(s.value("unit").toString()).arg(s.value("range_raw").toInt()).arg(s.value("low_alarm_raw").toInt()).arg(s.value("high_alarm_raw").toInt()));
  }
  const bool ptzFresh = v.ptzAge.isValid() && v.ptzAge.elapsed() < 2000;
  const bool validBounds = !v.bounds->isChecked() || v.minimum->value() < v.maximum->value();
  v.ptzStart->setEnabled(!offline_ && authorized_ && !v.ptzBlocked && validBounds);
  v.ptzStop->setEnabled(!offline_);
  v.speed->setEnabled(!v.ptzBlocked); v.seconds->setEnabled(!v.ptzBlocked); v.bounds->setEnabled(!v.ptzBlocked);
  v.minimum->setEnabled(!v.ptzBlocked && v.bounds->isChecked()); v.maximum->setEnabled(!v.ptzBlocked && v.bounds->isChecked());
  QString ptzText = !ptzFresh ? "尚无新鲜云台巡航状态" :
      v.ptz.value("state").toString() == "stopped" ? "巡航已停止，可使用手动云台" :
      v.ptz.value("state").toString() == "stopping" ? "正在停止，等待设备确认" :
      v.ptz.value("moving").toBool() ? "正在往返扫掠" : "等待相机 / 处理请求";
  if (ptzFresh) ptzText += QString("\n实际配置：速度 %1 档 · 单侧 %2 s").arg(v.ptz.value("pan_speed").toInt()).arg(v.ptz.value("sweep_seconds").toDouble());
  if (ptzFresh && !v.ptz.value("last_error").toString().isEmpty()) ptzText += "\n" + v.ptz.value("last_error").toString();
  if (v.ptzBlocked) ptzText += "\n手动云台已锁定；需收到巡航停止反馈后恢复";
  if (!authorized_) ptzText += "\n开始旋转前请勾选主界面“启用操作”";
  v.ptzStatus->setText(ptzText);
}
