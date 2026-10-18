#include "devices/gas_readout.h"
#include "devices/device_client.h"
#include "ui/console_theme.h"
#include <QLabel>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <cmath>

GasReadout::GasReadout(DeviceClient *client, bool offline, QWidget *parent)
  : QWidget(parent), offline_(offline) {
  setObjectName("consoleGasReadout");
  setMinimumWidth(120);
  auto *layout = new QVBoxLayout(this); layout->setContentsMargins(0, 0, 0, 0); layout->setSpacing(4);
  auto *open = new QToolButton; open->setObjectName("consoleGasOpen"); open->setText("气体检测 ↗");
  open->setAutoRaise(true); open->setCursor(Qt::PointingHandCursor);
  open->setToolTip("打开设备执勤，启动或停止气体检测");
  open->setStyleSheet("QToolButton { padding:0; border:0; text-align:left; }");
  layout->addWidget(open, 0, Qt::AlignLeft);
  value_ = new QLabel("—"); value_->setObjectName("consoleGasValue");
  value_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  state_ = new QLabel; state_->setObjectName("consoleGasState");
  state_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  layout->addWidget(value_); layout->addWidget(state_);
  connect(open, &QToolButton::clicked, this, &GasReadout::openRequested);
  connect(client, &DeviceClient::status, this, [this](const QString &component, const QJsonObject &value) {
    if (component != "gas") return;
    status_ = value; statusAge_.restart();
    if (!value.value("enabled").toBool() || !value.value("connected").toBool()) {
      sample_ = {}; sampleAge_.invalidate();
    }
    refresh();
  });
  connect(client, &DeviceClient::gasSample, this, [this](const QJsonObject &value) {
    sample_ = value; sampleAge_.restart(); refresh();
  });
  auto *timer = new QTimer(this); timer->setInterval(250);
  connect(timer, &QTimer::timeout, this, &GasReadout::refresh); timer->start(); refresh();
}

void GasReadout::refresh() {
  const bool freshStatus = statusAge_.isValid() && statusAge_.elapsed() < 2500;
  const bool enabled = status_.value("enabled").toBool();
  const bool connected = status_.value("connected").toBool();
  const double interval = status_.value("interval").toDouble(2.0);
  const double maxAge = std::isfinite(interval) ? qMax(6000., interval * 3000.) : 6000.;
  const bool freshSample = sampleAge_.isValid() && sampleAge_.elapsed() < maxAge;
  const auto error = status_.value("last_error").toString();
  QString value = "—", state, tooltip;
  const char *tone = "muted";
  if (offline_) state = "离线预览";
  else if (!freshStatus) state = statusAge_.isValid() ? "连接状态已过期" : "尚未采集";
  else if (!enabled) state = connected ? "正在停止" : "采集已停止";
  else if (!connected || !error.isEmpty()) { state = error.isEmpty() ? "正在连接" : "设备异常"; tone = "warning"; }
  else if (!freshSample) state = sampleAge_.isValid() ? "采样已过期" : "等待新采样";
  else if (!sample_.value("raw_value").isDouble()) { state = "采样数据无效"; tone = "warning"; }
  else {
    const auto unit = sample_.value("unit").toString("未知单位");
    const auto scaled = sample_.value("value_scaled");
    const bool hasScaled = scaled.isDouble() && std::isfinite(scaled.toDouble());
    const bool verified = sample_.value("decimal_scaling_verified").toBool();
    const auto raw = QString::number(sample_.value("raw_value").toDouble(), 'g', 8);
    if (hasScaled) {
      const int decimals = qBound(0, sample_.value("decimals").toInt(1), 3);
      value = (verified ? "" : "≈ ") + QString::number(scaled.toDouble(), 'f', decimals) + " " + unit;
    } else value = raw + "（原始）";
    const int alarm = sample_.value("state_code").toInt(-1);
    const QString alarmText = alarm == 0 ? "正常" : alarm == 1 ? "低报" : alarm == 2 ? "高报" : "状态未知";
    state = hasScaled && !verified ? "估算 · " + alarmText : hasScaled ? alarmText : unit + " · " + alarmText;
    tone = alarm == 1 || alarm == 2 ? "danger" : alarm == 0 ? "good" : "warning";
    tooltip = QString("原始寄存器值：%1\n设备单位：%2\n设备状态：%3\n").arg(raw, unit, alarmText);
    if (hasScaled && !verified) tooltip += "≈ 为推算的小数缩放值，尚需与仪表显示核对。报警来自设备上报状态。\n";
  }
  if (!error.isEmpty()) tooltip += "设备信息：" + error + "\n";
  tooltip += "点击“气体检测”打开设备执勤窗口。";
  value_->setText(value); state_->setText(state);
  value_->setToolTip(tooltip); state_->setToolTip(tooltip);
  setConsoleTone(value_, tone); setConsoleTone(state_, tone);
}
