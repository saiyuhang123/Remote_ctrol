#pragma once
#include "runtime/config.h"
#include <QObject>
#include <QJsonObject>
#include <memory>

// Independent ROS devices. Owns only processes explicitly created by this UI.
class DeviceClient : public QObject {
  Q_OBJECT
public:
  DeviceClient(const DesktopConfig &config, bool offline, QObject *parent = nullptr);
  ~DeviceClient() override;
  void startGas(const QString &port, int address, int baudrate, double interval);
  void stopGas();
  void startPtz(int speed, double seconds, double minimum, double maximum);
  void stopPtz();
  void setControlEnabled(bool enabled);
signals:
  void message(const QString &text);
  void status(const QString &component, const QJsonObject &value);
  void gasSample(const QJsonObject &value);
  void cameraRequested();
  void ptzBlocked(bool blocked);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
