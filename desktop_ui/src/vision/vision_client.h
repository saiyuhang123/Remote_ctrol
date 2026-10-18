#pragma once
#include "runtime/config.h"
#include <QObject>
#include <QJsonObject>
#include <QJsonArray>
#include <memory>

class VisionClient : public QObject {
  Q_OBJECT
public:
  VisionClient(const DesktopConfig &config, bool offline, QObject *parent = nullptr);
  ~VisionClient() override;
  bool supported() const;
  void startFace();
  void stopFace();
  void startDetector(bool person, bool plate);
  void stopDetector();
  void query(const QString &type = {}, const QString &cursor = {});
  void setMapLabel(const QString &label);
signals:
  void cameraRequested();
  void message(const QString &text);
  void status(const QString &component, const QJsonObject &value);
  void records(const QJsonArray &items, const QString &nextCursor, quint64 total);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
