#pragma once
#include "runtime/config.h"
#include <QWidget>
#include <QPointer>
class VisionClient;
class QDialog;
class VisionPanel : public QWidget {
  Q_OBJECT
public:
  VisionPanel(const DesktopConfig &config, bool offline, QWidget *parent = nullptr);
  ~VisionPanel() override;
  bool setRemoteFeature(const QString &parameter, bool enabled, QString *detail);
signals:
  void cameraRequested();
  void message(const QString &text);
private:
  VisionClient *client_;
  QPointer<QDialog> history_;
};
