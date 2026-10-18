#pragma once
#include "runtime/process_manager.h"
#include <QDateTime>
class RosAdapter;

class MapSaver : public QObject {
  Q_OBJECT
public:
  MapSaver(const DesktopConfig &config, RosAdapter *ros, QObject *parent = nullptr);
  bool busy() const { return busy_; }
  void save(const Profile &profile, const QString &prefix);
signals:
  void message(const QString &text);
  void busyChanged(bool busy);
  void saved(bool success, const QString &detail);
private:
  void finish(bool success, const QString &detail);
  RosAdapter *ros_;
  ProcessManager export_;
  ProcessManager webExport_;
  QString resourceDir_, projectRoot_;
  QString backend_, prefix_, localDetail_;
  bool webExportPending_ = false;
  QDateTime started_;
  quint64 generation_ = 0;
  bool busy_ = false;
};
