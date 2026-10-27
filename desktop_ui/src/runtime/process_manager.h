#pragma once
#include "runtime/config.h"
#include <QProcess>

class SessionProcess : public QProcess {
public:
  using QProcess::QProcess;
protected:
  void setupChildProcess() override;
};

// Supervises each session tree and replaces only matching local instances.
class ProcessManager : public QObject {
  Q_OBJECT
public:
  explicit ProcessManager(const DesktopConfig &config, QObject *parent = nullptr);
  ~ProcessManager() override;
  bool running() const;
  bool start(const QStringList &arguments);
  void stop();
  static QStringList launchArguments(const DesktopConfig &config, const Profile &profile,
                                    const QString &mapPrefix, bool sensors, bool chassis,
                                    bool control, const QString &odomSource,
                                    const QString &navigationYaml = QString(),
                                    bool enablePatrol = false);
  static QStringList environmentArguments(const DesktopConfig &config, const QStringList &arguments);
signals:
  void output(const QString &text);
  void runningChanged(bool running);
  void finished(int exitCode);
private:
  DesktopConfig config_;
  SessionProcess process_;
  qint64 group_ = 0;
  quint64 generation_ = 0;
  bool stopping_ = false;
  bool stopRequestedDuringStart_ = false;
};
