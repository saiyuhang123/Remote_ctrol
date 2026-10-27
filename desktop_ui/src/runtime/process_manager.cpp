#include "runtime/process_manager.h"
#include <QDir>
#include <QFileInfo>
#include <QTimer>
#include <csignal>
#include <unistd.h>

namespace {
QString supervisorPath(const DesktopConfig &config) {
  const QString installed = config.resourceDir + "/scripts/managed_process.py";
  return QFileInfo::exists(installed) ? installed : config.projectRoot + "/desktop_ui/scripts/managed_process.py";
}
}

void SessionProcess::setupChildProcess() { ::setsid(); }
ProcessManager::ProcessManager(const DesktopConfig &config, QObject *parent) : QObject(parent), config_(config) {
  process_.setProcessChannelMode(QProcess::MergedChannels);
  connect(&process_, &QProcess::readyReadStandardOutput, this, [this] { emit output(QString::fromUtf8(process_.readAllStandardOutput())); });
  connect(&process_, &QProcess::started, this, [this] {
    group_ = process_.processId(); emit runningChanged(true);
    if (stopRequestedDuringStart_) { stopRequestedDuringStart_ = false; stop(); }
  });
  connect(&process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) { emit output("启动器：" + process_.errorString()); if (error == QProcess::FailedToStart) { group_ = 0; stopping_ = false; ++generation_; emit runningChanged(false); emit finished(-1); } });
  connect(&process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, [this](int code, QProcess::ExitStatus) {
    // A failed launch can leave children. They still belong to this unique session group.
    if (group_ > 0) ::kill(-group_, SIGTERM);
    group_ = 0; stopping_ = false; stopRequestedDuringStart_ = false; ++generation_; emit runningChanged(false); emit finished(code);
  });
}
ProcessManager::~ProcessManager() {
  if (group_ > 0 && !stopping_) ::kill(group_, SIGINT);
  if (process_.state() != QProcess::NotRunning && !process_.waitForFinished(1500)) {
    if (group_ > 0) ::kill(-group_, SIGKILL);
    process_.waitForFinished(500);
  }
}
bool ProcessManager::running() const { return process_.state() != QProcess::NotRunning; }
QStringList ProcessManager::environmentArguments(const DesktopConfig &config, const QStringList &arguments) {
  // All user paths/arguments are positional parameters, never shell source text.
  return QStringList{"-c", "set -e\nsource /opt/ros/humble/setup.bash\nsource \"$1/ros2_ws/install/setup.bash\"\nexport ROBOT_PROJECT_ROOT=\"$1\"\nexport ROBOT_DESKTOP_SESSION_NODES=\"$2\"\nMANAGED_PROCESS=\"$3\"\nshift 3\nexec python3 \"$MANAGED_PROCESS\" -- \"$@\"", "robot-desktop-session", config.projectRoot, config.sessionNodeNames.join(','), supervisorPath(config)} + arguments;
}
bool ProcessManager::start(const QStringList &arguments) {
  if (running() || arguments.isEmpty()) return false;
  if (!QFileInfo::exists(config_.projectRoot + "/ros2_ws/install/setup.bash")) { emit output("缺少 ROS2 overlay，请先按原项目方式构建工作空间"); return false; }
  if (!QFileInfo::exists(supervisorPath(config_))) { emit output("缺少进程监督器 scripts/managed_process.py"); return false; }
  ++generation_;
  stopping_ = false;
  stopRequestedDuringStart_ = false;
  process_.setWorkingDirectory(config_.projectRoot);
  process_.start("bash", environmentArguments(config_, arguments));
  return true;
}
void ProcessManager::stop() {
  if (!running() || stopping_) return;
  if (group_ <= 0) { stopRequestedDuringStart_ = true; return; }
  stopping_ = true;
  const auto group = group_; const auto generation = generation_;
  // The supervisor forwards one interrupt and reaps the whole process tree.
  ::kill(group, SIGINT);
  QTimer::singleShot(5000, this, [this, group, generation] {
    if (group_ != group || generation_ != generation) return;
    ::kill(-group, SIGTERM);
    QTimer::singleShot(3000, this, [this, group, generation] { if (group_ == group && generation_ == generation) ::kill(-group, SIGKILL); });
  });
}
QStringList ProcessManager::launchArguments(const DesktopConfig &, const Profile &p, const QString &prefix,
                                          bool sensors, bool chassis, bool control, const QString &odom,
                                          const QString &navigationYaml, bool enablePatrol) {
  auto boolean = [](bool v) { return v ? "true" : "false"; };
  const QString navigationMap =
      navigationYaml.trimmed().isEmpty() ? prefix + ".yaml" : navigationYaml.trimmed();
  QStringList args{"ros2", "launch", "chassis_nav2", p.launchFile, "rviz:=false",
                   QString("start_sensors:=") + boolean(sensors), QString("start_chassis:=") + boolean(chassis),
                   QString("enable_control:=") + boolean(control)};
  // The patrol interlock changes twist_mux behavior and starts two companion
  // nodes. Only opt in explicitly; ordinary single-point/manual navigation
  // must not silently depend on a separately launched patrol service.
  // Ordinary point patrol needs the executor/guard, but no recorded-road
  // planners or controllers. Recorded roads remain an explicit CLI opt-in.
  if (enablePatrol && p.navigation && !p.mapping)
    args << "enable_patrol:=true" << "enable_recorded_routes:=false";
  if (p.backend == "fastlio") {
    args << "pcd_map:=" + prefix + ".pcd";
    args << "odom_source:=" + (p.mapping ? QString("lio") : odom);
    if (!p.mapping) args << "map:=" + navigationMap;
  } else if (!p.mapping) {
    args << "map:=" + navigationMap;
    if (p.backend == "cartographer") args << "map_file:=" + prefix;
  }
  return args;
}
