#include "runtime/map_saver.h"
#include "adapters/ros_adapter.h"
#include <QDir>
#include <QFileInfo>
#include <QTimer>

MapSaver::MapSaver(const DesktopConfig &c, RosAdapter *ros, QObject *parent) : QObject(parent), ros_(ros), export_(c) {
  connect(&export_, &ProcessManager::output, this, &MapSaver::message);
  connect(&export_, &ProcessManager::finished, this, [this](int code) {
    if (!busy_) return;
    const QFileInfo yaml(prefix_ + ".yaml"), pgm(prefix_ + ".pgm");
    const bool valid = code == 0 && yaml.size() > 0 && pgm.size() > 0 && yaml.lastModified() >= started_.addMSecs(-1000) && pgm.lastModified() >= started_.addMSecs(-1000);
    finish(valid, valid ? "状态文件与导航栅格已保存：" + prefix_ : "栅格导出失败或文件为空，请查看日志；建图会话仍保留");
  });
  connect(ros_, &RosAdapter::requestFinished, this, [this](const QString &op, bool ok, const QString &detail) {
    if (op != "save_state" || !busy_) return;
    emit message(detail);
    if (!ok) { finish(false, detail); return; }
    if (backend_ == "fastlio") {
      const QFileInfo pcd(prefix_ + ".pcd");
      const bool exists = pcd.size() > 0 && pcd.lastModified() >= started_.addMSecs(-1000);
      finish(exists, exists ? "PCD 已保存。二维导航栅格请使用现有 pcd2pgm 流程导出：" + prefix_ + ".pcd" : "服务返回成功，但所选本机路径未发现有效 PCD；检查实际建图输出路径");
      return;
    }
    auto validFile = [this](const QString &suffix) { const QFileInfo f(prefix_ + suffix); return f.size() > 0 && f.lastModified() >= started_.addMSecs(-1000); };
    const bool stateExists = backend_ == "cartographer" ? validFile(".pbstream") : validFile(".posegraph") && validFile(".data");
    if (!stateExists) { finish(false, "未找到所选本机目录中的序列化产物；远程模式请在机器人上核对文件"); return; }
    if (!export_.start({"ros2", "run", "nav2_map_server", "map_saver_cli", "-t", "/map", "-f", prefix_})) finish(false, "无法启动栅格保存工具");
    const auto generation = generation_;
    QTimer::singleShot(20000, this, [this, generation] { if (generation_ == generation && busy_ && export_.running()) { emit message("栅格保存超时，停止本次导出工具"); export_.stop(); } });
  });
}
void MapSaver::save(const Profile &p, const QString &prefix) {
  if (busy_ || !p.mapping || !QFileInfo(prefix).isAbsolute() || prefix.trimmed().isEmpty()) { emit message("请选择建图模式与本机绝对保存路径"); return; }
  if (!QDir().mkpath(QFileInfo(prefix).absolutePath())) { emit message("无法创建地图目录"); return; }
  ++generation_; backend_ = p.backend; prefix_ = prefix; started_ = QDateTime::currentDateTime(); busy_ = true; emit busyChanged(true);
  ros_->saveState(backend_, prefix_);
}
void MapSaver::finish(bool ok, const QString &detail) { busy_ = false; emit busyChanged(false); emit saved(ok, detail); }
