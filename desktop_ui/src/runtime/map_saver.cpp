#include "runtime/map_saver.h"
#include "adapters/ros_adapter.h"
#include <QDir>
#include <QCryptographicHash>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>

MapSaver::MapSaver(const DesktopConfig &c, RosAdapter *ros, QObject *parent)
  : QObject(parent), ros_(ros), export_(c), webExport_(c), resourceDir_(c.resourceDir),
    projectRoot_(c.projectRoot) {
  connect(&export_, &ProcessManager::output, this, &MapSaver::message);
  connect(&webExport_, &ProcessManager::output, this, &MapSaver::message);
  connect(&webExport_, &ProcessManager::finished, this, [this](int code) {
    if (!busy_ || !webExportPending_) return;
    webExportPending_ = false;
    const QString detail = localDetail_ +
      (code == 0 ? "；网页地图已自动导出" :
       "；本地地图已保存，但网页地图导出失败；网页任务页将因版本不一致禁止执行");
    finish(true, detail);
  });
  connect(&export_, &ProcessManager::finished, this, [this](int code) {
    if (!busy_) return;
    const QFileInfo yaml(prefix_ + ".yaml"), pgm(prefix_ + ".pgm");
    const bool valid = code == 0 && yaml.size() > 0 && pgm.size() > 0 && yaml.lastModified() >= started_.addMSecs(-1000) && pgm.lastModified() >= started_.addMSecs(-1000);
    if (!valid) {
      finish(false, "PCD 已保留于 " + prefix_ + ".pcd；二维切片导出未完成，请查看切片日志并再次保存重试。现有导航地图未替换。");
      return;
    }
    if (backend_ == "fastlio") {
      complete_ = true;
      const QString state = quality_ == "loop_checked" ? "回环检查通过" :
          quality_ == "no_loop" ? "单程图已保存，未完成回环验证" :
          quality_ == "geometry_rejected" ? "地图已归档，回环修正未采用" :
          quality_ == "sensor_error" ? "地图已归档，传感器数据需要检查" : "地图已保存，检查状态未知";
      finish(true, state + "；PCD、PGM、YAML：" + prefix_ + "。" + guidance_ + " 现有导航地图未替换。");
      return;
    }
    localDetail_ = (backend_ == "fastlio" ? "PCD、PGM、YAML 已保存：" : "状态文件与导航栅格已保存：") + prefix_;
    // Never make browser map export a second manual step. It runs only after
    // the Nav2 YAML/PGM pair is complete and uses the exact same YAML origin.
    webExportPending_ = true;
    if (!webExport_.start({"python3", projectRoot_ + "/scripts/export_web_map.py",
                           prefix_ + ".yaml", "--output-dir", projectRoot_ + "/web/maps"})) {
      webExportPending_ = false;
      finish(true, localDetail_ + "；网页地图导出启动失败，网页任务页将禁止执行");
    }
  });
  connect(ros_, &RosAdapter::requestFinished, this, [this](const QString &op, bool ok, const QString &detail) {
    if (op != "save_state" || !busy_) return;
    if (!ok) {
      finish(false, detail + (detail.contains("服务未就绪") ? "；请启动建图流程，升级后需停止并重新启动建图。" : QString()));
      return;
    }
    if (backend_ == "fastlio") {
      const auto payload = QJsonDocument::fromJson(detail.toUtf8()).object();
      if (payload.value("schema").toString() == "fastlio.saved_map.v1") {
        const QString artifact = payload.value("artifact_prefix").toString();
        if (!QFileInfo(artifact).isAbsolute() || artifact.isEmpty() ||
            QDir::cleanPath(payload.value("requested_prefix").toString()) != QDir::cleanPath(requestedPrefix_)) {
          finish(false, "服务保存目录与界面所选地图不一致，请使用启动建图时的地图前缀；已保存的数据保留在建图会话中。"); return;
        }
        prefix_ = artifact;
        quality_ = payload.value("quality").toString();
        guidance_ = payload.value("message").toString() + "。" + payload.value("next_action").toString();
        QFile reportFile(prefix_ + ".validation.json"), pcdFile(prefix_ + ".pcd");
        if (!reportFile.open(QIODevice::ReadOnly) || !pcdFile.open(QIODevice::ReadOnly)) {
          finish(false, "本次地图或检查报告无法读取，请检查保存目录权限：" + prefix_); return;
        }
        const auto report = QJsonDocument::fromJson(reportFile.readAll()).object();
        QCryptographicHash hash(QCryptographicHash::Sha256); hash.addData(&pcdFile);
        const bool verified = report.value("status").toString() == quality_ &&
            report.value("sha256").toString() == QString::fromLatin1(hash.result().toHex()) &&
            (quality_ != "loop_checked" || report.value("geometry_checks_passed").toBool());
        if (!verified) { finish(false, "本次 PCD 与检查报告不一致，请保留目录并重新保存：" + prefix_); return; }
        loadable_ = payload.value("localization_test_allowed").toBool() &&
            (quality_ == "loop_checked" || quality_ == "no_loop");
        emit message(guidance_);
      } else {
        // Compatibility for external/older service fixtures: never interpret
        // an unstructured success response as a validated navigation map.
        quality_ = "unknown"; guidance_ = "后端未提供检查状态，请核对地图来源。";
        emit message(detail);
      }
      const QFileInfo pcd(prefix_ + ".pcd");
      const bool exists = pcd.size() > 0 && pcd.lastModified() >= started_.addMSecs(-1000);
      if (!exists) { finish(false, "服务返回成功，但所选本机路径未发现有效 PCD；检查实际建图输出路径"); return; }
      savedPrefix_ = prefix_;
      if (!export_.start({"python3", "-u", resourceDir_ + "/scripts/export_fastlio_map.py",
                          "--prefix", prefix_, "--config", resourceDir_ + "/config/fastlio_map_export.yaml"}))
        finish(false, "PCD 已保存，但无法启动二维栅格导出工具");
      return;
    }
    emit message(detail);
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
  ++generation_; backend_ = p.backend; prefix_ = requestedPrefix_ = prefix; started_ = QDateTime::currentDateTime();
  savedPrefix_.clear(); quality_.clear(); guidance_.clear(); complete_ = loadable_ = false;
  localDetail_.clear(); webExportPending_ = false; busy_ = true; emit busyChanged(true);
  ros_->saveState(backend_, prefix_);
}
void MapSaver::finish(bool ok, const QString &detail) {
  busy_ = false;
  webExportPending_ = false;
  localDetail_.clear();
  emit busyChanged(false);
  emit saved(ok, detail);
}
