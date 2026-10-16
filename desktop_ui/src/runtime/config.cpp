#include "runtime/config.h"
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QUrl>
#include <cmath>
#include <stdexcept>

DesktopConfig DesktopConfig::load(const QString &path, const QString &resources, const QString &project) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) throw std::runtime_error(("Cannot read config: " + path).toStdString());
  QJsonParseError error;
  const auto doc = QJsonDocument::fromJson(f.readAll(), &error);
  if (error.error != QJsonParseError::NoError || !doc.isObject()) throw std::runtime_error("Invalid desktop JSON config");
  const auto o = doc.object();
  DesktopConfig c;
  c.resourceDir = QDir(resources).absolutePath(); c.projectRoot = QDir(project).absolutePath();
  c.backendUrl = o.value("backend_url").toString("http://127.0.0.1:8888");
  c.defaultProfile = o.value("default_profile").toString("fastlio_navigation");
  c.baseFrame = o.value("base_frame").toString("base_link");
  c.odomTopic = o.value("odom_topic").toString("/odom");
  c.imageTopic = o.value("image_topic").toString("/hk_camera/image_raw");
  c.cameraParamsFile = QDir(c.projectRoot).absoluteFilePath(o.value("camera_params_file").toString(
      "ros2_ws/install/hk_camera/share/hk_camera/config/hk_camera_params.yaml"));
  c.batteryTopic = o.value("battery_topic").toString("/battery_status");
  c.maxLinear = o.value("max_linear").toDouble(.3); c.maxAngular = o.value("max_angular").toDouble(.5);
  c.feedbackTimeoutMs = o.value("feedback_timeout_ms").toInt(1500);
  c.serviceTimeoutMs = o.value("service_timeout_ms").toInt(15000);
  const QUrl http(c.backendUrl);
  if (!http.isValid() || http.host().isEmpty() || (http.scheme() != "http" && http.scheme() != "https") ||
      !std::isfinite(c.maxLinear) || c.maxLinear <= 0 || c.maxLinear > .5 ||
      !std::isfinite(c.maxAngular) || c.maxAngular <= 0 || c.maxAngular > 1. ||
      c.feedbackTimeoutMs < 100 || c.serviceTimeoutMs < 1000)
    throw std::runtime_error("Invalid URLs, timeouts or speed limits in desktop config");
  return c;
}

const QVector<Profile> &desktopProfiles() {
  static const QVector<Profile> profiles = {
    {"fastlio_navigation", "FAST-LIO 定位 / Nav2 导航", "fastlio", "navigation.launch.py", "navigation_fastlio.rviz", "map", false, true},
    {"fastlio_mapping", "FAST-LIO 三维建图", "fastlio", "mapping.launch.py", "mapping_fastlio.rviz", "lio_odom", true, false},
    {"slam_navigation", "AMCL 定位 / Nav2 导航", "slam_toolbox", "amcl_localization.launch.py", "navigation_amcl.rviz", "map", false, true},
    {"slam_mapping", "SLAM Toolbox 二维建图", "slam_toolbox", "slam_2d_mapping.launch.py", "mapping_2d.rviz", "map", true, false},
    {"cartographer_navigation", "Cartographer 定位 / Nav2 导航", "cartographer", "cartographer_2d_localization.launch.py", "navigation.rviz", "map", false, true},
    {"cartographer_mapping", "Cartographer 二维建图", "cartographer", "cartographer_2d_mapping.launch.py", "mapping_2d.rviz", "map", true, false}
  };
  return profiles;
}
