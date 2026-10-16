#pragma once
#include <QJsonObject>
#include <QString>
#include <QVector>

struct DesktopConfig {
  QString resourceDir, projectRoot;
  QString backendUrl, defaultProfile, baseFrame, odomTopic, imageTopic, batteryTopic;
  QString cameraParamsFile;
  double maxLinear = .3, maxAngular = .5;
  int feedbackTimeoutMs = 1500, serviceTimeoutMs = 15000;
  static DesktopConfig load(const QString &path, const QString &resources, const QString &project);
};

struct Profile {
  QString id, name, backend, launchFile, rvizFile, fixedFrame;
  bool mapping = false;
  bool navigation = false;
};
const QVector<Profile> &desktopProfiles();
