#pragma once
#include "runtime/config.h"
#include <QImage>
#include <QObject>
#include <memory>

struct RobotSnapshot {
  bool poseValid = false, speedValid = false, batteryValid = false;
  bool estopValid = false, estopped = true, navigationKnown = false, navigationActive = false;
  bool navigationAvailable = false;
  QString frame;
  double x = 0., y = 0., yaw = 0., speed = 0., yawRate = 0., battery = 0., voltage = 0.;
};
Q_DECLARE_METATYPE(RobotSnapshot)

struct CameraPose {
  double pan = 0, tilt = 0, zoom = 1;
  bool limitsValid = false;
  double panMin = 0, panMax = 360, tiltMin = 0, tiltMax = 360, zoomMin = 1, zoomMax = 40;
};
Q_DECLARE_METATYPE(CameraPose)

class RosAdapter : public QObject {
  Q_OBJECT
public:
  explicit RosAdapter(const DesktopConfig &config, QObject *parent = nullptr);
  ~RosAdapter() override;
  void setFixedFrame(const QString &frame);
  QStringList capabilities() const;
  QStringList conflictingNodes() const;
  bool setInitialPose(double x, double y, double yaw);
  bool previewGoal(double x, double y, double yaw);
  bool navigate(double x, double y, double yaw);
  bool cancelNavigation();
  bool publishVelocity(double linear, double angular);
  bool setEmergencyStop(bool stopped);
  void captureBusiness();
  void speak(const QString &text);
  void captureCamera();
  void setPtz(double pan, double tilt, double zoom);
  bool cameraAvailable() const;
  bool cameraNodePresent() const;
  void cameraCommand(const QString &command);
  void readCameraPose();
  void jogCamera(int command, int speed, bool stop);
  void recognize();
  void saveState(const QString &backend, const QString &prefix);
signals:
  void snapshot(const RobotSnapshot &value);
  void image(const QImage &value);
  void capturedImage(const QImage &value);
  void cameraPose(const CameraPose &value);
  void ocrResult(const QString &value);
  void requestFinished(const QString &operation, bool success, const QString &detail);
  void goalRequested(double x, double y, double yaw);
  void initialPoseRequested(double x, double y, double yaw);
  void message(const QString &text);
  void navigationEvent(const QString &text);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
