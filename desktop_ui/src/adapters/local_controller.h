#pragma once
#include <QElapsedTimer>
#include <QObject>
#include <QTimer>

class RosAdapter;

// Explicit operator commands; transport is the local ROS2 graph.
class LocalController : public QObject {
  Q_OBJECT
public:
  LocalController(RosAdapter *ros, double maxLinear, double maxAngular,
                  int feedbackTimeoutMs, QObject *parent = nullptr);
  bool controlEnabled() const { return enabled_; }
  bool manualReady() const { return manualReady_; }
  void setControlEnabled(bool enabled);
  void updateFeedback(bool estopped, bool navActive, bool navKnown, bool fresh);
  bool takeManualControl();
  bool move(double linear, double angular);
  void stopManual();
  bool emergencyStop();
  bool releaseStop();
  bool navigate(double x, double y, double yaw);
  bool cancelNavigation();
signals:
  void controlChanged(bool enabled);
  void manualChanged(bool ready);
  void message(const QString &text);
private:
  bool feedbackFresh() const;
  RosAdapter *ros_;
  QTimer repeat_, preparation_;
  QElapsedTimer preparationAge_, feedbackAge_;
  int feedbackTimeoutMs_;
  double maxLinear_, maxAngular_, linear_ = 0., angular_ = 0.;
  bool enabled_ = false, manualReady_ = false, manualMoving_ = false, unlockWaiting_ = false;
  bool estopped_ = true, navActive_ = false, navKnown_ = false, fresh_ = false;
};
