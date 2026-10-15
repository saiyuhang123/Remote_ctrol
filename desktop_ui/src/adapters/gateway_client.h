#pragma once
#include <QElapsedTimer>
#include <QJsonObject>
#include <QTimer>
#include <QWebSocket>

// The only motion-command writer in the desktop. No disconnected command queue.
class GatewayClient : public QObject {
  Q_OBJECT
public:
  explicit GatewayClient(double maxLinear, double maxAngular, QObject *parent = nullptr, int feedbackTimeoutMs = 1500);
  void connectTo(const QUrl &url);
  void disconnectFrom();
  bool connected() const;
  bool controlEnabled() const { return enabled_; }
  bool manualReady() const { return manualReady_; }
  bool moving() const { return manualMoving_; }
  void setControlEnabled(bool enabled);
  void updateFeedback(bool estopped, bool navActive, bool navKnown, bool fresh);
  bool takeManualControl();
  bool move(double linear, double angular);
  void stopManual();
  bool emergencyStop();
  bool releaseStop();
  bool navigate(double x, double y, double yaw);
  bool initialPose(double x, double y, double yaw);
  bool cancelNavigation();
  bool capture();
  bool speak(const QString &text);
signals:
  void connectionChanged(bool connected);
  void controlChanged(bool enabled);
  void manualChanged(bool ready);
  void received(const QJsonObject &message);
  void message(const QString &text);
private:
  bool send(const QJsonObject &value, bool emergency = false);
  bool poseRequest(const QString &type, double x, double y, double yaw);
  void resetControl();
  bool feedbackFresh() const;
  QWebSocket socket_;
  QTimer repeat_, preparation_;
  QElapsedTimer preparationAge_;
  QElapsedTimer feedbackAge_;
  int feedbackTimeoutMs_;
  double maxLinear_, maxAngular_, linear_ = 0., angular_ = 0.;
  bool enabled_ = false, manualReady_ = false, manualMoving_ = false;
  bool estopped_ = true, navActive_ = false, navKnown_ = false, fresh_ = false;
};
