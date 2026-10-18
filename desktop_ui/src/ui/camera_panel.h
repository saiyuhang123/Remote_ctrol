#pragma once
#include "runtime/config.h"
#include <QElapsedTimer>
#include <QImage>
#include <QWidget>
#include <QVector>
class RosAdapter;
class ProcessManager;
class QLabel;
class QPushButton;
class QDoubleSpinBox;
class QTimer;

// Camera lifecycle is independent of navigation and of the business gateway.
class CameraPanel : public QWidget {
  Q_OBJECT
public:
  CameraPanel(const DesktopConfig &config, RosAdapter *ros, bool offline, QWidget *parent = nullptr);
  ~CameraPanel() override;
  void setControlEnabled(bool enabled);
  void stopInteraction();
  void ensureCamera();
  void setPatrolBlocked(bool blocked);
signals:
  void message(const QString &text);
  void patrolStopRequested();
protected:
  void resizeEvent(QResizeEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  bool eventFilter(QObject *object, QEvent *event) override;
private:
  void refresh();
  void renderFrame();
  void startCamera();
  void connectPreview();
  void saveFrame();
  void stopJog();
  DesktopConfig config_;
  RosAdapter *ros_;
  ProcessManager *process_;
  bool offline_, controlEnabled_ = false, waitingForNode_ = false, poseRead_ = false;
  QString pending_, detail_;
  QElapsedTimer lastFrame_, startup_, rateWindow_;
  int rateFrames_ = 0;
  double fps_ = 0;
  QImage frame_;
  QLabel *preview_, *status_, *poseStatus_;
  QPushButton *start_, *connect_, *stopStream_, *stopNode_, *read_, *set_, *save_;
  QDoubleSpinBox *pan_, *tilt_, *zoom_;
  QVector<QPushButton *> directions_;
  QTimer *jogTimer_;
  int jogCommand_ = 0;
  bool jogInFlight_ = false, absolutePanTilt_ = true;
  bool patrolBlocked_ = false;
};
