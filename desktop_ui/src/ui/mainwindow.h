#pragma once
#include "adapters/ros_adapter.h"
#include "runtime/config.h"
#include <QElapsedTimer>
#include <QJsonArray>
#include <QMainWindow>
#include <QVector>
class CameraPanel; class LocalController; class BusinessClient; class ProcessManager; class MapSaver; class RvizHost; class MapEditorDialog; class PatrolClient; class PatrolPage;
class QAction; class QCheckBox; class QComboBox; class QDoubleSpinBox; class QLabel; class QLineEdit; class QTimer;
class QPlainTextEdit; class QProgressBar; class QPushButton; class QTableWidget; class QStackedWidget;
class DashBoard;
namespace ads { class CDockManager; }
namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(const DesktopConfig &config, bool offline, QWidget *parent = nullptr);
  ~MainWindow() override;
  void setProfileIndex(int index);
  bool saveScreenshot(const QString &path);
protected:
  void closeEvent(QCloseEvent *event) override;
  void changeEvent(QEvent *event) override;
private:
  const Profile &profile() const;
  void setupUi();
  void applyProfile();
  void log(const QString &message);
  void updateControls();
  void syncPatrolState();
  bool cancelPatrolRun(const QString &reason);
  bool patrolRunActive() const;
  bool vehicleStopConfirmed() const;
  void pollCloseAfterSafetyStop();
  void updateSnapshot(const RobotSnapshot &state);
  void sendGoal(double x, double y, double yaw);
  void sendInitialPose(double x, double y, double yaw);
  void startSession();
  void openMapEditor();
  void openPatrolPage();
  bool mapEditorSourceMatchesCurrent() const;
  void refreshEditorNavigationFeedback();
  void showPoints(const QJsonArray &points);
  bool allowed() const;
  DesktopConfig config_;
  bool offline_;
  Ui::MainWindow *ui_ = nullptr;
  ads::CDockManager *docks_ = nullptr;
  LocalController *controller_ = nullptr;
  BusinessClient *business_ = nullptr;
  RosAdapter *ros_ = nullptr;
  ProcessManager *process_ = nullptr;
  MapSaver *saver_ = nullptr;
  RvizHost *rviz_ = nullptr;
  RobotSnapshot state_;
  QString lastNavigationEvent_;
  QComboBox *profiles_ = nullptr, *odom_ = nullptr;
  QCheckBox *enableControl_ = nullptr, *manage_ = nullptr, *sensors_ = nullptr, *chassis_ = nullptr, *bridge_ = nullptr, *patrolInterlock_ = nullptr;
  QLineEdit *backendUrl_ = nullptr, *mapPrefix_ = nullptr, *navigationYaml_ = nullptr;
  QDoubleSpinBox *goalX_ = nullptr, *goalY_ = nullptr, *goalYaw_ = nullptr, *speed_ = nullptr, *turn_ = nullptr;
  CameraPanel *cameraPanel_ = nullptr;
  QLabel *connection_ = nullptr, *pose_ = nullptr, *velocity_ = nullptr, *stopState_ = nullptr, *navState_ = nullptr;
  QLabel *power_ = nullptr, *picture_ = nullptr, *ocr_ = nullptr;
  QProgressBar *battery_ = nullptr;
  QStackedWidget *speedStack_ = nullptr;
  DashBoard *dashboard_ = nullptr;
  QPlainTextEdit *logs_ = nullptr;
  QTableWidget *points_ = nullptr;
  QPushButton *start_ = nullptr, *stop_ = nullptr, *save_ = nullptr, *take_ = nullptr, *unlock_ = nullptr;
  QPushButton *sendGoalButton_ = nullptr;
  MapEditorDialog *mapEditor_ = nullptr;
  PatrolClient *patrolClient_ = nullptr;
  PatrolPage *patrolPage_ = nullptr;
  QWidget *patrolWindow_ = nullptr;
  QAction *mapEditorAction_ = nullptr;
  QAction *patrolAction_ = nullptr;
  QString mapEditorProfileId_;
  QString mapEditorSourcePrefix_;
  QString mapEditorInitialSourceYaml_;
  int lastProfileIndex_ = -1;
  QVector<QPushButton *> directionButtons_, writeButtons_;
  QTimer *closeSafetyTimer_ = nullptr;
  QElapsedTimer closeSafetyAge_;
  bool closeAfterStop_ = false;
  bool sessionPatrolInterlock_ = false;
};
