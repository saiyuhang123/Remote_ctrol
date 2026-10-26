#pragma once
#include "adapters/ros_adapter.h"
#include "runtime/config.h"
#include <QElapsedTimer>
#include <QMainWindow>
#include <QVector>
class CameraPanel; class LocalController; class ProcessManager; class MapSaver; class RvizHost; class MapEditorDialog; class PatrolClient; class PatrolPage;
class VisionPanel;
class DesktopRemoteBridge;
class DeviceClient; class DevicePanel;
class QAction; class QCheckBox; class QComboBox; class QDoubleSpinBox; class QLabel; class QLineEdit; class QTimer;
class QPlainTextEdit; class QProgressBar; class QPushButton; class QTabWidget; class QToolButton;
class QBoxLayout;
namespace ads { class CDockManager; class CDockWidget; }
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
  void resizeEvent(QResizeEvent *event) override;
  bool eventFilter(QObject *object, QEvent *event) override;
private:
  const Profile &profile() const;
  void setupUi();
  void restoreConsoleLayout();
  void saveConsoleLayout();
  void resetConsoleLayout();
  void sizeConsolePanes();
  void adaptConsoleDensity();
  void refreshStatusSummary();
  void showOperationResult(const QString &operation, bool success, const QString &text);
  void applyProfile();
  void refreshOdometrySource();
  void log(const QString &message);
  void updateControls();
  void syncPatrolState();
  bool cancelPatrolRun(const QString &reason);
  void returnHome();
  bool patrolRunActive() const;
  bool vehicleStopConfirmed() const;
  void pollCloseAfterSafetyStop();
  void updateSnapshot(const RobotSnapshot &state);
  void sendGoal(double x, double y, double yaw);
  void sendInitialPose(double x, double y, double yaw);
  void startSession();
  void openMapEditor();
  void openPatrolPage();
  void openDeviceWindow();
  bool mapEditorSourceMatchesCurrent() const;
  void refreshEditorNavigationFeedback();
  bool allowed() const;
  DesktopConfig config_;
  bool offline_;
  Ui::MainWindow *ui_ = nullptr;
  ads::CDockManager *docks_ = nullptr;
  LocalController *controller_ = nullptr;
  RosAdapter *ros_ = nullptr;
  ProcessManager *process_ = nullptr;
  MapSaver *saver_ = nullptr;
  RvizHost *rviz_ = nullptr;
  RobotSnapshot state_;
  QString lastNavigationEvent_;
  QComboBox *profiles_ = nullptr, *odom_ = nullptr;
  QLabel *odomHint_ = nullptr;
  QString fastlioNavigationOdomSource_ = "lio";
  QCheckBox *enableControl_ = nullptr, *manage_ = nullptr, *sensors_ = nullptr, *chassis_ = nullptr, *bridge_ = nullptr, *patrolInterlock_ = nullptr;
  QLineEdit *mapPrefix_ = nullptr, *navigationYaml_ = nullptr;
  QDoubleSpinBox *goalX_ = nullptr, *goalY_ = nullptr, *goalYaw_ = nullptr, *speed_ = nullptr, *turn_ = nullptr;
  CameraPanel *cameraPanel_ = nullptr;
  VisionPanel *visionPanel_ = nullptr;
  DesktopRemoteBridge *remoteBridge_ = nullptr;
  DeviceClient *deviceClient_ = nullptr;
  DevicePanel *devicePanel_ = nullptr;
  QWidget *deviceWindow_ = nullptr;
  QLabel *connection_ = nullptr, *pose_ = nullptr, *velocity_ = nullptr, *stopState_ = nullptr, *navState_ = nullptr;
  QLabel *speedValue_ = nullptr, *localizationState_ = nullptr, *guardState_ = nullptr;
  QLabel *modeBadge_ = nullptr, *navigationHint_ = nullptr, *sessionHint_ = nullptr, *messageSummary_ = nullptr;
  QLabel *power_ = nullptr, *picture_ = nullptr, *ocr_ = nullptr;
  QProgressBar *battery_ = nullptr;
  QPlainTextEdit *logs_ = nullptr;
  QPushButton *start_ = nullptr, *stop_ = nullptr, *save_ = nullptr, *take_ = nullptr, *unlock_ = nullptr;
  QPushButton *sendGoalButton_ = nullptr;
  QLabel *mapSaveStatus_ = nullptr;
  QPushButton *openSavedMap_ = nullptr, *loadSavedMap_ = nullptr;
  QPushButton *cancelButton_ = nullptr;
  QPushButton *returnHomeButton_ = nullptr;
  QTabWidget *controlTabs_ = nullptr, *rightTabs_ = nullptr;
  QWidget *manualPage_ = nullptr, *statusDetails_ = nullptr;
  QToolButton *logToggle_ = nullptr;
  QBoxLayout *runtimeFooterLayout_ = nullptr;
  ads::CDockWidget *controlDock_ = nullptr, *monitorDock_ = nullptr, *mapDock_ = nullptr;
  QByteArray defaultDockState_;
  bool layoutReady_ = false, compactLayout_ = false, rightBeforeCompact_ = true;
  QString operationErrorKey_;
  QImage capturedPreview_;
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
