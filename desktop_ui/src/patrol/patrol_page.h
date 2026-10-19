#pragma once

#include "patrol/patrol_client.h"

#include <QWidget>
#include <QVector>
#include <functional>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTabWidget;
class RecordedRoadPage;
class QCloseEvent;

class PatrolPage : public QWidget {
  Q_OBJECT
public:
  PatrolPage(PatrolClient *client, const QString &initialMapYaml,
             const QString &backend, const QString &localizationFile,
             const QString &profileName, bool offline = false,
             std::function<bool(QString *)> startPermission = {},
             QWidget *parent = nullptr);

  PatrolClient *client() const { return client_; }
  QString activeMapId() const { return activeMapId_; }
  void setResumePermission(std::function<bool(QString *)> permission);
protected:
  void closeEvent(QCloseEvent *event) override;

private slots:
  void onMapsChanged();
  void onPointsChanged();
  void onRoutesChanged();
  void onRunsChanged();
  void onHomePointChanged();
  void onStatusChanged();
  void registerCurrentMap();
  void setActiveMap();
  void refreshAll();
  void saveCurrentPoint();
  void upsertSelectedPoint();
  void deleteSelectedPoint();
  void addStepFromPoint();
  void removeSelectedStep();
  void moveStepUp();
  void moveStepDown();
  void updateStepDwell();
  void newRoute();
  void saveRoute();
  void deleteSelectedRoute();
  void startRun();
  void pauseRun();
  void resumeRun();
  void cancelRun();
  void refreshHomePoint();
  void saveHomePoint();
  void clearHomePoint();
  void returnHome();
  void showOperation(const QString &operation, bool success, const QString &message);
  void showError(const QString &message);

private:
  void buildUi();
  void populateMaps();
  void populatePoints();
  void populateRoutes();
  void populateRuns();
  void populateAvailablePoints();
  void loadRouteIntoEditor(const PatrolRouteInfo &route);
  void clearRouteEditor();
  void syncStepTable();
  void updatePointFormFromSelection();
  void updateActionStates();
  void setMessage(const QString &message, bool error = false);
  bool confirmDraftDiscard();
  QString activeMapName() const;
  PatrolMapInfo activeMap() const;
  PatrolRouteInfo routeById(const QString &routeId) const;

  PatrolClient *client_ = nullptr;
  QString mapYaml_;
  QString backend_;
  QString localizationFile_;
  QString profileName_;
  bool offline_ = false;
  std::function<bool(QString *)> startPermission_;
  std::function<bool(QString *)> resumePermission_;
  QString activeMapId_;
  QString currentRouteId_;
  quint64 currentRouteRevision_ = 0;
  QVector<PatrolRouteStepInfo> currentSteps_;
  bool routeDirty_ = false;
  bool loadingRoute_ = false;

  QLabel *serviceLabel_ = nullptr;
  QLineEdit *mapYamlEdit_ = nullptr;
  QPushButton *registerMapButton_ = nullptr;
  QComboBox *mapsCombo_ = nullptr;
  QPushButton *setActiveButton_ = nullptr;
  QPushButton *refreshAllButton_ = nullptr;
  QLabel *activeMapLabel_ = nullptr;
  QLabel *homePointLabel_ = nullptr;
  QPushButton *saveHomeButton_ = nullptr;
  QPushButton *clearHomeButton_ = nullptr;
  QPushButton *returnHomeButton_ = nullptr;
  QLabel *messageLabel_ = nullptr;
  QLabel *pointsCountLabel_ = nullptr;
  QLabel *routesCountLabel_ = nullptr;
  QLabel *routeDraftLabel_ = nullptr;
  QTabWidget *tabs_ = nullptr;

  QTableWidget *pointsTable_ = nullptr;
  QLineEdit *pointNameEdit_ = nullptr;
  QDoubleSpinBox *pointX_ = nullptr;
  QDoubleSpinBox *pointY_ = nullptr;
  QDoubleSpinBox *pointYaw_ = nullptr;
  QPushButton *saveCurrentPointButton_ = nullptr;
  QPushButton *upsertPointButton_ = nullptr;
  QPushButton *deletePointButton_ = nullptr;

  QTableWidget *routesTable_ = nullptr;
  QLineEdit *routeNameEdit_ = nullptr;
  QComboBox *pointCombo_ = nullptr;
  QComboBox *navigationMode_ = nullptr;
  QComboBox *routeNetwork_ = nullptr;
  QPushButton *previewRecorded_ = nullptr;
  RecordedRoadPage *roadsPage_ = nullptr;
  QString planDigest_;
  bool recordingDraft_=false;
  QDoubleSpinBox *addStepDwell_ = nullptr;
  QPushButton *addStepButton_ = nullptr;
  QTableWidget *stepsTable_ = nullptr;
  QDoubleSpinBox *stepDwell_ = nullptr;
  QPushButton *updateDwellButton_ = nullptr;
  QPushButton *removeStepButton_ = nullptr;
  QPushButton *moveUpButton_ = nullptr;
  QPushButton *moveDownButton_ = nullptr;
  QPushButton *newRouteButton_ = nullptr;
  QPushButton *saveRouteButton_ = nullptr;
  QPushButton *deleteRouteButton_ = nullptr;
  QSpinBox *loopCount_ = nullptr;
  QPushButton *startButton_ = nullptr;
  QPushButton *pauseButton_ = nullptr;
  QPushButton *resumeButton_ = nullptr;
  QPushButton *cancelButton_ = nullptr;
  QLabel *startReadinessLabel_ = nullptr;
  QLabel *statusLabel_ = nullptr;
  QLabel *progressLabel_ = nullptr;

  QTableWidget *runsTable_ = nullptr;
  QPushButton *refreshRunsButton_ = nullptr;
};
