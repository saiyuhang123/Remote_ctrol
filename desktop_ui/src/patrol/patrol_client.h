#pragma once

#include <QObject>
#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <QVector>
#include <cstdint>
#include <memory>

struct PatrolMapInfo {
  QString mapId, name, backend, navigationYaml, localizationFile;
  double resolution = 0.0, originX = 0.0, originY = 0.0, originYaw = 0.0;
  int width = 0, height = 0;
  bool verified = false;
  quint64 revision = 0;
};

struct PatrolPointInfo {
  QString pointId, mapId, name, source;
  double x = 0.0, y = 0.0, yaw = 0.0;
  quint64 revision = 0;
  bool inspectionEnabled = false;
  int ptzPreset = 0;
  QString recognitionPrompt;
};

struct PatrolPointShotsInfo {
  PatrolPointInfo point;
  quint64 mapRevision = 0;
  int devicePanSign = 0;
  QJsonArray shots;
  bool loaded = false;
};

struct PatrolRouteStepInfo {
  QString stepId, pointId, pointName;
  double x = 0.0, y = 0.0, yaw = 0.0, dwellSeconds = 0.0;
  bool inspectionEnabled = false;
  int ptzPreset = 0;
  QString recognitionPrompt;
};

struct PatrolRouteInfo {
  QString routeId, mapId, name;
  QVector<PatrolRouteStepInfo> steps;
  quint64 revision = 0;
  QString navigationMode = "FREE_NAV", networkId;
  quint64 networkRevision = 0;
};

struct PatrolHomeInfo {
  QString mapId;
  quint64 mapRevision = 0;
  bool exists = false;
  double x = 0.0, y = 0.0, yaw = 0.0;
};

struct PatrolRunInfo {
  QString runId, routeId, mapId, errorCode, message;
  int state = 0;
  bool success = false;
  quint32 completedLoops = 0;
  quint32 completedSteps = 0;
  quint64 routeRevision = 0, mapRevision = 0;
};

struct PatrolInspectionResultInfo {
  QString resultId, runId, mapId, pointId, pointName, prompt, imagePath, text, model;
  quint32 loopIndex = 0, stepIndex = 0;
  int ptzPreset = 0;
  bool success = false;
  QString errorCode, message;
  qint64 startedAtNs = 0, finishedAtNs = 0;
  QString startedAtText;
};

struct PatrolStatusInfo {
  QString instanceId, runId, routeId, mapId, currentPointName, errorCode, message, currentGoalId;
  int state = 0;
  quint32 loopIndex = 0, totalLoops = 0, stepIndex = 0, stepCount = 0;
  double dwellRemaining = 0.0;
  bool active = false;
};

class PatrolClient : public QObject {
  Q_OBJECT
public:
  explicit PatrolClient(QObject *parent = nullptr);
  ~PatrolClient() override;

  bool available() const;
  bool routeServerReady() const;
  bool pointServerReady() const;
  bool returnServerReady() const;
  bool returnActiveOrPending() const;
  QString ownerSessionId() const;

  void refreshMaps();
  void registerMap(const QString &navigationYaml, const QString &backend,
                   const QString &localizationFile, const QString &name = QString());
  void setActiveMap(const QString &mapId, quint64 expectedRevision);
  void refreshPoints(const QString &mapId = QString());
  void upsertPoint(const QString &pointId, const QString &mapId, const QString &name,
                   double x, double y, double yaw, quint64 expectedRevision,
                   bool inspectionEnabled = false, int ptzPreset = 0,
                   const QString &recognitionPrompt = QString());
  void deletePoint(const QString &pointId, quint64 expectedRevision);
  bool pointShotsReady() const;
  bool pointShotsBusy() const;
  PatrolPointShotsInfo pointShots(const QString &pointId) const;
  void refreshPointShots(const QString &pointId);
  void recordPointShot(const QString &pointId, const QString &mapId, quint64 mapRevision,
                       quint64 pointRevision, const QString &shotId, const QString &name,
                       double settleSeconds);
  void removePointShot(const QString &pointId, const QString &mapId, quint64 mapRevision,
                       quint64 pointRevision, const QString &shotId);
  void clearPointShots(const QString &pointId, const QString &mapId, quint64 mapRevision,
                       quint64 pointRevision);
  void saveCurrentPoint(const QString &mapId, const QString &name,
                        bool inspectionEnabled = false, int ptzPreset = 0,
                        const QString &recognitionPrompt = QString());
  void refreshRoutes(const QString &mapId = QString());
  void upsertRoute(const QString &routeId, const QString &mapId, const QString &name,
                   const QVector<PatrolRouteStepInfo> &steps, quint64 expectedRevision,
                   const QString &navigationMode = QString(), const QString &networkId = QString(),
                   quint64 networkRevision = 0);
  void deleteRoute(const QString &routeId, quint64 expectedRevision);
  void refreshRuns(const QString &mapId = QString(), const QString &routeId = QString());
  void refreshInspectionResults(const QString &mapId = QString(),
                                const QString &runId = QString(), quint32 limit = 100);
  void refreshHomePoint(const QString &mapId = QString());
  void saveCurrentPoseAsHome(const QString &mapId, quint64 expectedMapRevision);
  void clearHomePoint(const QString &mapId, quint64 expectedMapRevision);
  void returnHome(const QString &mapId, quint64 mapRevision);
  void cancelReturnHome();
  void cancelActiveTask();
  void inspectPoint(const QString &mapId, quint64 mapRevision,
                    const QString &pointId, quint64 pointRevision,
                    double x, double y, double yaw, bool inspectionEnabled,
                    int ptzPreset, const QString &recognitionPrompt);
  void cancelInspectPoint();
  void runRoute(const QString &routeId, const QString &mapId,
                quint64 routeRevision, quint64 mapRevision, quint32 loopCount,
                const QString &planDigest = QString());
  bool roadServerReady() const;
  void roadCommand(const QString &operation, QJsonObject payload = {});
  QJsonArray roadNetworks() const;
  void cancelRunRoute();
  bool hasActiveOrPendingRun() const;
  void setOperatorAuthorized(bool enabled);
  void controlRun(quint8 command, const QString &runId);
  void resetInterlock();

  QVector<PatrolMapInfo> maps() const;
  QString activeMapId() const;
  QVector<PatrolPointInfo> points() const;
  QVector<PatrolRouteInfo> routes() const;
  QVector<PatrolRunInfo> runs() const;
  QVector<PatrolInspectionResultInfo> inspectionResults() const;
  PatrolHomeInfo homePoint() const;
  PatrolStatusInfo status() const;

  static QString stateName(int state);

signals:
  void mapsChanged();
  void pointsChanged();
  void pointShotsChanged(const QString &pointId);
  void routesChanged();
  void runsChanged();
  void inspectionResultsChanged();
  void homePointChanged();
  void statusChanged();
  void operationFinished(const QString &operation, bool success, const QString &message);
  void errorOccurred(const QString &message);
  void roadReply(const QString &operation, bool success, const QString &message, const QJsonObject &data);
  void roadNetworksChanged();

private:
  void sendPointShots(quint8 command, const QString &pointId, const QString &mapId = {},
                      quint64 mapRevision = 0, quint64 pointRevision = 0,
                      const QString &shotId = {}, const QString &name = {},
                      double settleSeconds = 1.5);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
