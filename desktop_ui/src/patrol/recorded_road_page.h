#pragma once
#include <QWidget>
#include <QJsonObject>
#include <QJsonArray>
class PatrolClient; class QComboBox; class QLineEdit; class QLabel; class QPushButton;
class QGraphicsView; class QGraphicsScene; class QTableWidget; class QDoubleSpinBox; class QCheckBox;

class RecordedRoadPage : public QWidget {
  Q_OBJECT
public:
  explicit RecordedRoadPage(PatrolClient *client, QWidget *parent = nullptr);
  bool hasDraft() const { return state_ != "IDLE"; }
  void showPlan(const QJsonObject &plan);
  void pauseForClose();
private:
  void refresh();
  void updateNetworks();
  void updatePoints();
  void updateBindingSummary();
  void redraw(bool preserveView=false);
  void reply(const QString &operation, bool success, const QString &message, const QJsonObject &data);
  void command(const QString &operation, QJsonObject payload = {});
  QJsonObject selectedNetwork() const;
  PatrolClient *client_;
  QJsonObject status_, plan_;
  QJsonArray drafts_;
  QString state_="IDLE";
  QString pendingNetworkId_;
  bool refreshing_=false;
  QComboBox *networks_, *startNode_, *endNode_, *point_, *bindingNode_, *draftsCombo_;
  QLineEdit *networkName_, *roadName_;
  QLabel *statusLabel_, *message_;
  QLabel *emptyPreview_, *networkSummary_, *bindingSummary_, *roadSelectionLabel_, *roadCount_;
  QGraphicsView *view_; QGraphicsScene *scene_;
  QTableWidget *roads_;
  QDoubleSpinBox *speed_; QCheckBox *edgeEnabled_;
  QPushButton *begin_, *pause_, *resume_, *finish_, *save_, *discard_, *publish_;
};
