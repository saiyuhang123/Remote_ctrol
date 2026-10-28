#pragma once

#include "patrol/patrol_client.h"
#include <QWidget>
#include <functional>

class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QSpinBox;
class QPlainTextEdit;
class QPushButton;
class QLabel;
class InspectionTargetMap;

class InspectionTargetPage : public QWidget {
  Q_OBJECT
public:
  InspectionTargetPage(PatrolClient *client, bool offline,
                       std::function<bool(QString *)> permission, QWidget *parent = nullptr);
  void setReturnPermission(std::function<bool(QString *)> permission);

private:
  void refreshMap();
  void refreshPoints();
  void chooseSavedPoint();
  void useCoordinates();
  void updateControls();
  void start();
  void message(const QString &text);
  PatrolClient *client_;
  bool offline_, targetValid_ = false, updating_ = false;
  PatrolMapInfo map_;
  QString selectedPointId_;
  quint64 selectedPointRevision_ = 0;
  std::function<bool(QString *)> permission_, returnPermission_;
  InspectionTargetMap *view_;
  QComboBox *points_;
  QDoubleSpinBox *x_, *y_, *yaw_;
  QCheckBox *inspect_;
  QSpinBox *preset_;
  QPlainTextEdit *prompt_;
  QPushButton *start_, *cancel_, *home_;
  QLabel *readiness_, *message_, *status_;
};
