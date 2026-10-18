#pragma once
#include <QElapsedTimer>
#include <QJsonObject>
#include <QWidget>

class DeviceClient;
class QLabel;

// Always-visible gas summary; consumes the existing device client's stream.
class GasReadout : public QWidget {
  Q_OBJECT
public:
  GasReadout(DeviceClient *client, bool offline, QWidget *parent = nullptr);
signals:
  void openRequested();
private:
  void refresh();
  bool offline_;
  QLabel *value_, *state_;
  QJsonObject status_, sample_;
  QElapsedTimer statusAge_, sampleAge_;
};
