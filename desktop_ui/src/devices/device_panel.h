#pragma once
#include <QWidget>
class DeviceClient;
class DevicePanel : public QWidget {
  Q_OBJECT
public:
  DevicePanel(DeviceClient *client, bool offline, QWidget *parent = nullptr);
  void setControlEnabled(bool enabled);
private:
  void refresh();
  DeviceClient *client_;
  bool offline_, authorized_ = false;
  struct View;
  View *view_;
};
