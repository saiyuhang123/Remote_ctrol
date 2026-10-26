#pragma once

#include <QObject>
#include <memory>

class CameraPanel;
class LocalController;
class VisionPanel;

// Web requests enter the same Qt controllers used by the local buttons.
class DesktopRemoteBridge : public QObject {
public:
  DesktopRemoteBridge(CameraPanel *camera, VisionPanel *vision,
                      LocalController *controller, bool offline, QObject *parent);
  ~DesktopRemoteBridge() override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
