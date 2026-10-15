#pragma once
#include <QWidget>
#include <memory>
namespace rviz_common { class VisualizationFrame; namespace ros_integration { class RosNodeAbstraction; } }
class RvizHost : public QWidget {
  Q_OBJECT
public:
  explicit RvizHost(const QString &configPath, QWidget *parent = nullptr);
  ~RvizHost() override;
  bool loadProfile(const QString &path);
  void selectTool(const QString &classId);
  void saveImage(const QString &path);
  uint64_t renderedFrames() const;
  void setPanelsVisible(bool visible);
private:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node_;
  rviz_common::VisualizationFrame *frame_ = nullptr;
  bool panelsVisible_ = false;
};
