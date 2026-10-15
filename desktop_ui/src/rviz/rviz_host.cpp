#include "rviz/rviz_host.h"
#include <QApplication>
#include <QFileInfo>
#include <QMenuBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/logging.hpp>
#include <rclcpp/rclcpp.hpp>

RvizHost::RvizHost(const QString &path, QWidget *parent) : QWidget(parent) {
  rviz_common::set_logging_handlers(
    [](const std::string &s, const std::string &, size_t) { RCLCPP_DEBUG(rclcpp::get_logger("robot_desktop_rviz"), "%s", s.c_str()); },
    [](const std::string &s, const std::string &, size_t) { RCLCPP_INFO(rclcpp::get_logger("robot_desktop_rviz"), "%s", s.c_str()); },
    [](const std::string &s, const std::string &, size_t) { RCLCPP_WARN(rclcpp::get_logger("robot_desktop_rviz"), "%s", s.c_str()); },
    [](const std::string &s, const std::string &, size_t) { RCLCPP_ERROR(rclcpp::get_logger("robot_desktop_rviz"), "%s", s.c_str()); });
  rviz_common::install_rviz_rendering_log_handlers();
  node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("robot_desktop_rviz");
  frame_ = new rviz_common::VisualizationFrame(node_, this);
  frame_->setApp(qApp); frame_->setSplashPath("");
  frame_->initialize(node_, path);
  frame_->setWindowFlags(Qt::Widget);
  // RViz retains pointers to its menus for config loading; keep them alive.
  frame_->menuBar()->hide();
  auto *layout = new QVBoxLayout(this); layout->setContentsMargins(0, 0, 0, 0); layout->addWidget(frame_);
  setPanelsVisible(false);
}
RvizHost::~RvizHost() { delete frame_; frame_ = nullptr; node_.reset(); }
bool RvizHost::loadProfile(const QString &path) {
  if (!QFileInfo::exists(path)) return false;
  frame_->loadDisplayConfig(path);
  for (auto *dock : frame_->findChildren<QDockWidget *>()) frame_->resizeDocks({dock}, {210}, Qt::Horizontal);
  setPanelsVisible(panelsVisible_);
  frame_->setWindowFlags(Qt::Widget); frame_->show(); return true;
}
void RvizHost::selectTool(const QString &id) {
  auto *manager = frame_->getManager()->getToolManager();
  for (int i = 0; i < manager->numTools(); ++i) if (manager->getTool(i)->getClassId() == id) { manager->setCurrentTool(manager->getTool(i)); return; }
}
void RvizHost::saveImage(const QString &path) { grab().save(path); }
uint64_t RvizHost::renderedFrames() const { return frame_->getManager()->getFrameCount(); }
void RvizHost::setPanelsVisible(bool visible) {
  panelsVisible_ = visible;
  for (auto *dock : frame_->findChildren<QDockWidget *>()) dock->setVisible(visible);
}
