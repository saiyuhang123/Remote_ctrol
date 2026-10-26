#include "adapters/desktop_remote_bridge.h"
#include "adapters/local_controller.h"
#include "ui/camera_panel.h"
#include "vision/vision_panel.h"

#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>

struct DesktopRemoteBridge::Impl {
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<rclcpp::ServiceBase::SharedPtr> services;
  QTimer timer;

  Impl(CameraPanel *camera, VisionPanel *vision, LocalController *controller) {
    node = std::make_shared<rclcpp::Node>("desktop_remote_bridge");
    executor.add_node(node);
    using Trigger = std_srvs::srv::Trigger;
    using SetBool = std_srvs::srv::SetBool;
    using Unlock = rcl_interfaces::srv::SetParametersAtomically;
    services.push_back(node->create_service<Trigger>("/desktop/remote/ensure_camera",
        [camera](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
          QString detail;
          response->success = camera->prepareRemote(&detail);
          response->message = detail.toStdString();
        }));
    services.push_back(node->create_service<Unlock>("/desktop/remote/release_estop",
        [controller](const std::shared_ptr<Unlock::Request> request, std::shared_ptr<Unlock::Response> response) {
          int64_t issued = 0, deadline = 0;
          for (const auto &parameter : request->parameters) {
            if (parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) continue;
            if (parameter.name == "issued_at_ns") issued = parameter.value.integer_value;
            if (parameter.name == "deadline_ns") deadline = parameter.value.integer_value;
          }
          const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count();
          if (request->parameters.size() != 2 || issued <= 0 || deadline <= issued ||
              deadline - issued > 2000000000LL || issued > now || now > deadline) {
            response->result.successful = false;
            response->result.reason = "解除急停请求无效或已过期，请确认现场状态后重新点击";
            return;
          }
          QString detail;
          response->result.successful = controller->releaseStopRemotely(issued, &detail);
          response->result.reason = detail.toStdString();
        }));
    for (const auto &feature : {QString("face"), QString("person"), QString("plate")}) {
      services.push_back(node->create_service<SetBool>(
          "/desktop/remote/vision_" + feature.toStdString(),
          [vision, feature](const std::shared_ptr<SetBool::Request> request,
                            std::shared_ptr<SetBool::Response> response) {
            QString detail;
            response->success = vision->setRemoteFeature("enable_" + feature, request->data, &detail);
            response->message = detail.toStdString();
          }));
    }
    // Service handlers run on the Qt thread. Existing widgets and process
    // managers keep their normal ownership and only start components once.
    timer.setInterval(30);
    QObject::connect(&timer, &QTimer::timeout, [this] {
      if (rclcpp::ok()) executor.spin_some(std::chrono::milliseconds(3));
    });
    timer.start();
  }
  ~Impl() { timer.stop(); executor.remove_node(node); }
};

DesktopRemoteBridge::DesktopRemoteBridge(CameraPanel *camera, VisionPanel *vision,
    LocalController *controller, bool offline, QObject *parent) : QObject(parent) {
  if (!offline) impl_ = std::make_unique<Impl>(camera, vision, controller);
}
DesktopRemoteBridge::~DesktopRemoteBridge() = default;
