#include "patrol/patrol_client.h"

#include <QUuid>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <type_traits>

#ifdef DESKTOP_HAS_PATROL_INTERFACES
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <patrol_interfaces/action/run_route.hpp>
#include <patrol_interfaces/msg/map_binding.hpp>
#include <patrol_interfaces/msg/operator_heartbeat.hpp>
#include <patrol_interfaces/msg/patrol_point.hpp>
#include <patrol_interfaces/msg/patrol_route.hpp>
#include <patrol_interfaces/msg/patrol_status.hpp>
#include <patrol_interfaces/msg/route_step.hpp>
#include <patrol_interfaces/msg/run_record.hpp>
#include <patrol_interfaces/srv/control_run.hpp>
#include <patrol_interfaces/srv/delete_point.hpp>
#include <patrol_interfaces/srv/delete_route.hpp>
#include <patrol_interfaces/srv/get_runs.hpp>
#include <patrol_interfaces/srv/list_maps.hpp>
#include <patrol_interfaces/srv/list_points.hpp>
#include <patrol_interfaces/srv/list_routes.hpp>
#include <patrol_interfaces/srv/register_map.hpp>
#include <patrol_interfaces/srv/save_current_point.hpp>
#include <patrol_interfaces/srv/set_active_map.hpp>
#include <patrol_interfaces/srv/upsert_point.hpp>
#include <patrol_interfaces/srv/upsert_route.hpp>

namespace {
PatrolMapInfo mapInfo(const patrol_interfaces::msg::MapBinding &message) {
  PatrolMapInfo info;
  info.mapId = QString::fromStdString(message.map_id);
  info.name = QString::fromStdString(message.name);
  info.backend = QString::fromStdString(message.backend);
  info.navigationYaml = QString::fromStdString(message.navigation_yaml);
  info.localizationFile = QString::fromStdString(message.localization_file);
  info.resolution = message.resolution;
  info.originX = message.origin_x;
  info.originY = message.origin_y;
  info.originYaw = message.origin_yaw;
  info.width = static_cast<int>(message.width);
  info.height = static_cast<int>(message.height);
  info.verified = message.verified;
  info.revision = message.revision;
  return info;
}

PatrolPointInfo pointInfo(const patrol_interfaces::msg::PatrolPoint &message) {
  PatrolPointInfo info;
  info.pointId = QString::fromStdString(message.point_id);
  info.mapId = QString::fromStdString(message.map_id);
  info.name = QString::fromStdString(message.name);
  info.x = message.x; info.y = message.y; info.yaw = message.yaw;
  info.source = QString::fromStdString(message.source);
  info.revision = message.revision;
  return info;
}

PatrolRouteInfo routeInfo(const patrol_interfaces::msg::PatrolRoute &message) {
  PatrolRouteInfo info;
  info.routeId = QString::fromStdString(message.route_id);
  info.mapId = QString::fromStdString(message.map_id);
  info.name = QString::fromStdString(message.name);
  info.revision = message.revision;
  for (const auto &stepMessage : message.steps) {
    PatrolRouteStepInfo step;
    step.stepId = QString::fromStdString(stepMessage.step_id);
    step.pointId = QString::fromStdString(stepMessage.point_id);
    step.pointName = QString::fromStdString(stepMessage.point_name);
    step.x = stepMessage.x; step.y = stepMessage.y; step.yaw = stepMessage.yaw;
    step.dwellSeconds = stepMessage.dwell_seconds;
    info.steps.push_back(step);
  }
  return info;
}

PatrolRunInfo runInfo(const patrol_interfaces::msg::RunRecord &message) {
  PatrolRunInfo info;
  info.runId = QString::fromStdString(message.run_id);
  info.routeId = QString::fromStdString(message.route_id);
  info.mapId = QString::fromStdString(message.map_id);
  info.state = message.state;
  info.success = message.success;
  info.completedLoops = message.completed_loops;
  info.completedSteps = message.completed_steps;
  info.errorCode = QString::fromStdString(message.error_code);
  info.message = QString::fromStdString(message.message);
  info.routeRevision = message.route_revision;
  info.mapRevision = message.map_revision;
  return info;
}

PatrolStatusInfo statusInfo(const patrol_interfaces::msg::PatrolStatus &message) {
  PatrolStatusInfo info;
  info.instanceId = QString::fromStdString(message.instance_id);
  info.runId = QString::fromStdString(message.run_id);
  info.routeId = QString::fromStdString(message.route_id);
  info.mapId = QString::fromStdString(message.map_id);
  info.state = message.state;
  info.loopIndex = message.loop_index;
  info.totalLoops = message.total_loops;
  info.stepIndex = message.step_index;
  info.stepCount = message.step_count;
  info.currentPointName = QString::fromStdString(message.current_point_name);
  info.dwellRemaining = message.dwell_remaining;
  info.errorCode = QString::fromStdString(message.error_code);
  info.message = QString::fromStdString(message.message);
  info.currentGoalId = QString::fromStdString(message.current_goal_id);
  info.active = message.active;
  return info;
}
}  // namespace
#endif

struct PatrolClient::Impl {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  PatrolClient *owner = nullptr;
  QString ownerSessionId;
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  rclcpp::Client<patrol_interfaces::srv::ListMaps>::SharedPtr listMaps;
  rclcpp::Client<patrol_interfaces::srv::RegisterMap>::SharedPtr registerMap;
  rclcpp::Client<patrol_interfaces::srv::SetActiveMap>::SharedPtr setActiveMap;
  rclcpp::Client<patrol_interfaces::srv::ListPoints>::SharedPtr listPoints;
  rclcpp::Client<patrol_interfaces::srv::UpsertPoint>::SharedPtr upsertPoint;
  rclcpp::Client<patrol_interfaces::srv::DeletePoint>::SharedPtr deletePoint;
  rclcpp::Client<patrol_interfaces::srv::SaveCurrentPoint>::SharedPtr saveCurrentPoint;
  rclcpp::Client<patrol_interfaces::srv::ListRoutes>::SharedPtr listRoutes;
  rclcpp::Client<patrol_interfaces::srv::UpsertRoute>::SharedPtr upsertRoute;
  rclcpp::Client<patrol_interfaces::srv::DeleteRoute>::SharedPtr deleteRoute;
  rclcpp::Client<patrol_interfaces::srv::GetRuns>::SharedPtr getRuns;
  rclcpp::Client<patrol_interfaces::srv::ControlRun>::SharedPtr controlRun;
  rclcpp_action::Client<patrol_interfaces::action::RunRoute>::SharedPtr runRoute;
  rclcpp::Subscription<patrol_interfaces::msg::PatrolStatus>::SharedPtr statusSubscription;
  rclcpp::Publisher<patrol_interfaces::msg::OperatorHeartbeat>::SharedPtr operatorHeartbeat;
  rclcpp::TimerBase::SharedPtr operatorHeartbeatTimer;
  rclcpp_action::Client<patrol_interfaces::action::RunRoute>::GoalHandle::SharedPtr currentGoal;
  std::atomic_bool operatorAuthorized{false};
  bool goalPending = false;
  bool cancelRequestedWhilePending = false;
  mutable std::mutex mutex;
  QVector<PatrolMapInfo> mapsData;
  QString activeMapId;
  QVector<PatrolPointInfo> pointsData;
  QVector<PatrolRouteInfo> routesData;
  QVector<PatrolRunInfo> runsData;
  PatrolStatusInfo statusData;

  Impl(PatrolClient *owner_, const QString &sessionId) : owner(owner_), ownerSessionId(sessionId) {
    node = std::make_shared<rclcpp::Node>("robot_desktop_patrol");
    listMaps = node->create_client<patrol_interfaces::srv::ListMaps>("/patrol/list_maps");
    registerMap = node->create_client<patrol_interfaces::srv::RegisterMap>("/patrol/register_map");
    setActiveMap = node->create_client<patrol_interfaces::srv::SetActiveMap>("/patrol/set_active_map");
    listPoints = node->create_client<patrol_interfaces::srv::ListPoints>("/patrol/list_points");
    upsertPoint = node->create_client<patrol_interfaces::srv::UpsertPoint>("/patrol/upsert_point");
    deletePoint = node->create_client<patrol_interfaces::srv::DeletePoint>("/patrol/delete_point");
    saveCurrentPoint = node->create_client<patrol_interfaces::srv::SaveCurrentPoint>("/patrol/save_current_point");
    listRoutes = node->create_client<patrol_interfaces::srv::ListRoutes>("/patrol/list_routes");
    upsertRoute = node->create_client<patrol_interfaces::srv::UpsertRoute>("/patrol/upsert_route");
    deleteRoute = node->create_client<patrol_interfaces::srv::DeleteRoute>("/patrol/delete_route");
    getRuns = node->create_client<patrol_interfaces::srv::GetRuns>("/patrol/get_runs");
    controlRun = node->create_client<patrol_interfaces::srv::ControlRun>("/patrol/control");
    runRoute = rclcpp_action::create_client<patrol_interfaces::action::RunRoute>(node, "/patrol/run_route");
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    statusSubscription = node->create_subscription<patrol_interfaces::msg::PatrolStatus>(
        "/patrol/status", qos, [this](patrol_interfaces::msg::PatrolStatus::ConstSharedPtr message) {
          PatrolStatusInfo info;
          { std::lock_guard<std::mutex> lock(mutex); statusData = statusInfo(*message); info = statusData; }
          emit owner->statusChanged();
        });
    operatorHeartbeat =
        node->create_publisher<patrol_interfaces::msg::OperatorHeartbeat>(
            "/patrol/operator_heartbeat", 10);
    operatorHeartbeatTimer = node->create_wall_timer(
        std::chrono::milliseconds(200), [this] {
          patrol_interfaces::msg::OperatorHeartbeat message;
          message.session_id = ownerSessionId.toStdString();
          message.client_id = "desktop_ui";
          message.enabled = operatorAuthorized.load();
          message.stamp = node->now();
          operatorHeartbeat->publish(message);
        });
    executor.add_node(node);
    thread = std::thread([this] {
      try { executor.spin(); } catch (const std::exception &error) {
        QMetaObject::invokeMethod(owner, [this, text = QString::fromUtf8(error.what())] {
          emit owner->errorOccurred(QStringLiteral("本地巡航 ROS 线程退出：") + text);
        }, Qt::QueuedConnection);
      }
    });
  }

  ~Impl() {
    executor.cancel();
    if (thread.joinable()) thread.join();
    executor.remove_node(node);
  }

  void error(const QString &message) { emit owner->errorOccurred(message); }
  void finished(const QString &operation, bool success, const QString &message) {
    emit owner->operationFinished(operation, success, message);
  }
  bool ready(const rclcpp::ClientBase::SharedPtr &client, const QString &name) {
    if (client->service_is_ready()) return true;
    error(name + QStringLiteral(" 服务未就绪；请先启动 robot_patrol"));
    return false;
  }

  template<class Service, class Callback>
  void call(const std::string &name, const QString &operation,
            const std::shared_ptr<typename Service::Request> &request, Callback callback) {
    typename rclcpp::Client<Service>::SharedPtr client;
    if constexpr (std::is_same_v<Service, patrol_interfaces::srv::ListMaps>) client = listMaps;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::RegisterMap>) client = registerMap;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::SetActiveMap>) client = setActiveMap;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::ListPoints>) client = listPoints;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::UpsertPoint>) client = upsertPoint;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::DeletePoint>) client = deletePoint;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::SaveCurrentPoint>) client = saveCurrentPoint;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::ListRoutes>) client = listRoutes;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::UpsertRoute>) client = upsertRoute;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::DeleteRoute>) client = deleteRoute;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::GetRuns>) client = getRuns;
    else if constexpr (std::is_same_v<Service, patrol_interfaces::srv::ControlRun>) client = controlRun;
    if (!ready(client, QString::fromStdString(name))) return;
    client->async_send_request(request, [this, operation, callback](
        typename rclcpp::Client<Service>::SharedFuture future) {
      try { callback(*future.get()); }
      catch (const std::exception &error) { finished(operation, false, QString::fromUtf8(error.what())); }
    });
  }
#else
  explicit Impl(PatrolClient *owner_) { Q_UNUSED(owner_); }
#endif
};

PatrolClient::PatrolClient(QObject *parent)
    : QObject(parent),
      impl_(std::make_unique<Impl>(this, QStringLiteral("desktop_") +
                                             QUuid::createUuid().toString(QUuid::WithoutBraces))) {}

PatrolClient::~PatrolClient() = default;

bool PatrolClient::available() const {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  return true;
#else
  return false;
#endif
}

QString PatrolClient::ownerSessionId() const {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  return impl_->ownerSessionId;
#else
  return {};
#endif
}

QVector<PatrolMapInfo> PatrolClient::maps() const {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->mapsData;
#else
  return {};
#endif
}

QString PatrolClient::activeMapId() const {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->activeMapId;
#else
  return {};
#endif
}

QVector<PatrolPointInfo> PatrolClient::points() const {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->pointsData;
#else
  return {};
#endif
}

QVector<PatrolRouteInfo> PatrolClient::routes() const {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->routesData;
#else
  return {};
#endif
}

QVector<PatrolRunInfo> PatrolClient::runs() const {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->runsData;
#else
  return {};
#endif
}

PatrolStatusInfo PatrolClient::status() const {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->statusData;
#else
  PatrolStatusInfo info;
  info.message = QStringLiteral("本地巡航接口未构建");
  return info;
#endif
}

QString PatrolClient::stateName(int state) {
  switch (state) {
    case 0: return QStringLiteral("空闲");
    case 1: return QStringLiteral("校验中");
    case 2: return QStringLiteral("派发目标");
    case 3: return QStringLiteral("导航中");
    case 4: return QStringLiteral("等待停稳");
    case 5: return QStringLiteral("停留中");
    case 6: return QStringLiteral("暂停中");
    case 7: return QStringLiteral("已暂停");
    case 8: return QStringLiteral("取消中");
    case 9: return QStringLiteral("已取消");
    case 10: return QStringLiteral("已完成");
    case 11: return QStringLiteral("失败");
    case 12: return QStringLiteral("已中断");
    case 13: return QStringLiteral("停止中");
    default: return QStringLiteral("未知");
  }
}

#ifdef DESKTOP_HAS_PATROL_INTERFACES
#else
#define PATROL_UNAVAILABLE(operation) \
  emit operationFinished(operation, false, QStringLiteral("未构建 patrol_interfaces/robot_patrol，本地巡航不可用"))
#endif

void PatrolClient::refreshMaps() {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  impl_->call<patrol_interfaces::srv::ListMaps>(
      "/patrol/list_maps", QStringLiteral("刷新地图"), std::make_shared<patrol_interfaces::srv::ListMaps::Request>(),
      [this](const patrol_interfaces::srv::ListMaps::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        { std::lock_guard<std::mutex> lock(impl_->mutex);
          impl_->mapsData.clear(); impl_->mapsData.reserve(response.maps.size());
          for (const auto &item : response.maps) impl_->mapsData.push_back(mapInfo(item));
          impl_->activeMapId = QString::fromStdString(response.active_map.map_id);
        }
        emit mapsChanged();
        emit operationFinished(QStringLiteral("刷新地图"), true, QStringLiteral("已刷新"));
      });
#else
  PATROL_UNAVAILABLE(QStringLiteral("刷新地图"));
#endif
}

void PatrolClient::registerMap(const QString &navigationYaml, const QString &backend,
                               const QString &localizationFile, const QString &name) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::RegisterMap::Request>();
  request->navigation_yaml = navigationYaml.toStdString();
  request->backend = backend.toStdString();
  request->localization_file = localizationFile.toStdString();
  request->name = name.toStdString();
  impl_->call<patrol_interfaces::srv::RegisterMap>(
      "/patrol/register_map", QStringLiteral("登记地图"), request,
      [this](const patrol_interfaces::srv::RegisterMap::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        emit operationFinished(QStringLiteral("登记地图"), true, QStringLiteral("地图已登记，可设为活跃地图"));
        refreshMaps();
      });
#else
  Q_UNUSED(navigationYaml); Q_UNUSED(backend); Q_UNUSED(localizationFile); Q_UNUSED(name);
  PATROL_UNAVAILABLE(QStringLiteral("登记地图"));
#endif
}

void PatrolClient::setActiveMap(const QString &mapId, quint64 expectedRevision) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::SetActiveMap::Request>();
  request->map_id = mapId.toStdString(); request->expected_revision = expectedRevision;
  impl_->call<patrol_interfaces::srv::SetActiveMap>(
      "/patrol/set_active_map", QStringLiteral("设置活跃地图"), request,
      [this](const patrol_interfaces::srv::SetActiveMap::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        { std::lock_guard<std::mutex> lock(impl_->mutex);
          impl_->activeMapId = QString::fromStdString(response.active_map.map_id); }
        emit mapsChanged();
        emit operationFinished(QStringLiteral("设置活跃地图"), true, QStringLiteral("已设为活跃地图"));
      });
#else
  Q_UNUSED(mapId); Q_UNUSED(expectedRevision);
  PATROL_UNAVAILABLE(QStringLiteral("设置活跃地图"));
#endif
}

void PatrolClient::refreshPoints(const QString &mapId) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::ListPoints::Request>();
  request->map_id = mapId.toStdString();
  impl_->call<patrol_interfaces::srv::ListPoints>(
      "/patrol/list_points", QStringLiteral("刷新点位"), request,
      [this](const patrol_interfaces::srv::ListPoints::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        { std::lock_guard<std::mutex> lock(impl_->mutex);
          impl_->pointsData.clear(); impl_->pointsData.reserve(response.points.size());
          for (const auto &item : response.points) impl_->pointsData.push_back(pointInfo(item)); }
        emit pointsChanged();
      });
#else
  Q_UNUSED(mapId);
  PATROL_UNAVAILABLE(QStringLiteral("刷新点位"));
#endif
}

void PatrolClient::upsertPoint(const QString &pointId, const QString &mapId, const QString &name,
                               double x, double y, double yaw, quint64 expectedRevision) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::UpsertPoint::Request>();
  request->point_id = pointId.toStdString(); request->map_id = mapId.toStdString();
  request->name = name.toStdString(); request->x = x; request->y = y; request->yaw = yaw;
  request->expected_revision = expectedRevision;
  impl_->call<patrol_interfaces::srv::UpsertPoint>(
      "/patrol/upsert_point", QStringLiteral("保存点位"), request,
      [this, mapId](const patrol_interfaces::srv::UpsertPoint::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        emit operationFinished(QStringLiteral("保存点位"), true, QStringLiteral("点位已保存"));
        refreshPoints(mapId);
        refreshRoutes(mapId);
      });
#else
  Q_UNUSED(pointId); Q_UNUSED(mapId); Q_UNUSED(name); Q_UNUSED(x); Q_UNUSED(y); Q_UNUSED(yaw);
  Q_UNUSED(expectedRevision);
  PATROL_UNAVAILABLE(QStringLiteral("保存点位"));
#endif
}

void PatrolClient::deletePoint(const QString &pointId, quint64 expectedRevision) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::DeletePoint::Request>();
  request->point_id = pointId.toStdString(); request->expected_revision = expectedRevision;
  impl_->call<patrol_interfaces::srv::DeletePoint>(
      "/patrol/delete_point", QStringLiteral("删除点位"), request,
      [this](const patrol_interfaces::srv::DeletePoint::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        emit operationFinished(QStringLiteral("删除点位"), true, QStringLiteral("点位已删除"));
        refreshPoints();
        const QString mapId = activeMapId();
        if (!mapId.isEmpty()) refreshRoutes(mapId);
      });
#else
  Q_UNUSED(pointId); Q_UNUSED(expectedRevision);
  PATROL_UNAVAILABLE(QStringLiteral("删除点位"));
#endif
}

void PatrolClient::saveCurrentPoint(const QString &mapId, const QString &name) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::SaveCurrentPoint::Request>();
  request->map_id = mapId.toStdString(); request->name = name.toStdString();
  request->request_id = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
  impl_->call<patrol_interfaces::srv::SaveCurrentPoint>(
      "/patrol/save_current_point", QStringLiteral("保存当前位置为点位"), request,
      [this, mapId](const patrol_interfaces::srv::SaveCurrentPoint::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        emit operationFinished(QStringLiteral("保存当前位置为点位"), true,
                               QString::fromStdString(response.message));
        refreshPoints(mapId);
        refreshRoutes(mapId);
      });
#else
  Q_UNUSED(mapId); Q_UNUSED(name);
  PATROL_UNAVAILABLE(QStringLiteral("保存当前位置为点位"));
#endif
}

void PatrolClient::refreshRoutes(const QString &mapId) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::ListRoutes::Request>();
  request->map_id = mapId.toStdString();
  impl_->call<patrol_interfaces::srv::ListRoutes>(
      "/patrol/list_routes", QStringLiteral("刷新路线"), request,
      [this](const patrol_interfaces::srv::ListRoutes::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        { std::lock_guard<std::mutex> lock(impl_->mutex);
          impl_->routesData.clear(); impl_->routesData.reserve(response.routes.size());
          for (const auto &item : response.routes) impl_->routesData.push_back(routeInfo(item)); }
        emit routesChanged();
      });
#else
  Q_UNUSED(mapId);
  PATROL_UNAVAILABLE(QStringLiteral("刷新路线"));
#endif
}

void PatrolClient::upsertRoute(const QString &routeId, const QString &mapId, const QString &name,
                               const QVector<PatrolRouteStepInfo> &steps, quint64 expectedRevision) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::UpsertRoute::Request>();
  request->route_id = routeId.toStdString(); request->map_id = mapId.toStdString();
  request->name = name.toStdString(); request->expected_revision = expectedRevision;
  for (const auto &stepInfo : steps) {
    patrol_interfaces::msg::RouteStep step;
    step.point_id = stepInfo.pointId.toStdString();
    step.dwell_seconds = stepInfo.dwellSeconds;
    request->steps.push_back(step);
  }
  impl_->call<patrol_interfaces::srv::UpsertRoute>(
      "/patrol/upsert_route", QStringLiteral("保存路线"), request,
      [this, mapId](const patrol_interfaces::srv::UpsertRoute::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        emit operationFinished(QStringLiteral("保存路线"), true, QStringLiteral("路线已保存"));
        refreshRoutes(mapId);
      });
#else
  Q_UNUSED(routeId); Q_UNUSED(mapId); Q_UNUSED(name); Q_UNUSED(steps); Q_UNUSED(expectedRevision);
  PATROL_UNAVAILABLE(QStringLiteral("保存路线"));
#endif
}

void PatrolClient::deleteRoute(const QString &routeId, quint64 expectedRevision) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::DeleteRoute::Request>();
  request->route_id = routeId.toStdString(); request->expected_revision = expectedRevision;
  impl_->call<patrol_interfaces::srv::DeleteRoute>(
      "/patrol/delete_route", QStringLiteral("删除路线"), request,
      [this](const patrol_interfaces::srv::DeleteRoute::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        emit operationFinished(QStringLiteral("删除路线"), true, QStringLiteral("路线已删除"));
        refreshRoutes();
      });
#else
  Q_UNUSED(routeId); Q_UNUSED(expectedRevision);
  PATROL_UNAVAILABLE(QStringLiteral("删除路线"));
#endif
}

void PatrolClient::refreshRuns(const QString &mapId, const QString &routeId) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::GetRuns::Request>();
  request->map_id = mapId.toStdString(); request->route_id = routeId.toStdString(); request->limit = 100;
  impl_->call<patrol_interfaces::srv::GetRuns>(
      "/patrol/get_runs", QStringLiteral("刷新任务记录"), request,
      [this](const patrol_interfaces::srv::GetRuns::Response &response) {
        if (!response.success) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        { std::lock_guard<std::mutex> lock(impl_->mutex);
          impl_->runsData.clear(); impl_->runsData.reserve(response.runs.size());
          for (const auto &item : response.runs) impl_->runsData.push_back(runInfo(item)); }
        emit runsChanged();
      });
#else
  Q_UNUSED(mapId); Q_UNUSED(routeId);
  PATROL_UNAVAILABLE(QStringLiteral("刷新任务记录"));
#endif
}

void PatrolClient::runRoute(const QString &routeId, const QString &mapId,
                            quint64 routeRevision, quint64 mapRevision, quint32 loopCount) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  if (!impl_->runRoute->action_server_is_ready()) {
    emit errorOccurred(QStringLiteral("巡航 Action 未就绪；请先启动 robot_patrol"));
    return;
  }
  patrol_interfaces::action::RunRoute::Goal goal;
  goal.request_id = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
  goal.owner_session_id = impl_->ownerSessionId.toStdString();
  goal.route_id = routeId.toStdString(); goal.map_id = mapId.toStdString();
  goal.route_revision = routeRevision; goal.map_revision = mapRevision;
  goal.loop_count = loopCount;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->goalPending = true;
    impl_->cancelRequestedWhilePending = false;
  }
  rclcpp_action::Client<patrol_interfaces::action::RunRoute>::SendGoalOptions options;
  options.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<patrol_interfaces::action::RunRoute>::SharedPtr handle) {
    bool cancelRequested = false;
    {
      std::lock_guard<std::mutex> lock(impl_->mutex);
      impl_->goalPending = false;
      if (handle) impl_->currentGoal = handle;
      cancelRequested = impl_->cancelRequestedWhilePending;
      impl_->cancelRequestedWhilePending = false;
    }
    if (!handle) { emit errorOccurred(QStringLiteral("巡航目标被拒绝")); return; }
    if (cancelRequested) {
      impl_->runRoute->async_cancel_goal(handle);
      emit operationFinished(QStringLiteral("取消巡航"), true,
                             QStringLiteral("目标接受后已按待处理取消意图立即取消"));
    } else {
      emit operationFinished(QStringLiteral("启动巡航"), true,
                             QStringLiteral("巡航目标已接受"));
    }
  };
  options.feedback_callback = [this](rclcpp_action::ClientGoalHandle<patrol_interfaces::action::RunRoute>::SharedPtr,
                                     const std::shared_ptr<const patrol_interfaces::action::RunRoute::Feedback> feedback) {
    { std::lock_guard<std::mutex> lock(impl_->mutex); impl_->statusData = statusInfo(feedback->status); }
    emit statusChanged();
  };
  options.result_callback = [this](const rclcpp_action::ClientGoalHandle<patrol_interfaces::action::RunRoute>::WrappedResult &result) {
    { std::lock_guard<std::mutex> lock(impl_->mutex); impl_->currentGoal.reset(); }
    QString message = QStringLiteral("巡航 Action 结束");
    if (result.result) message = QString::fromStdString(result.result->message);
    emit operationFinished(QStringLiteral("巡航任务"), result.code == rclcpp_action::ResultCode::SUCCEEDED, message);
  };
  impl_->runRoute->async_send_goal(goal, options);
#else
  Q_UNUSED(routeId); Q_UNUSED(mapId); Q_UNUSED(routeRevision); Q_UNUSED(mapRevision); Q_UNUSED(loopCount);
  PATROL_UNAVAILABLE(QStringLiteral("启动巡航"));
#endif
}

void PatrolClient::cancelRunRoute() {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  rclcpp_action::Client<patrol_interfaces::action::RunRoute>::GoalHandle::SharedPtr goal;
  bool pending = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    goal = impl_->currentGoal;
    if (!goal && impl_->goalPending) {
      impl_->cancelRequestedWhilePending = true;
      pending = true;
    }
  }
  if (goal) {
    impl_->runRoute->async_cancel_goal(goal);
    emit operationFinished(QStringLiteral("取消巡航"), true,
                           QStringLiteral("已请求取消巡航任务"));
  } else if (pending) {
    emit operationFinished(QStringLiteral("取消巡航"), true,
                           QStringLiteral("已记录取消意图；目标接受后将立即取消"));
  } else {
    emit errorOccurred(QStringLiteral("当前没有可取消的本地巡航 Action"));
  }
#else
  PATROL_UNAVAILABLE(QStringLiteral("取消巡航"));
#endif
}

bool PatrolClient::hasActiveOrPendingRun() const {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->statusData.active || impl_->goalPending || static_cast<bool>(impl_->currentGoal);
#else
  return false;
#endif
}

void PatrolClient::setOperatorAuthorized(bool enabled) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  impl_->operatorAuthorized.store(enabled);
#else
  Q_UNUSED(enabled);
#endif
}

void PatrolClient::controlRun(quint8 command, const QString &runId) {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
  auto request = std::make_shared<patrol_interfaces::srv::ControlRun::Request>();
  request->command = command; request->run_id = runId.toStdString();
  request->owner_session_id = impl_->ownerSessionId.toStdString();
  request->command_id = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
  impl_->call<patrol_interfaces::srv::ControlRun>(
      "/patrol/control", QStringLiteral("巡航控制"), request,
      [this](const patrol_interfaces::srv::ControlRun::Response &response) {
        if (!response.accepted) { emit errorOccurred(QString::fromStdString(response.message)); return; }
        { std::lock_guard<std::mutex> lock(impl_->mutex); impl_->statusData = statusInfo(response.status); }
        emit statusChanged();
        emit operationFinished(QStringLiteral("巡航控制"), true, QString::fromStdString(response.message));
      });
#else
  Q_UNUSED(command); Q_UNUSED(runId);
  PATROL_UNAVAILABLE(QStringLiteral("巡航控制"));
#endif
}
