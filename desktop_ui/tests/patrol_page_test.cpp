#include "patrol/patrol_client.h"
#include "patrol/patrol_page.h"

#include <QDir>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStringList>
#include <QTableWidget>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
#include <atomic>
#include <chrono>
#include <thread>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <patrol_interfaces/action/run_route.hpp>
#include <patrol_interfaces/msg/patrol_status.hpp>
#include <patrol_interfaces/srv/control_run.hpp>
#include <patrol_interfaces/srv/list_maps.hpp>
#include <patrol_interfaces/srv/list_routes.hpp>

class PatrolPageTest : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() {
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  }
  void cleanupTestCase() { rclcpp::shutdown(); }

  void pendingRunCancelIsRememberedBeforeGoalAcceptance() {
    using RunRoute = patrol_interfaces::action::RunRoute;
    auto node = std::make_shared<rclcpp::Node>("pending_run_cancel_fixture");
    std::atomic<int> cancelCalls{0};
    std::atomic<bool> executeStarted{false};
    auto callbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto server = rclcpp_action::create_server<RunRoute>(
        node, "/patrol/run_route",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const RunRoute::Goal>) {
          std::this_thread::sleep_for(std::chrono::milliseconds(400));
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunRoute>>) {
          cancelCalls.fetch_add(1);
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunRoute>> goal) {
          executeStarted.store(true);
          while (rclcpp::ok() && !goal->is_canceling())
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          goal->canceled(std::make_shared<RunRoute::Result>());
        }, rcl_action_server_get_default_options(), callbackGroup);
    Q_UNUSED(server);
    rclcpp::executors::MultiThreadedExecutor fakeExecutor(rclcpp::ExecutorOptions(), 2);
    fakeExecutor.add_node(node);
    std::thread fakeSpin([&] { fakeExecutor.spin(); });
    PatrolClient client;
    if (!client.available()) {
      fakeExecutor.cancel();
      fakeSpin.join();
      QSKIP("patrol_interfaces/robot_patrol is not built");
    }
    QTest::qWait(800);  // allow action client/server discovery

    client.setOperatorAuthorized(true);
    client.runRoute("route", "map", 1, 1, 1);
    QVERIFY(client.hasActiveOrPendingRun());
    client.cancelRunRoute();
    const bool cancelDelivered = QTest::qWaitFor([&] { return cancelCalls.load() >= 1; }, 3000);
    const bool started = QTest::qWaitFor([&] { return executeStarted.load(); }, 1000);
    const bool finished = QTest::qWaitFor(
        [&] { return !client.hasActiveOrPendingRun(); }, 3000);
    fakeExecutor.cancel();
    fakeSpin.join();
    QVERIFY2(cancelDelivered, "pre-accept cancel intent was not honored");
    QVERIFY(started);
    QVERIFY(finished);
  }

  void pausedRunUsesResumePermissionNotStartPermission() {
    auto node = std::make_shared<rclcpp::Node>("paused_resume_permission_fixture");
    std::atomic<int> controlCommand{-1};
    auto control = node->create_service<patrol_interfaces::srv::ControlRun>(
        "/patrol/control",
        [&](const std::shared_ptr<patrol_interfaces::srv::ControlRun::Request> request,
            std::shared_ptr<patrol_interfaces::srv::ControlRun::Response> response) {
          controlCommand.store(static_cast<int>(request->command));
          response->accepted = true;
        });
    Q_UNUSED(control);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    auto status = node->create_publisher<patrol_interfaces::msg::PatrolStatus>("/patrol/status", qos);
    auto timer = node->create_wall_timer(std::chrono::milliseconds(50), [status] {
      patrol_interfaces::msg::PatrolStatus message;
      message.instance_id = "paused-ui-fixture";
      message.run_id = "run-paused";
      message.route_id = "route-paused";
      message.map_id = "map-paused";
      message.state = 7;
      message.active = true;
      message.step_count = 1;
      status->publish(message);
    });
    Q_UNUSED(timer);
    rclcpp::executors::MultiThreadedExecutor fakeExecutor(rclcpp::ExecutorOptions(), 2);
    fakeExecutor.add_node(node);
    std::thread fakeSpin([&] { fakeExecutor.spin(); });

    PatrolClient client;
    if (!client.available()) {
      fakeExecutor.cancel();
      fakeSpin.join();
      QSKIP("patrol_interfaces/robot_patrol is not built");
    }
    PatrolPage page(&client, QString(), "amcl", QString(), "测试模式", false,
                    [](QString *reason) {
                      if (reason) *reason = QStringLiteral("活动任务不能启动新任务");
                      return false;
                    });
    page.setResumePermission([](QString *) { return true; });
    page.resize(1000, 700);
    page.show();
    QVERIFY(QTest::qWaitForWindowExposed(&page));

    auto *resume = page.findChild<QPushButton *>(QStringLiteral("patrolResume"));
    QVERIFY(resume);
    QTRY_VERIFY_WITH_TIMEOUT(resume->isEnabled(), 3000);
    resume->click();
    QTRY_COMPARE_WITH_TIMEOUT(controlCommand.load(), 2, 2000);

    fakeExecutor.cancel();
    fakeSpin.join();
    page.close();
  }

  void pageBuildsWithLocalMapAndOfflineService() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString image = directory.filePath("map.pgm");
    const QString yaml = directory.filePath("map.yaml");
    QFile imageFile(image);
    QVERIFY(imageFile.open(QIODevice::WriteOnly));
    QVERIFY(imageFile.write(QByteArray("P5\n20 20\n255\n") + QByteArray(400, char(205))) > 0);
    imageFile.close();
    QFile yamlFile(yaml);
    QVERIFY(yamlFile.open(QIODevice::WriteOnly));
    QVERIFY(yamlFile.write("image: map.pgm\nmode: trinary\nresolution: 0.05\n"
                           "origin: [-1.0, -1.0, 0.0]\nnegate: 0\n"
                           "free_thresh: 0.25\noccupied_thresh: 0.65\n") > 0);
    yamlFile.close();

    PatrolClient client;
    if (!client.available()) QSKIP("patrol_interfaces/robot_patrol is not built");
    PatrolPage page(&client, yaml, "amcl", "", "测试模式", false);
    page.resize(1000, 700);
    page.show();
    QVERIFY(QTest::qWaitForWindowExposed(&page));

    auto *mapEdit = page.findChild<QLineEdit *>(QStringLiteral("patrolMapYaml"));
    auto *tabs = page.findChild<QTabWidget *>(QStringLiteral("patrolTabs"));
    auto *registerMap = page.findChild<QPushButton *>(QStringLiteral("patrolRegisterMap"));
    auto *saveCurrent = page.findChild<QPushButton *>(QStringLiteral("patrolSaveCurrentPoint"));
    auto *start = page.findChild<QPushButton *>(QStringLiteral("patrolStart"));
    QVERIFY(mapEdit && tabs && registerMap && saveCurrent && start);
    QCOMPARE(mapEdit->text(), yaml);
    QCOMPARE(tabs->count(), 3);
    QVERIFY(registerMap->isEnabled());
    QVERIFY(!saveCurrent->isEnabled()); // no active map yet
    QVERIFY(!start->isEnabled());
    auto *readiness = page.findChild<QLabel *>(QStringLiteral("patrolStartReadiness"));
    QVERIFY(readiness);
    QVERIFY(readiness->text().contains(QStringLiteral("未设置活跃地图")));
    QVERIFY(readiness->text().contains(QStringLiteral("未选择已保存路线")));
    QVERIFY(readiness->text().contains(QStringLiteral("巡航后台未就绪")));
    QTest::qWait(300);
    page.close();
  }

  void startReadinessTracksPermissionsBackendAndUnsavedRoute() {
    using RunRoute = patrol_interfaces::action::RunRoute;
    auto node = std::make_shared<rclcpp::Node>("patrol_readiness_fixture");
    patrol_interfaces::msg::MapBinding map;
    map.map_id = "readiness-map"; map.name = "测试地图"; map.revision = 1;
    patrol_interfaces::msg::PatrolRoute route;
    route.route_id = "readiness-route"; route.map_id = map.map_id;
    route.name = "测试路线"; route.revision = 1;
    route.steps.resize(1); route.steps.front().point_name = "测试点";
    auto maps = node->create_service<patrol_interfaces::srv::ListMaps>(
        "/patrol/list_maps",
        [map](const std::shared_ptr<patrol_interfaces::srv::ListMaps::Request>,
              std::shared_ptr<patrol_interfaces::srv::ListMaps::Response> response) {
          response->success = true; response->maps.push_back(map); response->active_map = map;
        });
    auto routes = node->create_service<patrol_interfaces::srv::ListRoutes>(
        "/patrol/list_routes",
        [route](const std::shared_ptr<patrol_interfaces::srv::ListRoutes::Request>,
                std::shared_ptr<patrol_interfaces::srv::ListRoutes::Response> response) {
          response->success = true; response->routes.push_back(route);
        });
    Q_UNUSED(maps); Q_UNUSED(routes);
    int goalsReceived = 0;
    auto createServer = [&] {
      return rclcpp_action::create_server<RunRoute>(
          node, "/patrol/run_route",
          [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const RunRoute::Goal>) {
            ++goalsReceived;
            return rclcpp_action::GoalResponse::REJECT;
          },
          [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunRoute>>) {
            return rclcpp_action::CancelResponse::REJECT;
          },
          [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunRoute>>) {});
    };
    auto server = createServer();
    QTimer spin;
    connect(&spin, &QTimer::timeout, this, [node] { rclcpp::spin_some(node); });
    spin.start(10);
    PatrolClient client;
    if (!client.available()) QSKIP("patrol_interfaces/robot_patrol is not built");
    QStringList missing{
        QStringLiteral("尚未启用操作，请勾选主窗口顶部“启用操作”"),
        QStringLiteral("急停保持中，请明确解除急停"),
        QStringLiteral("巡航联锁尚未就绪，请检查心跳")};
    PatrolPage page(&client, QString(), "amcl", QString(), "测试模式", false,
                    [&missing](QString *reason) {
                      if (reason) *reason = missing.join('\n');
                      return missing.isEmpty();
                    });
    page.resize(1180, 760);
    page.show();
    auto *tabs = page.findChild<QTabWidget *>(QStringLiteral("patrolTabs"));
    auto *table = page.findChild<QTableWidget *>(QStringLiteral("patrolRoutesTable"));
    auto *start = page.findChild<QPushButton *>(QStringLiteral("patrolStart"));
    auto *readiness = page.findChild<QLabel *>(QStringLiteral("patrolStartReadiness"));
    auto *name = page.findChild<QLineEdit *>(QStringLiteral("patrolRouteName"));
    QVERIFY(tabs && table && start && readiness && name);
    tabs->setCurrentIndex(1);
    QVERIFY(QTest::qWaitForWindowExposed(&page));
    QTRY_VERIFY_WITH_TIMEOUT(client.routeServerReady(), 3000);
    // Let the constructor's initial discovery failure recover explicitly,
    // just as the operator's Refresh button does after backend startup.
    client.refreshMaps();
    QTRY_COMPARE_WITH_TIMEOUT(table->rowCount(), 1, 3000);
    table->selectRow(0);
    QTRY_VERIFY(readiness->text().contains(QStringLiteral("尚未启用操作")));
    QVERIFY(readiness->text().contains(QStringLiteral("急停保持中")));
    QVERIFY(readiness->text().contains(QStringLiteral("巡航联锁尚未就绪")));
    QVERIFY(!start->isEnabled());
    QVERIFY(readiness->isVisible());
    const QRect visibleBar(readiness->mapTo(&page, QPoint()), readiness->size());
    QVERIFY(page.rect().contains(visibleBar));

    missing.clear();
    QTRY_VERIFY(start->isEnabled());
    QVERIFY(readiness->text().contains(QStringLiteral("启动条件：已就绪")));
    QVERIFY(!readiness->text().contains(QStringLiteral("急停保持中")));

    // Losing the real action endpoint must remove the ready indication.
    server.reset();
    QTRY_VERIFY_WITH_TIMEOUT(!client.routeServerReady(), 5000);
    QTRY_VERIFY(!start->isEnabled());
    QVERIFY(readiness->text().contains(QStringLiteral("巡航后台未就绪")));
    server = createServer();
    QTRY_VERIFY_WITH_TIMEOUT(start->isEnabled(), 5000);
    name->setText(QStringLiteral("修改后的路线"));
    QTRY_VERIFY(!start->isEnabled());
    QVERIFY(readiness->text().contains(QStringLiteral("未保存修改")));
    QCOMPARE(goalsReceived, 0); // Readiness display never starts a task.
    page.close();
  }
};

QTEST_MAIN(PatrolPageTest)
#include "patrol_page_test.moc"
