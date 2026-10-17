#include "patrol/patrol_client.h"
#include "patrol/patrol_page.h"

#include <QDir>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QTest>
#include <atomic>
#include <chrono>
#include <thread>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <patrol_interfaces/action/run_route.hpp>
#include <patrol_interfaces/msg/patrol_status.hpp>
#include <patrol_interfaces/srv/control_run.hpp>

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
    QTest::qWait(300);
    page.close();
  }
};

QTEST_MAIN(PatrolPageTest)
#include "patrol_page_test.moc"
