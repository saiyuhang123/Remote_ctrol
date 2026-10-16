#include "adapters/local_controller.h"
#include "adapters/ros_adapter.h"
#include "runtime/config.h"
#include <QDir>
#include <QTest>
#include <QTimer>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <vector>
#include <atomic>
#include <thread>
#include <limits>

struct ActionFixture {
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("delayed_local_nav_fixture");
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread worker;
  void start() { executor.add_node(node); worker = std::thread([this] { executor.spin(); }); }
  ~ActionFixture() { executor.cancel(); if (worker.joinable()) worker.join(); }
};

class LocalControllerTest : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() { rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None); }
  void cleanupTestCase() { rclcpp::shutdown(); }
  void observerManualStopAndFeedbackTimeout() {
    const QString root = DESKTOP_UI_SOURCE_DIR;
    auto config = DesktopConfig::load(root + "/config/desktop.json", root, QDir(root).absoluteFilePath(".."));
    config.feedbackTimeoutMs = 400;
    auto fixture = std::make_shared<rclcpp::Node>("local_control_fixture");
    std::vector<geometry_msgs::msg::Twist> commands; std::vector<bool> stops;
    auto velocity = fixture->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_teleop", 10,
      [&](geometry_msgs::msg::Twist::ConstSharedPtr msg) { commands.push_back(*msg); });
    auto stop = fixture->create_subscription<std_msgs::msg::Bool>("/emergency_stop", 10,
      [&](std_msgs::msg::Bool::ConstSharedPtr msg) { stops.push_back(msg->data); });
    QTimer spin; connect(&spin, &QTimer::timeout, this, [&] { rclcpp::spin_some(fixture); }); spin.start(20);
    RosAdapter ros(config); LocalController controller(&ros, .3, .5, config.feedbackTimeoutMs);
    RobotSnapshot state;
    auto feedback = connect(&ros, &RosAdapter::snapshot, &controller, [&](const RobotSnapshot &s) {
      state = s; controller.updateFeedback(s.estopped, s.navigationActive, s.navigationKnown, s.estopValid);
    });
    QTest::qWait(500); QCOMPARE(commands.size(), 0); QCOMPARE(stops.size(), 0);
    QVERIFY(!controller.move(.2, .2)); QVERIFY(!controller.navigate(1., 2., 0.));
    controller.setControlEnabled(true);
    QVERIFY(controller.takeManualControl()); QTRY_VERIFY(controller.manualReady());
    QVERIFY(state.estopped); QVERIFY(!controller.move(.2, 0.));
    QVERIFY(controller.releaseStop()); QTRY_VERIFY(state.estopValid && !state.estopped);
    QVERIFY(controller.move(.9, 2.)); QTRY_VERIFY(!commands.empty() && commands.back().linear.x > 0.);
    QCOMPARE(commands.back().linear.x, .3); QCOMPARE(commands.back().angular.z, .5);
    controller.stopManual(); QTRY_COMPARE(commands.back().linear.x, 0.);
    const int count = commands.size(); QTest::qWait(180); QCOMPARE(commands.size(), count);
    QVERIFY(controller.move(.2, .1)); QTRY_COMPARE(commands.back().linear.x, .2);
    disconnect(feedback);
    QTRY_VERIFY_WITH_TIMEOUT(!stops.empty() && stops.back(), 2000);
    QTRY_COMPARE(commands.back().linear.x, 0.);
    QVERIFY(!controller.manualReady());
    QVERIFY(ros.publishVelocity(0., 0.)); QTest::qWait(250); QVERIFY(stops.back());
    controller.setControlEnabled(false); QVERIFY(!controller.move(.2, 0.));
  }
  void initialPoseAndOtherControllerConflict() {
    const QString root = DESKTOP_UI_SOURCE_DIR;
    auto config = DesktopConfig::load(root + "/config/desktop.json", root, root);
    auto fixture = std::make_shared<rclcpp::Node>("other_control_fixture");
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> poses;
    auto initial = fixture->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
      [&](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) { poses.push_back(*msg); });
    auto remote = fixture->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teleop", 10);
    QTimer spin; connect(&spin, &QTimer::timeout, this, [&] { rclcpp::spin_some(fixture); }); spin.start(20);
    RosAdapter ros(config); QTest::qWait(500);
    QVERIFY(!ros.setEmergencyStop(false));
    QVERIFY(!ros.navigate(1., 0., 0.));
    QVERIFY(ros.setEmergencyStop(true));
    QVERIFY(!ros.setInitialPose(std::numeric_limits<double>::quiet_NaN(), 0., 0.));
    QVERIFY(ros.setInitialPose(1., 2., M_PI / 2.)); QTRY_COMPARE(poses.size(), 1);
    QCOMPARE(poses.back().header.frame_id, std::string("map"));
    QVERIFY(std::abs(poses.back().pose.pose.orientation.z - std::sqrt(.5)) < 1e-9);
  }
  void emergencyAlsoCancelsGoalAcceptedAfterCancelRequest() {
    using Navigate = nav2_msgs::action::NavigateToPose;
    using Handle = rclcpp_action::ServerGoalHandle<Navigate>;
    ActionFixture fixture;
    std::shared_ptr<Handle> active;
    std::atomic_bool accepted{false}, canceled{false};
    auto server = rclcpp_action::create_server<Navigate>(fixture.node, "/navigate_to_pose",
      [](const auto &, auto) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }, [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<Handle> goal) { active = goal; accepted.store(true); });
    auto timer = fixture.node->create_wall_timer(std::chrono::milliseconds(20), [&] {
      if (active && active->is_canceling()) {
        active->canceled(std::make_shared<Navigate::Result>()); active.reset(); canceled.store(true);
      }
    });
    fixture.start();
    const QString root = DESKTOP_UI_SOURCE_DIR;
    auto config = DesktopConfig::load(root + "/config/desktop.json", root, root);
    RosAdapter ros(config); RobotSnapshot state;
    connect(&ros, &RosAdapter::snapshot, this, [&](const RobotSnapshot &s) { state = s; });
    QTRY_VERIFY_WITH_TIMEOUT(state.navigationAvailable, 3000);
    QVERIFY(ros.setEmergencyStop(false));
    QVERIFY(ros.navigate(1., 2., .3));
    QVERIFY(ros.cancelNavigation()); QVERIFY(ros.setEmergencyStop(true));
    QVERIFY(!ros.setEmergencyStop(false));
    QVERIFY(!ros.publishVelocity(.2, 0.));
    QTRY_VERIFY_WITH_TIMEOUT(accepted.load() && canceled.load(), 4000);
    QTRY_VERIFY(!state.navigationActive);
    QVERIFY(ros.setEmergencyStop(false));
  }
};
QTEST_GUILESS_MAIN(LocalControllerTest)
#include "local_controller_test.moc"
