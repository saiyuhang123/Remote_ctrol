#include "ui/mainwindow.h"
#include "runtime/config.h"
#include "rviz/rviz_host.h"
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTest>
#include <QTimer>
#include <QWebSocketServer>
#include <QWebSocket>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class UiIntegrationTest : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() { rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None); }
  void cleanupTestCase() { rclcpp::shutdown(); }
  void realWidgetsRosFeedbackAndMockGateway() {
    QWebSocketServer server("desktop-offline-integration", QWebSocketServer::NonSecureMode);
    QVERIFY(server.listen(QHostAddress::LocalHost, 0));
    QWebSocket *peer = nullptr; QVector<QJsonObject> messages; bool estopped = false;
    connect(&server, &QWebSocketServer::newConnection, this, [&] {
      peer = server.nextPendingConnection();
      connect(peer, &QWebSocket::textMessageReceived, this, [&](const QString &text) {
        auto message = QJsonDocument::fromJson(text.toUtf8()).object(); messages << message;
        if (message.value("type") == "estop") estopped = true;
        if (message.value("type") == "cmd_vel") estopped = false;
      });
    });
    auto node = std::make_shared<rclcpp::Node>("desktop_isolated_fixture");
    auto stop = node->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
    auto odom = node->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
    auto status = node->create_publisher<action_msgs::msg::GoalStatusArray>("/navigate_to_pose/_action/status", rclcpp::QoS(1).reliable().transient_local());
    auto goal = node->create_publisher<geometry_msgs::msg::PoseStamped>("/desktop_ui/goal_request", 10);
    auto initial = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/desktop_ui/initial_pose_request", 10);
    tf2_ros::TransformBroadcaster tf(node);
    QTimer publish;
    connect(&publish, &QTimer::timeout, this, [&] {
      std_msgs::msg::Bool stopMessage; stopMessage.data = estopped; stop->publish(stopMessage);
      nav_msgs::msg::Odometry odomMessage; odomMessage.header.stamp = node->now(); odomMessage.header.frame_id = "odom"; odomMessage.child_frame_id = "base_link"; odomMessage.pose.pose.orientation.w = 1.; odomMessage.twist.twist.linear.x = .123; odom->publish(odomMessage);
      status->publish(action_msgs::msg::GoalStatusArray{});
      geometry_msgs::msg::TransformStamped transform; transform.header.stamp = node->now(); transform.header.frame_id = "map"; transform.child_frame_id = "base_link"; transform.transform.translation.x = 1.; transform.transform.rotation.w = 1.; tf.sendTransform(transform);
    }); publish.start(50);
    const QString root = DESKTOP_UI_SOURCE_DIR;
    auto config = DesktopConfig::load(root + "/config/desktop.json", root, QDir(root).absoluteFilePath(".."));
    config.gatewayUrl = "ws://127.0.0.1:" + QString::number(server.serverPort());
    config.backendUrl = "http://127.0.0.1:9";
    MainWindow window(config, false); window.show(); window.activateWindow();
    QVERIFY(QTest::qWaitForWindowExposed(&window));
    auto *arm = window.findChild<QCheckBox *>("enableControl");
    auto *pose = window.findChild<QLabel *>("poseStatus");
    QVERIFY(arm && pose);
    QTRY_VERIFY_WITH_TIMEOUT(peer && arm->isEnabled() && pose->text().contains("x="), 10000);
    auto *rviz = window.findChild<RvizHost *>(); QVERIFY(rviz); QTRY_VERIFY(rviz->renderedFrames() > 5);
    QTest::qWait(200); QCOMPARE(messages.size(), 0);
    auto *send = window.findChild<QPushButton *>("sendGoal"); QVERIFY(send); QVERIFY(!send->isEnabled());
    arm->setChecked(true);
    auto *x = window.findChild<QDoubleSpinBox *>("goalX"); auto *y = window.findChild<QDoubleSpinBox *>("goalY"); auto *yaw = window.findChild<QDoubleSpinBox *>("goalYaw");
    QVERIFY(x && y && yaw); x->setValue(2.); y->setValue(-1.); yaw->setValue(45.);
    QTest::mouseClick(send, Qt::LeftButton);
    QTRY_COMPARE(messages.size(), 1); QCOMPARE(messages.last().value("type").toString(), QString("nav_goal"));
    QVERIFY(std::abs(messages.last().value("yaw").toDouble() - 0.785398163397) < 1e-9);
    geometry_msgs::msg::PoseStamped clicked; clicked.header.frame_id = "map"; clicked.header.stamp = node->now(); clicked.pose.position.x = 4.; clicked.pose.orientation.w = 1.; goal->publish(clicked);
    QTRY_COMPARE(messages.size(), 2); QCOMPARE(messages.last().value("x").toDouble(), 4.);
    geometry_msgs::msg::PoseWithCovarianceStamped relocated; relocated.header = clicked.header; relocated.pose.pose = clicked.pose; initial->publish(relocated);
    QTRY_COMPARE(messages.size(), 3); QCOMPARE(messages.last().value("type").toString(), QString("initial_pose"));
    auto *take = window.findChild<QPushButton *>("takeManual"); auto *unlock = window.findChild<QPushButton *>("releaseStop"); auto *forward = window.findChild<QPushButton *>("manualDirection1");
    QVERIFY(take && unlock && forward);
    QTest::mouseClick(take, Qt::LeftButton);
    QTRY_COMPARE(messages.last().value("type").toString(), QString("estop"));
    QTRY_VERIFY(take->text() == "手动已接管" && unlock->isEnabled());
    QVERIFY(!forward->isEnabled()); QTest::mouseClick(unlock, Qt::LeftButton);
    QTRY_VERIFY(forward->isEnabled());
    QTest::mousePress(forward, Qt::LeftButton); QTest::qWait(120);
    QTRY_VERIFY(messages.last().value("linear").toDouble() > 0.);
    QTest::mouseRelease(forward, Qt::LeftButton);
    QTRY_COMPARE(messages.last().value("linear").toDouble(), 0.);
    int count = messages.size(); QTest::qWait(180); QCOMPARE(messages.size(), count);
    arm->setChecked(false); count = messages.size();
    QTest::qWait(120); QCOMPARE(messages.size(), count);
    QVERIFY(window.saveScreenshot("/tmp/robot_desktop_integration.png"));
    publish.stop();
    server.close();
  }
};
QTEST_MAIN(UiIntegrationTest)
#include "ui_integration_test.moc"
