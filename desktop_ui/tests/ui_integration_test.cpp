#include "ui/mainwindow.h"
#include "ui/map_editor_dialog.h"
#include "runtime/config.h"
#include "rviz/rviz_host.h"
#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScreen>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
QString propertyText(rviz_common::properties::Property *property) {
  if (!property) return {};
  QString text = property->getName() + ":" + property->getValue().toString() + "\n";
  for (int i = 0; i < property->numChildren(); ++i) text += propertyText(property->childAt(i));
  return text;
}
rviz_common::Display *display(rviz_common::VisualizationManager *manager, const QString &name) {
  auto *group = manager->getRootDisplayGroup();
  for (int i = 0; i < group->numDisplays(); ++i) if (group->getDisplayAt(i)->getName() == name) return group->getDisplayAt(i);
  return nullptr;
}
QImage renderedImage(rviz_common::RenderPanel *panel) {
  const QPoint position = panel->mapToGlobal(QPoint(0, 0));
  return panel->screen()->grabWindow(0, position.x(), position.y(), panel->width(), panel->height()).toImage();
}
int coloredPixels(const QImage &image) {
  int count = 0;
  for (int y = 0; y < image.height(); ++y) for (int x = 0; x < image.width(); ++x) {
    const QColor c = image.pixelColor(x, y);
    if (std::max({c.red(), c.green(), c.blue()}) - std::min({c.red(), c.green(), c.blue()}) > 60) ++count;
  }
  return count;
}
int brightMapPixels(const QImage &image) {
  int count = 0;
  for (int y = 0; y < image.height(); ++y) for (int x = 0; x < image.width(); ++x) {
    const QColor c = image.pixelColor(x, y);
    if (c.red() > 160 && std::abs(c.red() - c.green()) < 3 && std::abs(c.green() - c.blue()) < 3) ++count;
  }
  return count;
}
int redArrowPixels(const QImage &image) {
  int count = 0;
  for (int y = 0; y < image.height(); ++y) for (int x = 0; x < image.width(); ++x) {
    const QColor c = image.pixelColor(x, y);
    if (c.red() > 140 && c.red() > c.green() + 60 && c.red() > c.blue() + 60) ++count;
  }
  return count;
}
}

class UiIntegrationTest : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() { rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None); }
  void cleanupTestCase() { rclcpp::shutdown(); }
  void localNavigationInitialPoseAndRealPointCloudRendering() {
    using Navigate = nav2_msgs::action::NavigateToPose;
    using Handle = rclcpp_action::ServerGoalHandle<Navigate>;
    auto node = std::make_shared<rclcpp::Node>("desktop_isolated_fixture");
    std::vector<Navigate::Goal> goals;
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> initialPoses;
    std::vector<geometry_msgs::msg::PoseStamped> previews;
    std::vector<geometry_msgs::msg::Twist> commands; std::vector<bool> stops;
    std::shared_ptr<Handle> activeGoal; int cancellations = 0;
    auto action = rclcpp_action::create_server<Navigate>(node, "/navigate_to_pose",
      [](const auto &, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [&](auto) { ++cancellations; return rclcpp_action::CancelResponse::ACCEPT; },
      [&](std::shared_ptr<Handle> handle) { activeGoal = handle; goals.push_back(*handle->get_goal()); });
    auto initialFeedback = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
      [&](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) { initialPoses.push_back(*msg); });
    auto previewFeedback = node->create_subscription<geometry_msgs::msg::PoseStamped>("/desktop_ui/selected_goal", 10,
      [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { previews.push_back(*msg); });
    auto velocity = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_teleop", 10,
      [&](geometry_msgs::msg::Twist::ConstSharedPtr msg) { commands.push_back(*msg); });
    auto stop = node->create_subscription<std_msgs::msg::Bool>("/emergency_stop", 10,
      [&](std_msgs::msg::Bool::ConstSharedPtr msg) { stops.push_back(msg->data); });
    auto odom = node->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
    auto initial = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/desktop_ui/initial_pose_request", 10);
    auto map = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable().transient_local());
    auto global = node->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", rclcpp::QoS(1).reliable().transient_local());
    auto registered = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", rclcpp::SensorDataQoS());
    auto scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", rclcpp::SensorDataQoS());
    sensor_msgs::msg::PointCloud2 cloud; cloud.header.frame_id = "map";
    sensor_msgs::PointCloud2Modifier modifier(cloud); modifier.setPointCloud2FieldsByString(1, "xyz"); modifier.resize(441);
    sensor_msgs::PointCloud2Iterator<float> cx(cloud, "x"), cy(cloud, "y"), cz(cloud, "z");
    for (int y = -10; y <= 10; ++y) for (int x = -10; x <= 10; ++x, ++cx, ++cy, ++cz) {
      *cx = x * .35f; *cy = y * .35f; *cz = .5f + .02f * x;
    }
    cloud.header.stamp = node->now(); global->publish(cloud);
    nav_msgs::msg::OccupancyGrid grid; grid.header.frame_id = "map"; grid.header.stamp = node->now();
    grid.info.resolution = .1; grid.info.width = 100; grid.info.height = 100;
    grid.info.origin.position.x = -5.; grid.info.origin.position.y = -5.; grid.info.origin.orientation.w = 1.;
    grid.data.resize(10000);
    for (int y = 0; y < 100; ++y) for (int x = 0; x < 100; ++x) grid.data[y * 100 + x] = ((x / 10 + y / 10) % 2) ? 0 : 100;
    map->publish(grid); // Both retained maps precede the UI subscribers.
    bool localized = false;
    tf2_ros::TransformBroadcaster tf(node);
    QTimer publish;
    connect(&publish, &QTimer::timeout, this, [&] {
      rclcpp::spin_some(node);
      if (activeGoal && activeGoal->is_canceling()) { activeGoal->canceled(std::make_shared<Navigate::Result>()); activeGoal.reset(); }
      nav_msgs::msg::Odometry msg; msg.header.stamp = node->now(); msg.header.frame_id = "odom";
      msg.child_frame_id = "base_link"; msg.pose.pose.orientation.w = 1.; odom->publish(msg);
      cloud.header.stamp = node->now(); cloud.header.frame_id = "lio_odom"; registered->publish(cloud); scan->publish(cloud);
      if (localized) {
        geometry_msgs::msg::TransformStamped transform; transform.header.stamp = node->now();
        transform.header.frame_id = "map"; transform.child_frame_id = "lio_odom"; transform.transform.rotation.w = 1.; tf.sendTransform(transform);
        transform.header.frame_id = "lio_odom"; transform.child_frame_id = "base_link"; transform.transform.translation.x = 1.; tf.sendTransform(transform);
      }
    }); publish.start(50);
    const QString root = DESKTOP_UI_SOURCE_DIR;
    auto config = DesktopConfig::load(root + "/config/desktop.json", root, QDir(root).absoluteFilePath(".."));
    config.backendUrl = "http://127.0.0.1:9";
    MainWindow window(config, false); window.show(); window.activateWindow();
    QVERIFY(QTest::qWaitForWindowExposed(&window));
    auto *rviz = window.findChild<RvizHost *>(); QVERIFY(rviz);
    // Main-window map-editor entry must work even while vehicle control is not enabled.
    auto *editorAction = window.findChild<QAction *>(QStringLiteral("mapEditorAction"));
    auto *navigationYaml = window.findChild<QLineEdit *>(QStringLiteral("navigationYaml"));
    auto *mapPrefix = window.findChild<QLineEdit *>(QStringLiteral("mapPrefix"));
    QVERIFY(editorAction && navigationYaml && mapPrefix);
    QVERIFY(editorAction->isEnabled());
    QTemporaryDir editorMapDirectory;
    QVERIFY(editorMapDirectory.isValid());
    const QString editorImage = editorMapDirectory.filePath(QStringLiteral("map.pgm"));
    const QString editorYaml = editorMapDirectory.filePath(QStringLiteral("map.yaml"));
    {
      QFile image(editorImage);
      QVERIFY(image.open(QIODevice::WriteOnly));
      QVERIFY(image.write(QByteArray("P5\n4 3\n255\n") + QByteArray(12, char(205))) == 23);
      QFile yaml(editorYaml);
      QVERIFY(yaml.open(QIODevice::WriteOnly));
      QVERIFY(yaml.write("image: map.pgm\nmode: trinary\nresolution: 0.05\n"
                         "origin: [-1.25, 3.5, 0.0]\n"
                         "negate: 0\nfree_thresh: 0.25\noccupied_thresh: 0.65\n") > 0);
    }
    mapPrefix->setText(editorMapDirectory.filePath(QStringLiteral("map")));
    editorAction->trigger();
    QTRY_VERIFY(window.findChild<MapEditorDialog *>() != nullptr);
    auto *editorDialog = window.findChild<MapEditorDialog *>();
    QVERIFY(editorDialog && editorDialog->document());
    QCOMPARE(editorDialog->document()->width(), 4);
    QCOMPARE(editorDialog->document()->height(), 3);
    auto *profileSelector = window.findChild<QComboBox *>(QStringLiteral("profileSelector"));
    QVERIFY(profileSelector != nullptr);
    const int originalProfile = profileSelector->currentIndex();
    window.setProfileIndex(4); // Cartographer navigation: different localization source
    QCOMPARE(profileSelector->currentIndex(), 4);
    QVERIFY(navigationYaml->text().isEmpty());
    const QString mismatchTarget = editorMapDirectory.filePath(QStringLiteral("mismatch_edited.yaml"));
    QString saveError;
    QVERIFY2(editorDialog->saveToPath(mismatchTarget, &saveError), qPrintable(saveError));
    QVERIFY(navigationYaml->text().isEmpty()); // never fill a map from another backend
    window.setProfileIndex(originalProfile);
    QTRY_COMPARE(profileSelector->currentIndex(), originalProfile);
    editorDialog->close();
    QTRY_VERIFY(window.findChild<MapEditorDialog *>() == nullptr);
    auto *frame = rviz->findChild<rviz_common::VisualizationFrame *>(); QVERIFY(frame);
    auto *manager = frame->getManager(); auto *panel = manager->getRenderPanel();
    auto *globalDisplay = display(manager, "全局点云地图"); QVERIFY(globalDisplay && globalDisplay->isEnabled());
    QTRY_VERIFY_WITH_TIMEOUT(propertyText(globalDisplay).contains("441"), 10000);
    QTRY_VERIFY_WITH_TIMEOUT(propertyText(display(manager, "栅格地图")).contains("100"), 10000);
    QTRY_VERIFY(brightMapPixels(renderedImage(panel)) > 2000);
    QTRY_VERIFY(coloredPixels(renderedImage(panel)) > 200);
    QVERIFY(window.saveScreenshot("/tmp/robot_desktop_before_localization.png"));
    auto *arm = window.findChild<QCheckBox *>("enableControl"); auto *pose = window.findChild<QLabel *>("poseStatus");
    auto *send = window.findChild<QPushButton *>("sendGoal"); auto *relocate = window.findChild<QPushButton *>("setInitialPose");
    QVERIFY(arm && pose && send && relocate);
    QVERIFY(!pose->text().contains("x=")); QVERIFY(!send->isEnabled()); QVERIFY(!relocate->isEnabled());
    QCOMPARE(commands.size(), 0); QCOMPARE(stops.size(), 0); QCOMPARE(initialPoses.size(), 0);
    auto *arrow = display(manager, "所选目标箭头"); QVERIFY(arrow && arrow->isEnabled());
    QCOMPARE(arrow->subProp("Topic")->getValue().toString(), QString("/desktop_ui/selected_goal"));
    const int redBefore = redArrowPixels(renderedImage(panel));
    rviz->selectTool("rviz_default_plugins/SetGoal");
    const QPoint goalPosition = panel->rect().center() + QPoint(80, -20);
    QTest::mousePress(panel, Qt::LeftButton, Qt::NoModifier, goalPosition);
    QTest::mouseMove(panel, goalPosition + QPoint(40, 0), 50);
    QTest::mouseRelease(panel, Qt::LeftButton, Qt::NoModifier, goalPosition + QPoint(40, 0));
    QTRY_COMPARE(previews.size(), 1);
    QTRY_VERIFY(propertyText(arrow).contains("1 messages received"));
    QTest::qWait(200); // The RViz tool's temporary arrow is already gone.
    qInfo().noquote() << propertyText(arrow);
    qInfo() << "arrow pixels" << redBefore << redArrowPixels(renderedImage(panel));
    QVERIFY(window.saveScreenshot("/tmp/robot_desktop_selected_goal.png"));
    QTRY_VERIFY(redArrowPixels(renderedImage(panel)) > redBefore + 8);
    QCOMPARE(goals.size(), 0); QCOMPARE(commands.size(), 0); QCOMPARE(stops.size(), 0);
    QVERIFY(window.saveScreenshot("/tmp/robot_desktop_selected_goal.png"));
    arm->setChecked(true);
    auto *x = window.findChild<QDoubleSpinBox *>("goalX"); auto *y = window.findChild<QDoubleSpinBox *>("goalY"); auto *yaw = window.findChild<QDoubleSpinBox *>("goalYaw");
    QVERIFY(x && y && yaw); x->setValue(2.); y->setValue(-1.); yaw->setValue(45.);
    QTest::mouseClick(relocate, Qt::LeftButton); QTRY_COMPARE(initialPoses.size(), 1);
    QVERIFY(std::abs(initialPoses.back().pose.pose.orientation.z - std::sin(M_PI / 8.)) < 1e-9);
    window.setProfileIndex(2); // AMCL must accept initial pose before map TF exists.
    QVERIFY(window.findChild<QComboBox *>("profileSelector")->currentText().contains("AMCL"));
    QVERIFY(display(manager, "AMCL定位位姿"));
    QVERIFY(display(manager, "AMCL粒子云"));
    QCOMPARE(manager->getFixedFrame(), QString("map"));
    const auto framesAfterSwitch = rviz->renderedFrames();
    QTRY_VERIFY_WITH_TIMEOUT(rviz->renderedFrames() > framesAfterSwitch + 3, 3000);
    rviz->selectTool("rviz_default_plugins/SetInitialPose");
    const QPoint center = panel->rect().center();
    QTest::mousePress(panel, Qt::LeftButton, Qt::NoModifier, center);
    QTest::mouseMove(panel, center + QPoint(40, 0), 50);
    QTest::mouseRelease(panel, Qt::LeftButton, Qt::NoModifier, center + QPoint(40, 0));
    QTRY_COMPARE(initialPoses.size(), 2); // Actual RViz tool works without map->base_link.
    localized = true; QTRY_VERIFY_WITH_TIMEOUT(pose->text().contains("x="), 5000);
    auto *take = window.findChild<QPushButton *>("takeManual"); auto *unlock = window.findChild<QPushButton *>("releaseStop"); auto *forward = window.findChild<QPushButton *>("manualDirection1");
    QVERIFY(take && unlock && forward); QTRY_VERIFY(unlock->isEnabled());
    QTest::mouseClick(unlock, Qt::LeftButton); QTRY_VERIFY(!stops.empty() && !stops.back());
    QTest::qWait(250); QTest::mouseClick(send, Qt::LeftButton); QTRY_COMPARE(goals.size(), 1);
    QTRY_COMPARE(previews.size(), 2);
    QCOMPARE(previews.back().pose.position.x, 2.);
    QCOMPARE(goals.back().pose.header.frame_id, std::string("map")); QCOMPARE(goals.back().pose.pose.position.x, 2.);
    QVERIFY(std::abs(goals.back().pose.pose.orientation.z - std::sin(M_PI / 8.)) < 1e-9);
    QTest::mouseClick(take, Qt::LeftButton); QTRY_VERIFY(cancellations > 0);
    QTRY_VERIFY(take->text() == "手动已接管" && unlock->isEnabled());
    QVERIFY(!forward->isEnabled()); QTest::mouseClick(unlock, Qt::LeftButton); QTRY_VERIFY(forward->isEnabled());
    QTest::mousePress(forward, Qt::LeftButton); QTRY_VERIFY(!commands.empty() && commands.back().linear.x > 0.);
    QTest::mouseRelease(forward, Qt::LeftButton); QTRY_COMPARE(commands.back().linear.x, 0.);
    const int count = commands.size(); QTest::qWait(180); QCOMPARE(commands.size(), count);
    arm->setChecked(false);
    geometry_msgs::msg::PoseWithCovarianceStamped ignored; ignored.header.frame_id = "map"; ignored.pose.pose.orientation.w = 1.; initial->publish(ignored);
    QTest::qWait(200); QCOMPARE(initialPoses.size(), 2);
    QVERIFY(window.saveScreenshot("/tmp/robot_desktop_navigation_render.png"));
    window.setProfileIndex(1);
    auto *cloudDisplay = display(manager, "CloudRegistered"); QVERIFY(cloudDisplay && cloudDisplay->isEnabled());
    QTRY_VERIFY_WITH_TIMEOUT(propertyText(cloudDisplay).contains("441"), 10000);
    QTRY_VERIFY(coloredPixels(renderedImage(panel)) > 500);
    QVERIFY(window.saveScreenshot("/tmp/robot_desktop_mapping_render.png"));
    publish.stop();
  }
};
QTEST_MAIN(UiIntegrationTest)
#include "ui_integration_test.moc"
