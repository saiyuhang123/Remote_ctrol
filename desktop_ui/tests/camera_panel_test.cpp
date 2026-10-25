#include "ui/camera_panel.h"
#include "adapters/ros_adapter.h"
#include <QDir>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSignalSpy>
#include <QTest>
#include <QToolButton>
#include <QTabWidget>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <hk_camera/srv/get_ptz_pose.hpp>
#include <hk_camera/srv/set_ptz_pose.hpp>
#include <hk_camera/srv/control_ptz.hpp>
#include <atomic>
#include <thread>

struct CameraFixture {
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("camera_panel_fixture");
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread worker;
  std::atomic_int login{0}, start{0}, stop{0}, moves{0}, poseReads{0};
  std::atomic_int jogs{0}, jogStops{0}, direction{0};
  std::atomic_bool streaming{false};
  std::atomic_bool poseReadFails{false};
  double pan = 45, tilt = 10, zoom = 2;
  bool absolutePanTilt;
  bool signedCoordinates;
  std::vector<rclcpp::ServiceBase::SharedPtr> services;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image;
  rclcpp::TimerBase::SharedPtr timer;
  explicit CameraFixture(bool absolute = true, bool signedAngles = false)
      : absolutePanTilt(absolute), signedCoordinates(signedAngles) {
    if (!absolute) { pan = 0; tilt = 0; }
    if (signedAngles) tilt = -33.6;
    auto trigger = [this](const char *command, auto callback) {
      services.push_back(node->create_service<std_srvs::srv::Trigger>(std::string("/hk_camera/") + command,
        [callback](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r) {
          callback(); r->success = true;
        }));
    };
    trigger("login", [this] { ++login; });
    trigger("start_stream", [this] { ++start; streaming = true; });
    trigger("stop_stream", [this] { ++stop; streaming = false; });
    services.push_back(node->create_service<hk_camera::srv::GetPTZPose>("/hk_camera/get_ptz_pose",
      [this](hk_camera::srv::GetPTZPose::Request::SharedPtr, hk_camera::srv::GetPTZPose::Response::SharedPtr r) {
        ++poseReads;
        if (poseReadFails) { r->success = false; r->message = "485 read failed"; return; }
        r->success = true; r->pan = pan; r->tilt = tilt; r->zoom = zoom;
        r->limits_valid = true; r->pan_min = 0; r->pan_max = absolutePanTilt ? 360 : 0;
        r->tilt_min = signedCoordinates ? -90 : 0; r->tilt_max = signedCoordinates ? 25 : absolutePanTilt ? 90 : 0;
        r->zoom_min = 1; r->zoom_max = signedCoordinates ? 23 : absolutePanTilt ? 30 : 23;
      }));
    services.push_back(node->create_service<hk_camera::srv::SetPTZPose>("/hk_camera/set_ptz_pose",
      [this](hk_camera::srv::SetPTZPose::Request::SharedPtr q, hk_camera::srv::SetPTZPose::Response::SharedPtr r) {
        pan = q->pan; tilt = q->tilt; zoom = q->zoom; ++moves; r->success = true;
      }));
    services.push_back(node->create_service<hk_camera::srv::ControlPTZ>("/hk_camera/control_ptz",
      [this](hk_camera::srv::ControlPTZ::Request::SharedPtr q, hk_camera::srv::ControlPTZ::Response::SharedPtr r) {
        if (!q->stop && (q->speed < 1 || q->speed > 7)) {
          r->message = "Invalid SDK speed"; return;
        }
        if (q->stop) { ++jogStops; direction = 0; }
        else { ++jogs; direction = q->command; }
        r->success = true;
      }));
    image = node->create_publisher<sensor_msgs::msg::Image>("/hk_camera/image_raw", rclcpp::SensorDataQoS());
    timer = node->create_wall_timer(std::chrono::milliseconds(50), [this] {
      if (!streaming) return;
      sensor_msgs::msg::Image m; m.width = 160; m.height = 90; m.step = 480; m.encoding = "bgr8";
      m.data.resize(m.step * m.height);
      for (size_t i = 0; i < m.data.size(); i += 3) { m.data[i] = 0; m.data[i+1] = 0; m.data[i+2] = 255; }
      image->publish(m);
    });
    executor.add_node(node); worker = std::thread([this] { executor.spin(); });
  }
  ~CameraFixture() { executor.cancel(); if (worker.joinable()) worker.join(); }
};

class CameraPanelTest : public QObject {
  Q_OBJECT
  DesktopConfig config() {
    const auto project = QDir(DESKTOP_UI_SOURCE_DIR).absoluteFilePath("..");
    return DesktopConfig::load(QString(DESKTOP_UI_SOURCE_DIR) + "/config/desktop.json", DESKTOP_UI_SOURCE_DIR, project);
  }
private slots:
  void initTestCase() { rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None); }
  void cleanupTestCase() { rclcpp::shutdown(); }
  void previewAndPtzAreIndependentOfNavigation() {
    CameraFixture fixture; RosAdapter ros(config()); CameraPanel panel(config(), &ros, false);
    panel.resize(430, 680); panel.show();
    QTest::mouseClick(panel.findChild<QToolButton *>("cameraDetailsToggle"), Qt::LeftButton);
    auto button = [&](const char *name) { return panel.findChild<QPushButton *>(name); };
    QTRY_VERIFY_WITH_TIMEOUT(button("cameraConnect")->isEnabled(), 5000);
    QVERIFY(!button("cameraStart")->isEnabled()); QVERIFY(!button("cameraStopNode")->isEnabled());
    QVERIFY(!button("cameraSetPtz")->isEnabled());
    QTest::mouseClick(button("cameraConnect"), Qt::LeftButton);
    QTRY_COMPARE(fixture.login.load(), 1); QTRY_COMPARE(fixture.start.load(), 1);
    auto *preview = panel.findChild<QLabel *>("cameraPreview");
    QTRY_VERIFY_WITH_TIMEOUT(!preview->pixmap(Qt::ReturnByValue).isNull(), 5000);
    const auto frame = preview->pixmap(Qt::ReturnByValue).toImage();
    QCOMPARE(frame.pixelColor(frame.width()/2, frame.height()/2), QColor(Qt::red));
    QVERIFY(button("cameraSave")->isEnabled());
    panel.setControlEnabled(true); QVERIFY(!button("cameraSetPtz")->isEnabled());
    QTest::mouseClick(button("cameraReadPtz"), Qt::LeftButton);
    QTRY_VERIFY(button("cameraSetPtz")->isEnabled());
    auto *pan = panel.findChild<QDoubleSpinBox *>("cameraPan");
    auto *tilt = panel.findChild<QDoubleSpinBox *>("cameraTilt");
    auto *zoom = panel.findChild<QDoubleSpinBox *>("cameraZoom");
    QCOMPARE(pan->value(), 45.); QCOMPARE(tilt->value(), 10.); QCOMPARE(zoom->value(), 2.);
    QCOMPARE(tilt->maximum(), 90.); QCOMPARE(zoom->maximum(), 30.);
    pan->setValue(100); tilt->setValue(25); zoom->setValue(3);
    QTest::mouseClick(button("cameraSetPtz"), Qt::LeftButton);
    QTRY_COMPARE(fixture.moves.load(), 1);
    QTRY_VERIFY(button("cameraReadPtz")->isEnabled());
    QTest::mouseClick(button("cameraReadPtz"), Qt::LeftButton);
    QTRY_VERIFY(panel.findChild<QLabel *>("cameraPoseStatus")->text().contains("100.0"));
    auto *left = button("cameraDirection23");
    panel.setPatrolBlocked(true);
    QVERIFY(!left->isEnabled()); QVERIFY(!button("cameraSetPtz")->isEnabled());
    QSignalSpy patrolStop(&panel, &CameraPanel::patrolStopRequested);
    QTest::mouseClick(button("cameraDirection0"), Qt::LeftButton); QCOMPARE(patrolStop.count(), 1);
    panel.setPatrolBlocked(false); QVERIFY(left->isEnabled());
    QTest::mousePress(left, Qt::LeftButton);
    QTRY_VERIFY(fixture.jogs.load() >= 3); QCOMPARE(fixture.direction.load(), 23);
    QTest::mouseRelease(left, Qt::LeftButton);
    QTRY_VERIFY(fixture.jogStops.load() >= 1); QTRY_COMPARE(fixture.direction.load(), 0);
    const int releasedCount = fixture.jogs.load(); QTest::qWait(350); QCOMPARE(fixture.jogs.load(), releasedCount);
    QTest::mousePress(left, Qt::LeftButton); QTRY_COMPARE(fixture.direction.load(), 23);
    panel.setControlEnabled(false); QTRY_COMPARE(fixture.direction.load(), 0);
    QTest::mouseRelease(left, Qt::LeftButton);
    panel.setControlEnabled(false); QVERIFY(!button("cameraSetPtz")->isEnabled());
    QTest::mouseClick(button("cameraStopStream"), Qt::LeftButton); QTRY_COMPARE(fixture.stop.load(), 1);
    QTRY_VERIFY(preview->pixmap(Qt::ReturnByValue).isNull());
    QTRY_VERIFY(!button("cameraSave")->isEnabled());
    QTest::mouseClick(button("cameraConnect"), Qt::LeftButton); QTRY_COMPARE(fixture.start.load(), 2);
    QTRY_VERIFY(!preview->pixmap(Qt::ReturnByValue).isNull());
    panel.setControlEnabled(true);
    QTest::mousePress(left, Qt::LeftButton); QTRY_COMPARE(fixture.direction.load(), 23);
    QEvent deactivate(QEvent::WindowDeactivate); QApplication::sendEvent(&panel, &deactivate);
    QTRY_COMPARE(fixture.direction.load(), 0); QTest::mouseRelease(left, Qt::LeftButton);
    QTest::mousePress(left, Qt::LeftButton); QTRY_COMPARE(fixture.direction.load(), 23);
    fixture.streaming = false;
    QTRY_VERIFY_WITH_TIMEOUT(preview->text().contains("过期"), 5000);
    QTRY_COMPARE(fixture.direction.load(), 0); QTest::mouseRelease(left, Qt::LeftButton);
    QVERIFY(!button("cameraSave")->isEnabled());
  }
  void zoomOnlyDeviceDoesNotOfferAbsolutePanTilt() {
    CameraFixture fixture(false); RosAdapter ros(config()); CameraPanel panel(config(), &ros, false);
    panel.setControlEnabled(true); panel.show();
    QTest::mouseClick(panel.findChild<QToolButton *>("cameraDetailsToggle"), Qt::LeftButton);
    auto *read = panel.findChild<QPushButton *>("cameraReadPtz");
    QTRY_VERIFY_WITH_TIMEOUT(read->isEnabled(), 5000); QTest::mouseClick(read, Qt::LeftButton);
    QTRY_VERIFY(!panel.findChild<QDoubleSpinBox *>("cameraPan")->isEnabled());
    QVERIFY(!panel.findChild<QDoubleSpinBox *>("cameraTilt")->isEnabled());
    QCOMPARE(panel.findChild<QDoubleSpinBox *>("cameraZoom")->maximum(), 23.);
    QVERIFY(panel.findChild<QPushButton *>("cameraSetPtz")->text().contains("光学变倍"));
    QVERIFY(panel.findChild<QLabel *>("cameraPoseStatus")->text().contains("角度反馈不可用"));
  }
  void signedTiltAndMaximumOpticalZoomRoundTripAndAutoReadBack() {
    CameraFixture fixture(true, true); RosAdapter ros(config()); CameraPanel panel(config(), &ros, false);
    panel.setControlEnabled(true); panel.show();
    QTest::mouseClick(panel.findChild<QToolButton *>("cameraDetailsToggle"), Qt::LeftButton);
    auto *read = panel.findChild<QPushButton *>("cameraReadPtz");
    auto *set = panel.findChild<QPushButton *>("cameraSetPtz");
    auto *pan = panel.findChild<QDoubleSpinBox *>("cameraPan");
    auto *tilt = panel.findChild<QDoubleSpinBox *>("cameraTilt");
    auto *zoom = panel.findChild<QDoubleSpinBox *>("cameraZoom");
    QTRY_VERIFY(read->isEnabled()); QTest::mouseClick(read, Qt::LeftButton);
    QTRY_VERIFY(set->isEnabled());
    QCOMPARE(tilt->minimum(), -90.); QCOMPARE(tilt->maximum(), 25.); QCOMPARE(tilt->value(), -33.6);
    QCOMPARE(zoom->maximum(), 23.);
    pan->setValue(125.7); tilt->setValue(-45.); zoom->setValue(23.);
    QTest::mouseClick(set, Qt::LeftButton);
    QTRY_COMPARE(fixture.moves.load(), 1);
    QCOMPARE(fixture.tilt, -45.); QCOMPARE(fixture.zoom, 23.);
    QTRY_VERIFY_WITH_TIMEOUT(fixture.poseReads.load() >= 2, 4000);
    QTRY_VERIFY(set->isEnabled());
    QCOMPARE(tilt->value(), -45.); QCOMPARE(zoom->value(), 23.);
    fixture.poseReadFails = true;
    QTest::mouseClick(read, Qt::LeftButton);
    QTRY_VERIFY(panel.findChild<QLabel *>("cameraPoseStatus")->text().contains("读取失败"));
    QVERIFY(!set->isEnabled());
  }
  void offlineCannotStartOrControlCamera() {
    CameraFixture fixture; RosAdapter ros(config()); CameraPanel panel(config(), &ros, true);
    panel.setControlEnabled(true); QTest::qWait(500);
    for (const char *name : {"cameraStart", "cameraConnect", "cameraStopStream", "cameraStopNode", "cameraReadPtz", "cameraSetPtz", "cameraSave"})
      QVERIFY(!panel.findChild<QPushButton *>(name)->isEnabled());
    QCOMPARE(fixture.login.load(), 0); QCOMPARE(fixture.moves.load(), 0);
  }
  void hiddenCameraPageStopsHeldJogAndDoesNotResume() {
    CameraFixture fixture; RosAdapter ros(config()); QTabWidget workspace;
    CameraPanel panel(config(), &ros, false); workspace.addTab(&panel, "相机"); workspace.addTab(new QWidget, "其他");
    workspace.resize(430, 750); workspace.show(); panel.setControlEnabled(true);
    auto *connect = panel.findChild<QPushButton *>("cameraConnect");
    QTRY_VERIFY(connect->isEnabled()); QTest::mouseClick(connect, Qt::LeftButton);
    auto *left = panel.findChild<QPushButton *>("cameraDirection23"); QTRY_VERIFY(left->isEnabled());
    QTest::mousePress(left, Qt::LeftButton); QTRY_COMPARE(fixture.direction.load(), 23);
    workspace.setCurrentIndex(1); QTRY_COMPARE(fixture.direction.load(), 0);
    QTest::mouseRelease(left, Qt::LeftButton);
    const int stopped = fixture.jogs.load(); QTest::qWait(350); QCOMPARE(fixture.jogs.load(), stopped);
    workspace.setCurrentIndex(0); QTest::qWait(200); QCOMPARE(fixture.direction.load(), 0);
    QTest::mousePress(left, Qt::LeftButton); QTRY_COMPARE(fixture.direction.load(), 23);
    workspace.hide(); QTRY_COMPARE(fixture.direction.load(), 0);
    QTest::mouseRelease(left, Qt::LeftButton);
  }
  void liveCameraPreview() {
    if (!qEnvironmentVariableIsSet("DESKTOP_CAMERA_LIVE_TEST")) QSKIP("Explicit opt-in required for physical camera read-only preview");
    RosAdapter ros(config()); CameraPanel panel(config(), &ros, false);
    panel.resize(720, 850); panel.show();
    QTest::mouseClick(panel.findChild<QToolButton *>("cameraDetailsToggle"), Qt::LeftButton);
    QSignalSpy frames(&ros, &RosAdapter::image), poses(&ros, &RosAdapter::cameraPose);
    auto *start = panel.findChild<QPushButton *>("cameraStart");
    auto *connect = panel.findChild<QPushButton *>("cameraConnect");
    if (start->isEnabled()) QTest::mouseClick(start, Qt::LeftButton);
    else { QTRY_VERIFY_WITH_TIMEOUT(connect->isEnabled(), 5000); QTest::mouseClick(connect, Qt::LeftButton); }
    QTRY_VERIFY_WITH_TIMEOUT(frames.size() >= 5, 30000);
    const auto image = qvariant_cast<QImage>(frames.last().at(0));
    qInfo() << "Live camera frame:" << image.size();
    QVERIFY(image.width() >= 640); QVERIFY(image.height() >= 360);
    auto *read = panel.findChild<QPushButton *>("cameraReadPtz");
    QTRY_VERIFY(read->isEnabled()); QTest::mouseClick(read, Qt::LeftButton);
    QTRY_VERIFY_WITH_TIMEOUT(!poses.isEmpty(), 20000);
    const auto pose = qvariant_cast<CameraPose>(poses.last().at(0));
    qInfo() << "Actual PTZ:" << pose.pan << pose.tilt << pose.zoom << "Limits:" << pose.limitsValid
            << pose.panMin << pose.panMax << pose.tiltMin << pose.tiltMax << pose.zoomMin << pose.zoomMax;
    QVERIFY(!panel.findChild<QPushButton *>("cameraSetPtz")->isEnabled());
    QVERIFY(panel.grab().save("/tmp/robot_desktop_camera_live.png"));
    auto *stop = panel.findChild<QPushButton *>("cameraStopNode");
    if (stop->isEnabled()) { QTest::mouseClick(stop, Qt::LeftButton); QTRY_VERIFY_WITH_TIMEOUT(!stop->isEnabled(), 15000); }
  }
};
QTEST_MAIN(CameraPanelTest)
#include "camera_panel_test.moc"
