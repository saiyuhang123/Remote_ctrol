#include "ui/mainwindow.h"
#include "adapters/ros_adapter.h"
#include "runtime/config.h"
#include "runtime/process_manager.h"
#include "DockWidget.h"
#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QLabel>
#include <QPushButton>
#include <QScreen>
#include <QSettings>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QTest>
#include <QToolButton>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#ifdef DESKTOP_HAS_PATROL_INTERFACES
#include <patrol_interfaces/msg/patrol_status.hpp>
#endif

class ConsoleLayoutTest : public QObject {
  Q_OBJECT
  QTemporaryDir settingsDirectory_;
  DesktopConfig config() const {
    const QString root = DESKTOP_UI_SOURCE_DIR;
    auto value = DesktopConfig::load(root + "/config/desktop.json", root, QDir(root).absoluteFilePath(".."));
    value.backendUrl = "http://127.0.0.1:9";
    return value;
  }
private slots:
  void initTestCase() {
    QVERIFY(settingsDirectory_.isValid());
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope,
                       qEnvironmentVariable("DESKTOP_CONSOLE_SETTINGS_DIR", settingsDirectory_.path()));
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  }
  void cleanupTestCase() { rclcpp::shutdown(); }
  void initialWindowFitsAvailableScreen() {
    MainWindow window(config(), true); window.show();
    QVERIFY(QTest::qWaitForWindowExposed(&window)); QTest::qWait(100);
    const auto available = window.screen()->availableGeometry();
    QVERIFY2(window.width() <= available.width(), qPrintable(QString("window %1 > screen %2").arg(window.width()).arg(available.width())));
    QVERIFY2(window.height() <= available.height(), qPrintable(QString("window %1 > screen %2").arg(window.height()).arg(available.height())));
    for (const auto &name : {"emergencyStop", "cancelNavigation", "enableControl"}) {
      auto *control = window.findChild<QWidget *>(name); QVERIFY(control && control->isVisible());
      QVERIFY(window.rect().contains(QRect(control->mapTo(&window, {}), control->size())));
    }
    auto *tabs = window.findChild<QTabWidget *>("controlWorkspaceTabs");
    auto *start = window.findChild<QPushButton *>("startSession"); QVERIFY(tabs && start);
    QVERIFY(start->isVisible()); QVERIFY(tabs->rect().contains(QRect(start->mapTo(tabs, {}), start->size())));
    auto *panel = window.findChild<rviz_common::RenderPanel *>(); QVERIFY(panel && panel->height() >= 140);
  }
  void odometrySourceFollowsModeAndPreservesNavigationChoice() {
    const auto settings = config();
    MainWindow window(settings, true); window.show();
    QVERIFY(QTest::qWaitForWindowExposed(&window));
    auto *source = window.findChild<QComboBox *>("odometrySource");
    auto *description = window.findChild<QLabel *>("odometrySourceHint");
    QVERIFY(source && description);
    window.findChild<QToolButton *>("runtimeAdvancedToggle")->setChecked(true);
    window.setProfileIndex(0);
    QVERIFY(source->isEnabled()); QCOMPARE(source->currentText(), QString("lio"));
    // Check both non-default choices survive every fixed mode and periodic telemetry refresh.
    for (const auto &choice : {QString("wheel"), QString("wheel_imu")}) {
      source->setCurrentText(choice);
      QVERIFY(description->text().contains(choice == "wheel" ? "转角" : "融合"));
      for (int i = 1; i < desktopProfiles().size(); ++i) {
        window.setProfileIndex(i);
        const auto &mode = desktopProfiles().at(i);
        QVERIFY(!source->isEnabled());
        QCOMPARE(source->currentText(), mode.backend == "fastlio" ? QString("lio") : QString("wheel_imu"));
        QVERIFY(description->text().contains("固定"));
        QVERIFY(description->isVisible());
        const auto arguments = ProcessManager::launchArguments(settings, mode, "/tmp/map", false, false, false, source->currentText());
        if (mode.backend == "fastlio") QVERIFY(arguments.contains("odom_source:=lio"));
        else for (const auto &argument : arguments) QVERIFY(!argument.startsWith("odom_source:="));
        window.setProfileIndex(0);
        QVERIFY(source->isEnabled()); QCOMPARE(source->currentText(), choice);
        window.findChild<RosAdapter *>()->snapshot(RobotSnapshot{});
        QCOMPARE(source->currentText(), choice);
        QVERIFY(ProcessManager::launchArguments(settings, desktopProfiles().at(0), "/tmp/map", false, false, false, source->currentText())
                    .contains("odom_source:=" + choice));
      }
    }
    // Exercise the UI running lock with an inert process, without launching ROS hardware.
    auto *process = window.findChild<ProcessManager *>(); QVERIFY(process);
    QVERIFY(process->start({"/usr/bin/sleep", "30"}));
    QTRY_VERIFY(!source->isEnabled()); QVERIFY(description->text().contains("停止后"));
    process->stop(); QTRY_VERIFY_WITH_TIMEOUT(!process->running(), 3000);
    QTRY_VERIFY(source->isEnabled()); QCOMPARE(source->currentText(), QString("wheel_imu"));
  }
  void responsiveLayout_data() {
    QTest::addColumn<QSize>("size");
    QTest::newRow("full-hd") << QSize(1920, 1080);
    QTest::newRow("laptop") << QSize(1366, 768);
    QTest::newRow("compact") << QSize(1100, 680);
  }
  void responsiveLayout() {
    QFETCH(QSize, size);
    MainWindow window(config(), true); window.show();
    QVERIFY(QTest::qWaitForWindowExposed(&window)); window.resize(size); QTest::qWait(150);
    QCOMPARE(window.size(), size);
    auto *left = window.findChild<QTabWidget *>("controlWorkspaceTabs");
    auto *right = window.findChild<QTabWidget *>("rightWorkspaceTabs");
    auto *monitor = window.findChild<ads::CDockWidget *>("consoleMonitorDock");
    auto *panel = window.findChild<rviz_common::RenderPanel *>();
    QVERIFY(left && right && monitor && panel); QCOMPARE(left->count(), 3); QCOMPARE(right->count(), 3);
    QCOMPARE(monitor->isClosed(), size.width() < 1180);
    if (size.width() >= 1180) {
      right->setCurrentIndex(1); QTest::qWait(30);
      for (const auto &name : {"visionStartFace", "visionStartDetector"}) {
        auto *control = window.findChild<QPushButton *>(name); QVERIFY(control && control->isVisible());
        QVERIFY2(right->rect().contains(QRect(control->mapTo(right, {}), control->size())), name);
      }
      right->setCurrentIndex(0);
    }
    qInfo() << "render area" << size << panel->size();
    if (size.width() == 1366) { QVERIFY(panel->width() >= 700); QVERIFY(panel->height() >= 440); }
    if (size.width() == 1920) QVERIFY(panel->width() >= size.width() * .65);
    for (const auto &name : {"emergencyStop", "cancelNavigation", "enableControl"}) {
      auto *control = window.findChild<QWidget *>(name); QVERIFY(control && control->isVisible());
      QVERIFY(window.rect().contains(QRect(control->mapTo(&window, {}), control->size())));
    }
    left->setCurrentIndex(2);
    auto *forward = window.findChild<QPushButton *>("manualDirection1"); QVERIFY(forward && forward->isVisible());
    QVERIFY(!forward->isEnabled());
    auto *reset = window.findChild<QAction *>("consoleResetLayout"); QVERIFY(reset);
    left->setCurrentIndex(0);
    auto *start = window.findChild<QPushButton *>("startSession"); QVERIFY(start && start->isVisible());
    QVERIFY(left->rect().contains(QRect(start->mapTo(left, {}), start->size())));
    auto *controlDock = window.findChild<ads::CDockWidget *>("consoleControlDock");
    controlDock->toggleView(false);
    window.findChild<QToolButton *>("consoleLogToggle")->setChecked(true);
    reset->trigger(); QTest::qWait(50);
    QVERIFY(!controlDock->isClosed()); QCOMPARE(left->currentIndex(), 0); QCOMPARE(right->currentIndex(), 0);
    QVERIFY(!window.findChild<QToolButton *>("consoleLogToggle")->isChecked());
    const auto captureDir = qEnvironmentVariable("DESKTOP_CONSOLE_CAPTURE_DIR");
    if (!captureDir.isEmpty()) {
      QDir().mkpath(captureDir);
      QVERIFY(window.saveScreenshot(captureDir + QString("/console_%1x%2.png").arg(size.width()).arg(size.height())));
      if (size.width() == 1366) {
        left->setCurrentIndex(2); QTest::qWait(80); QVERIFY(window.saveScreenshot(captureDir + "/console_manual.png"));
        right->setCurrentIndex(1); QTest::qWait(80); QVERIFY(window.saveScreenshot(captureDir + "/console_capture.png"));
      }
    }
  }
  void presentationKeepsUnknownReverseAndInterlockSeparate() {
    MainWindow window(config(), true); window.show(); QVERIFY(QTest::qWaitForWindowExposed(&window));
    auto *ros = window.findChild<RosAdapter *>(); QVERIFY(ros);
    auto *speed = window.findChild<QLabel *>("consoleSpeed"); auto *guard = window.findChild<QLabel *>("consoleGuardStatus");
    auto *pose = window.findChild<QLabel *>("poseStatus"); QVERIFY(speed && guard && pose);
    RobotSnapshot state; ros->snapshot(state);
    QVERIFY(speed->text().startsWith("—")); QVERIFY(!pose->text().contains("x="));
    state.speedValid = true; state.speed = -.15; state.yawRate = -.2;
    state.poseValid = true; state.frame = "map"; state.x = 1.2;
    state.guardValid = true; state.guardReady = true; state.patrolInterlockValid = true; state.patrolLocked = true;
    ros->snapshot(state);
    QVERIFY(speed->text().startsWith("-0.15")); QVERIFY(pose->text().contains("x=1.200"));
    QVERIFY(guard->text().contains("锁定")); QVERIFY(!guard->text().contains("就绪"));
    state.speedValid = false; state.poseValid = false; state.guardValid = false; ros->snapshot(state);
    QVERIFY(speed->text().startsWith("—")); QVERIFY(!pose->text().contains("x=")); QVERIFY(guard->text().contains("过期"));
    auto *message = window.findChild<QLabel *>("consoleMessageSummary"); QVERIFY(message);
    ros->requestFinished("capture", false, "模拟抓拍失败");
    ros->requestFinished("ptz_read", true, "模拟读取成功");
    QCOMPARE(message->text(), QString("模拟抓拍失败"));
    ros->requestFinished("capture", true, "模拟抓拍成功");
    QCOMPARE(message->text(), QString("模拟抓拍成功"));
    const auto captureDir = qEnvironmentVariable("DESKTOP_CONSOLE_CAPTURE_DIR");
    if (!captureDir.isEmpty()) {
      QDir().mkpath(captureDir);
      state.speedValid = true; state.speed = 0; state.poseValid = true;
      state.guardValid = true; state.guardReady = true; state.patrolLocked = true;
      ros->snapshot(state);
      window.findChild<QLabel *>("consoleSource")->setText("隔离测试 · 模拟遥测与联锁，未连接实车");
      window.repaint(); QCoreApplication::sendPostedEvents(nullptr, QEvent::UpdateRequest);
      QVERIFY(window.saveScreenshot(captureDir + "/console_interlock_simulated.png"));
    }
  }
  void patrolSummarySurvivesIdleNavigationUpdates() {
#ifdef DESKTOP_HAS_PATROL_INTERFACES
    MainWindow window(config(), true); window.show(); QVERIFY(QTest::qWaitForWindowExposed(&window));
    auto node = std::make_shared<rclcpp::Node>("console_patrol_status_fixture");
    auto publisher = node->create_publisher<patrol_interfaces::msg::PatrolStatus>("/patrol/status", rclcpp::QoS(1).reliable().transient_local());
    patrol_interfaces::msg::PatrolStatus status;
    status.instance_id = "console-test"; status.run_id = "simulated-run"; status.active = true;
    status.state = status.PAUSED; status.loop_index = 1; status.total_loops = 3; status.step_index = 2; status.step_count = 4;
    status.message = "隔离测试中的模拟任务，未发送导航目标";
    QTimer timer; connect(&timer, &QTimer::timeout, this, [&] { ++status.sequence; publisher->publish(status); }); timer.start(50);
    auto *label = window.findChild<QLabel *>("consoleNavigationStatus"); QVERIFY(label);
    QTRY_VERIFY_WITH_TIMEOUT(label->text().contains("已暂停"), 3000);
    RobotSnapshot state; state.navigationAvailable = true;
    auto *ros = window.findChild<RosAdapter *>(); ros->snapshot(state);
    QVERIFY(label->text().contains("已暂停")); QVERIFY(label->text().contains("1/3"));
    QVERIFY(label->text().contains("2/4"));
    const auto captureDir = qEnvironmentVariable("DESKTOP_CONSOLE_CAPTURE_DIR");
    if (!captureDir.isEmpty()) {
      QDir().mkpath(captureDir);
      window.findChild<QLabel *>("consoleSource")->setText("隔离测试 · 模拟暂停任务，未连接实车");
      window.repaint(); QCoreApplication::sendPostedEvents(nullptr, QEvent::UpdateRequest);
      QVERIFY(window.saveScreenshot(captureDir + "/console_patrol_simulated.png"));
    }
    timer.stop(); status.active = false; status.state = status.IDLE; publisher->publish(status);
#else
    QSKIP("patrol_interfaces not available");
#endif
  }
  void writesOnlyLayoutPreferences() {
    {
      MainWindow window(config(), false); window.show(); QVERIFY(QTest::qWaitForWindowExposed(&window)); QTest::qWait(100);
      window.resize(1366, 768);
      window.findChild<QTabWidget *>("controlWorkspaceTabs")->setCurrentIndex(1);
      window.findChild<QTabWidget *>("rightWorkspaceTabs")->setCurrentIndex(1);
      window.findChild<QCheckBox *>("enableControl")->setChecked(true);
      window.findChild<QCheckBox *>("enablePatrolInterlock")->setChecked(true);
    }
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, "RemoteCtrol", "desktop_console_v1");
    QCOMPARE(settings.value("leftTab").toInt(), 1); QCOMPARE(settings.value("rightTab").toInt(), 1);
    QVERIFY(!settings.contains("enableControl")); QVERIFY(!settings.contains("enablePatrolInterlock"));
    QVERIFY(!settings.contains("manageSession"));
  }
  void restoresLayoutWithoutAuthorization() {
    MainWindow restored(config(), false); restored.show();
    QVERIFY(QTest::qWaitForWindowExposed(&restored)); QTest::qWait(100);
    QCOMPARE(restored.findChild<QTabWidget *>("controlWorkspaceTabs")->currentIndex(), 1);
    QCOMPARE(restored.findChild<QTabWidget *>("rightWorkspaceTabs")->currentIndex(), 1);
    QVERIFY(!restored.findChild<QCheckBox *>("enableControl")->isChecked());
    QVERIFY(!restored.findChild<QCheckBox *>("manageSession")->isChecked());
    QVERIFY(!restored.findChild<QCheckBox *>("enablePatrolInterlock")->isChecked());
    QVERIFY(!restored.findChild<QPushButton *>("manualDirection1")->isEnabled());
  }
  void corruptedLayoutFallsBackToAccessiblePanels() {
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, "RemoteCtrol", "desktop_console_v1");
    settings.setValue("version", 1); settings.setValue("docks", QByteArray("invalid dock state"));
    settings.setValue("geometry", QByteArray("invalid geometry")); settings.sync();
    MainWindow window(config(), false); window.show(); QVERIFY(QTest::qWaitForWindowExposed(&window)); QTest::qWait(100);
    QVERIFY(!window.findChild<ads::CDockWidget *>("consoleControlDock")->isClosed());
    QVERIFY(window.findChild<QPushButton *>("emergencyStop")->isVisible());
    QVERIFY(!window.findChild<QCheckBox *>("enableControl")->isChecked());
  }
};

QTEST_MAIN(ConsoleLayoutTest)
#include "console_layout_test.moc"
