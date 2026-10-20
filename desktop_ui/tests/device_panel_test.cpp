#include "devices/device_client.h"
#include "devices/device_panel.h"
#include "devices/gas_readout.h"
#include "ui/camera_panel.h"
#include "adapters/ros_adapter.h"
#include <QCheckBox>
#include <QJsonDocument>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
#include <QToolButton>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <atomic>
#include <thread>

struct DeviceFixture {
  rclcpp::Node::SharedPtr gas = std::make_shared<rclcpp::Node>("gas_detector");
  rclcpp::Node::SharedPtr ptz = std::make_shared<rclcpp::Node>("ptz_patrol");
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread worker;
  std::atomic_bool gasEnabled{false}, holdStop{false}, everStarted{false};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gasStatus, gasData, ptzStatus;
  QTimer timer;
  bool publishPtz = true;
  DeviceFixture() {
    ptz->declare_parameter("enabled", false); ptz->declare_parameter("pan_speed", 2);
    ptz->declare_parameter("sweep_seconds", 20.); ptz->declare_parameter("pan_min", -1.); ptz->declare_parameter("pan_max", -1.);
    service = gas->create_service<std_srvs::srv::SetBool>("/gas_detector/set_enabled",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        gasEnabled = req->data; res->success = true; res->message = "isolated gas fixture";
      });
    gasStatus = gas->create_publisher<std_msgs::msg::String>("/gas_detector/status", 10);
    gasData = gas->create_publisher<std_msgs::msg::String>("/gas_detector/data", 10);
    ptzStatus = ptz->create_publisher<std_msgs::msg::String>("/ptz_patrol/status", 10);
    executor.add_node(gas); executor.add_node(ptz); worker = std::thread([this] { executor.spin(); });
    QObject::connect(&timer, &QTimer::timeout, [&] {
      auto send = [](const auto &pub, const QJsonObject &data) {
        std_msgs::msg::String msg; msg.data = QJsonDocument(data).toJson(QJsonDocument::Compact).toStdString(); pub->publish(msg);
      };
      send(gasStatus, {{"component", "gas"}, {"enabled", gasEnabled.load()}, {"connected", gasEnabled.load()},
        {"state", gasEnabled ? "sampling" : "stopped"}, {"interval", 2.0}, {"port", "/fake/gas"}, {"address", 1}, {"baudrate", 2400}});
      if (gasEnabled) send(gasData, {{"raw_value", 150}, {"value_scaled", 15.0}, {"unit", "ppm"}, {"state_code", 2}});
      const bool enabled = ptz->get_parameter("enabled").as_bool();
      if (enabled) everStarted = true;
      const bool active = enabled || (everStarted && holdStop);
      if (publishPtz) send(ptzStatus, {{"component", "ptz"}, {"enabled", enabled}, {"active", active}, {"moving", active},
        {"status_time_ns", static_cast<double>(ptz->now().nanoseconds())},
        {"state", enabled ? "sweeping" : active ? "stopping" : "stopped"},
        {"pan_speed", static_cast<int>(ptz->get_parameter("pan_speed").as_int())}, {"sweep_seconds", ptz->get_parameter("sweep_seconds").as_double()}});
    });
    timer.start(100);
  }
  ~DeviceFixture() { timer.stop(); executor.cancel(); worker.join(); }
};

class DevicePanelTest : public QObject {
  Q_OBJECT
  QTemporaryDir project_;
  DesktopConfig config() const {
    return DesktopConfig::load(QString(DESKTOP_UI_SOURCE_DIR) + "/config/desktop.json", DESKTOP_UI_SOURCE_DIR, project_.path());
  }
private slots:
  void initTestCase() { rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None); }
  void cleanupTestCase() { rclcpp::shutdown(); }
  void independentGasAndPtzNeedNoNavigationAndRetainStopInterlock() {
    DeviceFixture fixture;
    DeviceClient client(config(), false); DevicePanel panel(&client, false); panel.show();
    QSignalSpy camera(&client, &DeviceClient::cameraRequested);
    auto button = [&](const char *name) { return panel.findChild<QPushButton *>(name); };
    QTRY_VERIFY(panel.findChild<QLabel *>("ptzPatrolStatus")->text().contains("已停止"));
    QVERIFY(!button("ptzPatrolStart")->isEnabled()); // Camera motion retains the existing authorization switch.
    QVERIFY(button("gasStart")->isEnabled());
    QTest::mouseClick(button("gasStart"), Qt::LeftButton);
    QTRY_VERIFY(fixture.gasEnabled.load());
    QTRY_COMPARE(panel.findChild<QLabel *>("gasAlarm")->text(), QString("高报"));
    panel.hide(); QTest::qWait(200); QVERIFY(fixture.gasEnabled.load()); panel.show();
    QTest::mouseClick(button("gasStop"), Qt::LeftButton); QTRY_VERIFY(!fixture.gasEnabled.load());
    QTRY_VERIFY(panel.findChild<QLabel *>("gasFreshness")->text().contains("无新鲜采样"));
    panel.findChild<QTabWidget *>("deviceTabs")->setCurrentIndex(1);
    client.setControlEnabled(true); panel.setControlEnabled(true);
    QTRY_VERIFY(button("ptzPatrolStart")->isEnabled());
    QTest::mouseClick(button("ptzPatrolStart"), Qt::LeftButton);
    QTRY_VERIFY(fixture.ptz->get_parameter("enabled").as_bool()); QCOMPARE(camera.count(), 1);
    QTRY_VERIFY(fixture.everStarted.load());
    fixture.holdStop = true;
    QTest::mouseClick(button("ptzPatrolStop"), Qt::LeftButton);
    QTRY_VERIFY(!fixture.ptz->get_parameter("enabled").as_bool());
    QTRY_VERIFY(panel.findChild<QLabel *>("ptzPatrolStatus")->text().contains("正在停止"));
    QVERIFY(!button("ptzPatrolStart")->isEnabled());
    fixture.holdStop = false; QTRY_VERIFY(button("ptzPatrolStart")->isEnabled());
    QTest::mouseClick(button("ptzPatrolStart"), Qt::LeftButton);
    QTRY_VERIFY(fixture.ptz->get_parameter("enabled").as_bool());
    fixture.publishPtz = false; QTest::qWait(2300);
    QVERIFY(!button("ptzPatrolStart")->isEnabled());
    QVERIFY(panel.findChild<QLabel *>("ptzPatrolStatus")->text().contains("新鲜"));
    // Revoking authorization stops this UI's sweep, even while the panel is hidden.
    panel.hide(); client.setControlEnabled(false); panel.setControlEnabled(false);
    QTRY_VERIFY(!fixture.ptz->get_parameter("enabled").as_bool());
    fixture.publishPtz = true;
  }
  void offlineCannotStartDevices() {
    DeviceClient client(config(), true); DevicePanel panel(&client, true);
    client.setControlEnabled(true); panel.setControlEnabled(true);
    QVERIFY(!panel.findChild<QPushButton *>("gasStart")->isEnabled());
    QVERIFY(!panel.findChild<QPushButton *>("ptzPatrolStart")->isEnabled());
  }
  void layoutKeepsActionsVisibleAndTelemetryClearsOnStop() {
    DeviceClient client(config(), true); // Display injection only; no real device commands.
    DevicePanel panel(&client, false);
    panel.setControlEnabled(true);
    QSignalSpy camera(&client, &DeviceClient::cameraRequested);
    panel.resize(640, 760); panel.show();
    QVERIFY(QTest::qWaitForWindowExposed(&panel));
    QJsonObject gas{{"enabled",true},{"connected",true},{"state","sampling"},{"interval",2.0},
      {"port","/fake/gas"},{"address",1},{"baudrate",2400}};
    client.status("gas", gas);
    client.gasSample({{"raw_value",150},{"value_scaled",15.0},{"decimals",1},{"unit","ppm"},
      {"state_code",2},{"range_raw",1000},{"low_alarm_raw",30},{"high_alarm_raw",60}});
    QJsonObject ptz{{"state","stopped"},{"active",false},{"moving",false},{"pan_speed",2},{"sweep_seconds",20.0}};
    client.status("ptz", ptz);
    QCOMPARE(panel.findChild<QLabel *>("gasValue")->text(), QString("150"));
    QVERIFY(panel.findChild<QLabel *>("gasEstimate")->text().contains("≈"));
    QVERIFY(panel.findChild<QLabel *>("deviceGasSummary")->text().contains("150"));
    auto *tabs=panel.findChild<QTabWidget *>("deviceTabs");
    QVERIFY(tabs && tabs->count()==2);
    for(const auto size:{QSize(480,640),QSize(580,680),QSize(640,760),QSize(900,800)}) {
      panel.resize(size);QTest::qWait(30);
      QCOMPARE(panel.size(),size);
      for(int tab=0;tab<2;++tab) {
        tabs->setCurrentIndex(tab);QTest::qWait(20);
        QCOMPARE(panel.size(),size);
        const QStringList names=tab==0 ? QStringList{"gasStart","gasStop"}
                                       : QStringList{"ptzPatrolStart","ptzPatrolStop"};
        for(const auto &name:names) {
          auto *button=panel.findChild<QPushButton *>(name);QVERIFY(button && button->isVisible());
          QVERIFY(panel.rect().contains(QRect(button->mapTo(&panel,QPoint()),button->size())));
          QVERIFY(tabs->widget(tab)->rect().contains(QRect(button->mapTo(tabs->widget(tab),QPoint()),button->size())));
        }
        QVERIFY(panel.grab().save(QString("/tmp/device_duty_tab%1_%2.png").arg(tab).arg(size.width())));
      }
    }
    panel.resize(640,760);tabs->setCurrentIndex(0);QTest::qWait(30);
    auto *settings=panel.findChild<QWidget *>("gasSettings");
    auto *toggle=panel.findChild<QPushButton *>("gasSettingsToggle");
    QVERIFY(settings && settings->isHidden());
    toggle->click();QVERIFY(settings->isVisible());
    QVERIFY(!panel.findChild<QLineEdit *>("gasPort")->isEnabled()); // Running device keeps configuration locked.
    QTest::qWait(30);QVERIFY(panel.grab().save("/tmp/device_duty_gas_settings.png"));
    toggle->click();QTest::qWait(30);
    tabs->setCurrentIndex(1);
    auto *bounds=panel.findChild<QCheckBox *>("ptzBounds");
    bounds->setChecked(true);
    auto *minimum=panel.findChild<QDoubleSpinBox *>("ptzPanMinimum");
    auto *maximum=panel.findChild<QDoubleSpinBox *>("ptzPanMaximum");
    minimum->setValue(200);maximum->setValue(100);
    QVERIFY(!panel.findChild<QPushButton *>("ptzPatrolStart")->isEnabled());
    QVERIFY(panel.findChild<QLabel *>("ptzReadiness")->text().contains("最小水平角"));
    minimum->setValue(0);maximum->setValue(180);
    QVERIFY(panel.findChild<QPushButton *>("ptzPatrolStart")->isEnabled());
    QTest::qWait(30);QVERIFY(panel.grab().save("/tmp/device_duty_ptz_bounds.png"));
    client.ptzBlocked(true);ptz["state"]="sweeping";ptz["moving"]=true;ptz["active"]=true;
    client.status("ptz",ptz);
    QVERIFY(!panel.findChild<QPushButton *>("ptzPatrolStart")->isEnabled());
    QVERIFY(panel.findChild<QPushButton *>("ptzPatrolStop")->isEnabled());
    bounds->setChecked(false);QTest::qWait(30);
    QVERIFY(panel.grab().save("/tmp/device_duty_active.png"));
    QCOMPARE(camera.count(),0); // Viewing tabs/settings never starts a camera or sweep.
    gas["enabled"]=false;gas["connected"]=false;client.status("gas",gas);
    QCOMPARE(panel.findChild<QLabel *>("gasValue")->text(),QString("—"));
    QVERIFY(!panel.findChild<QLabel *>("gasEstimate")->text().contains("15.0"));
    gas["enabled"]=true;gas["connected"]=true;client.status("gas",gas);
    QCOMPARE(panel.findChild<QLabel *>("gasValue")->text(),QString("—"));
    client.gasSample({{"raw_value",20},{"unit","%LEL"},{"state_code",0}});
    QCOMPARE(panel.findChild<QLabel *>("gasValue")->text(),QString("20"));
    gas["last_error"]="isolated sensor error";client.status("gas",gas);
    QCOMPARE(panel.findChild<QLabel *>("gasValue")->text(),QString("—"));
    QVERIFY(panel.findChild<QLabel *>("gasStatus")->text().contains("isolated sensor error"));
    ptz["last_error"]="isolated camera failure";ptz["state"]="error";client.status("ptz",ptz);
    QCOMPARE(panel.findChild<QLabel *>("ptzActivity")->text(),QString("设备异常"));
    QVERIFY(panel.findChild<QLabel *>("ptzPatrolStatus")->text().contains("isolated camera failure"));
  }
  void gasSummaryShowsLiveUnitsAlarmsAndExpiresWithoutNavigation() {
    DeviceClient client(config(), true); // Inject display events without starting hardware.
    GasReadout readout(&client, false); readout.show();
    auto *value = readout.findChild<QLabel *>("consoleGasValue");
    auto *state = readout.findChild<QLabel *>("consoleGasState");
    QCOMPARE(value->text(), QString("—"));
    QSignalSpy open(&readout, &GasReadout::openRequested);
    QTest::mouseClick(readout.findChild<QToolButton *>("consoleGasOpen"), Qt::LeftButton);
    QCOMPARE(open.count(), 1);
    QJsonObject status{{"enabled", true}, {"connected", true}, {"interval", 2.0}};
    client.status("gas", status);
    client.gasSample({{"raw_value", 150}, {"value_scaled", 15.0}, {"decimals", 1},
      {"decimal_scaling_verified", false}, {"unit", "ppm"}, {"state_code", 2}});
    QCOMPARE(value->text(), QString("≈ 15.0 ppm"));
    QVERIFY(state->text().contains("高报")); QVERIFY(state->text().contains("估算"));
    QVERIFY(value->toolTip().contains("150")); QCOMPARE(value->property("consoleTone").toString(), QString("danger"));
    status["enabled"] = false; status["connected"] = false;
    client.status("gas", status); QCOMPARE(value->text(), QString("—"));
    QVERIFY(state->text().contains("已停止"));
    status["enabled"] = true; status["connected"] = true;
    client.status("gas", status); QCOMPARE(value->text(), QString("—")); // New session requires a new sample.
    client.gasSample({{"raw_value", 20}, {"value_scaled", 2.0}, {"unit", "%LEL"}, {"state_code", 0}});
    QVERIFY(value->text().contains("%LEL"));
    QTRY_VERIFY_WITH_TIMEOUT(state->text().contains("过期"), 3500);
    QCOMPARE(value->text(), QString("—"));
    status["last_error"] = "isolated unplugged sensor";
    client.status("gas", status); QCOMPARE(value->text(), QString("—"));
    QVERIFY(state->text().contains("异常"));
  }
};
QTEST_MAIN(DevicePanelTest)
#include "device_panel_test.moc"
