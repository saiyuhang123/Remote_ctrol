#include "devices/device_client.h"
#include "devices/device_panel.h"
#include "devices/gas_readout.h"
#include "ui/camera_panel.h"
#include "adapters/ros_adapter.h"
#include <QCheckBox>
#include <QJsonDocument>
#include <QLabel>
#include <QPushButton>
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
    QTRY_VERIFY(panel.findChild<QLabel *>("gasValue")->text().contains("高报"));
    panel.hide(); QTest::qWait(200); QVERIFY(fixture.gasEnabled.load()); panel.show();
    QTest::mouseClick(button("gasStop"), Qt::LeftButton); QTRY_VERIFY(!fixture.gasEnabled.load());
    QTRY_VERIFY(panel.findChild<QLabel *>("gasValue")->text().contains("无新鲜采样"));
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
