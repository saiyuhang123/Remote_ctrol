#include "adapters/ros_adapter.h"
#include "runtime/map_saver.h"
#include <QDir>
#include <QFile>
#include <QRegularExpression>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QTest>
#include <QTextStream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <atomic>
#include <thread>

namespace {
QByteArray readFile(const QString &path) {
  QFile file(path);
  return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray();
}
bool writeFile(const QString &path, const QByteArray &data) {
  QFile file(path);
  return file.open(QIODevice::WriteOnly) && file.write(data) == data.size();
}
QByteArray cloud(bool valid) {
  QByteArray data;
  QTextStream stream(&data);
  stream.setRealNumberPrecision(12);
  stream << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            "COUNT 1 1 1\nWIDTH 121\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 121\nDATA ascii\n";
  for (int x = 0; x <= 10; ++x)
    for (int y = 0; y <= 10; ++y)
      stream << 10.12345678 + .04*x << ' ' << 20.23456789 + .04*y << ' ' << (valid ? .3 : 10.) << '\n';
  stream.flush();
  return data;
}
struct MappingFixture {
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("desktop_mapping_fixture");
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  std::atomic_bool valid{true};
  std::atomic_bool writePcd{true};
  std::atomic_int requests{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;
  MappingFixture(const QString &prefix) {
    service = node->create_service<std_srvs::srv::Trigger>("/loop_closure/save_map",
      [this, prefix](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        ++requests;
        response->success = !writePcd || writeFile(prefix + ".pcd", cloud(valid));
        response->message = "fixture PCD saved";
      });
    executor.add_node(node);
    thread = std::thread([this] { executor.spin(); });
  }
  ~MappingFixture() { executor.cancel(); if (thread.joinable()) thread.join(); }
};
}

class MapSaverTest : public QObject {
  Q_OBJECT
  DesktopConfig config() {
    return DesktopConfig::load(QString(DESKTOP_UI_SOURCE_DIR) + "/config/desktop.json",
                               DESKTOP_UI_SOURCE_DIR, QString(DESKTOP_UI_SOURCE_DIR) + "/..");
  }
private slots:
  void initTestCase() { rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None); }
  void cleanupTestCase() { rclcpp::shutdown(); }

  void exportsOwnPcdAndPreservesOldGridOnFailure() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const auto prefix = directory.filePath("export with space ' 中文.v2");
    MappingFixture fixture(prefix);
    // A retained map from another conversion must never be selected for this save.
    auto stale = fixture.node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/projected_map", rclcpp::QoS(1).reliable().transient_local());
    nav_msgs::msg::OccupancyGrid wrong;
    wrong.header.frame_id = "map"; wrong.info.resolution = 1.;
    wrong.info.width = wrong.info.height = 1;
    wrong.info.origin.position.x = wrong.info.origin.position.y = 999.;
    wrong.info.origin.orientation.w = 1.; wrong.data = {0}; stale->publish(wrong);
    RosAdapter adapter(config());
    MapSaver saver(config(), &adapter);
    QSignalSpy saved(&saver, &MapSaver::saved);
    QSignalSpy messages(&saver, &MapSaver::message);
    connect(&saver, &MapSaver::message, [](const QString &text) { qInfo().noquote() << text.trimmed(); });
    QTest::qWait(800);
    saver.save(desktopProfiles().at(1), prefix);
    QVERIFY(saver.busy());
    // Repeated clicks cannot issue another save while the first export runs.
    saver.save(desktopProfiles().at(1), prefix);
    QTRY_COMPARE_WITH_TIMEOUT(saved.size(), 1, 30000);
    QVERIFY2(saved.at(0).at(0).toBool(), qPrintable(saved.at(0).at(1).toString()));
    QVERIFY(!saver.busy()); QCOMPARE(fixture.requests.load(), 1);
    const auto pgm = readFile(prefix + ".pgm");
    const auto yaml = readFile(prefix + ".yaml");
    QVERIFY(pgm.startsWith("P5"));
    const auto text = QString::fromUtf8(yaml);
    QVERIFY(text.contains("export with space ' 中文.v2.pgm"));
    QVERIFY(text.contains("resolution: 0.05"));
    QVERIFY(QRegularExpression("origin:\\s*- 10\\.1234[0-9]*\\s*- 20\\.2345[0-9]*\\s*- 0").match(text).hasMatch());
    QVERIFY(!text.contains("999"));

    fixture.valid = false; // All points outside the configured height band.
    saver.save(desktopProfiles().at(1), prefix);
    QTRY_COMPARE_WITH_TIMEOUT(saved.size(), 2, 30000);
    QVERIFY(!saved.at(1).at(0).toBool()); QVERIFY(!saver.busy());
    QCOMPARE(readFile(prefix + ".pcd"), cloud(false));
    QCOMPARE(readFile(prefix + ".pgm"), pgm);
    QCOMPARE(readFile(prefix + ".yaml"), yaml);
    QCOMPARE(fixture.requests.load(), 2);
    QVERIFY(QDir(directory.path()).entryList({".fastlio-export-*"}, QDir::Dirs | QDir::Hidden).isEmpty());

    fixture.valid = true;
    saver.save(desktopProfiles().at(1), prefix);
    QTRY_COMPARE_WITH_TIMEOUT(saved.size(), 3, 30000);
    QVERIFY(saved.at(2).at(0).toBool()); QCOMPARE(fixture.requests.load(), 3);
    // Check actual processes: an aborted ROS node can remain in DDS discovery
    // until its lease expires even after the underlying process has exited.
    QTRY_VERIFY_WITH_TIMEOUT(([&directory] {
      for (const auto &pid : QDir("/proc").entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
        const auto command = readFile("/proc/" + pid + "/cmdline");
        if (command.contains(directory.path().toUtf8()) &&
            (command.contains("pcd2pgm_node") || command.contains("map_saver_cli") ||
             command.contains("export_fastlio_map.py"))) return false;
      }
      return true;
    })(), 5000);
  }

  void serviceSuccessWithoutPcdDoesNotExportOldGrid() {
    QTemporaryDir directory;
    const auto prefix = directory.filePath("missing");
    MappingFixture fixture(prefix); fixture.writePcd = false;
    QVERIFY(writeFile(prefix + ".pgm", "previous pgm"));
    QVERIFY(writeFile(prefix + ".yaml", "previous yaml"));
    RosAdapter adapter(config()); MapSaver saver(config(), &adapter);
    QSignalSpy saved(&saver, &MapSaver::saved);
    QTest::qWait(800); saver.save(desktopProfiles().at(1), prefix);
    QTRY_COMPARE_WITH_TIMEOUT(saved.size(), 1, 5000);
    QVERIFY(!saved.at(0).at(0).toBool());
    QCOMPARE(readFile(prefix + ".pgm"), QByteArray("previous pgm"));
    QCOMPARE(readFile(prefix + ".yaml"), QByteArray("previous yaml"));
  }

  void projectionTimeoutPreservesPcdAndOldGrid() {
    QTemporaryDir directory;
    const auto prefix = directory.filePath("timeout map");
    auto c = config();
    QDir resources(directory.filePath("resources"));
    QVERIFY(resources.mkpath("scripts")); QVERIFY(resources.mkpath("config"));
    QVERIFY(QFile::copy(c.resourceDir + "/scripts/export_fastlio_map.py",
                       resources.filePath("scripts/export_fastlio_map.py")));
    auto parameters = readFile(c.resourceDir + "/config/fastlio_map_export.yaml");
    parameters.replace("timeout_sec: 180.0", "timeout_sec: 0.1");
    QVERIFY(writeFile(resources.filePath("config/fastlio_map_export.yaml"), parameters));
    c.resourceDir = resources.absolutePath();
    MappingFixture fixture(prefix);
    QVERIFY(writeFile(prefix + ".pgm", "previous pgm"));
    QVERIFY(writeFile(prefix + ".yaml", "previous yaml"));
    RosAdapter adapter(c); MapSaver saver(c, &adapter);
    QSignalSpy saved(&saver, &MapSaver::saved), messages(&saver, &MapSaver::message);
    QTest::qWait(800); saver.save(desktopProfiles().at(1), prefix);
    QTRY_COMPARE_WITH_TIMEOUT(saved.size(), 1, 15000);
    QVERIFY(!saved.at(0).at(0).toBool()); QVERIFY(!saver.busy());
    QString log; for (const auto &message : messages) log += message.at(0).toString();
    QVERIFY2(log.contains("超时"), qPrintable(log));
    QCOMPARE(readFile(prefix + ".pcd"), cloud(true));
    QCOMPARE(readFile(prefix + ".pgm"), QByteArray("previous pgm"));
    QCOMPARE(readFile(prefix + ".yaml"), QByteArray("previous yaml"));
    QVERIFY(QDir(directory.path()).entryList({".fastlio-export-*"}, QDir::Dirs | QDir::Hidden).isEmpty());
  }
};
QTEST_GUILESS_MAIN(MapSaverTest)
#include "map_saver_test.moc"
