#include "adapters/ros_adapter.h"
#include "runtime/process_manager.h"
#include <QDir>
#include <QFileInfo>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QTest>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <atomic>
#include <thread>
#ifdef DESKTOP_HAS_HK_CAMERA
#include <hk_camera/srv/capture_picture.hpp>
#include <hk_camera/srv/set_ptz_pose.hpp>
#endif
#ifdef DESKTOP_HAS_OCR_INTERFACES
#include <ocr_interfaces/srv/recognize_text.hpp>
#endif
#ifdef DESKTOP_HAS_SLAM_TOOLBOX
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#endif
#ifdef DESKTOP_HAS_CARTOGRAPHER_ROS_MSGS
#include <cartographer_ros_msgs/srv/write_state.hpp>
#endif

struct FixtureExecutor {
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("desktop_service_fixture");
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  void start() { executor.add_node(node); thread = std::thread([this] { executor.spin(); }); }
  ~FixtureExecutor() { executor.cancel(); if (thread.joinable()) thread.join(); }
};
template<class Service, class Callback>
auto mockService(const rclcpp::Node::SharedPtr &node, const std::string &name, Callback callback) {
  return node->create_service<Service>(name,
    [callback](const std::shared_ptr<typename Service::Request> request,
               std::shared_ptr<typename Service::Response> response) { callback(request, response); });
}
class ServicesTest : public QObject {
  Q_OBJECT
  DesktopConfig config() { return DesktopConfig::load(QString(DESKTOP_UI_SOURCE_DIR) + "/config/desktop.json", DESKTOP_UI_SOURCE_DIR, "/tmp"); }
private slots:
  void initTestCase() { rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None); }
  void cleanupTestCase() { rclcpp::shutdown(); }
  void amclProfileUsesOccupancyMapAndPreservesOtherBackends() {
    const auto &amcl = desktopProfiles().at(2);
    QCOMPARE(amcl.id, QString("slam_navigation"));
    QVERIFY(amcl.name.contains("AMCL")); QCOMPARE(amcl.backend, QString("slam_toolbox"));
    auto args = ProcessManager::launchArguments(config(), amcl, "/tmp/slam_map", true, false, false, "lio");
    QVERIFY(args.contains("amcl_localization.launch.py")); QVERIFY(args.contains("map:=/tmp/slam_map.yaml"));
    QVERIFY(!args.contains("map_file:=/tmp/slam_map"));
    args = ProcessManager::launchArguments(config(), desktopProfiles().at(4), "/tmp/carto", true, false, false, "lio");
    QVERIFY(args.contains("map_file:=/tmp/carto"));
    args = ProcessManager::launchArguments(config(), desktopProfiles().at(3), "/tmp/slam_map", true, false, false, "lio");
    QVERIFY(args.contains("slam_2d_mapping.launch.py"));
  }
  void launchPreservesLiteralArgumentsAndStopsOnlyOwnedProcess() {
    auto c = config(); c.projectRoot = QDir(DESKTOP_UI_SOURCE_DIR).absoluteFilePath("..");
    if (!QFileInfo::exists(c.projectRoot + "/ros2_ws/install/setup.bash")) QSKIP("Project ROS overlay is required for local launch tests");
    QTemporaryDir directory;
    const auto marker = directory.filePath("must_not_exist");
    const auto literal = "space ' \" $(touch " + marker + ") `touch " + marker + "`";
    ProcessManager process(c); QSignalSpy output(&process, &ProcessManager::output); QSignalSpy finished(&process, &ProcessManager::finished);
    QVERIFY(process.start({"python3", "-c", "import sys; print(sys.argv[1], flush=True)", literal}));
    QTRY_COMPARE_WITH_TIMEOUT(finished.size(), 1, 5000); QCOMPARE(finished.at(0).at(0).toInt(), 0);
    QString text; for (const auto &line : output) text += line.at(0).toString(); QVERIFY(text.contains(literal)); QVERIFY(!QFileInfo::exists(marker));
    QProcess unrelated; unrelated.start("python3", {"-c", "import time; time.sleep(30)"}); QVERIFY(unrelated.waitForStarted(1000));
    output.clear(); finished.clear();
    QVERIFY(process.start({"python3", "-c", "import time; print('owned_ready',flush=True); time.sleep(30)"}));
    QTRY_VERIFY_WITH_TIMEOUT(!output.isEmpty(), 5000);
    process.stop(); QTRY_COMPARE_WITH_TIMEOUT(finished.size(), 1, 3000);
    QCOMPARE(unrelated.state(), QProcess::Running);
    unrelated.kill(); QVERIFY(unrelated.waitForFinished(1000));
  }
  void mapSaveTimeoutIgnoresLateResponse() {
    FixtureExecutor fixture;
    auto service = mockService<std_srvs::srv::Trigger>(fixture.node, "/map_save", [](auto, auto response) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1500)); response->success = true;
    }); fixture.start();
    auto c = config(); c.serviceTimeoutMs = 1000;
    RosAdapter adapter(c); QSignalSpy results(&adapter, &RosAdapter::requestFinished);
    QTest::qWait(300); adapter.saveState("fastlio", "/tmp/unused");
    QTRY_COMPARE_WITH_TIMEOUT(results.size(), 1, 3000);
    QCOMPARE(results.at(0).at(1).toBool(), false);
    QVERIFY(results.at(0).at(2).toString().contains("超时"));
    QTest::qWait(700); QCOMPARE(results.size(), 1);
  }
  void mapSaveUsesBackendServiceTypes() {
    FixtureExecutor fixture;
    auto trigger = mockService<std_srvs::srv::Trigger>(fixture.node, "/map_save", [](auto, auto r) { r->success = true; r->message = "mock PCD saved"; });
#ifdef DESKTOP_HAS_SLAM_TOOLBOX
    auto slam = mockService<slam_toolbox::srv::SerializePoseGraph>(fixture.node, "/slam_toolbox/serialize_map", [](auto q, auto r) { r->result = q->filename == "/tmp/desktop_map" ? 0 : 255; });
#endif
#ifdef DESKTOP_HAS_CARTOGRAPHER_ROS_MSGS
    auto carto = mockService<cartographer_ros_msgs::srv::WriteState>(fixture.node, "/write_state", [](auto q, auto r) { r->status.code = q->filename == "/tmp/desktop_map.pbstream" && q->include_unfinished_submaps ? 0 : 1; });
#endif
    fixture.start(); RosAdapter adapter(config()); QSignalSpy results(&adapter, &RosAdapter::requestFinished); QTest::qWait(300);
    adapter.saveState("fastlio", "/tmp/desktop_map"); QTRY_COMPARE(results.size(), 1); QVERIFY(results.last().at(1).toBool());
#ifdef DESKTOP_HAS_SLAM_TOOLBOX
    adapter.saveState("slam_toolbox", "/tmp/desktop_map"); QTRY_COMPARE(results.size(), 2); QVERIFY(results.last().at(1).toBool());
#endif
#ifdef DESKTOP_HAS_CARTOGRAPHER_ROS_MSGS
    int count = results.size(); adapter.saveState("cartographer", "/tmp/desktop_map"); QTRY_COMPARE(results.size(), count + 1); QVERIFY(results.last().at(1).toBool());
#endif
  }
  void cameraAndOcrRoundTrips() {
    FixtureExecutor fixture;
#ifdef DESKTOP_HAS_HK_CAMERA
    std::atomic_int pan{0};
    auto capture = mockService<hk_camera::srv::CapturePicture>(fixture.node, "/hk_camera/capture", [](auto q, auto r) {
      r->success = q->quality == 0 && q->save_path.empty(); r->image.width = 2; r->image.height = 1; r->image.step = 6; r->image.encoding = "rgb8"; r->image.data = {255, 0, 0, 0, 255, 0};
    });
    auto ptz = mockService<hk_camera::srv::SetPTZPose>(fixture.node, "/hk_camera/set_ptz_pose", [&pan](auto q, auto r) { pan = q->pan; r->success = q->zoom == 2 && q->tilt == 10; });
#endif
#ifdef DESKTOP_HAS_OCR_INTERFACES
    auto ocr = mockService<ocr_interfaces::srv::RecognizeText>(fixture.node, "/ocr/recognize", [](auto, auto r) {
      r->success = true; r->detections.resize(1); r->detections[0].text = "12.34"; r->detections[0].confidence = .99; r->processing_time_ms = 12.;
    });
#endif
    fixture.start(); RosAdapter adapter(config()); QSignalSpy results(&adapter, &RosAdapter::requestFinished);
    QSignalSpy images(&adapter, &RosAdapter::capturedImage); QSignalSpy texts(&adapter, &RosAdapter::ocrResult); QTest::qWait(300);
#ifdef DESKTOP_HAS_HK_CAMERA
    adapter.captureCamera(); QTRY_COMPARE(images.size(), 1); QVERIFY(results.last().at(1).toBool());
    auto image = qvariant_cast<QImage>(images.at(0).at(0)); QCOMPARE(image.size(), QSize(2, 1)); QCOMPARE(image.pixelColor(0, 0), QColor(Qt::red));
    int before = results.size(); adapter.setPtz(45, 10, 2); QTRY_COMPARE(results.size(), before + 1); QVERIFY(results.last().at(1).toBool()); QCOMPARE(pan.load(), 45);
#endif
#ifdef DESKTOP_HAS_OCR_INTERFACES
    adapter.recognize(); QTRY_COMPARE(texts.size(), 1); QVERIFY(texts.at(0).at(0).toString().contains("12.34")); QVERIFY(results.last().at(1).toBool());
#endif
  }
};
QTEST_GUILESS_MAIN(ServicesTest)
#include "ros_services_test.moc"
