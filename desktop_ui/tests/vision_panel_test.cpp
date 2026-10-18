#include "vision/vision_panel.h"
#include "runtime/config.h"
#include <QCheckBox>
#include <QDialog>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTest>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vision_interfaces/srv/query_events.hpp>
#include <vision_interfaces/srv/set_recording_context.hpp>
#include <atomic>
#include <thread>

struct VisionFixture {
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("detector_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  std::atomic_int faceStarts{0}, faceStops{0}, queries{0};
  std::vector<rclcpp::ServiceBase::SharedPtr> services;
  VisionFixture() {
    node->declare_parameter("enable_person", false); node->declare_parameter("enable_plate", false);
    services.push_back(node->create_service<std_srvs::srv::SetBool>("/hk_camera/set_face_capture",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        if (req->data) ++faceStarts; else ++faceStops;
        if (req->data) std::this_thread::sleep_for(std::chrono::milliseconds(200));
        res->success = true; res->message = "isolated camera fixture";
      }));
    services.push_back(node->create_service<vision_interfaces::srv::QueryEvents>("/vision/query_events",
      [this](vision_interfaces::srv::QueryEvents::Request::SharedPtr, vision_interfaces::srv::QueryEvents::Response::SharedPtr res) {
        ++queries; res->success = true; res->total = 1;
        res->events_json = QJsonDocument(QJsonArray{QJsonObject{{"event_type", "face_capture"},
            {"label", "人脸抓拍"}, {"pose_valid", false}, {"pose_status", "map_unassociated"},
            {"event_time_ns", 1.79e18}}}).toJson(QJsonDocument::Compact).toStdString();
      }));
    services.push_back(node->create_service<vision_interfaces::srv::SetRecordingContext>("/vision/set_recording_context",
      [](vision_interfaces::srv::SetRecordingContext::Request::SharedPtr, vision_interfaces::srv::SetRecordingContext::Response::SharedPtr res) { res->success = true; }));
    executor.add_node(node); thread = std::thread([this] { executor.spin(); });
  }
  ~VisionFixture() { executor.cancel(); if (thread.joinable()) thread.join(); }
};

class VisionPanelTest : public QObject {
  Q_OBJECT
  QTemporaryDir project_;
  DesktopConfig config() const {
    return DesktopConfig::load(QString(DESKTOP_UI_SOURCE_DIR) + "/config/desktop.json", DESKTOP_UI_SOURCE_DIR, project_.path());
  }
private slots:
  void initTestCase() { rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None); }
  void cleanupTestCase() { rclcpp::shutdown(); }
  void standaloneControlsAndHistoryNeedNoNavigationOrMotionAuthorization() {
    VisionFixture fixture;
    VisionPanel panel(config(), false); panel.resize(350, 500); panel.show();
    QTest::qWait(300);
    auto button = [&](const char *name) { return panel.findChild<QPushButton *>(name); };
    QVERIFY(button("visionStartFace")->isEnabled());
    QTest::mouseClick(button("visionStartFace"), Qt::LeftButton);
    QTRY_COMPARE_WITH_TIMEOUT(fixture.faceStarts.load(), 1, 5000);
    panel.hide(); QTest::qWait(150); QCOMPARE(fixture.faceStops.load(), 0); panel.show();
    QTest::mouseClick(button("visionStopFace"), Qt::LeftButton); QTRY_COMPARE(fixture.faceStops.load(), 1);
    panel.findChild<QCheckBox *>("visionPlate")->setChecked(false);
    QTest::mouseClick(button("visionStartDetector"), Qt::LeftButton);
    QTRY_VERIFY_WITH_TIMEOUT(fixture.node->get_parameter("enable_person").as_bool(), 5000);
    QVERIFY(!fixture.node->get_parameter("enable_plate").as_bool());
    QTest::mouseClick(button("visionStopDetector"), Qt::LeftButton);
    QTRY_VERIFY(!fixture.node->get_parameter("enable_person").as_bool());
    QTest::mouseClick(button("visionHistory"), Qt::LeftButton);
    auto *dialog = panel.findChild<QDialog *>("visionHistoryDialog"); QVERIFY(dialog && dialog->isVisible());
    auto *table = dialog->findChild<QTableWidget *>(); QVERIFY(table);
    QTRY_COMPARE(table->rowCount(), 1); QCOMPARE(table->item(0, 2)->text(), QString("人脸抓拍"));
    QCOMPARE(table->item(0, 3)->text(), QString("—"));
    dialog->close(); QTest::qWait(50);
  }
  void offlinePreviewCannotStartHardware() {
    VisionPanel panel(config(), true);
    QVERIFY(!panel.findChild<QPushButton *>("visionStartFace")->isEnabled());
    QVERIFY(!panel.findChild<QPushButton *>("visionStartDetector")->isEnabled());
  }
};
QTEST_MAIN(VisionPanelTest)
#include "vision_panel_test.moc"
