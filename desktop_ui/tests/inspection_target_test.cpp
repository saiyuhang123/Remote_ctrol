#include "patrol/inspection_target_page.h"
#include "patrol/patrol_ui_style.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QGraphicsView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalSpy>
#include <QSpinBox>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
#include <atomic>
#include <cmath>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <patrol_interfaces/action/inspect_point.hpp>
#include <patrol_interfaces/action/return_home.hpp>
#include <patrol_interfaces/srv/get_home_point.hpp>
#include <patrol_interfaces/srv/list_maps.hpp>
#include <patrol_interfaces/srv/list_points.hpp>

using Inspect = patrol_interfaces::action::InspectPoint;
using Home = patrol_interfaces::action::ReturnHome;

// Only fake action servers and temporary maps in a private localhost domain.
struct TargetFixture {
  QTemporaryDir dir;
  rclcpp::Node::SharedPtr node=std::make_shared<rclcpp::Node>("inspection_target_fixture");
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(),4};
  std::thread spin;
  patrol_interfaces::msg::MapBinding map;
  patrol_interfaces::msg::PatrolPoint point;
  std::atomic<int> pointGoals{0}, pointCancels{0}, homeGoals{0}, homeCancels{0};
  std::atomic<bool> release{false}, reject{false};
  Inspect::Goal received;
  int delayMs=0;
  std::vector<rclcpp::ServiceBase::SharedPtr> services;
  rclcpp_action::Server<Inspect>::SharedPtr inspect;
  rclcpp_action::Server<Home>::SharedPtr home;
  TargetFixture(int delay=0) : delayMs(delay) {
    QFile pgm(dir.filePath("map.pgm")); pgm.open(QIODevice::WriteOnly);
    pgm.write("P5\n100 80\n255\n"); pgm.write(QByteArray(8000,char(254))); pgm.close();
    QFile yaml(dir.filePath("map.yaml")); yaml.open(QIODevice::WriteOnly);
    yaml.write("image: map.pgm\nresolution: 0.2\norigin: [3.0, -4.0, 1.5707963267948966]\nnegate: 0\nfree_thresh: 0.25\noccupied_thresh: 0.65\n"); yaml.close();
    map.map_id="test-map"; map.name="测试地图"; map.revision=1; map.backend="amcl";
    map.navigation_yaml=yaml.fileName().toStdString(); map.width=100; map.height=80;
    map.resolution=.2; map.origin_x=3.; map.origin_y=-4.; map.origin_yaw=1.5707963267948966;
    point.point_id="saved"; point.map_id=map.map_id; point.name="多视角仪表点"; point.revision=7;
    point.x=-3.; point.y=5.; point.yaw=.4; point.inspection_enabled=true; point.ptz_preset=9;
    point.recognition_prompt="读取仪表";
    services.push_back(node->create_service<patrol_interfaces::srv::ListMaps>("/patrol/list_maps",
        [this](const std::shared_ptr<patrol_interfaces::srv::ListMaps::Request>,
               std::shared_ptr<patrol_interfaces::srv::ListMaps::Response> response) {response->success=true; response->active_map=map; response->maps.push_back(map);}));
    services.push_back(node->create_service<patrol_interfaces::srv::ListPoints>("/patrol/list_points",
        [this](const std::shared_ptr<patrol_interfaces::srv::ListPoints::Request>,
               std::shared_ptr<patrol_interfaces::srv::ListPoints::Response> response) {response->success=true; response->points.push_back(point);}));
    services.push_back(node->create_service<patrol_interfaces::srv::GetHomePoint>("/patrol/get_home_point",
        [this](const std::shared_ptr<patrol_interfaces::srv::GetHomePoint::Request>,
               std::shared_ptr<patrol_interfaces::srv::GetHomePoint::Response> response) {response->success=true; response->exists=true; response->map_revision=map.revision;}));
    auto group=node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // Retain the callback group with the fixture node.
    callbackGroup=group;
    inspect=rclcpp_action::create_server<Inspect>(node,"/patrol/inspect_point",
        [this](const auto &,std::shared_ptr<const Inspect::Goal> goal) {
          received=*goal; ++pointGoals;
          std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
          return reject.load()?rclcpp_action::GoalResponse::REJECT:rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](auto) {++pointCancels; return rclcpp_action::CancelResponse::ACCEPT;},
        [this](auto goal) {
          while(rclcpp::ok() && !release && !goal->is_canceling()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
          auto result=std::make_shared<Inspect::Result>(); result->message="测试任务结束";
          if(goal->is_canceling()) {result->final_state=9; goal->canceled(result);}
          else {result->success=true; result->final_state=10; goal->succeed(result);}
        },rcl_action_server_get_default_options(),group);
    home=rclcpp_action::create_server<Home>(node,"/patrol/return_home",
        [this](const auto &,std::shared_ptr<const Home::Goal>) {
          ++homeGoals; std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](auto) {++homeCancels; return rclcpp_action::CancelResponse::ACCEPT;},
        [this](auto goal) {
          while(rclcpp::ok() && !release && !goal->is_canceling()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
          auto result=std::make_shared<Home::Result>();
          if(goal->is_canceling()) goal->canceled(result);
          else {result->success=true; goal->succeed(result);}
        },rcl_action_server_get_default_options(),group);
    executor.add_node(node); spin=std::thread([this]{executor.spin();});
  }
  ~TargetFixture() {release=true; executor.cancel(); if(spin.joinable()) spin.join();}
  rclcpp::CallbackGroup::SharedPtr callbackGroup;
};

class InspectionTargetTest : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() {rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);}
  void cleanupTestCase() {rclcpp::shutdown();}

  void mapDragUsesRotatedOriginAndStartsOnlyOnClick() {
    TargetFixture fake;
    PatrolClient client;
    bool permitted=true;
    InspectionTargetPage page(&client,false,[&](QString *reason){if(!permitted && reason)*reason="急停保持"; return permitted;});
    page.setObjectName("patrolPage"); page.setStyleSheet(patrolui::styleSheet());
    page.resize(1100,650); page.show();
    QTRY_VERIFY_WITH_TIMEOUT(client.pointServerReady(),4000);
    client.refreshMaps(); client.refreshPoints("test-map"); client.refreshHomePoint("test-map");
    auto *view=page.findChild<QGraphicsView *>("inspectionTargetMap");
    auto *start=page.findChild<QPushButton *>("inspectionTargetStart");
    auto *x=page.findChild<QDoubleSpinBox *>("inspectionTargetX");
    auto *y=page.findChild<QDoubleSpinBox *>("inspectionTargetY");
    auto *yaw=page.findChild<QDoubleSpinBox *>("inspectionTargetYaw");
    QVERIFY(view && start && x && y && yaw);
    QTRY_COMPARE(view->sceneRect().width(),100.);
    QVERIFY(!start->isEnabled());
    const auto from=view->mapFromScene(QPointF(50,40)),to=view->mapFromScene(QPointF(50,25));
    QTest::mousePress(view->viewport(),Qt::LeftButton,Qt::NoModifier,from);
    QTest::mouseMove(view->viewport(),to,20); QTest::mouseRelease(view->viewport(),Qt::LeftButton,Qt::NoModifier,to);
    QVERIFY(std::abs(x->value()+5.)<.05); QVERIFY(std::abs(y->value()-6.)<.05);
    QVERIFY(std::abs(std::abs(yaw->value())-180.)<1.);
    QCOMPARE(fake.pointGoals.load(),0);
    QTRY_VERIFY(start->isEnabled());
    permitted=false; QTRY_VERIFY(!start->isEnabled());
    permitted=true; QTRY_VERIFY(start->isEnabled());
    auto *inspect=page.findChild<QCheckBox *>("inspectionTargetEnabled"); inspect->setChecked(false);
    start->click(); QTRY_COMPARE(fake.pointGoals.load(),1);
    QVERIFY(!fake.received.inspection_enabled); QVERIFY(fake.received.point_id.empty());
    QVERIFY(std::abs(fake.received.x+5.)<.05);
    QTRY_VERIFY(!start->isEnabled());
    QVERIFY(page.findChild<QPushButton *>("inspectionTargetHome")->isEnabled());
    page.grab().save("/tmp/inspection_target_desktop_test.png");
    client.cancelActiveTask(); QTRY_COMPARE(fake.pointCancels.load(),1);
    QTRY_VERIFY(!client.hasActiveOrPendingRun());
    x->setValue(999.); QTRY_VERIFY(!start->isEnabled());
  }

  void savedPointUsesRevisionAndInvalidatesOnChange() {
    TargetFixture fake;
    PatrolClient client;
    InspectionTargetPage page(&client,false,[](QString *){return true;});
    page.resize(1000,650); page.show();
    QTRY_VERIFY_WITH_TIMEOUT(client.pointServerReady(),4000);
    client.refreshMaps(); client.refreshPoints("test-map");
    auto *combo=page.findChild<QComboBox *>("inspectionTargetPoint");
    QTRY_COMPARE(combo->count(),2); combo->setCurrentIndex(1);
    auto *start=page.findChild<QPushButton *>("inspectionTargetStart");
    QTRY_VERIFY(start->isEnabled());
    QVERIFY(!page.findChild<QCheckBox *>("inspectionTargetEnabled")->isEnabled());
    // Inspecting an unchanged coordinate must retain saved multi-view work.
    page.findChild<QDoubleSpinBox *>("inspectionTargetX")->setFocus();
    start->setFocus(); QTest::qWait(20);
    QCOMPARE(combo->currentIndex(),1);
    start->click(); QTRY_COMPARE(fake.pointGoals.load(),1);
    QCOMPARE(QString::fromStdString(fake.received.point_id),QString("saved"));
    QCOMPARE(fake.received.point_revision,uint64_t(7));
    client.cancelActiveTask(); QTRY_VERIFY(!client.hasActiveOrPendingRun());
    ++fake.point.revision; client.refreshPoints("test-map");
    QTRY_COMPARE(combo->currentIndex(),0); QVERIFY(!start->isEnabled());
    combo->setCurrentIndex(1); QTRY_VERIFY(start->isEnabled());
    ++fake.map.revision; client.refreshMaps(); QTRY_VERIFY(!start->isEnabled());
    QCOMPARE(fake.pointGoals.load(),1);
  }

  void pendingPointAndReturnCanBothBeCanceledAndReturnIsDeduplicated() {
    TargetFixture fake(400);
    PatrolClient client;
    QTRY_VERIFY_WITH_TIMEOUT(client.pointServerReady() && client.returnServerReady(),4000);
    client.inspectPoint("test-map",1,"",0,-5,6,0,true,9,"读取仪表");
    QVERIFY(client.hasActiveOrPendingRun());
    client.returnHome("test-map",1);
    QVERIFY(client.returnActiveOrPending());
    client.returnHome("test-map",1); // double click cannot submit another goal
    client.cancelActiveTask();
    QTRY_COMPARE_WITH_TIMEOUT(fake.pointCancels.load(),1,4000);
    QTRY_COMPARE_WITH_TIMEOUT(fake.homeCancels.load(),1,4000);
    QTRY_VERIFY_WITH_TIMEOUT(!client.hasActiveOrPendingRun(),4000);
    QCOMPARE(fake.homeGoals.load(),1); QCOMPARE(fake.pointGoals.load(),1);
    fake.reject=true;
    client.inspectPoint("test-map",1,"",0,-5,6,0,true,9,"");
    QTRY_COMPARE(fake.pointGoals.load(),2); QTRY_VERIFY(!client.hasActiveOrPendingRun());
    fake.reject=false;
    client.inspectPoint("test-map",1,"",0,-5,6,0,true,9,"");
    QTRY_COMPARE(fake.pointGoals.load(),3); client.cancelActiveTask();
    QTRY_VERIFY(!client.hasActiveOrPendingRun());
  }
};
QTEST_MAIN(InspectionTargetTest)
#include "inspection_target_test.moc"
