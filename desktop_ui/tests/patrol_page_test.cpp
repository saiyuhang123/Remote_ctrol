#include "patrol/patrol_client.h"
#include "patrol/patrol_page.h"

#include <QDir>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStringList>
#include <QTableWidget>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <patrol_interfaces/action/run_route.hpp>
#include <patrol_interfaces/msg/patrol_status.hpp>
#include <patrol_interfaces/srv/control_run.hpp>
#include <patrol_interfaces/srv/list_maps.hpp>
#include <patrol_interfaces/srv/list_routes.hpp>
#include <QMessageBox>
#include <QTimer>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QJsonDocument>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QGraphicsView>
#include <QScrollArea>
#include <patrol_interfaces/srv/list_points.hpp>
#include <patrol_interfaces/srv/get_runs.hpp>
#include <patrol_interfaces/srv/get_home_point.hpp>
#include <patrol_interfaces/srv/road_command.hpp>

class PatrolPageTest : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() {
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  }
  void cleanupTestCase() { rclcpp::shutdown(); }

  void pendingRunCancelIsRememberedBeforeGoalAcceptance() {
    using RunRoute = patrol_interfaces::action::RunRoute;
    auto node = std::make_shared<rclcpp::Node>("pending_run_cancel_fixture");
    std::atomic<int> cancelCalls{0};
    std::atomic<bool> executeStarted{false};
    auto callbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto server = rclcpp_action::create_server<RunRoute>(
        node, "/patrol/run_route",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const RunRoute::Goal>) {
          std::this_thread::sleep_for(std::chrono::milliseconds(400));
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunRoute>>) {
          cancelCalls.fetch_add(1);
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunRoute>> goal) {
          executeStarted.store(true);
          while (rclcpp::ok() && !goal->is_canceling())
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          goal->canceled(std::make_shared<RunRoute::Result>());
        }, rcl_action_server_get_default_options(), callbackGroup);
    Q_UNUSED(server);
    rclcpp::executors::MultiThreadedExecutor fakeExecutor(rclcpp::ExecutorOptions(), 2);
    fakeExecutor.add_node(node);
    std::thread fakeSpin([&] { fakeExecutor.spin(); });
    PatrolClient client;
    if (!client.available()) {
      fakeExecutor.cancel();
      fakeSpin.join();
      QSKIP("patrol_interfaces/robot_patrol is not built");
    }
    QTest::qWait(800);  // allow action client/server discovery

    client.setOperatorAuthorized(true);
    client.runRoute("route", "map", 1, 1, 1);
    QVERIFY(client.hasActiveOrPendingRun());
    client.cancelRunRoute();
    const bool cancelDelivered = QTest::qWaitFor([&] { return cancelCalls.load() >= 1; }, 3000);
    const bool started = QTest::qWaitFor([&] { return executeStarted.load(); }, 1000);
    const bool finished = QTest::qWaitFor(
        [&] { return !client.hasActiveOrPendingRun(); }, 3000);
    fakeExecutor.cancel();
    fakeSpin.join();
    QVERIFY2(cancelDelivered, "pre-accept cancel intent was not honored");
    QVERIFY(started);
    QVERIFY(finished);
  }

  void pausedRunUsesResumePermissionNotStartPermission() {
    auto node = std::make_shared<rclcpp::Node>("paused_resume_permission_fixture");
    std::atomic<int> controlCommand{-1};
    auto control = node->create_service<patrol_interfaces::srv::ControlRun>(
        "/patrol/control",
        [&](const std::shared_ptr<patrol_interfaces::srv::ControlRun::Request> request,
            std::shared_ptr<patrol_interfaces::srv::ControlRun::Response> response) {
          controlCommand.store(static_cast<int>(request->command));
          response->accepted = true;
        });
    Q_UNUSED(control);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    auto status = node->create_publisher<patrol_interfaces::msg::PatrolStatus>("/patrol/status", qos);
    auto timer = node->create_wall_timer(std::chrono::milliseconds(50), [status] {
      patrol_interfaces::msg::PatrolStatus message;
      message.instance_id = "paused-ui-fixture";
      message.run_id = "run-paused";
      message.route_id = "route-paused";
      message.map_id = "map-paused";
      message.state = 7;
      message.active = true;
      message.step_count = 1;
      status->publish(message);
    });
    Q_UNUSED(timer);
    rclcpp::executors::MultiThreadedExecutor fakeExecutor(rclcpp::ExecutorOptions(), 2);
    fakeExecutor.add_node(node);
    std::thread fakeSpin([&] { fakeExecutor.spin(); });

    PatrolClient client;
    if (!client.available()) {
      fakeExecutor.cancel();
      fakeSpin.join();
      QSKIP("patrol_interfaces/robot_patrol is not built");
    }
    PatrolPage page(&client, QString(), "amcl", QString(), "测试模式", false,
                    [](QString *reason) {
                      if (reason) *reason = QStringLiteral("活动任务不能启动新任务");
                      return false;
                    });
    page.setResumePermission([](QString *) { return true; });
    page.resize(1000, 700);
    page.show();
    QVERIFY(QTest::qWaitForWindowExposed(&page));

    auto *resume = page.findChild<QPushButton *>(QStringLiteral("patrolResume"));
    QVERIFY(resume);
    QTRY_VERIFY_WITH_TIMEOUT(resume->isEnabled(), 3000);
    resume->click();
    QTRY_COMPARE_WITH_TIMEOUT(controlCommand.load(), 2, 2000);

    fakeExecutor.cancel();
    fakeSpin.join();
    page.close();
  }

  void pageBuildsWithLocalMapAndOfflineService() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString image = directory.filePath("map.pgm");
    const QString yaml = directory.filePath("map.yaml");
    QFile imageFile(image);
    QVERIFY(imageFile.open(QIODevice::WriteOnly));
    QVERIFY(imageFile.write(QByteArray("P5\n20 20\n255\n") + QByteArray(400, char(205))) > 0);
    imageFile.close();
    QFile yamlFile(yaml);
    QVERIFY(yamlFile.open(QIODevice::WriteOnly));
    QVERIFY(yamlFile.write("image: map.pgm\nmode: trinary\nresolution: 0.05\n"
                           "origin: [-1.0, -1.0, 0.0]\nnegate: 0\n"
                           "free_thresh: 0.25\noccupied_thresh: 0.65\n") > 0);
    yamlFile.close();

    PatrolClient client;
    if (!client.available()) QSKIP("patrol_interfaces/robot_patrol is not built");
    PatrolPage page(&client, yaml, "amcl", "", "测试模式", false);
    page.resize(1000, 700);
    page.show();
    QVERIFY(QTest::qWaitForWindowExposed(&page));

    auto *mapEdit = page.findChild<QLineEdit *>(QStringLiteral("patrolMapYaml"));
    auto *tabs = page.findChild<QTabWidget *>(QStringLiteral("patrolTabs"));
    auto *registerMap = page.findChild<QPushButton *>(QStringLiteral("patrolRegisterMap"));
    auto *saveCurrent = page.findChild<QPushButton *>(QStringLiteral("patrolSaveCurrentPoint"));
    auto *start = page.findChild<QPushButton *>(QStringLiteral("patrolStart"));
    QVERIFY(mapEdit && tabs && registerMap && saveCurrent && start);
    QCOMPARE(mapEdit->text(), yaml);
    QCOMPARE(tabs->count(), 4);
    QVERIFY(registerMap->isEnabled());
    QVERIFY(!saveCurrent->isEnabled()); // no active map yet
    QVERIFY(!start->isEnabled());
    auto *readiness = page.findChild<QLabel *>(QStringLiteral("patrolStartReadiness"));
    QVERIFY(readiness);
    QVERIFY(readiness->text().contains(QStringLiteral("未设置活跃地图")));
    QVERIFY(readiness->text().contains(QStringLiteral("未选择已保存路线")));
    QVERIFY(readiness->text().contains(QStringLiteral("巡航后台未就绪")));
    QTest::qWait(300);
    page.close();
  }

  void startReadinessTracksPermissionsBackendAndUnsavedRoute() {
    using RunRoute = patrol_interfaces::action::RunRoute;
    auto node = std::make_shared<rclcpp::Node>("patrol_readiness_fixture");
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    // Synthetic floor plan and roads, for layout/rendering checks only.
    QImage floor(480, 320, QImage::Format_RGB32);
    floor.fill(QColor("#f1f3f5"));
    QPainter painter(&floor);
    painter.setPen(QPen(QColor("#8793a3"), 6));
    painter.drawRect(14, 14, 452, 292);
    painter.setBrush(QColor("#c1cbd8"));
    painter.drawRect(120, 100, 100, 110);
    painter.drawRect(260, 100, 100, 110);
    painter.end();
    QVERIFY(floor.save(directory.filePath("map.png")));
    QFile yamlFile(directory.filePath("map.yaml"));
    QVERIFY(yamlFile.open(QIODevice::WriteOnly));
    yamlFile.write("image: map.png\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\nfree_thresh: 0.25\noccupied_thresh: 0.65\n");
    yamlFile.close();
    patrol_interfaces::msg::MapBinding map;
    map.map_id = "readiness-map"; map.name = "厂区一层（测试）"; map.revision = 1;
    map.backend = "amcl"; map.navigation_yaml = yamlFile.fileName().toStdString();
    map.resolution = .05; map.width = 480; map.height = 320;
    patrol_interfaces::msg::PatrolRoute route;
    route.route_id = "readiness-route"; route.map_id = map.map_id;
    route.name = "日常设备巡检"; route.revision = 1;
    std::vector<patrol_interfaces::msg::PatrolPoint> fixturePoints;
    const QStringList pointNames{"出入口", "设备间", "仓库", "值班室"};
    const QVector<QPointF> positions{{3,3},{20,4},{19,13},{3,12}};
    QJsonObject nodes, edges, bindings;
    for(int i=0;i<4;++i) {
      const auto id=QString("point-%1").arg(i), nodeId=QString("node-%1").arg(i);
      patrol_interfaces::msg::PatrolPoint point;
      point.point_id=id.toStdString();point.map_id=map.map_id;point.name=pointNames[i].toStdString();
      point.x=positions[i].x();point.y=positions[i].y();point.yaw=(i==3?-1:i)*1.57079632679;point.revision=1;
      fixturePoints.push_back(point);
      patrol_interfaces::msg::RouteStep step;
      step.point_id=point.point_id;step.point_name=point.name;step.x=point.x;step.y=point.y;step.yaw=point.yaw;step.dwell_seconds=5;
      route.steps.push_back(step);
      nodes[nodeId]=QJsonObject{{"name",pointNames[i]},{"x",point.x},{"y",point.y},{"yaw",point.yaw}};
      bindings[id]=QJsonArray{nodeId};
    }
    const QStringList edgeNames{"入口至设备间", "东侧巡检道", "仓库至值班室", "返回入口"};
    QJsonArray planBlocks;
    for(int i=0;i<4;++i) {
      QPainterPath path(positions[i]);
      if(i==0) {path.lineTo(19,3);path.quadTo(20,3,20,4);}
      if(i==1) {path.lineTo(20,12);path.quadTo(20,13,19,13);}
      if(i==2) {path.lineTo(4,13);path.quadTo(3,13,3,12);}
      if(i==3) path.lineTo(positions[0]);
      QJsonArray samples;
      for(int k=0;k<=60;++k) {const auto p=path.pointAtPercent(k/60.0);samples.append(QJsonObject{{"x",p.x()},{"y",p.y()}});}
      QJsonObject block{{"direction",i==3?-1:1},{"samples",samples}};
      edges[QString("edge-%1").arg(i)]=QJsonObject{{"name",edgeNames[i]},{"from",QString("node-%1").arg(i)},
        {"to",QString("node-%1").arg((i+1)%4)},{"speed",.2},{"enabled",true},{"blocks",QJsonArray{block}}};
      if(i<2) planBlocks.append(block);
    }
    QJsonObject document{{"nodes",nodes},{"edges",edges},{"bindings",bindings}};
    auto maps = node->create_service<patrol_interfaces::srv::ListMaps>(
        "/patrol/list_maps",
        [map](const std::shared_ptr<patrol_interfaces::srv::ListMaps::Request>,
              std::shared_ptr<patrol_interfaces::srv::ListMaps::Response> response) {
          response->success = true; response->maps.push_back(map); response->active_map = map;
        });
    auto routes = node->create_service<patrol_interfaces::srv::ListRoutes>(
        "/patrol/list_routes",
        [&route](const std::shared_ptr<patrol_interfaces::srv::ListRoutes::Request>,
                std::shared_ptr<patrol_interfaces::srv::ListRoutes::Response> response) {
          response->success = true; response->routes.push_back(route);
        });
    Q_UNUSED(maps); Q_UNUSED(routes);
    auto points=node->create_service<patrol_interfaces::srv::ListPoints>("/patrol/list_points",
      [fixturePoints](const std::shared_ptr<patrol_interfaces::srv::ListPoints::Request>, std::shared_ptr<patrol_interfaces::srv::ListPoints::Response> response) {
        response->success=true;for(const auto &point:fixturePoints) response->points.push_back(point);
      });
    auto home=node->create_service<patrol_interfaces::srv::GetHomePoint>("/patrol/get_home_point",
      [](const std::shared_ptr<patrol_interfaces::srv::GetHomePoint::Request>, std::shared_ptr<patrol_interfaces::srv::GetHomePoint::Response> response) {
        response->success=true;response->exists=true;response->map_revision=1;response->x=3;response->y=3;
      });
    auto runs=node->create_service<patrol_interfaces::srv::GetRuns>("/patrol/get_runs",
      [route](const std::shared_ptr<patrol_interfaces::srv::GetRuns::Request>, std::shared_ptr<patrol_interfaces::srv::GetRuns::Response> response) {
        response->success=true;
        patrol_interfaces::msg::RunRecord run;
        run.run_id="test-run-20261019-01";run.route_id=route.route_id;run.map_id=route.map_id;
        run.state=8;run.completed_loops=2;run.completed_steps=8;run.success=true;run.message="巡检任务完成";
        response->runs.push_back(run);
      });
    Q_UNUSED(points);Q_UNUSED(home);Q_UNUSED(runs);
    QString fakeRecordingState="IDLE";QJsonObject lastSavedRoad;QString lastRoadOperation;
    int planRequests=0;
    auto roads=node->create_service<patrol_interfaces::srv::RoadCommand>("/patrol/road_command",
      [&route,&fakeRecordingState,&lastSavedRoad,&lastRoadOperation,&planRequests,&document,planBlocks](const std::shared_ptr<patrol_interfaces::srv::RoadCommand::Request> request,
               std::shared_ptr<patrol_interfaces::srv::RoadCommand::Response> response){
        const auto payload=QJsonDocument::fromJson(QByteArray::fromStdString(request->payload_json)).object();
        QJsonObject data;
        if(request->operation=="LIST") {
          data["networks"]=QJsonArray{
            QJsonObject{{"network_id","network-test"},{"name","厂区巡检路网"},{"revision",2},{"published",true},{"document",document}},
            QJsonObject{{"network_id","restore-test"},{"name","恢复草稿路网"},{"revision",7},{"published",true},{"document",QJsonObject{}}}};
        } else if(request->operation=="PLAN") {
          ++planRequests;
          data={{"route_id",QString::fromStdString(route.route_id)},{"route_revision",1},
            {"loop_count",payload["loop_count"]},{"digest","verified-digest"},{"network_id","network-test"},
            {"entry",QJsonObject{{"required",true},{"target_name","厂区巡检 · 点1"},{"length",1.2},{"direction_changes",2}}},
            {"legs",QJsonArray{QJsonObject{{"blocks",planBlocks},{"edge_ids",QJsonArray{"edge-0","edge-1"}}}}}};
        } else if(request->operation=="STATUS") data={{"state",fakeRecordingState},{"enabled",true},{"network_id","restore-test"}};
        else if(request->operation=="SAVE_RECORD") lastSavedRoad=payload;
        else if(request->operation=="START_PATH" || request->operation=="MARK_PATH_POINT" || request->operation=="SAVE_PATH") {
          lastRoadOperation=QString::fromStdString(request->operation);
          if(request->operation=="SAVE_PATH") {
            data={{"route_id",QString::fromStdString(route.route_id)},{"point_count",4},{"closed",false}};
          }
        }
        response->success=true;response->data_json=QJsonDocument(data).toJson(QJsonDocument::Compact).toStdString();
      });
    Q_UNUSED(roads);
    int goalsReceived = 0;QString receivedPlanDigest;
    auto createServer = [&] {
      return rclcpp_action::create_server<RunRoute>(
          node, "/patrol/run_route",
          [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const RunRoute::Goal> goal) {
            ++goalsReceived;
            receivedPlanDigest=QString::fromStdString(goal->plan_digest);
            return rclcpp_action::GoalResponse::REJECT;
          },
          [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunRoute>>) {
            return rclcpp_action::CancelResponse::REJECT;
          },
          [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunRoute>>) {});
    };
    auto server = createServer();
    QTimer spin;
    connect(&spin, &QTimer::timeout, this, [node] { rclcpp::spin_some(node); });
    spin.start(10);
    PatrolClient client;
    if (!client.available()) QSKIP("patrol_interfaces/robot_patrol is not built");
    QStringList missing{
        QStringLiteral("尚未启用操作，请勾选主窗口顶部“启用操作”"),
        QStringLiteral("急停保持中，请明确解除急停"),
        QStringLiteral("巡航联锁尚未就绪，请检查心跳")};
    PatrolPage page(&client, QString::fromStdString(map.navigation_yaml), "amcl", QString(), "AMCL 导航 · 测试数据", false,
                    [&missing](QString *reason) {
                      if (reason) *reason = missing.join('\n');
                      return missing.isEmpty();
                    });
    page.resize(1180, 760);
    page.show();
    auto *tabs = page.findChild<QTabWidget *>(QStringLiteral("patrolTabs"));
    auto *table = page.findChild<QTableWidget *>(QStringLiteral("patrolRoutesTable"));
    auto *start = page.findChild<QPushButton *>(QStringLiteral("patrolStart"));
    auto *readiness = page.findChild<QLabel *>(QStringLiteral("patrolStartReadiness"));
    auto *name = page.findChild<QLineEdit *>(QStringLiteral("patrolRouteName"));
    QVERIFY(tabs && table && start && readiness && name);
    tabs->setCurrentIndex(1);
    QVERIFY(QTest::qWaitForWindowExposed(&page));
    QTRY_VERIFY_WITH_TIMEOUT(client.routeServerReady(), 3000);
    // Let the constructor's initial discovery failure recover explicitly,
    // just as the operator's Refresh button does after backend startup.
    client.refreshMaps();
    QTRY_COMPARE_WITH_TIMEOUT(table->rowCount(), 1, 3000);
    table->selectRow(0);
    QTRY_VERIFY(readiness->text().contains(QStringLiteral("尚未启用操作")));
    QVERIFY(readiness->text().contains(QStringLiteral("急停保持中")));
    QVERIFY(readiness->text().contains(QStringLiteral("巡航联锁尚未就绪")));
    QVERIFY(!start->isEnabled());
    QVERIFY(readiness->isVisible());
    const QRect visibleBar(readiness->mapTo(&page, QPoint()), readiness->size());
    QVERIFY(page.rect().contains(visibleBar));

    missing.clear();
    QTRY_VERIFY(start->isEnabled());
    QVERIFY(readiness->text().contains(QStringLiteral("启动条件：已就绪")));
    QVERIFY(!readiness->text().contains(QStringLiteral("急停保持中")));

    // Losing the real action endpoint must remove the ready indication.
    server.reset();
    QTRY_VERIFY_WITH_TIMEOUT(!client.routeServerReady(), 5000);
    QTRY_VERIFY(!start->isEnabled());
    QVERIFY(readiness->text().contains(QStringLiteral("巡航后台未就绪")));
    server = createServer();
    QTRY_VERIFY_WITH_TIMEOUT(start->isEnabled(), 5000);
    // A stored fixed route cannot start until the complete road plan has
    // been previewed; changing loop count invalidates that preview.
    route.navigation_mode="RECORDED_NETWORK";route.network_id="network-test";route.network_revision=2;
    client.roadCommand("LIST");client.refreshRoutes(QString::fromStdString(map.map_id));
    auto *mode=page.findChild<QComboBox*>("patrolNavigationMode");
    auto *preview=page.findChild<QPushButton*>("patrolPreviewRecorded");
    auto *loops=page.findChild<QSpinBox*>("patrolLoopCount");
    QVERIFY(mode && preview && loops);
    QTRY_COMPARE(mode->currentData().toString(),QString("RECORDED_NETWORK"));
    QTRY_VERIFY(!start->isEnabled());QTRY_VERIFY(preview->isEnabled());
    preview->click();QTRY_VERIFY(start->isEnabled());
    QTRY_VERIFY(page.findChild<QLabel*>("patrolMessage")->text().contains("将自动接入 厂区巡检 · 点1"));
    QVERIFY(page.findChild<QLabel*>("patrolMessage")->text().contains("2 次前进/倒车换向"));
    loops->setValue(2);QTRY_VERIFY(!start->isEnabled());
    preview->click();QTRY_VERIFY(start->isEnabled());
    fakeRecordingState="REVIEW";client.roadCommand("STATUS");
    QTRY_VERIFY(!start->isEnabled());QTRY_VERIFY(!preview->isEnabled());
    QVERIFY(readiness->text().contains("录制草稿"));
    auto *roadNetworks=page.findChild<QComboBox*>("roadNetworks");
    QTRY_COMPARE(roadNetworks->currentData().toString(),QString("restore-test"));
    auto *saveRoad=page.findChild<QPushButton*>("roadSave");QTRY_VERIFY(saveRoad->isEnabled());saveRoad->click();
    QTRY_COMPARE(lastSavedRoad["expected_revision"].toInt(),7);
    QCOMPARE(lastSavedRoad["network_id"].toString(),QString("restore-test"));
    QVERIFY(!lastSavedRoad["owner_session_id"].toString().isEmpty());
    auto *advanced=page.findChild<QCheckBox *>("roadAdvancedSettings");
    QVERIFY(advanced && !advanced->isChecked());
    auto *savePath=page.findChild<QPushButton *>("roadSavePath");
    QVERIFY(savePath && !savePath->isHidden());
    QTRY_VERIFY(savePath->isEnabled());savePath->click();
    QTRY_COMPARE(lastRoadOperation,QString("SAVE_PATH"));
    fakeRecordingState="IDLE";client.roadCommand("STATUS");QTRY_VERIFY(preview->isEnabled());
    QVERIFY(!start->isEnabled());preview->click();QTRY_VERIFY(start->isEnabled());
    auto *mapSettings = page.findChild<QWidget *>("patrolMapSettings");
    auto *settingsToggle = page.findChild<QPushButton *>("patrolMapSettingsToggle");
    QVERIFY(mapSettings && settingsToggle && mapSettings->isHidden());
    settingsToggle->click();QVERIFY(mapSettings->isVisible());
    settingsToggle->click();QVERIFY(mapSettings->isHidden());
    QTest::qWait(30); // Let Qt invalidate the expanded panel's minimum size.
    advanced->setChecked(true);
    for(const auto size:{QSize(1100,680),QSize(1180,760),QSize(1366,768),QSize(1920,1080)}) {
      page.resize(size);
      for(int tab=0;tab<tabs->count();++tab) {
        tabs->setCurrentIndex(tab);QTest::qWait(30);
        if(page.size()!=size) {
          qWarning()<<"Page minimum"<<page.minimumSize()<<page.minimumSizeHint();
          for(auto *widget:page.findChildren<QWidget *>())
            if(widget->property("patrolCard").toBool() || qobject_cast<QTabWidget *>(widget) || widget->objectName()=="roadRecordingTab" || qobject_cast<QScrollArea *>(widget))
              qWarning()<<widget->objectName()<<widget->geometry()<<widget->minimumSize()<<widget->minimumSizeHint();
          page.grab().save("/tmp/patrol_layout_failure.png");
        }
        QCOMPARE(page.size(),size);
        QVERIFY(page.rect().contains(QRect(tabs->mapTo(&page,QPoint()),tabs->size())));
        if(tab==1) {
          for(auto *control : {static_cast<QWidget *>(start), static_cast<QWidget *>(readiness)})
            QVERIFY(page.rect().contains(QRect(control->mapTo(&page,QPoint()),control->size())));
        }
        if(tab==3) {
          auto *publish=page.findChild<QPushButton *>("roadPublish");
          auto *begin=page.findChild<QPushButton *>("roadBegin");
          auto *recordTab=page.findChild<QTabWidget *>("roadControlTabs")->widget(0);
          QVERIFY(publish && begin && recordTab);
          QVERIFY(page.rect().contains(QRect(publish->mapTo(&page,QPoint()),publish->size())));
          for(auto *control : {begin,page.findChild<QPushButton *>("roadSave"),page.findChild<QPushButton *>("roadFinish")})
            QVERIFY(recordTab->rect().contains(QRect(control->mapTo(recordTab,QPoint()),control->size())));
        }
        QVERIFY(page.grab().save(QString("/tmp/patrol_tab%1_%2.png").arg(tab).arg(size.width())));
      }
    }
    page.resize(1366,768);tabs->setCurrentIndex(3);
    auto *roadTabs=page.findChild<QTabWidget *>("roadControlTabs");
    roadTabs->setCurrentIndex(1);QTest::qWait(30);
    QVERIFY(page.findChild<QLabel *>("roadBindingSummary")->text().contains("出入口"));
    QVERIFY(page.grab().save("/tmp/patrol_road_bindings.png"));
    page.findChild<QTableWidget *>("roadEdgesTable")->selectRow(0);
    roadTabs->setCurrentIndex(2);QTest::qWait(30);
    QVERIFY(page.grab().save("/tmp/patrol_road_management.png"));
    roadTabs->setCurrentIndex(0);
    auto *view=page.findChild<QGraphicsView *>("roadPreview");
    const auto beforeZoom=view->transform().m11();
    page.findChild<QPushButton *>("roadZoomIn")->click();
    QVERIFY(view->transform().m11()>beforeZoom);
    page.findChild<QPushButton *>("roadFitView")->click();
    QVERIFY(qAbs(view->transform().m11()-beforeZoom)<.001);
    page.resize(1180,760);tabs->setCurrentIndex(1);
    QCOMPARE(goalsReceived,0);
    client.setOperatorAuthorized(true);start->click();
    QTRY_COMPARE(goalsReceived,1);QCOMPARE(receivedPlanDigest,QString("verified-digest"));
    QTRY_VERIFY(!client.hasActiveOrPendingRun());
    // A teaching path starts from one explicit click; PLAN happens internally.
    QJsonObject pointIds;
    for(int i=0;i<4;++i) pointIds[QString("node-%1").arg(i)]=QString("point-%1").arg(i);
    document["inspection_path"]=QJsonObject{{"route_id",QString::fromStdString(route.route_id)},
      {"closed",false},{"point_ids",pointIds},{"edge_ids",QJsonArray{"edge-0","edge-1","edge-2"}}};
    client.roadCommand("LIST");
    QTRY_VERIFY(!client.roadNetworks().first().toObject()["document"].toObject()["inspection_path"].toObject().isEmpty());
    loops->setValue(3);QTRY_VERIFY(start->isEnabled());
    const int previousPlans=planRequests;
    start->click();
    QTRY_COMPARE(planRequests,previousPlans+1);
    QTRY_COMPARE(goalsReceived,2);
    QTRY_VERIFY(!client.hasActiveOrPendingRun());
    // The default recording view also fits compact screens, with no manual
    // point binding or publication controls visible.
    advanced->setChecked(false);tabs->setCurrentIndex(3);page.resize(1100,680);QTest::qWait(30);
    QVERIFY(page.findChild<QPushButton *>("roadPublish")->isHidden());
    QVERIFY(page.rect().contains(QRect(savePath->mapTo(&page,QPoint()),savePath->size())));
    QVERIFY(page.grab().save("/tmp/patrol_simple_recording.png"));
    auto *beginPath=page.findChild<QPushButton *>("roadBegin");
    QTRY_VERIFY(beginPath->isEnabled());beginPath->click();QTRY_COMPARE(lastRoadOperation,QString("START_PATH"));
    name->setText(QStringLiteral("修改后的路线"));
    QTRY_VERIFY(!start->isEnabled());
    QVERIFY(readiness->text().contains(QStringLiteral("未保存修改")));
    QCOMPARE(goalsReceived, 2); // Only explicit Start clicks send goals.
    QTimer::singleShot(0, [] {
      if(auto *box=qobject_cast<QMessageBox *>(QApplication::activeModalWidget())) box->done(QMessageBox::Cancel);
    });
    page.close();
    QVERIFY(page.isVisible());
    QTimer::singleShot(0, [] {
      if(auto *box=qobject_cast<QMessageBox *>(QApplication::activeModalWidget())) box->done(QMessageBox::Discard);
    });
    page.close();
  }
};

QTEST_MAIN(PatrolPageTest)
#include "patrol_page_test.moc"
