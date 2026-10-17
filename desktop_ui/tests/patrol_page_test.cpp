#include "patrol/patrol_client.h"
#include "patrol/patrol_page.h"

#include <QDir>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QTest>
#include <rclcpp/rclcpp.hpp>

class PatrolPageTest : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() {
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  }
  void cleanupTestCase() { rclcpp::shutdown(); }

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
    QCOMPARE(tabs->count(), 3);
    QVERIFY(registerMap->isEnabled());
    QVERIFY(!saveCurrent->isEnabled()); // no active map yet
    QVERIFY(!start->isEnabled());
    QTest::qWait(300);
    page.close();
  }
};

QTEST_MAIN(PatrolPageTest)
#include "patrol_page_test.moc"
