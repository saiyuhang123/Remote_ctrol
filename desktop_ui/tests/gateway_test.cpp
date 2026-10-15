#include "adapters/gateway_client.h"
#include "runtime/config.h"
#include <QFile>
#include <QJsonDocument>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QTest>
#include <QWebSocketServer>
#include <limits>
#include <cmath>
#include <stdexcept>

class GatewayTest : public QObject {
  Q_OBJECT
  QWebSocketServer *server_ = nullptr;
  GatewayClient *client_ = nullptr;
  QWebSocket *peer_ = nullptr;
  QVector<QJsonObject> messages_;
private slots:
  void init() {
    server_ = new QWebSocketServer("isolated-desktop-test", QWebSocketServer::NonSecureMode, this);
    QVERIFY(server_->listen(QHostAddress::LocalHost, 0));
    connect(server_, &QWebSocketServer::newConnection, this, [this] {
      peer_ = server_->nextPendingConnection();
      connect(peer_, &QWebSocket::textMessageReceived, this, [this](const QString &s) { messages_ << QJsonDocument::fromJson(s.toUtf8()).object(); });
    });
    client_ = new GatewayClient(.3, .5, this);
    client_->connectTo(QUrl("ws://127.0.0.1:" + QString::number(server_->serverPort())));
    QTRY_VERIFY(client_->connected() && peer_);
  }
  void cleanup() { delete client_; client_ = nullptr; server_->close(); delete server_; server_ = nullptr; peer_ = nullptr; messages_.clear(); }
  void observerNeverWritesOrReplays() {
    QTest::qWait(120); QCOMPARE(messages_.size(), 0);
    QVERIFY(!client_->capture()); QVERIFY(!client_->initialPose(1, 2, 0));
    QVERIFY(!client_->move(.2, .2)); QVERIFY(!client_->cancelNavigation());
    QTest::qWait(100); QCOMPARE(messages_.size(), 0);
    client_->setControlEnabled(true); client_->disconnectFrom();
    QTRY_VERIFY(!client_->connected());
    client_->connectTo(QUrl("ws://127.0.0.1:" + QString::number(server_->serverPort())));
    QTRY_VERIFY(client_->connected()); QVERIFY(!client_->controlEnabled());
    QTest::qWait(100); QCOMPARE(messages_.size(), 0);
  }
  void manualRequiresCancelStopAndExplicitRelease() {
    client_->setControlEnabled(true); client_->updateFeedback(false, false, true, true);
    QVERIFY(client_->takeManualControl());
    QTRY_COMPARE(messages_.size(), 1); QCOMPARE(messages_.last().value("type").toString(), QString("estop"));
    client_->updateFeedback(true, true, true, true);
    QTest::qWait(350); QVERIFY(!client_->manualReady());
    client_->updateFeedback(true, false, true, true);
    QTRY_VERIFY(client_->manualReady()); QVERIFY(!client_->move(.2, 0));
    QVERIFY(client_->releaseStop()); QTRY_COMPARE(messages_.size(), 2);
    QCOMPARE(messages_.last().value("linear").toDouble(), 0.);
    client_->updateFeedback(false, false, true, true);
    QVERIFY(client_->move(.9, 2.));
    QTRY_VERIFY(messages_.size() >= 3);
    QCOMPARE(messages_.last().value("linear").toDouble(), .3);
    QCOMPARE(messages_.last().value("angular").toDouble(), .5);
    client_->stopManual();
    QTRY_COMPARE(messages_.last().value("linear").toDouble(), 0.);
    const int count = messages_.size(); QTest::qWait(180); QCOMPARE(messages_.size(), count);
  }
  void staleFeedbackStopsAndDoesNotClearStop() {
    client_->setControlEnabled(true); client_->updateFeedback(true, false, true, true);
    QVERIFY(client_->takeManualControl()); QTRY_VERIFY(client_->manualReady());
    QVERIFY(client_->releaseStop()); client_->updateFeedback(false, false, true, true);
    QVERIFY(client_->move(-.2, -.2)); QTest::qWait(80);
    client_->updateFeedback(false, false, true, false);
    QTRY_COMPARE(messages_.last().value("type").toString(), QString("estop"));
    QVERIFY(!client_->moving()); QVERIFY(!client_->manualReady());
    const int count = messages_.size(); QTest::qWait(150); QCOMPARE(messages_.size(), count);
    QVERIFY(!client_->releaseStop());
  }
  void goalCoordinatesAreFiniteAndYawIsRadians() {
    client_->setControlEnabled(true); client_->updateFeedback(false, false, true, true);
    QVERIFY(!client_->navigate(std::numeric_limits<double>::quiet_NaN(), 0, 0));
    QVERIFY(client_->navigate(1.25, -2.5, 7.));
    QTRY_COMPARE(messages_.size(), 1);
    QCOMPARE(messages_.last().value("x").toDouble(), 1.25);
    QCOMPARE(messages_.last().value("y").toDouble(), -2.5);
    QVERIFY(std::abs(messages_.last().value("yaw").toDouble() - .71681469282) < 1e-8);
  }
  void stoppedRosThreadCannotKeepRepeatingMotion() {
    client_->setControlEnabled(true); client_->updateFeedback(true, false, true, true);
    QVERIFY(client_->takeManualControl()); QTRY_VERIFY(client_->manualReady());
    QVERIFY(client_->releaseStop()); client_->updateFeedback(false, false, true, true);
    QVERIFY(client_->move(.1, 0));
    QTRY_VERIFY_WITH_TIMEOUT(!client_->moving(), 2500);
    QTRY_COMPARE(messages_.last().value("type").toString(), QString("estop"));
    const auto count = messages_.size(); QTest::qWait(120); QCOMPARE(messages_.size(), count);
  }
  void configRejectsInvalidLimits() {
    QTemporaryDir d; QFile f(d.filePath("desktop.json")); QVERIFY(f.open(QIODevice::WriteOnly));
    f.write("{\"max_linear\":-1}"); f.close();
    QVERIFY_EXCEPTION_THROWN(DesktopConfig::load(f.fileName(), d.path(), d.path()), std::runtime_error);
  }
};
QTEST_GUILESS_MAIN(GatewayTest)
#include "gateway_test.moc"
