#include "ui/mainwindow.h"
#include "ui/map_editor_dialog.h"
#include "runtime/config.h"

#include <QAction>
#include <QDir>
#include <QFile>
#include <QLineEdit>
#include <QTemporaryDir>
#include <QTest>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class UiLifecycleTest : public QObject {
  Q_OBJECT
private slots:
  void initTestCase() {
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  }
  void cleanupTestCase() { rclcpp::shutdown(); }

  void closingMainWindowWithOpenEditorDoesNotTouchDestroyedMembers() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString imagePath = directory.filePath(QStringLiteral("map.pgm"));
    const QString yamlPath = directory.filePath(QStringLiteral("map.yaml"));
    {
      QFile image(imagePath);
      QVERIFY(image.open(QIODevice::WriteOnly));
      QVERIFY(image.write(QByteArray("P5\n4 3\n255\n") + QByteArray(12, char(205))) == 23);
      QFile yaml(yamlPath);
      QVERIFY(yaml.open(QIODevice::WriteOnly));
      QVERIFY(yaml.write("image: map.pgm\nmode: trinary\nresolution: 0.05\n"
                         "origin: [-1.25, 3.5, 0.0]\n"
                         "negate: 0\nfree_thresh: 0.25\noccupied_thresh: 0.65\n") > 0);
    }

    const QString root = DESKTOP_UI_SOURCE_DIR;
    auto config = DesktopConfig::load(root + "/config/desktop.json", root,
                                      QDir(root).absoluteFilePath(".."));
    config.backendUrl = "http://127.0.0.1:9";
    auto window = std::make_unique<MainWindow>(config, true);
    window->show();
    QVERIFY(QTest::qWaitForWindowExposed(window.get()));
    auto *mapPrefix = window->findChild<QLineEdit *>(QStringLiteral("mapPrefix"));
    auto *editorAction = window->findChild<QAction *>(QStringLiteral("mapEditorAction"));
    QVERIFY(mapPrefix && editorAction);
    mapPrefix->setText(directory.filePath(QStringLiteral("map")));
    editorAction->trigger();
    QTRY_VERIFY(window->findChild<MapEditorDialog *>() != nullptr);
    QVERIFY(window->findChild<MapEditorDialog *>()->isVisible());

    // Deliberately destroy the main window while the editor is still its child.
    // The editor's destroyed callback must not run after MainWindow's QString
    // members have been destroyed.
    window.reset();
    QTest::qWait(50);
  }
};

QTEST_MAIN(UiLifecycleTest)
#include "ui_lifecycle_test.moc"
