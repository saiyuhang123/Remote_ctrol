#include "map_editor/map_document.h"
#include "map_editor/map_io.h"
#include "ui/map_editor_dialog.h"
#include "ui/widgets/map_canvas.h"

#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QPushButton>
#include <QTemporaryDir>
#include <QTest>
#include <vector>

namespace {
bool writeBytes(const QString &path, const QByteArray &data) {
  QFile file(path);
  return file.open(QIODevice::WriteOnly) && file.write(data) == data.size();
}

QString writeSyntheticMap(const QString &directory) {
  QDir().mkpath(directory);
  const QByteArray pixels(12, char(205));
  const QByteArray pgm = QByteArray("P5\n4 3\n255\n") + pixels;
  const QString image = QDir(directory).absoluteFilePath("map.pgm");
  if (!writeBytes(image, pgm)) return {};
  const QByteArray yaml =
      "image: map.pgm\nmode: trinary\nresolution: 0.05\n"
      "origin: [-1.25, 3.5, 0.0]\n"
      "negate: 0\nfree_thresh: 0.25\noccupied_thresh: 0.65\n";
  const QString yamlPath = QDir(directory).absoluteFilePath("map.yaml");
  return writeBytes(yamlPath, yaml) ? yamlPath : QString();
}
}

class MapEditorUiTest : public QObject {
  Q_OBJECT
private slots:
  void mousePaintUnknownAndUndoRedo() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString yaml = writeSyntheticMap(directory.path());
    QVERIFY(!yaml.isEmpty());

    MapEditorDialog dialog(yaml);
    dialog.resize(800, 600);
    dialog.show();
    QVERIFY(QTest::qWaitForWindowExposed(&dialog));
    auto *canvas = dialog.findChild<MapCanvas *>(QStringLiteral("mapCanvas"));
    QVERIFY(canvas != nullptr);
    QVERIFY(dialog.document() != nullptr);
    canvas->fitToView();

    const QPointF topLeft = canvas->cellToScreen(1, 1);
    const QPointF next = canvas->cellToScreen(2, 2);
    const QPointF center = (topLeft + next) / 2.0;
    const QPointF center2 = canvas->cellToScreen(3, 3) + QPointF(1, 1);

    QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier, center.toPoint());
    QTest::mouseMove(canvas, center2.toPoint(), 20);
    QTest::mouseRelease(canvas, Qt::LeftButton, Qt::NoModifier, center2.toPoint());
    QTRY_VERIFY(dialog.isDirty());
    QVERIFY(dialog.document()->pixel(1, 1) == 0);

    auto *undo = dialog.findChild<QPushButton *>(QStringLiteral("mapEditorUndo"));
    auto *redo = dialog.findChild<QPushButton *>(QStringLiteral("mapEditorRedo"));
    QVERIFY(undo && redo);
    QVERIFY(undo->isEnabled());
    QTest::mouseClick(undo, Qt::LeftButton);
    QVERIFY(dialog.document()->pixel(1, 1) == 205);
    QVERIFY(redo->isEnabled());
    QTest::mouseClick(redo, Qt::LeftButton);
    QVERIFY(dialog.document()->pixel(1, 1) == 0);
    QVERIFY(dialog.isDirty());

    auto *tool = dialog.findChild<QComboBox *>(QStringLiteral("mapEditorTool"));
    QVERIFY(tool != nullptr);
    tool->setCurrentIndex(2); // Unknown
    const QPointF unknownCenter = (canvas->cellToScreen(0, 0) + canvas->cellToScreen(1, 1)) / 2.0;
    QTest::mouseClick(canvas, Qt::LeftButton, Qt::NoModifier, unknownCenter.toPoint());
    const uint8_t value = dialog.document()->pixel(0, 0);
    QVERIFY(value >= 90 && value <= 191);
    QCOMPARE(dialog.document()->semanticValue(value), -1);

    tool->setCurrentIndex(3); // obstacle line
    const QPointF cellStep = canvas->cellToScreen(1, 1) - canvas->cellToScreen(0, 0);
    const QPointF lineStart = canvas->cellToScreen(0, 2) + cellStep / 2.0;
    const QPointF lineEnd = canvas->cellToScreen(3, 2) + cellStep / 2.0;
    QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier, lineStart.toPoint());
    QTest::mouseMove(canvas, lineEnd.toPoint(), 20);
    QTest::mouseRelease(canvas, Qt::LeftButton, Qt::NoModifier, lineEnd.toPoint());
    QCOMPARE(dialog.document()->pixel(0, 2), uint8_t(0));
    QCOMPARE(dialog.document()->pixel(3, 2), uint8_t(0));
    QVERIFY(dialog.isDirty());
  }

  void savesEditedMapAndMarksClean() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString yaml = writeSyntheticMap(directory.path());
    QVERIFY(!yaml.isEmpty());

    MapEditorDialog dialog(yaml);
    dialog.resize(640, 480);
    dialog.show();
    QVERIFY(QTest::qWaitForWindowExposed(&dialog));
    auto *canvas = dialog.findChild<MapCanvas *>(QStringLiteral("mapCanvas"));
    QVERIFY(canvas != nullptr);
    canvas->fitToView();

    const QPointF center = (canvas->cellToScreen(1, 1) + canvas->cellToScreen(2, 2)) / 2.0;
    QTest::mouseClick(canvas, Qt::LeftButton, Qt::NoModifier, center.toPoint());
    QVERIFY(dialog.isDirty());

    const QString target = directory.filePath("edited.yaml");
    QString error;
    QVERIFY2(dialog.saveToPath(target, &error), qPrintable(error));
    QVERIFY(!dialog.isDirty());
    QVERIFY(QFileInfo::exists(target));

    MapDocument reloaded;
    QVERIFY2(MapIo::load(target, &reloaded, &error), qPrintable(error));
    QCOMPARE(reloaded.pixel(1, 1), uint8_t(0));
  }
};

QTEST_MAIN(MapEditorUiTest)
#include "map_editor_ui_test.moc"
