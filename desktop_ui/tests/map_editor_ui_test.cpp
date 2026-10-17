#include "map_editor/map_document.h"
#include "map_editor/map_io.h"
#include "ui/map_editor_dialog.h"
#include "ui/widgets/map_canvas.h"

#include <QApplication>
#include <QClipboard>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QLabel>
#include <QMessageBox>
#include <QRegularExpression>
#include <QShortcut>
#include <QMouseEvent>
#include <QPushButton>
#include <QSpinBox>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
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

QPushButton *button(MapEditorDialog *dialog, const QString &name) {
  return dialog->findChild<QPushButton *>(name);
}

void selectTool(MapEditorDialog *dialog, const QString &shapeName, const QString &cellName) {
  auto *shape = button(dialog, shapeName);
  auto *cell = button(dialog, cellName);
  QVERIFY(shape && cell);
  if (!shape->isChecked()) shape->click();
  if (!cell->isChecked()) cell->click();
}

QPointF cellCenter(MapCanvas *canvas, int u, int v) {
  return canvas->cellToScreen(u, v) +
         QPointF(canvas->viewScale() / 2.0, canvas->viewScale() / 2.0);
}

void clickCell(MapCanvas *canvas, int u, int v) {
  QTest::mouseClick(canvas, Qt::LeftButton, Qt::NoModifier, cellCenter(canvas, u, v).toPoint());
}

void dragLine(MapCanvas *canvas, int u0, int v0, int u1, int v1) {
  QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier, cellCenter(canvas, u0, v0).toPoint());
  QTest::mouseRelease(canvas, Qt::LeftButton, Qt::NoModifier, cellCenter(canvas, u1, v1).toPoint());
}

void sendHover(MapCanvas *canvas, const QPointF &point) {
  QMouseEvent event(QEvent::MouseMove, point, Qt::NoButton, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(canvas, &event);
}
}  // namespace

class MapEditorUiTest : public QObject {
  Q_OBJECT
private slots:
  void autoFitManualViewAndStatusSeparation() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString yaml = writeSyntheticMap(directory.path());
    QVERIFY(!yaml.isEmpty());

    MapEditorDialog dialog(yaml);
    dialog.show();
    QVERIFY(QTest::qWaitForWindowExposed(&dialog));
    auto *canvas = dialog.findChild<MapCanvas *>(QStringLiteral("mapCanvas"));
    QVERIFY(canvas && dialog.document());
    QTRY_VERIFY(canvas->autoFitEnabled());
    QTest::qWait(30);
    QVERIFY(canvas->cellToScreen(0, 0).x() >= 0.0);
    QVERIFY(canvas->cellToScreen(0, 0).y() >= 0.0);
    QVERIFY(canvas->cellToScreen(dialog.document()->width(), dialog.document()->height()).x() <=
            canvas->width() + 1.0);
    QVERIFY(canvas->cellToScreen(dialog.document()->width(), dialog.document()->height()).y() <=
            canvas->height() + 1.0);

    auto *operation = dialog.findChild<QLabel *>(QStringLiteral("mapEditorOperationMessage"));
    auto *cursor = dialog.findChild<QLabel *>(QStringLiteral("mapEditorCursor"));
    QVERIFY(operation && cursor);
    const QString openedMessage = operation->text();
    QVERIFY(openedMessage.contains(QStringLiteral("已打开")));
    sendHover(canvas, cellCenter(canvas, 1, 1));
    QVERIFY(cursor->text().contains(QStringLiteral("x=")));
    QVERIFY(QRegularExpression(QStringLiteral("x=-?\\d+\\.\\d{2} m"))
                .match(cursor->text())
                .hasMatch());
    QCOMPARE(operation->text(), openedMessage);
    dialog.resize(760, 480);
    QTest::qWait(80);
    auto *statusPanel = dialog.findChild<QWidget *>(QStringLiteral("mapEditorStatusBar"));
    auto *toolSummary = dialog.findChild<QLabel *>(QStringLiteral("mapEditorToolSummary"));
    QVERIFY(statusPanel && toolSummary && cursor);
    QVERIFY(!toolSummary->text().isEmpty());
    QVERIFY(toolSummary->toolTip().contains(QStringLiteral("直径")));
    QVERIFY(toolSummary->geometry().right() <= statusPanel->width());
    QVERIFY(cursor->geometry().right() <= statusPanel->width());
    QVERIFY(QRegularExpression(QStringLiteral("x=-?\\d+\\.\\d{2} m"))
                .match(cursor->text())
                .hasMatch());
    QEvent leaveEvent(QEvent::Leave);
    QApplication::sendEvent(canvas, &leaveEvent);
    QCOMPARE(cursor->text(), QStringLiteral("光标：—"));
    QCOMPARE(operation->text(), openedMessage);

    auto *infoPanel = dialog.findChild<QWidget *>(QStringLiteral("mapEditorInfoPanel"));
    auto *infoToggle = button(&dialog, QStringLiteral("mapEditorInfoToggle"));
    QVERIFY(infoPanel && infoToggle);
    const bool infoVisible = infoPanel->isVisible();
    infoToggle->click();
    QCOMPARE(infoPanel->isVisible(), !infoVisible);
    infoToggle->click();
    auto *copySourceYaml = button(&dialog, QStringLiteral("mapEditorCopySourceYaml"));
    QVERIFY(copySourceYaml != nullptr);
    copySourceYaml->click();
    QCOMPARE(QApplication::clipboard()->text(), dialog.document()->metadata().sourceYamlPath);

    const double fitScale = canvas->viewScale();
    canvas->zoomIn();
    QVERIFY(!canvas->autoFitEnabled());
    QVERIFY(canvas->viewScale() > fitScale);
    const double manualScale = canvas->viewScale();
    dialog.resize(dialog.width() + 80, dialog.height() + 20);
    QTest::qWait(30);
    QCOMPARE(canvas->viewScale(), manualScale);
    auto *fit = button(&dialog, QStringLiteral("mapEditorFit"));
    QVERIFY(fit != nullptr);
    fit->click();
    QTRY_VERIFY(canvas->autoFitEnabled());
  }

  void toolGroupsWriteSixSemanticsAndShortcutsWork() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString yaml = writeSyntheticMap(directory.path());
    QVERIFY(!yaml.isEmpty());

    MapEditorDialog dialog(yaml);
    dialog.show();
    QVERIFY(QTest::qWaitForWindowExposed(&dialog));
    auto *canvas = dialog.findChild<MapCanvas *>(QStringLiteral("mapCanvas"));
    auto *brush = dialog.findChild<QSpinBox *>(QStringLiteral("mapEditorBrush"));
    auto *summary = dialog.findChild<QLabel *>(QStringLiteral("mapEditorBrushSummary"));
    QVERIFY(canvas && brush && summary);
    brush->setValue(2);
    QVERIFY(summary->text().contains(QStringLiteral("5 格")));
    QVERIFY(summary->text().contains(QStringLiteral("0.25 m")));

    brush->setValue(0);
    selectTool(&dialog, QStringLiteral("mapEditorShapeBrush"), QStringLiteral("mapEditorCellObstacle"));
    clickCell(canvas, 0, 0);
    QCOMPARE(dialog.document()->semanticValue(dialog.document()->pixel(0, 0)), 100);

    selectTool(&dialog, QStringLiteral("mapEditorShapeBrush"), QStringLiteral("mapEditorCellFree"));
    clickCell(canvas, 1, 0);
    QCOMPARE(dialog.document()->semanticValue(dialog.document()->pixel(1, 0)), 0);

    selectTool(&dialog, QStringLiteral("mapEditorShapeBrush"), QStringLiteral("mapEditorCellUnknown"));
    clickCell(canvas, 2, 0);
    QCOMPARE(dialog.document()->semanticValue(dialog.document()->pixel(2, 0)), -1);

    selectTool(&dialog, QStringLiteral("mapEditorShapeLine"), QStringLiteral("mapEditorCellUnknown"));
    dragLine(canvas, 0, 0, 2, 0);
    QCOMPARE(dialog.document()->semanticValue(dialog.document()->pixel(0, 0)), -1);
    QCOMPARE(dialog.document()->semanticValue(dialog.document()->pixel(2, 0)), -1);

    selectTool(&dialog, QStringLiteral("mapEditorShapeLine"), QStringLiteral("mapEditorCellFree"));
    dragLine(canvas, 0, 1, 2, 1);
    QCOMPARE(dialog.document()->semanticValue(dialog.document()->pixel(0, 1)), 0);
    QCOMPARE(dialog.document()->semanticValue(dialog.document()->pixel(2, 1)), 0);

    selectTool(&dialog, QStringLiteral("mapEditorShapeLine"), QStringLiteral("mapEditorCellObstacle"));
    dragLine(canvas, 0, 2, 2, 2);
    QCOMPARE(dialog.document()->semanticValue(dialog.document()->pixel(0, 2)), 100);
    QCOMPARE(dialog.document()->semanticValue(dialog.document()->pixel(2, 2)), 100);

    // Keyboard undo/redo is scoped to the editor and does not write data.
    canvas->setFocus();
    const auto beforeUndo = dialog.document()->pixels();
    clickCell(canvas, 3, 0);
    QVERIFY(dialog.document()->pixels() != beforeUndo);
    QTest::keyClick(canvas, Qt::Key_Z, Qt::ControlModifier);
    QVERIFY(dialog.document()->pixels() == beforeUndo);
    QTest::keyClick(canvas, Qt::Key_Y, Qt::ControlModifier);
    QVERIFY(dialog.document()->pixels() != beforeUndo);
    QTest::keyClick(canvas, Qt::Key_Z, Qt::ControlModifier);
    QVERIFY(dialog.document()->pixels() == beforeUndo);
    QTest::keyClick(canvas, Qt::Key_Z, Qt::ControlModifier | Qt::ShiftModifier);
    QVERIFY(dialog.document()->pixels() != beforeUndo);

    // Text/number input focus must not turn Ctrl+Z into a map undo.
    const auto beforeSpinUndo = dialog.document()->pixels();
    brush->setFocus();
    QTest::keyClick(brush, Qt::Key_Z, Qt::ControlModifier);
    QVERIFY(dialog.document()->pixels() == beforeSpinUndo);

    // F enters fitted view after a manual zoom.
    canvas->zoomIn();
    QVERIFY(!canvas->autoFitEnabled());
    canvas->setFocus();
    QTest::keyClick(canvas, Qt::Key_F);
    QTRY_VERIFY(canvas->autoFitEnabled());
  }

  void escapeOnDirtyDocumentPromptsBeforeClosing() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString yaml = writeSyntheticMap(directory.path());
    QVERIFY(!yaml.isEmpty());

    MapEditorDialog dialog(yaml);
    dialog.show();
    QVERIFY(QTest::qWaitForWindowExposed(&dialog));
    auto *canvas = dialog.findChild<MapCanvas *>(QStringLiteral("mapCanvas"));
    QVERIFY(canvas != nullptr);
    clickCell(canvas, 1, 1);
    QVERIFY(dialog.isDirty());

    bool promptShown = false;
    QTimer::singleShot(100, [&promptShown] {
      auto *message = qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
      if (message) {
        promptShown = true;
        if (auto *discard = message->button(QMessageBox::Discard)) discard->click();
      }
    });
    QTest::keyClick(&dialog, Qt::Key_Escape);
    QTest::qWait(150);
    QVERIFY(promptShown);
    QVERIFY(!dialog.isVisible());
  }

  void escapeCancelsLinePreviewWithoutClosing() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString yaml = writeSyntheticMap(directory.path());
    QVERIFY(!yaml.isEmpty());

    MapEditorDialog dialog(yaml);
    dialog.show();
    QVERIFY(QTest::qWaitForWindowExposed(&dialog));
    auto *canvas = dialog.findChild<MapCanvas *>(QStringLiteral("mapCanvas"));
    QVERIFY(canvas != nullptr);
    clickCell(canvas, 1, 1);
    QVERIFY(dialog.isDirty());

    selectTool(&dialog, QStringLiteral("mapEditorShapeLine"), QStringLiteral("mapEditorCellObstacle"));
    const uint8_t before = dialog.document()->pixel(2, 1);
    QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier, cellCenter(canvas, 2, 1).toPoint());
    QTest::keyClick(&dialog, Qt::Key_Escape);
    QVERIFY(dialog.isVisible());
    QCOMPARE(dialog.document()->pixel(2, 1), before);
  }

  void saveAsShortcutSharesButtonEnabledState() {
    MapEditorDialog dialog{QString()};
    dialog.show();
    QVERIFY(QTest::qWaitForWindowExposed(&dialog));
    auto *saveAs = button(&dialog, QStringLiteral("mapEditorSaveAs"));
    auto *shortcut = dialog.findChild<QShortcut *>(QStringLiteral("mapEditorSaveAsShortcut"));
    QVERIFY(saveAs && shortcut);
    QVERIFY(!saveAs->isEnabled());
    QVERIFY(!shortcut->isEnabled());

    bool modalShown = false;
    QTimer::singleShot(100, [&modalShown] {
      if (auto *modal = QApplication::activeModalWidget()) {
        modalShown = true;
        modal->close();
      }
    });
    QTest::keyClick(&dialog, Qt::Key_S, Qt::ControlModifier | Qt::ShiftModifier);
    QTest::qWait(150);
    QVERIFY(!modalShown);

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString yaml = writeSyntheticMap(directory.path());
    QVERIFY(!yaml.isEmpty());
    QString error;
    QVERIFY2(dialog.loadMap(yaml, &error), qPrintable(error));
    QVERIFY(saveAs->isEnabled());
    QVERIFY(shortcut->isEnabled());
  }

  void savesEditedMapAndMarksClean() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString yaml = writeSyntheticMap(directory.path());
    QVERIFY(!yaml.isEmpty());

    MapEditorDialog dialog(yaml);
    dialog.show();
    QVERIFY(QTest::qWaitForWindowExposed(&dialog));
    auto *canvas = dialog.findChild<MapCanvas *>(QStringLiteral("mapCanvas"));
    QVERIFY(canvas != nullptr);
    clickCell(canvas, 1, 1);
    QVERIFY(dialog.isDirty());

    const QString target = directory.filePath("edited.yaml");
    QString error;
    QVERIFY2(dialog.saveToPath(target, &error), qPrintable(error));
    QVERIFY(!dialog.isDirty());
    auto *nav = dialog.findChild<QLabel *>(QStringLiteral("mapEditorNavigationSelection"));
    QVERIFY(nav != nullptr);
    QVERIFY(nav->text().contains(QStringLiteral("未自动选用")));

    MapDocument reloaded;
    QVERIFY2(MapIo::load(target, &reloaded, &error), qPrintable(error));
    QCOMPARE(reloaded.semanticValue(reloaded.pixel(1, 1)), 100);
  }
};

QTEST_MAIN(MapEditorUiTest)
#include "map_editor_ui_test.moc"
