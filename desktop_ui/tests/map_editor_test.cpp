#include "map_editor/map_document.h"
#include "map_editor/map_edit_command.h"
#include "map_editor/map_io.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <QTest>
#include <cmath>
#include <vector>

namespace {
bool writeBytes(const QString &path, const QByteArray &data) {
  QFile file(path);
  return file.open(QIODevice::WriteOnly) && file.write(data) == data.size();
}

QByteArray readBytes(const QString &path) {
  QFile file(path);
  return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray();
}

QByteArray pgmHeader(int width, int height) {
  return "P5\n" + QByteArray::number(width) + " " + QByteArray::number(height) + "\n255\n";
}

MapMetadata metadataFor(int width, int height, bool negate = false,
                        double freeThresh = 0.25, double occupiedThresh = 0.65) {
  MapMetadata metadata;
  metadata.width = width;
  metadata.height = height;
  metadata.resolution = 0.05;
  metadata.originX = -1.25;
  metadata.originY = 3.5;
  metadata.originYaw = 0.0;
  metadata.negate = negate;
  metadata.freeThresh = freeThresh;
  metadata.occupiedThresh = occupiedThresh;
  return metadata;
}

QString writeSyntheticMap(const QString &directory, const QString &stem,
                          const std::vector<uint8_t> &pixels, int width, int height) {
  QDir().mkpath(directory);
  const QString imagePath = QDir(directory).absoluteFilePath(stem + ".pgm");
  if (!writeBytes(imagePath, pgmHeader(width, height) +
                                QByteArray(reinterpret_cast<const char *>(pixels.data()),
                                           static_cast<int>(pixels.size())))) {
    return {};
  }
  const QString yamlPath = QDir(directory).absoluteFilePath(stem + ".yaml");
  const QByteArray yaml =
      "image: " + stem.toUtf8() +
      ".pgm\nmode: trinary\nresolution: 0.05\n"
      "origin: [-1.25, 3.5, 0.0]\n"
      "negate: 0\nfree_thresh: 0.25\noccupied_thresh: 0.65\ncustom_field: keep_me\n";
  return writeBytes(yamlPath, yaml) ? yamlPath : QString();
}

bool samePixels(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b) {
  return a == b;
}
}  // namespace

class MapEditorTest : public QObject {
  Q_OBJECT
private slots:
  void semanticsMatchNav2Thresholds() {
    MapMetadata metadata = metadataFor(1, 1);
    MapDocument document(metadata, {0});
    QVERIFY(document.valid());
    QCOMPARE(document.semanticValue(0), 100);
    QCOMPARE(document.semanticValue(89), 100);
    QCOMPARE(document.semanticValue(90), -1);
    QCOMPARE(document.semanticValue(191), -1);
    QCOMPARE(document.semanticValue(192), 0);
    QCOMPARE(document.semanticValue(205), 0);
    QCOMPARE(document.semanticValue(254), 0);
    QCOMPARE(document.semanticValue(255), 0);

    uint8_t value = 0;
    QString error;
    QVERIFY2(document.encodingForState(MapCellState::Unknown, &value, &error), qPrintable(error));
    QVERIFY(value >= 90 && value <= 191);
    QCOMPARE(document.semanticValue(value), -1);
    QVERIFY(document.encodingForState(MapCellState::Free, &value, &error));
    QCOMPARE(value, uint8_t(255));
    QVERIFY(document.encodingForState(MapCellState::Occupied, &value, &error));
    QCOMPARE(value, uint8_t(0));

    MapMetadata standard = metadataFor(1, 1, false, 0.196, 0.65);
    MapDocument standardDocument(standard, {0});
    QCOMPARE(standardDocument.semanticValue(205), -1);
  }

  void loadSaveRoundTripKeepsPixelsMetadataAndExtraFields() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString subdir = directory.filePath(QStringLiteral("目录 with space 'quote'"));
    const std::vector<uint8_t> pixels{0, 9, 10, 13, 32, 35, 90, 128, 191, 205, 254, 255};
    const QString source = writeSyntheticMap(subdir, QStringLiteral("source"), pixels, 4, 3);
    QVERIFY(!source.isEmpty());

    MapDocument document;
    QString error;
    QVERIFY2(MapIo::load(source, &document, &error), qPrintable(error));
    QCOMPARE(document.width(), 4);
    QCOMPARE(document.height(), 3);
    QCOMPARE(document.metadata().resolution, 0.05);
    QVERIFY(std::abs(document.metadata().originX + 1.25) < 1e-12);
    QVERIFY(std::abs(document.metadata().originY - 3.5) < 1e-12);
    QVERIFY(samePixels(document.pixels(), pixels));

    document.pixels()[0] = 0;
    document.pixels()[5] = 205;
    const QString target = QDir(subdir).absoluteFilePath(QStringLiteral("map_edited.yaml"));
    QString backup;
    QVERIFY2(MapIo::saveAs(&document, target, true, &error, &backup), qPrintable(error));
    QVERIFY(backup.isEmpty());
    QVERIFY(QFileInfo::exists(target));

    MapDocument reloaded;
    QVERIFY2(MapIo::load(target, &reloaded, &error), qPrintable(error));
    QVERIFY(samePixels(reloaded.pixels(), document.pixels()));
    QVERIFY(std::abs(reloaded.metadata().originX + 1.25) < 1e-12);
    QVERIFY(std::abs(reloaded.metadata().freeThresh - 0.25) < 1e-12);
    const QByteArray targetYaml = readBytes(target);
    QVERIFY(targetYaml.contains("custom_field: keep_me"));
    QVERIFY(targetYaml.contains("image:"));
    QVERIFY(!targetYaml.contains("source.pgm"));
    QVERIFY(QFileInfo::exists(QDir(subdir).absoluteFilePath("source.pgm")));
  }

  void pgmParserHandlesCommentsWhitespacePixelsAndRejectsInvalidData() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath("edge.pgm");
    const QByteArray data = QByteArray("P5\n# comment\n4 2\n255\n") +
                            QByteArray::fromHex("00090a0d2023cdfe");
    QVERIFY(writeBytes(path, data));
    int width = 0;
    int height = 0;
    std::vector<uint8_t> pixels;
    QString error;
    QVERIFY2(MapIo::readPgm(path, &width, &height, &pixels, &error), qPrintable(error));
    QCOMPARE(width, 4);
    QCOMPARE(height, 2);
    QVERIFY(pixels == (std::vector<uint8_t>{0x00, 0x09, 0x0a, 0x0d, 0x20, 0x23, 0xcd, 0xfe}));

    const QString shortPath = directory.filePath("short.pgm");
    QVERIFY(writeBytes(shortPath, pgmHeader(2, 2) + QByteArray(3, '\0')));
    QVERIFY(!MapIo::readPgm(shortPath, nullptr, nullptr, nullptr, &error));

    const QString p2Path = directory.filePath("p2.pgm");
    QVERIFY(writeBytes(p2Path, "P2\n2 2\n255\n0 1 2 3\n"));
    QVERIFY(!MapIo::readPgm(p2Path, nullptr, nullptr, nullptr, &error));

    const QString sixteenBit = directory.filePath("16.pgm");
    QVERIFY(writeBytes(sixteenBit, "P5\n2 2\n65535\n" + QByteArray(8, '\0')));
    QVERIFY(!MapIo::readPgm(sixteenBit, nullptr, nullptr, nullptr, &error));
  }

  void saveAsBacksUpExistingTargetAndRejectsSilentOverwrite() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const std::vector<uint8_t> pixels(20, 205);
    const QString source = writeSyntheticMap(directory.path(), QStringLiteral("source"), pixels, 5, 4);
    QVERIFY(!source.isEmpty());
    MapDocument document;
    QString error;
    QVERIFY2(MapIo::load(source, &document, &error), qPrintable(error));

    const QString target = directory.filePath("edited.yaml");
    QVERIFY2(MapIo::saveAs(&document, target, true, &error, nullptr), qPrintable(error));
    QVERIFY(QFileInfo::exists(target));

    QVERIFY(!MapIo::saveAs(&document, target, false, &error, nullptr));
    QVERIFY(error.contains(QStringLiteral("已存在")));

    document.pixels()[1] = 0;
    QString backup;
    QVERIFY2(MapIo::saveAs(&document, target, true, &error, &backup), qPrintable(error));
    QVERIFY(QFileInfo(backup).isDir());
    QVERIFY(QFileInfo::exists(QDir(backup).absoluteFilePath("edited.yaml")));
    const QByteArray backupYaml = readBytes(QDir(backup).absoluteFilePath("edited.yaml"));
    QVERIFY(!backupYaml.isEmpty());

    MapDocument reloaded;
    QVERIFY2(MapIo::load(target, &reloaded, &error), qPrintable(error));
    QCOMPARE(reloaded.pixels()[1], uint8_t(0));
  }

  void undoRedoRestoresExactPixelsAndRevision() {
    MapMetadata metadata = metadataFor(8, 6);
    MapDocument document(metadata, std::vector<uint8_t>(48, 205));
    MapHistory history;
    const auto original = document.pixels();

    auto circle = std::make_unique<MapEditCommand>(&document);
    circle->paintCircle(2, 2, 1, 0);
    QVERIFY(!circle->empty());
    history.push(std::move(circle));
    QVERIFY(document.pixels() != original);
    QVERIFY(history.canUndo());
    QVERIFY(history.undo());
    QVERIFY(document.pixels() == original);
    QVERIFY(history.canRedo());
    QVERIFY(history.redo());
    QVERIFY(document.pixels() != original);
    const auto afterCircle = document.pixels();

    auto line = std::make_unique<MapEditCommand>(&document);
    line->paintLine(0, 0, 7, 5, 0, 128);
    history.push(std::move(line));
    QVERIFY(history.undo());
    QVERIFY(document.pixels() == afterCircle);
    QVERIFY(history.undo());
    QVERIFY(document.pixels() == original);
    QVERIFY(!history.canUndo());
  }

  void worldCoordinatesRoundTripWithNegativeOriginAndYaw() {
    MapMetadata metadata = metadataFor(4, 3);
    metadata.resolution = 0.5;
    metadata.originX = 2.0;
    metadata.originY = -1.0;
    metadata.originYaw = 0.37;
    MapDocument document(metadata, std::vector<uint8_t>(12, 254));
    for (int v = 0; v < document.height(); ++v) {
      for (int u = 0; u < document.width(); ++u) {
        const QPointF world = document.cellCenterWorld(u, v);
        int resultU = -1;
        int resultV = -1;
        QVERIFY(document.worldToCell(world.x(), world.y(), &resultU, &resultV));
        QCOMPARE(resultU, u);
        QCOMPARE(resultV, v);
      }
    }
  }

  void invalidMetadataIsRejected() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString image = directory.filePath("map.pgm");
    QVERIFY(writeBytes(image, pgmHeader(1, 1) + QByteArray(1, '\0')));
    const QString yaml = directory.filePath("map.yaml");
    QVERIFY(writeBytes(yaml, "image: map.pgm\nmode: trinary\nresolution: 0.0\n"
                             "origin: [0, 0, 0]\nnegate: 0\nfree_thresh: 0.25\n"
                             "occupied_thresh: 0.65\n"));
    MapDocument document;
    QString error;
    QVERIFY(!MapIo::load(yaml, &document, &error));
    QVERIFY(!error.isEmpty());
  }
};

QTEST_GUILESS_MAIN(MapEditorTest)
#include "map_editor_test.moc"
