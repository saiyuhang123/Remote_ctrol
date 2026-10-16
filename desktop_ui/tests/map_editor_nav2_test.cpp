#include "map_editor/map_document.h"

#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <QTest>
#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

namespace {
bool writeFile(const QString &path, const QByteArray &data) {
  QFile file(path);
  return file.open(QIODevice::WriteOnly) && file.write(data) == data.size();
}

QString writeTripleMap(const QString &directory, const QString &stem, bool negate,
                       const std::vector<uint8_t> &pixels) {
  const QString image = QDir(directory).absoluteFilePath(stem + ".pgm");
  if (!writeFile(image, "P5\n3 1\n255\n" +
                             QByteArray(reinterpret_cast<const char *>(pixels.data()),
                                        static_cast<int>(pixels.size())))) {
    return {};
  }
  const QString yaml = QDir(directory).absoluteFilePath(stem + ".yaml");
  const QByteArray text =
      "image: " + stem.toUtf8() + ".pgm\nmode: trinary\nresolution: 0.1\n"
      "origin: [0.0, 0.0, 0.0]\nnegate: " + (negate ? "1" : "0") +
      "\nfree_thresh: 0.25\noccupied_thresh: 0.65\n";
  return writeFile(yaml, text) ? yaml : QString();
}
}  // namespace

class MapEditorNav2Test : public QObject {
  Q_OBJECT
private slots:
  void realMapServerLoadsEditedEncodingAsExpected() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());

    // negate=0, thresholds 0.25/0.65: 0 occupied, 128 unknown, 255 free.
    const QString normal = writeTripleMap(directory.path(), QStringLiteral("normal"), false,
                                          {0, 128, 255});
    QVERIFY(!normal.isEmpty());
    auto parameters = nav2_map_server::loadMapYaml(normal.toStdString());
    QCOMPARE(parameters.negate, false);
    QCOMPARE(parameters.free_thresh, 0.25);
    QCOMPARE(parameters.occupied_thresh, 0.65);
    nav_msgs::msg::OccupancyGrid normalGrid;
    nav2_map_server::loadMapFromFile(parameters, normalGrid);
    QCOMPARE(normalGrid.info.width, 3u);
    QCOMPARE(normalGrid.info.height, 1u);
    QCOMPARE(normalGrid.data.at(0), int8_t(100));
    QCOMPARE(normalGrid.data.at(1), int8_t(-1));
    QCOMPARE(normalGrid.data.at(2), int8_t(0));

    // Existing project maps use 205 with free_thresh=0.25, so the real loader
    // must treat 205 as free, not as unknown.
    const QString legacy = writeTripleMap(directory.path(), QStringLiteral("legacy"), false,
                                          {205, 205, 205});
    QVERIFY(!legacy.isEmpty());
    nav_msgs::msg::OccupancyGrid legacyGrid;
    auto legacyParameters = nav2_map_server::loadMapYaml(legacy.toStdString());
    nav2_map_server::loadMapFromFile(legacyParameters, legacyGrid);
    for (const auto value : legacyGrid.data) QCOMPARE(value, int8_t(0));

    // negate=1: 0 free, 128 unknown, 255 occupied.
    const QString negated = writeTripleMap(directory.path(), QStringLiteral("negated"), true,
                                           {0, 128, 255});
    QVERIFY(!negated.isEmpty());
    auto negatedParameters = nav2_map_server::loadMapYaml(negated.toStdString());
    QCOMPARE(negatedParameters.negate, true);
    nav_msgs::msg::OccupancyGrid negatedGrid;
    nav2_map_server::loadMapFromFile(negatedParameters, negatedGrid);
    QCOMPARE(negatedGrid.data.at(0), int8_t(0));
    QCOMPARE(negatedGrid.data.at(1), int8_t(-1));
    QCOMPARE(negatedGrid.data.at(2), int8_t(100));
  }

  void editorEncodingForStateMatchesRealLoader() {
    MapMetadata metadata;
    metadata.width = 3;
    metadata.height = 1;
    metadata.resolution = 0.1;
    metadata.negate = false;
    metadata.freeThresh = 0.25;
    metadata.occupiedThresh = 0.65;
    MapDocument document(metadata, {0, 0, 0});
    uint8_t freeValue = 0;
    uint8_t unknownValue = 0;
    uint8_t occupiedValue = 0;
    QString error;
    QVERIFY2(document.encodingForState(MapCellState::Free, &freeValue, &error), qPrintable(error));
    QVERIFY2(document.encodingForState(MapCellState::Unknown, &unknownValue, &error), qPrintable(error));
    QVERIFY2(document.encodingForState(MapCellState::Occupied, &occupiedValue, &error), qPrintable(error));

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QByteArray pgm = "P5\n3 1\n255\n" +
                           QByteArray(1, static_cast<char>(freeValue)) +
                           QByteArray(1, static_cast<char>(unknownValue)) +
                           QByteArray(1, static_cast<char>(occupiedValue));
    const QString image = directory.filePath("encoding.pgm");
    QVERIFY(writeFile(image, pgm));
    const QString yaml = directory.filePath("encoding.yaml");
    QVERIFY(writeFile(yaml,
                      "image: encoding.pgm\nmode: trinary\nresolution: 0.1\n"
                      "origin: [0.0, 0.0, 0.0]\nnegate: 0\n"
                      "free_thresh: 0.25\noccupied_thresh: 0.65\n"));
    auto parameters = nav2_map_server::loadMapYaml(yaml.toStdString());
    nav_msgs::msg::OccupancyGrid grid;
    nav2_map_server::loadMapFromFile(parameters, grid);
    QCOMPARE(grid.data.at(0), int8_t(0));
    QCOMPARE(grid.data.at(1), int8_t(-1));
    QCOMPARE(grid.data.at(2), int8_t(100));
  }
};

QTEST_GUILESS_MAIN(MapEditorNav2Test)
#include "map_editor_nav2_test.moc"
