#pragma once

#include <QByteArray>
#include <QPointF>
#include <QString>
#include <cstdint>
#include <vector>

enum class MapCellState {
  Free,
  Occupied,
  Unknown,
};

struct MapMetadata {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double originX = 0.0;
  double originY = 0.0;
  double originYaw = 0.0;
  bool negate = false;
  double freeThresh = 0.25;
  double occupiedThresh = 0.65;
  QString mode = QStringLiteral("trinary");

  // Immutable source identity and the latest export target. Hashes are SHA-256
  // of the complete files and are used to detect external modifications.
  QString sourceYamlPath;
  QString sourceImagePath;
  QString lastExportYamlPath;
  QByteArray sourceYamlHash;
  QByteArray sourceImageHash;
  QByteArray lastExportYamlHash;
};

class MapDocument {
public:
  MapDocument() = default;
  MapDocument(MapMetadata metadata, std::vector<uint8_t> pixels);

  bool valid() const;
  int width() const { return metadata_.width; }
  int height() const { return metadata_.height; }
  int cellCount() const { return metadata_.width * metadata_.height; }
  bool contains(int u, int v) const;
  int index(int u, int v) const { return v * metadata_.width + u; }

  const MapMetadata &metadata() const { return metadata_; }
  void setLastExportInfo(const QString &yamlPath, const QByteArray &yamlHash);

  const std::vector<uint8_t> &pixels() const { return pixels_; }
  std::vector<uint8_t> &pixels() { return pixels_; }
  uint8_t pixel(int u, int v) const;
  uint8_t pixelByIndex(int index) const;
  void setPixelByIndex(int index, uint8_t value);

  bool isDirty() const { return pixels_ != originalPixels_; }
  void markClean() { originalPixels_ = pixels_; }

  // Returns -1 / 0 / 100 exactly as nav2_map_server's trinary loader does for
  // this document's negate/threshold configuration.
  int semanticValue(uint8_t pixel) const;
  bool encodingForState(MapCellState state, uint8_t *pixel, QString *error = nullptr) const;

  bool pixelToCell(double localX, double localY, int *u, int *v) const;
  bool worldToCell(double worldX, double worldY, int *u, int *v) const;
  QPointF cellCenterWorld(int u, int v) const;
  QPointF cellCenterLocal(int u, int v) const;

private:
  MapMetadata metadata_;
  std::vector<uint8_t> pixels_;
  std::vector<uint8_t> originalPixels_;
};
