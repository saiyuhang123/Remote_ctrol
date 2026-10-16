#include "map_editor/map_document.h"

#include <QtGlobal>
#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

MapDocument::MapDocument(MapMetadata metadata, std::vector<uint8_t> pixels)
    : metadata_(std::move(metadata)), pixels_(std::move(pixels)) {
  originalPixels_ = pixels_;
}

bool MapDocument::valid() const {
  return metadata_.width > 0 && metadata_.height > 0 &&
         metadata_.width <= 25000000 / metadata_.height &&
         pixels_.size() == static_cast<size_t>(cellCount()) &&
         std::isfinite(metadata_.resolution) && metadata_.resolution > 0.0 &&
         std::isfinite(metadata_.originX) && std::isfinite(metadata_.originY) &&
         std::isfinite(metadata_.originYaw) &&
         std::isfinite(metadata_.freeThresh) && std::isfinite(metadata_.occupiedThresh) &&
         metadata_.freeThresh > 0.0 && metadata_.freeThresh < metadata_.occupiedThresh &&
         metadata_.occupiedThresh < 1.0;
}

bool MapDocument::contains(int u, int v) const {
  return u >= 0 && u < metadata_.width && v >= 0 && v < metadata_.height;
}

uint8_t MapDocument::pixel(int u, int v) const {
  if (!contains(u, v)) return 0;
  return pixels_[static_cast<size_t>(index(u, v))];
}

uint8_t MapDocument::pixelByIndex(int index) const {
  if (index < 0 || index >= cellCount()) return 0;
  return pixels_[static_cast<size_t>(index)];
}

void MapDocument::setPixelByIndex(int index, uint8_t value) {
  if (index < 0 || index >= cellCount()) return;
  pixels_[static_cast<size_t>(index)] = value;
}

void MapDocument::setLastExportInfo(const QString &yamlPath, const QByteArray &yamlHash) {
  metadata_.lastExportYamlPath = yamlPath;
  metadata_.lastExportYamlHash = yamlHash;
}

int MapDocument::semanticValue(uint8_t pixel) const {
  const double occupancy = metadata_.negate
                               ? static_cast<double>(pixel) / 255.0
                               : static_cast<double>(255 - pixel) / 255.0;
  if (occupancy >= metadata_.occupiedThresh) return 100;
  if (occupancy <= metadata_.freeThresh) return 0;
  return -1;
}

bool MapDocument::encodingForState(MapCellState state, uint8_t *pixel, QString *error) const {
  if (!pixel) return false;
  const auto fail = [error](const QString &text) {
    if (error) *error = text;
    return false;
  };

  if (state == MapCellState::Occupied) {
    *pixel = metadata_.negate ? 255 : 0;
  } else if (state == MapCellState::Free) {
    *pixel = metadata_.negate ? 0 : 255;
  } else {
    int first = -1;
    int last = -1;
    for (int value = 0; value < 256; ++value) {
      if (semanticValue(static_cast<uint8_t>(value)) == -1) {
        if (first < 0) first = value;
        last = value;
      }
    }
    if (first < 0) return fail(QStringLiteral("当前 YAML 阈值下没有 8 位灰度可以表示未知区域"));
    *pixel = static_cast<uint8_t>((first + last) / 2);
  }

  if (semanticValue(*pixel) != (state == MapCellState::Free ? 0 : state == MapCellState::Occupied ? 100 : -1)) {
    return fail(QStringLiteral("当前 YAML 阈值配置无法可靠表示目标状态"));
  }
  if (error) error->clear();
  return true;
}

QPointF MapDocument::cellCenterLocal(int u, int v) const {
  const double localX = (static_cast<double>(u) + 0.5) * metadata_.resolution;
  const double localY = (static_cast<double>(metadata_.height - v) - 0.5) * metadata_.resolution;
  return QPointF(localX, localY);
}

QPointF MapDocument::cellCenterWorld(int u, int v) const {
  const QPointF local = cellCenterLocal(u, v);
  const double c = std::cos(metadata_.originYaw);
  const double s = std::sin(metadata_.originYaw);
  return QPointF(metadata_.originX + local.x() * c - local.y() * s,
                 metadata_.originY + local.x() * s + local.y() * c);
}

bool MapDocument::worldToCell(double worldX, double worldY, int *u, int *v) const {
  if (!u || !v) return false;
  const double dx = worldX - metadata_.originX;
  const double dy = worldY - metadata_.originY;
  const double c = std::cos(metadata_.originYaw);
  const double s = std::sin(metadata_.originYaw);
  // Inverse of the ROS map-axis rotation.
  const double localX = dx * c + dy * s;
  const double localY = -dx * s + dy * c;
  return pixelToCell(localX, localY, u, v);
}

bool MapDocument::pixelToCell(double localX, double localY, int *u, int *v) const {
  if (!u || !v) return false;
  const int candidateU = static_cast<int>(std::floor(localX / metadata_.resolution));
  const int bottomUpV = static_cast<int>(std::floor(localY / metadata_.resolution));
  const int candidateV = metadata_.height - 1 - bottomUpV;
  if (!contains(candidateU, candidateV)) return false;
  *u = candidateU;
  *v = candidateV;
  return true;
}
