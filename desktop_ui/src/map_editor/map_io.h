#pragma once

#include "map_editor/map_document.h"

#include <QByteArray>
#include <QString>
#include <cstdint>
#include <vector>

class MapIo {
public:
  static bool load(const QString &yamlPath, MapDocument *out, QString *error = nullptr);
  static bool save(MapDocument *document, QString *error = nullptr,
                   QString *backupDir = nullptr);
  static bool saveAs(MapDocument *document, const QString &targetYamlPath,
                     bool allowOverwriteExisting, QString *error = nullptr,
                     QString *backupDir = nullptr);

  static bool readPgm(const QString &path, int *width, int *height,
                      std::vector<uint8_t> *pixels, QString *error = nullptr);
  static QByteArray fileHash(const QString &path, QString *error = nullptr);
  static bool fileMatchesHash(const QString &path, const QByteArray &hash,
                              QString *error = nullptr);
};
