#include "map_editor/map_io.h"

#include <QDateTime>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QtGlobal>
#include <cmath>
#include <string>
#include <yaml-cpp/yaml.h>

namespace {
constexpr qint64 kMaxPixels = 25000000;

struct PgmImage {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> pixels;
};

bool readWholeFile(const QString &path, QByteArray *bytes, QString *error) {
  if (!bytes) return false;
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("无法读取文件：%1（%2）").arg(path, file.errorString());
    return false;
  }
  *bytes = file.readAll();
  if (file.error() != QFile::NoError) {
    if (error) *error = QStringLiteral("读取文件失败：%1（%2）").arg(path, file.errorString());
    return false;
  }
  return true;
}

bool isHeaderSpace(char c) {
  const unsigned char value = static_cast<unsigned char>(c);
  return value == ' ' || value == '\t' || value == '\n' || value == '\r' ||
         value == '\v' || value == '\f';
}

bool nextPgmToken(const QByteArray &bytes, int *position, QByteArray *token) {
  if (!position || !token) return false;
  int index = *position;
  while (index < bytes.size()) {
    if (bytes.at(index) == '#') {
      while (index < bytes.size() && bytes.at(index) != '\n' && bytes.at(index) != '\r') ++index;
    } else if (isHeaderSpace(bytes.at(index))) {
      ++index;
    } else {
      break;
    }
  }
  if (index >= bytes.size()) {
    *position = index;
    return false;
  }
  const int start = index;
  while (index < bytes.size() && !isHeaderSpace(bytes.at(index))) ++index;
  *token = bytes.mid(start, index - start);
  *position = index;
  return true;
}

bool parsePgmData(const QByteArray &bytes, PgmImage *image, QString *error) {
  if (!image) return false;
  const auto fail = [error](const QString &text) {
    if (error) *error = text;
    return false;
  };

  int position = 0;
  QByteArray magic;
  QByteArray widthToken;
  QByteArray heightToken;
  QByteArray maxvalToken;
  if (!nextPgmToken(bytes, &position, &magic) || magic != "P5" ||
      !nextPgmToken(bytes, &position, &widthToken) ||
      !nextPgmToken(bytes, &position, &heightToken) ||
      !nextPgmToken(bytes, &position, &maxvalToken)) {
    return fail(QStringLiteral("PGM 头部不完整或不是 P5 格式"));
  }

  bool widthOk = false;
  bool heightOk = false;
  bool maxvalOk = false;
  const qint64 width = widthToken.toLongLong(&widthOk);
  const qint64 height = heightToken.toLongLong(&heightOk);
  const qint64 maxval = maxvalToken.toLongLong(&maxvalOk);
  if (!widthOk || !heightOk || !maxvalOk || width <= 0 || height <= 0) {
    return fail(QStringLiteral("PGM 尺寸或 maxval 不是有效正整数"));
  }
  if (maxval != 255) {
    return fail(QStringLiteral("当前只支持 maxval=255 的 8 位 PGM"));
  }
  if (width > kMaxPixels / height) {
    return fail(QStringLiteral("PGM 像素数超过上限 %1").arg(kMaxPixels));
  }
  const qint64 pixelCount = width * height;
  if (pixelCount > kMaxPixels) {
    return fail(QStringLiteral("PGM 像素数超过上限 %1").arg(kMaxPixels));
  }

  // PGM has exactly one whitespace byte between the maxval token and raster
  // data. The first pixel may itself be '\n', '\r', ' ', or '#'.
  if (position >= bytes.size() || !isHeaderSpace(bytes.at(position))) {
    return fail(QStringLiteral("PGM 头部与像素数据之间缺少分隔空白"));
  }
  ++position;
  if (bytes.size() - position != pixelCount) {
    return fail(QStringLiteral("PGM 像素数据长度不符：期望 %1 字节，实际 %2 字节")
                    .arg(pixelCount)
                    .arg(bytes.size() - position));
  }

  image->width = static_cast<int>(width);
  image->height = static_cast<int>(height);
  image->pixels.assign(reinterpret_cast<const uint8_t *>(bytes.constData() + position),
                       reinterpret_cast<const uint8_t *>(bytes.constData() + position + pixelCount));
  return true;
}

bool readPgmFile(const QString &path, PgmImage *image, QByteArray *rawBytes, QString *error) {
  QByteArray bytes;
  if (!readWholeFile(path, &bytes, error)) return false;
  if (!parsePgmData(bytes, image, error)) return false;
  if (rawBytes) *rawBytes = std::move(bytes);
  return true;
}

QString canonicalOrAbsolute(const QString &path) {
  const QFileInfo info(path);
  const QString canonical = info.canonicalFilePath();
  return canonical.isEmpty() ? info.absoluteFilePath() : canonical;
}

QString resolveImagePath(const QString &yamlPath, const QString &imageReference) {
  QFileInfo reference(imageReference);
  if (reference.isAbsolute()) return QDir::cleanPath(reference.absoluteFilePath());
  return QDir::cleanPath(QDir(QFileInfo(yamlPath).absolutePath()).absoluteFilePath(imageReference));
}

bool scalarString(const YAML::Node &node, QString *value) {
  if (!node || !node.IsScalar() || !value) return false;
  try {
    *value = QString::fromStdString(node.as<std::string>());
    return true;
  } catch (...) {
    return false;
  }
}

bool scalarDouble(const YAML::Node &node, double *value) {
  if (!node || !node.IsScalar() || !value) return false;
  try {
    *value = node.as<double>();
    return std::isfinite(*value);
  } catch (...) {
    return false;
  }
}

bool scalarBool(const YAML::Node &node, bool *value) {
  if (!node || !node.IsScalar() || !value) return false;
  const std::string text = node.Scalar();
  if (text == "0" || text == "false" || text == "False" || text == "FALSE") {
    *value = false;
    return true;
  }
  if (text == "1" || text == "true" || text == "True" || text == "TRUE") {
    *value = true;
    return true;
  }
  return false;
}

bool writeYamlAtomically(const QString &path, const YAML::Node &root, QString *error) {
  std::string dumped;
  try {
    dumped = YAML::Dump(root);
  } catch (const std::exception &exception) {
    if (error) *error = QStringLiteral("生成 YAML 失败：%1").arg(QString::fromUtf8(exception.what()));
    return false;
  }
  if (dumped.empty()) {
    if (error) *error = QStringLiteral("生成 YAML 失败：内容为空");
    return false;
  }
  try {
    YAML::Node verification = YAML::Load(dumped);
    if (!verification.IsMap()) {
      if (error) *error = QStringLiteral("生成的 YAML 不是映射结构");
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = QStringLiteral("生成的 YAML 无法回读：%1").arg(QString::fromUtf8(exception.what()));
    return false;
  }

  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    if (error) *error = QStringLiteral("无法写入 YAML：%1（%2）").arg(path, file.errorString());
    return false;
  }
  const QByteArray data = QByteArray::fromStdString(dumped);
  if (file.write(data) != data.size() || !file.commit()) {
    if (error) *error = QStringLiteral("提交 YAML 失败：%1").arg(path);
    return false;
  }
  return true;
}

bool writePgmAtomically(const QString &path, int width, int height,
                        const std::vector<uint8_t> &pixels, QString *error) {
  QByteArray header = "P5\n" + QByteArray::number(width) + " " + QByteArray::number(height) + "\n255\n";
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    if (error) *error = QStringLiteral("无法写入 PGM：%1（%2）").arg(path, file.errorString());
    return false;
  }
  const qint64 pixelBytes = static_cast<qint64>(pixels.size());
  if (file.write(header) != header.size() ||
      file.write(reinterpret_cast<const char *>(pixels.data()), pixelBytes) != pixelBytes ||
      !file.commit()) {
    if (error) *error = QStringLiteral("提交 PGM 失败：%1").arg(path);
    return false;
  }
  return true;
}

QString uniqueImageName(const QString &targetYamlPath) {
  const QFileInfo target(targetYamlPath);
  const QDir directory(target.absolutePath());
  const QString stem = target.completeBaseName().isEmpty() ? QStringLiteral("map") : target.completeBaseName();
  const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
  QString name = stem + "." + stamp + ".pgm";
  int suffix = 1;
  while (QFileInfo::exists(directory.absoluteFilePath(name))) {
    name = stem + "." + stamp + "_" + QString::number(suffix++) + ".pgm";
  }
  return name;
}

bool backupExistingTarget(const QString &targetYamlPath, QString *backupDir, QString *error) {
  if (backupDir) backupDir->clear();
  if (!QFileInfo::exists(targetYamlPath)) return true;

  YAML::Node root;
  try {
    root = YAML::LoadFile(targetYamlPath.toStdString());
  } catch (const std::exception &exception) {
    if (error) *error = QStringLiteral("目标 YAML 无法解析，拒绝备份覆盖：%1").arg(QString::fromUtf8(exception.what()));
    return false;
  }
  if (!root.IsMap()) {
    if (error) *error = QStringLiteral("目标 YAML 不是映射结构，拒绝备份覆盖");
    return false;
  }

  QString imageReference;
  if (!scalarString(root["image"], &imageReference)) {
    if (error) *error = QStringLiteral("目标 YAML 缺少 image 字段，拒绝备份覆盖");
    return false;
  }
  const QFileInfo targetInfo(targetYamlPath);
  const QString oldImage = resolveImagePath(targetYamlPath, imageReference);
  if (!QFileInfo::exists(oldImage) || !QFileInfo(oldImage).isFile()) {
    if (error) *error = QStringLiteral("目标 YAML 引用的图片不存在：%1").arg(oldImage);
    return false;
  }

  const QString baseName = targetInfo.completeBaseName().isEmpty() ? QStringLiteral("map") : targetInfo.completeBaseName();
  const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
  QDir parent(targetInfo.absolutePath());
  QString directoryName = baseName + "_backup_" + stamp;
  int suffix = 1;
  while (QFileInfo::exists(parent.absoluteFilePath(directoryName))) {
    directoryName = baseName + "_backup_" + stamp + "_" + QString::number(suffix++);
  }
  const QString backupDirectory = parent.absoluteFilePath(directoryName);
  if (!QDir().mkpath(backupDirectory)) {
    if (error) *error = QStringLiteral("无法创建备份目录：%1").arg(backupDirectory);
    return false;
  }

  const QString oldImageName = QFileInfo(oldImage).fileName();
  const QString backupImage = QDir(backupDirectory).absoluteFilePath(oldImageName);
  if (!QFile::copy(oldImage, backupImage)) {
    if (error) *error = QStringLiteral("无法复制原图片到备份目录：%1").arg(backupImage);
    return false;
  }

  root["image"] = oldImageName.toStdString();
  const QString backupYaml = QDir(backupDirectory).absoluteFilePath(targetInfo.fileName());
  if (!writeYamlAtomically(backupYaml, root, error)) return false;
  if (backupDir) *backupDir = backupDirectory;
  return true;
}

YAML::Node baseYamlForSave(MapDocument *document, QString *error) {
  YAML::Node root;
  if (!document->metadata().sourceYamlPath.isEmpty()) {
    try {
      root = YAML::LoadFile(document->metadata().sourceYamlPath.toStdString());
    } catch (const std::exception &exception) {
      if (error) *error = QStringLiteral("重新读取源 YAML 失败：%1").arg(QString::fromUtf8(exception.what()));
      return {};
    }
    if (!root.IsMap()) {
      if (error) *error = QStringLiteral("源 YAML 不是映射结构");
      return {};
    }
  } else {
    root = YAML::Node(YAML::NodeType::Map);
  }

  const MapMetadata &metadata = document->metadata();
  if (!root["resolution"]) root["resolution"] = metadata.resolution;
  if (!root["origin"]) {
    YAML::Node origin(YAML::NodeType::Sequence);
    origin.push_back(metadata.originX);
    origin.push_back(metadata.originY);
    origin.push_back(metadata.originYaw);
    root["origin"] = origin;
  }
  if (!root["negate"]) root["negate"] = metadata.negate;
  if (!root["free_thresh"]) root["free_thresh"] = metadata.freeThresh;
  if (!root["occupied_thresh"]) root["occupied_thresh"] = metadata.occupiedThresh;
  if (!root["mode"] && !metadata.mode.isEmpty()) root["mode"] = metadata.mode.toStdString();
  return root;
}

bool verifySavedMap(const QString &targetYamlPath, const QString &expectedImage,
                    int expectedWidth, int expectedHeight, QString *error) {
  try {
    const YAML::Node root = YAML::LoadFile(targetYamlPath.toStdString());
    QString imageReference;
    if (!root.IsMap() || !scalarString(root["image"], &imageReference) ||
        imageReference != expectedImage) {
      if (error) *error = QStringLiteral("导出后的 YAML 校验失败：image 字段不正确");
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = QStringLiteral("导出后的 YAML 无法回读：%1").arg(QString::fromUtf8(exception.what()));
    return false;
  }

  PgmImage image;
  const QString imagePath = resolveImagePath(targetYamlPath, expectedImage);
  if (!readPgmFile(imagePath, &image, nullptr, error)) return false;
  if (image.width != expectedWidth || image.height != expectedHeight) {
    if (error) *error = QStringLiteral("导出后的 PGM 尺寸校验失败");
    return false;
  }
  return true;
}
}  // namespace

bool MapIo::readPgm(const QString &path, int *width, int *height,
                    std::vector<uint8_t> *pixels, QString *error) {
  PgmImage image;
  if (!readPgmFile(path, &image, nullptr, error)) return false;
  if (width) *width = image.width;
  if (height) *height = image.height;
  if (pixels) *pixels = std::move(image.pixels);
  return true;
}

QByteArray MapIo::fileHash(const QString &path, QString *error) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("无法读取文件：%1（%2）").arg(path, file.errorString());
    return {};
  }
  QCryptographicHash hash(QCryptographicHash::Sha256);
  if (!hash.addData(&file)) {
    if (error) *error = QStringLiteral("计算文件摘要失败：%1").arg(path);
    return {};
  }
  return hash.result();
}

bool MapIo::fileMatchesHash(const QString &path, const QByteArray &hash, QString *error) {
  if (hash.isEmpty()) return QFileInfo::exists(path);
  const QByteArray actual = fileHash(path, error);
  if (actual.isEmpty()) return false;
  if (actual != hash) {
    if (error) *error = QStringLiteral("文件内容已被外部修改：%1").arg(path);
    return false;
  }
  return true;
}

bool MapIo::load(const QString &yamlPath, MapDocument *out, QString *error) {
  if (!out) {
    if (error) *error = QStringLiteral("内部错误：空的文档指针");
    return false;
  }
  const QString yamlAbsolute = QFileInfo(yamlPath).absoluteFilePath();
  QByteArray yamlBytes;
  if (!readWholeFile(yamlAbsolute, &yamlBytes, error)) return false;

  YAML::Node root;
  try {
    root = YAML::Load(std::string(yamlBytes.constData(), static_cast<size_t>(yamlBytes.size())));
  } catch (const std::exception &exception) {
    if (error) *error = QStringLiteral("YAML 解析失败：%1").arg(QString::fromUtf8(exception.what()));
    return false;
  }
  if (!root.IsMap()) {
    if (error) *error = QStringLiteral("地图 YAML 不是映射结构");
    return false;
  }

  QString imageReference;
  if (!scalarString(root["image"], &imageReference) || imageReference.isEmpty()) {
    if (error) *error = QStringLiteral("地图 YAML 缺少有效 image 字段");
    return false;
  }
  const QString imagePath = resolveImagePath(yamlAbsolute, imageReference);

  MapMetadata metadata;
  if (!scalarDouble(root["resolution"], &metadata.resolution) || metadata.resolution <= 0.0) {
    if (error) *error = QStringLiteral("地图 YAML 的 resolution 无效");
    return false;
  }
  const YAML::Node origin = root["origin"];
  if (!origin || !origin.IsSequence() || origin.size() != 3) {
    if (error) *error = QStringLiteral("地图 YAML 的 origin 必须是三项序列");
    return false;
  }
  if (!scalarDouble(origin[0], &metadata.originX) ||
      !scalarDouble(origin[1], &metadata.originY) ||
      !scalarDouble(origin[2], &metadata.originYaw)) {
    if (error) *error = QStringLiteral("地图 YAML 的 origin 包含无效数值");
    return false;
  }
  if (!scalarBool(root["negate"], &metadata.negate)) {
    if (error) *error = QStringLiteral("地图 YAML 的 negate 必须为 0/1 或 true/false");
    return false;
  }
  if (!scalarDouble(root["free_thresh"], &metadata.freeThresh) ||
      !scalarDouble(root["occupied_thresh"], &metadata.occupiedThresh) ||
      metadata.freeThresh <= 0.0 ||
      metadata.freeThresh >= metadata.occupiedThresh ||
      metadata.occupiedThresh >= 1.0) {
    if (error) *error = QStringLiteral("地图 YAML 的 free_thresh/occupied_thresh 无效");
    return false;
  }
  QString mode = QStringLiteral("trinary");
  if (root["mode"]) {
    if (!scalarString(root["mode"], &mode) || mode != QStringLiteral("trinary")) {
      if (error) *error = QStringLiteral("当前只支持 mode=trinary 的地图");
      return false;
    }
  }
  metadata.mode = mode;

  PgmImage pgm;
  QByteArray imageBytes;
  if (!readPgmFile(imagePath, &pgm, &imageBytes, error)) return false;
  metadata.width = pgm.width;
  metadata.height = pgm.height;
  metadata.sourceYamlPath = yamlAbsolute;
  metadata.sourceImagePath = QFileInfo(imagePath).absoluteFilePath();
  metadata.sourceYamlHash = QCryptographicHash::hash(yamlBytes, QCryptographicHash::Sha256);
  metadata.sourceImageHash = QCryptographicHash::hash(imageBytes, QCryptographicHash::Sha256);

  MapDocument document(std::move(metadata), std::move(pgm.pixels));
  if (!document.valid()) {
    if (error) *error = QStringLiteral("地图文档校验失败：尺寸、分辨率或像素长度不一致");
    return false;
  }
  uint8_t ignored = 0;
  if (!document.encodingForState(MapCellState::Unknown, &ignored, error)) return false;
  *out = std::move(document);
  return true;
}

bool MapIo::saveAs(MapDocument *document, const QString &targetYamlPath,
                   bool allowOverwriteExisting, QString *error, QString *backupDir) {
  if (backupDir) backupDir->clear();
  if (!document || !document->valid()) {
    if (error) *error = QStringLiteral("没有可保存的有效地图文档");
    return false;
  }

  QString target = targetYamlPath.trimmed();
  if (target.isEmpty()) {
    if (error) *error = QStringLiteral("导出路径为空");
    return false;
  }
  QString suffix = QFileInfo(target).suffix();
  if (suffix.isEmpty()) {
    target += QStringLiteral(".yaml");
    suffix = QStringLiteral("yaml");
  }
  if (suffix.compare(QStringLiteral("yaml"), Qt::CaseInsensitive) != 0 &&
      suffix.compare(QStringLiteral("yml"), Qt::CaseInsensitive) != 0) {
    if (error) *error = QStringLiteral("导航地图只能另存为 .yaml 或 .yml 文件");
    return false;
  }

  const QFileInfo targetInfo(target);
  const QString targetAbsolute = targetInfo.absoluteFilePath();
  const MapMetadata &metadata = document->metadata();

  if (!metadata.sourceYamlPath.isEmpty()) {
    if (canonicalOrAbsolute(targetAbsolute) == canonicalOrAbsolute(metadata.sourceYamlPath)) {
      if (error) *error = QStringLiteral("第一版不允许把源 YAML 作为导出目标，请另存为");
      return false;
    }
    if (!fileMatchesHash(metadata.sourceYamlPath, metadata.sourceYamlHash, error)) return false;
    if (!fileMatchesHash(metadata.sourceImagePath, metadata.sourceImageHash, error)) return false;
  }

  const bool targetExists = QFileInfo::exists(targetAbsolute);
  if (targetExists && !allowOverwriteExisting) {
    if (error) *error = QStringLiteral("目标文件已存在，需要确认后才能备份覆盖");
    return false;
  }
  if (!QDir().mkpath(targetInfo.absolutePath())) {
    if (error) *error = QStringLiteral("无法创建导出目录：%1").arg(targetInfo.absolutePath());
    return false;
  }

  QString backup;
  if (targetExists && !backupExistingTarget(targetAbsolute, &backup, error)) return false;

  const QString imageName = uniqueImageName(targetAbsolute);
  const QString imagePath = QDir(targetInfo.absolutePath()).absoluteFilePath(imageName);
  if (!writePgmAtomically(imagePath, document->width(), document->height(),
                          document->pixels(), error)) {
    return false;
  }

  // Verify the staged image before the YAML is committed. If this fails, the
  // previous target YAML still references its complete old image.
  PgmImage stagedImage;
  if (!readPgmFile(imagePath, &stagedImage, nullptr, error)) return false;
  if (stagedImage.width != document->width() || stagedImage.height != document->height() ||
      stagedImage.pixels != document->pixels()) {
    if (error) *error = QStringLiteral("导出 PGM 的尺寸或像素与文档不一致");
    return false;
  }

  YAML::Node root = baseYamlForSave(document, error);
  if (!root.IsMap()) return false;
  root["image"] = imageName.toStdString();
  if (!writeYamlAtomically(targetAbsolute, root, error)) return false;
  if (!verifySavedMap(targetAbsolute, imageName, document->width(), document->height(), error)) {
    return false;
  }

  document->setLastExportInfo(targetAbsolute, QCryptographicHash::hash(
      [&targetAbsolute] {
        QFile file(targetAbsolute);
        return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray();
      }(),
      QCryptographicHash::Sha256));
  if (backupDir) *backupDir = backup;
  return true;
}
