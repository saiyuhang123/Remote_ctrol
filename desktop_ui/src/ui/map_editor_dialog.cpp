#include "ui/map_editor_dialog.h"
#include "map_editor/map_document.h"
#include "map_editor/map_io.h"
#include "ui/widgets/map_canvas.h"

#include <QCloseEvent>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeyEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;

QString displayNumber(double value) {
  return QString::number(value, 'g', 12);
}
}  // namespace

MapEditorDialog::MapEditorDialog(const QString &initialYamlPath, QWidget *parent)
    : QDialog(parent) {
  setObjectName(QStringLiteral("mapEditorDialog"));
  setWindowTitle(QStringLiteral("地图编辑"));
  resize(1120, 780);

  auto *root = new QVBoxLayout(this);
  auto *toolbar = new QHBoxLayout;
  openButton_ = new QPushButton(QStringLiteral("打开地图"), this);
  openButton_->setObjectName(QStringLiteral("mapEditorOpen"));
  saveAsButton_ = new QPushButton(QStringLiteral("另存为"), this);
  saveAsButton_->setObjectName(QStringLiteral("mapEditorSaveAs"));
  undoButton_ = new QPushButton(QStringLiteral("撤销"), this);
  undoButton_->setObjectName(QStringLiteral("mapEditorUndo"));
  redoButton_ = new QPushButton(QStringLiteral("重做"), this);
  redoButton_->setObjectName(QStringLiteral("mapEditorRedo"));

  toolCombo_ = new QComboBox(this);
  toolCombo_->setObjectName(QStringLiteral("mapEditorTool"));
  toolCombo_->setToolTip(QStringLiteral("未知表示未观测区域，不等于禁止通行；要阻止通行请使用障碍工具"));
  toolCombo_->addItem(QStringLiteral("障碍"), static_cast<int>(MapCanvas::Tool::Obstacle));
  toolCombo_->addItem(QStringLiteral("空闲"), static_cast<int>(MapCanvas::Tool::Free));
  toolCombo_->addItem(QStringLiteral("未知"), static_cast<int>(MapCanvas::Tool::Unknown));
  toolCombo_->addItem(QStringLiteral("障碍直线"), static_cast<int>(MapCanvas::Tool::ObstacleLine));
  toolCombo_->addItem(QStringLiteral("空闲直线"), static_cast<int>(MapCanvas::Tool::FreeLine));
  toolCombo_->addItem(QStringLiteral("未知直线"), static_cast<int>(MapCanvas::Tool::UnknownLine));

  brushSpin_ = new QSpinBox(this);
  brushSpin_->setObjectName(QStringLiteral("mapEditorBrush"));
  brushSpin_->setRange(0, 200);
  brushSpin_->setValue(2);
  brushSpin_->setSuffix(QStringLiteral(" 格"));
  brushMeters_ = new QLabel(this);

  fitButton_ = new QPushButton(QStringLiteral("适应窗口"), this);
  fitButton_->setObjectName(QStringLiteral("mapEditorFit"));
  zoomInButton_ = new QPushButton(QStringLiteral("+"), this);
  zoomOutButton_ = new QPushButton(QStringLiteral("-"), this);

  toolbar->addWidget(openButton_);
  toolbar->addWidget(saveAsButton_);
  toolbar->addSpacing(12);
  toolbar->addWidget(undoButton_);
  toolbar->addWidget(redoButton_);
  toolbar->addSpacing(12);
  toolbar->addWidget(new QLabel(QStringLiteral("工具"), this));
  toolbar->addWidget(toolCombo_);
  toolbar->addWidget(new QLabel(QStringLiteral("笔刷"), this));
  toolbar->addWidget(brushSpin_);
  toolbar->addWidget(brushMeters_);
  toolbar->addStretch();
  toolbar->addWidget(fitButton_);
  toolbar->addWidget(zoomOutButton_);
  toolbar->addWidget(zoomInButton_);
  root->addLayout(toolbar);

  pathLabel_ = new QLabel(this);
  pathLabel_->setObjectName(QStringLiteral("mapEditorPaths"));
  pathLabel_->setWordWrap(true);
  pathLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  root->addWidget(pathLabel_);

  metadataLabel_ = new QLabel(this);
  metadataLabel_->setObjectName(QStringLiteral("mapEditorMetadata"));
  metadataLabel_->setWordWrap(true);
  metadataLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  root->addWidget(metadataLabel_);

  canvas_ = new MapCanvas(this);
  canvas_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  root->addWidget(canvas_, 1);

  statusLabel_ = new QLabel(QStringLiteral("打开一张已有 YAML 地图开始编辑"), this);
  statusLabel_->setObjectName(QStringLiteral("mapEditorStatus"));
  statusLabel_->setWordWrap(true);
  statusLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  root->addWidget(statusLabel_);

  connect(openButton_, &QPushButton::clicked, this, &MapEditorDialog::chooseOpen);
  connect(saveAsButton_, &QPushButton::clicked, this, [this] { saveAs(); });
  connect(undoButton_, &QPushButton::clicked, this, [this] {
    if (history_.undo()) canvas_->refreshImage();
    updateUi();
  });
  connect(redoButton_, &QPushButton::clicked, this, [this] {
    if (history_.redo()) canvas_->refreshImage();
    updateUi();
  });
  connect(toolCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] {
    canvas_->setTool(static_cast<MapCanvas::Tool>(toolCombo_->currentData().toInt()));
  });
  connect(brushSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
    canvas_->setBrushRadius(value);
    if (document_) {
      brushMeters_->setText(QStringLiteral("≈ %1 m")
                                .arg(displayNumber((value * 2 + 1) * document_->metadata().resolution)));
    }
  });
  connect(fitButton_, &QPushButton::clicked, canvas_, &MapCanvas::fitToView);
  connect(zoomInButton_, &QPushButton::clicked, canvas_, &MapCanvas::zoomIn);
  connect(zoomOutButton_, &QPushButton::clicked, canvas_, &MapCanvas::zoomOut);
  connect(canvas_, &MapCanvas::cursorMoved, this, [this](double x, double y, bool valid) {
    if (valid) {
      statusLabel_->setText(QStringLiteral("光标世界坐标：x=%1 m，y=%2 m")
                                .arg(displayNumber(x), displayNumber(y)));
    }
  });
  connect(canvas_, &MapCanvas::statusMessage, statusLabel_, &QLabel::setText);
  connect(canvas_, &MapCanvas::viewScaleChanged, this, [this](double scale) {
    if (document_) {
      statusLabel_->setText(QStringLiteral("缩放 %1 倍；滚动滚轮缩放，右键/中键拖拽平移")
                                .arg(QString::number(scale, 'f', 2)));
    }
  });
  canvas_->setPendingCommandHandler([this](std::unique_ptr<MapEditCommand> command) {
    history_.push(std::move(command));
    updateUi();
  });

  updateUi();
  if (!initialYamlPath.trimmed().isEmpty()) {
    QString error;
    if (!loadMap(initialYamlPath.trimmed(), &error)) {
      statusLabel_->setText(QStringLiteral("默认地图未打开：%1").arg(error));
    }
  }
}

bool MapEditorDialog::loadMap(const QString &yamlPath, QString *error) {
  MapDocument loaded;
  if (!MapIo::load(yamlPath, &loaded, error)) return false;
  setDocument(std::make_unique<MapDocument>(std::move(loaded)));
  statusLabel_->setText(QStringLiteral("已打开：%1").arg(document_->metadata().sourceYamlPath));
  return true;
}

bool MapEditorDialog::saveToPath(const QString &targetYamlPath, QString *error) {
  if (!document_) {
    if (error) *error = QStringLiteral("当前没有打开地图");
    return false;
  }
  QString backup;
  QString internalError;
  if (!MapIo::saveAs(document_.get(), targetYamlPath, true, &internalError, &backup)) {
    if (error) *error = internalError;
    statusLabel_->setText(QStringLiteral("保存失败：%1").arg(internalError));
    return false;
  }
  savedRevision_ = history_.currentRevision();
  const QString source = document_->metadata().sourceYamlPath;
  updateUi();
  QString message = QStringLiteral("已保存：%1").arg(document_->metadata().lastExportYamlPath);
  if (!backup.isEmpty()) message += QStringLiteral("；原目标备份：%1").arg(backup);
  statusLabel_->setText(message);
  emit mapSaved(document_->metadata().lastExportYamlPath, source);
  if (error) error->clear();
  return true;
}

void MapEditorDialog::closeEvent(QCloseEvent *event) {
  if (!maybeSave()) {
    event->ignore();
    return;
  }
  event->accept();
}

void MapEditorDialog::keyPressEvent(QKeyEvent *event) {
  // Esc during a line preview cancels the preview instead of leaving the
  // editor or discarding the whole document.
  if (event->key() == Qt::Key_Escape && canvas_ && canvas_->cancelLinePreview()) {
    event->accept();
    return;
  }
  QDialog::keyPressEvent(event);
}

void MapEditorDialog::reject() {
  if (!maybeSave()) return;
  QDialog::reject();
}

bool MapEditorDialog::maybeSave() {
  if (!document_ || !isDirty()) return true;
  const auto choice = QMessageBox::warning(
      this, QStringLiteral("未保存修改"), QStringLiteral("地图存在未保存修改，是否保存？"),
      QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);
  if (choice == QMessageBox::Save) return saveAs();
  return choice == QMessageBox::Discard;
}

void MapEditorDialog::chooseOpen() {
  if (!maybeSave()) return;
  const QString initial = document_ ? document_->metadata().sourceYamlPath : QString();
  const QString selected = QFileDialog::getOpenFileName(
      this, QStringLiteral("打开地图"),
      initial.isEmpty() ? QDir::homePath() : QFileInfo(initial).absolutePath(),
      QStringLiteral("Nav2 地图 YAML (*.yaml *.yml);;PGM 图像 (*.pgm);;所有文件 (*)"));
  if (selected.isEmpty()) return;
  if (QFileInfo(selected).suffix().compare(QStringLiteral("pgm"), Qt::CaseInsensitive) == 0) {
    const QFileInfo pgm(selected);
    const QDir directory(pgm.absolutePath());
    QString yaml = directory.absoluteFilePath(pgm.completeBaseName() + QStringLiteral(".yaml"));
    if (!QFileInfo::exists(yaml)) {
      yaml = directory.absoluteFilePath(pgm.completeBaseName() + QStringLiteral(".yml"));
    }
    if (!QFileInfo::exists(yaml)) {
      QMessageBox::warning(this, QStringLiteral("缺少 YAML"),
                           QStringLiteral("只有 PGM 而没有对应 YAML 时不能猜测原点或分辨率，请选择地图 YAML。"));
      return;
    }
    QString pgmError;
    if (!loadMap(yaml, &pgmError)) {
      QMessageBox::critical(this, QStringLiteral("打开失败"), pgmError);
    }
    return;
  }
  QString error;
  if (!loadMap(selected, &error)) {
    QMessageBox::critical(this, QStringLiteral("打开失败"), error);
  }
}

bool MapEditorDialog::saveAs() {
  if (!document_) {
    QMessageBox::warning(this, QStringLiteral("无法保存"), QStringLiteral("当前没有打开地图。"));
    return false;
  }
  QString selected = QFileDialog::getSaveFileName(this, QStringLiteral("另存导航地图"),
                                                   initialSavePath(),
                                                   QStringLiteral("Nav2 地图 YAML (*.yaml *.yml)"));
  if (selected.isEmpty()) return false;
  if (QFileInfo(selected).suffix().isEmpty()) selected += QStringLiteral(".yaml");
  const QFileInfo target(selected);
  const QString targetAbsolute = target.absoluteFilePath();
  if (QFileInfo::exists(targetAbsolute)) {
    const bool isOurLastExport =
        document_ &&
        document_->metadata().lastExportYamlPath == targetAbsolute &&
        !document_->metadata().lastExportYamlHash.isEmpty() &&
        MapIo::fileHash(targetAbsolute) == document_->metadata().lastExportYamlHash;
    if (!isOurLastExport) {
      const auto answer = QMessageBox::question(
          this, QStringLiteral("目标已存在"),
          QStringLiteral("目标文件已存在且不是本会话最近导出的版本。\n"
                         "继续会在保存前备份旧 YAML 和它引用的图片，是否继续？"),
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
      if (answer != QMessageBox::Yes) return false;
    }
  }
  QString error;
  if (!saveToPath(targetAbsolute, &error)) {
    QMessageBox::critical(this, QStringLiteral("保存失败"), error);
    return false;
  }
  return true;
}

void MapEditorDialog::setDocument(std::unique_ptr<MapDocument> document) {
  document_ = std::move(document);
  history_.clear();
  savedRevision_ = history_.currentRevision();
  canvas_->setDocument(document_.get());
  updateUi();
}

void MapEditorDialog::updateUi() {
  undoButton_->setEnabled(history_.canUndo());
  redoButton_->setEnabled(history_.canRedo());
  saveAsButton_->setEnabled(document_ != nullptr);
  if (!document_) {
    setWindowTitle(QStringLiteral("地图编辑 - 未打开地图"));
    pathLabel_->setText(QStringLiteral("来源：未打开"));
    metadataLabel_->setText(QString());
    brushMeters_->setText(QString());
    return;
  }

  const MapMetadata &metadata = document_->metadata();
  const QString dirtyMark = isDirty() ? QStringLiteral(" *") : QString();
  setWindowTitle(QStringLiteral("地图编辑 - %1%2")
                     .arg(QFileInfo(metadata.sourceYamlPath).fileName(), dirtyMark));
  QString paths = QStringLiteral("源 YAML：%1\n源图片：%2")
                      .arg(metadata.sourceYamlPath, metadata.sourceImagePath);
  if (!metadata.lastExportYamlPath.isEmpty()) {
    paths += QStringLiteral("\n最近导出：%1").arg(metadata.lastExportYamlPath);
  }
  pathLabel_->setText(paths);
  uint8_t unknownValue = 0;
  const bool unknownUsable = document_->encodingForState(MapCellState::Unknown, &unknownValue, nullptr);
  metadataLabel_->setText(
      QStringLiteral("尺寸：%1 × %2；分辨率：%3 m/栅格；原点：[%4, %5, %6] rad；"
                     "mode：%7；negate：%8；free：%9；occupied：%10；未知编码：%11")
          .arg(document_->width())
          .arg(document_->height())
          .arg(displayNumber(metadata.resolution))
          .arg(displayNumber(metadata.originX))
          .arg(displayNumber(metadata.originY))
          .arg(displayNumber(metadata.originYaw))
          .arg(metadata.mode)
          .arg(metadata.negate ? 1 : 0)
          .arg(displayNumber(metadata.freeThresh))
          .arg(displayNumber(metadata.occupiedThresh))
          .arg(unknownUsable ? QString::number(unknownValue) : QStringLiteral("不可用")));
  brushMeters_->setText(QStringLiteral("≈ %1 m")
                            .arg(displayNumber((brushSpin_->value() * 2 + 1) * metadata.resolution)));
}

QString MapEditorDialog::initialSavePath() const {
  if (document_ && !document_->metadata().lastExportYamlPath.isEmpty()) {
    return document_->metadata().lastExportYamlPath;
  }
  if (document_ && !document_->metadata().sourceYamlPath.isEmpty()) {
    const QFileInfo source(document_->metadata().sourceYamlPath);
    return QDir(source.absolutePath())
        .absoluteFilePath(source.completeBaseName() + QStringLiteral("_edited.yaml"));
  }
  return QDir::homePath() + QStringLiteral("/map_edited.yaml");
}
