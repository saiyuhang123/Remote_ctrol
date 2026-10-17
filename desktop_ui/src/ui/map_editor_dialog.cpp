#include "ui/map_editor_dialog.h"
#include "map_editor/map_document.h"
#include "map_editor/map_io.h"
#include "ui/widgets/map_canvas.h"

#include <QAbstractSpinBox>
#include <QApplication>
#include <QButtonGroup>
#include <QClipboard>
#include <QCloseEvent>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontMetrics>
#include <QFrame>
#include <QGuiApplication>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QScreen>
#include <QShortcut>
#include <QSpinBox>
#include <QSplitter>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace {
QString displayNumber(double value) {
  return QString::number(value, 'g', 12);
}

QString pathIdentity(const QString &path) {
  if (path.trimmed().isEmpty()) return {};
  const QFileInfo info(path);
  const QString canonical = info.canonicalFilePath();
  return canonical.isEmpty() ? info.absoluteFilePath() : canonical;
}

QIcon stateIcon(const QColor &color) {
  QPixmap pixmap(16, 16);
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QColor(QStringLiteral("#7a8699")));
  painter.setBrush(color);
  painter.drawRoundedRect(QRectF(1.5, 1.5, 13.0, 13.0), 2.5, 2.5);
  return QIcon(pixmap);
}

QLabel *sectionTitle(const QString &text, QWidget *parent) {
  auto *label = new QLabel(text, parent);
  label->setProperty("sectionTitle", true);
  return label;
}

QLabel *legendSwatch(const QColor &color, QWidget *parent) {
  auto *label = new QLabel(parent);
  label->setFixedSize(14, 14);
  label->setStyleSheet(QStringLiteral("background:%1;border:1px solid #7a8699;border-radius:2px;")
                           .arg(color.name()));
  return label;
}

class CanvasContainer : public QWidget {
public:
  explicit CanvasContainer(QWidget *parent = nullptr) : QWidget(parent) {
    setObjectName(QStringLiteral("mapEditorCanvasContainer"));
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    canvas_ = new MapCanvas(this);
    layout->addWidget(canvas_, 1);

    viewBar_ = new QFrame(this);
    viewBar_->setObjectName(QStringLiteral("mapEditorViewBar"));
    auto *bar = new QHBoxLayout(viewBar_);
    bar->setContentsMargins(7, 4, 7, 4);
    bar->setSpacing(4);
    zoomOutButton_ = new QPushButton(QStringLiteral("−"), viewBar_);
    zoomOutButton_->setObjectName(QStringLiteral("mapEditorZoomOut"));
    zoomOutButton_->setFixedSize(28, 28);
    zoomOutButton_->setToolTip(QStringLiteral("缩小"));
    zoomValueLabel_ = new QLabel(QStringLiteral("1.00×"), viewBar_);
    zoomValueLabel_->setObjectName(QStringLiteral("mapEditorZoom"));
    zoomValueLabel_->setAlignment(Qt::AlignCenter);
    zoomValueLabel_->setMinimumWidth(54);
    fitButton_ = new QPushButton(QStringLiteral("适应"), viewBar_);
    fitButton_->setObjectName(QStringLiteral("mapEditorFit"));
    fitButton_->setToolTip(QStringLiteral("适应窗口 (F)"));
    fitButton_->setMinimumWidth(50);
    zoomInButton_ = new QPushButton(QStringLiteral("+"), viewBar_);
    zoomInButton_->setObjectName(QStringLiteral("mapEditorZoomIn"));
    zoomInButton_->setFixedSize(28, 28);
    zoomInButton_->setToolTip(QStringLiteral("放大"));
    bar->addWidget(zoomOutButton_);
    bar->addWidget(zoomValueLabel_);
    bar->addWidget(fitButton_);
    bar->addWidget(zoomInButton_);
    viewBar_->adjustSize();
    viewBar_->raise();
  }

  MapCanvas *canvas() const { return canvas_; }
  QPushButton *zoomOutButton() const { return zoomOutButton_; }
  QPushButton *zoomInButton() const { return zoomInButton_; }
  QPushButton *fitButton() const { return fitButton_; }
  QLabel *zoomValueLabel() const { return zoomValueLabel_; }

protected:
  void resizeEvent(QResizeEvent *event) override {
    QWidget::resizeEvent(event);
    if (!viewBar_) return;
    viewBar_->adjustSize();
    viewBar_->move(std::max(8, width() - viewBar_->width() - 12), 12);
    viewBar_->raise();
  }

private:
  MapCanvas *canvas_ = nullptr;
  QFrame *viewBar_ = nullptr;
  QPushButton *zoomOutButton_ = nullptr;
  QPushButton *zoomInButton_ = nullptr;
  QPushButton *fitButton_ = nullptr;
  QLabel *zoomValueLabel_ = nullptr;
};
}  // namespace

MapEditorDialog::MapEditorDialog(const QString &initialYamlPath, QWidget *parent)
    : QDialog(parent) {
  setObjectName(QStringLiteral("mapEditorDialog"));
  setWindowTitle(QStringLiteral("地图编辑"));
  setMinimumSize(760, 480);

  QScreen *screen = parentWidget() ? parentWidget()->screen() : QGuiApplication::primaryScreen();
  const QRect available = screen ? screen->availableGeometry() : QRect(0, 0, 1440, 900);
  const int targetWidth = std::min(1120, std::max(760, available.width() - 60));
  const int targetHeight = std::min(780, std::max(480, available.height() - 60));
  resize(targetWidth, targetHeight);

  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(8);

  // Top file bar.
  auto *topBar = new QFrame(this);
  topBar->setObjectName(QStringLiteral("mapEditorTopBar"));
  auto *topLayout = new QHBoxLayout(topBar);
  topLayout->setContentsMargins(10, 7, 10, 7);
  topLayout->setSpacing(8);
  openButton_ = new QPushButton(QStringLiteral("打开地图"), topBar);
  openButton_->setObjectName(QStringLiteral("mapEditorOpen"));
  openButton_->setIcon(style()->standardIcon(QStyle::SP_DialogOpenButton));
  undoButton_ = new QPushButton(QStringLiteral("撤销"), topBar);
  undoButton_->setObjectName(QStringLiteral("mapEditorUndo"));
  undoButton_->setIcon(style()->standardIcon(QStyle::SP_ArrowBack));
  undoButton_->setToolTip(QStringLiteral("撤销 (Ctrl+Z)"));
  redoButton_ = new QPushButton(QStringLiteral("重做"), topBar);
  redoButton_->setObjectName(QStringLiteral("mapEditorRedo"));
  redoButton_->setIcon(style()->standardIcon(QStyle::SP_ArrowForward));
  redoButton_->setToolTip(QStringLiteral("重做 (Ctrl+Y / Ctrl+Shift+Z)"));
  currentFileLabel_ = new QLabel(QStringLiteral("未打开地图"), topBar);
  currentFileLabel_->setObjectName(QStringLiteral("mapEditorCurrentFile"));
  currentFileLabel_->setAlignment(Qt::AlignCenter);
  currentFileLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  currentFileLabel_->setMinimumWidth(0);
  currentFileLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  infoToggleButton_ = new QPushButton(QStringLiteral("地图信息"), topBar);
  infoToggleButton_->setObjectName(QStringLiteral("mapEditorInfoToggle"));
  infoToggleButton_->setCheckable(true);
  infoToggleButton_->setIcon(style()->standardIcon(QStyle::SP_FileDialogDetailedView));
  saveAsButton_ = new QPushButton(QStringLiteral("另存为"), topBar);
  saveAsButton_->setObjectName(QStringLiteral("mapEditorSaveAs"));
  saveAsButton_->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
  saveAsButton_->setProperty("primary", true);
  topLayout->addWidget(openButton_);
  topLayout->addWidget(undoButton_);
  topLayout->addWidget(redoButton_);
  topLayout->addSpacing(6);
  topLayout->addWidget(currentFileLabel_, 1);
  topLayout->addWidget(infoToggleButton_);
  topLayout->addWidget(saveAsButton_);
  root->addWidget(topBar);

  // Left tool panel.
  auto *leftPanel = new QFrame(this);
  leftPanel->setObjectName(QStringLiteral("mapEditorToolPanel"));
  leftPanel->setMinimumWidth(190);
  leftPanel->setMaximumWidth(235);
  auto *leftOuter = new QVBoxLayout(leftPanel);
  leftOuter->setContentsMargins(0, 0, 0, 0);
  auto *leftScroll = new QScrollArea(leftPanel);
  leftScroll->setWidgetResizable(true);
  leftScroll->setFrameShape(QFrame::NoFrame);
  leftScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto *toolsContent = new QWidget(leftScroll);
  auto *left = new QVBoxLayout(toolsContent);
  left->setContentsMargins(12, 12, 12, 12);
  left->setSpacing(10);

  left->addWidget(sectionTitle(QStringLiteral("绘制方式"), toolsContent));
  auto *shapeRow = new QHBoxLayout;
  shapeBrushButton_ = new QPushButton(QStringLiteral("画笔"), toolsContent);
  shapeBrushButton_->setObjectName(QStringLiteral("mapEditorShapeBrush"));
  shapeBrushButton_->setCheckable(true);
  shapeLineButton_ = new QPushButton(QStringLiteral("直线"), toolsContent);
  shapeLineButton_->setObjectName(QStringLiteral("mapEditorShapeLine"));
  shapeLineButton_->setCheckable(true);
  shapeRow->addWidget(shapeBrushButton_);
  shapeRow->addWidget(shapeLineButton_);
  left->addLayout(shapeRow);
  auto *shapeGroup = new QButtonGroup(this);
  shapeGroup->setExclusive(true);
  shapeGroup->addButton(shapeBrushButton_);
  shapeGroup->addButton(shapeLineButton_);
  shapeBrushButton_->setChecked(true);

  left->addWidget(sectionTitle(QStringLiteral("栅格类型"), toolsContent));
  auto *cellGroupLayout = new QVBoxLayout;
  cellGroupLayout->setSpacing(6);
  cellObstacleButton_ = new QPushButton(QStringLiteral("障碍"), toolsContent);
  cellObstacleButton_->setObjectName(QStringLiteral("mapEditorCellObstacle"));
  cellObstacleButton_->setIcon(stateIcon(QColor(34, 38, 42)));
  cellObstacleButton_->setCheckable(true);
  cellFreeButton_ = new QPushButton(QStringLiteral("空闲"), toolsContent);
  cellFreeButton_->setObjectName(QStringLiteral("mapEditorCellFree"));
  cellFreeButton_->setIcon(stateIcon(QColor(238, 240, 242)));
  cellFreeButton_->setCheckable(true);
  cellUnknownButton_ = new QPushButton(QStringLiteral("未知"), toolsContent);
  cellUnknownButton_->setObjectName(QStringLiteral("mapEditorCellUnknown"));
  cellUnknownButton_->setIcon(stateIcon(QColor(156, 156, 156)));
  cellUnknownButton_->setCheckable(true);
  cellObstacleButton_->setToolTip(QStringLiteral("绘制静态障碍，车辆不可通行"));
  cellFreeButton_->setToolTip(QStringLiteral("绘制空闲区域，车辆可通行"));
  cellUnknownButton_->setToolTip(QStringLiteral("未知表示未观测区域，不等于禁止通行"));
  cellGroupLayout->addWidget(cellObstacleButton_);
  cellGroupLayout->addWidget(cellFreeButton_);
  cellGroupLayout->addWidget(cellUnknownButton_);
  left->addLayout(cellGroupLayout);
  auto *cellGroup = new QButtonGroup(this);
  cellGroup->setExclusive(true);
  cellGroup->addButton(cellObstacleButton_);
  cellGroup->addButton(cellFreeButton_);
  cellGroup->addButton(cellUnknownButton_);
  cellObstacleButton_->setChecked(true);

  left->addWidget(sectionTitle(QStringLiteral("笔刷"), toolsContent));
  auto *brushLabel = new QLabel(QStringLiteral("半径（格）"), toolsContent);
  brushSpin_ = new QSpinBox(toolsContent);
  brushSpin_->setObjectName(QStringLiteral("mapEditorBrush"));
  brushSpin_->setRange(0, 200);
  brushSpin_->setValue(2);
  brushSpin_->setSuffix(QStringLiteral(" 格"));
  brushSummaryLabel_ = new QLabel(toolsContent);
  brushSummaryLabel_->setObjectName(QStringLiteral("mapEditorBrushSummary"));
  brushSummaryLabel_->setWordWrap(true);
  left->addWidget(brushLabel);
  left->addWidget(brushSpin_);
  left->addWidget(brushSummaryLabel_);

  left->addSpacing(4);
  left->addWidget(sectionTitle(QStringLiteral("图例"), toolsContent));
  auto *legendObstacleRow = new QHBoxLayout;
  legendObstacleRow->addWidget(legendSwatch(QColor(34, 38, 42), toolsContent));
  legendObstacleRow->addWidget(new QLabel(QStringLiteral("障碍"), toolsContent));
  legendObstacleRow->addStretch();
  left->addLayout(legendObstacleRow);
  auto *legendFreeRow = new QHBoxLayout;
  legendFreeRow->addWidget(legendSwatch(QColor(238, 240, 242), toolsContent));
  legendFreeRow->addWidget(new QLabel(QStringLiteral("空闲"), toolsContent));
  legendFreeRow->addStretch();
  left->addLayout(legendFreeRow);
  auto *legendUnknownRow = new QHBoxLayout;
  legendUnknownRow->addWidget(legendSwatch(QColor(156, 156, 156), toolsContent));
  legendUnknownRow->addWidget(new QLabel(QStringLiteral("未知"), toolsContent));
  legendUnknownRow->addStretch();
  left->addLayout(legendUnknownRow);
  auto *unknownNote = new QLabel(QStringLiteral("未知表示未观测，不等于禁止通行；需要阻行请使用“障碍”。"), toolsContent);
  unknownNote->setWordWrap(true);
  unknownNote->setStyleSheet(QStringLiteral("color:#667085;"));
  left->addWidget(unknownNote);
  left->addStretch();
  leftScroll->setWidget(toolsContent);
  leftOuter->addWidget(leftScroll);

  // Central canvas with corner view controls.
  auto *canvasContainer = new CanvasContainer(this);
  canvas_ = canvasContainer->canvas();
  fitButton_ = canvasContainer->fitButton();
  zoomInButton_ = canvasContainer->zoomInButton();
  zoomOutButton_ = canvasContainer->zoomOutButton();
  zoomValueLabel_ = canvasContainer->zoomValueLabel();

  // Right information panel.
  infoPanel_ = new QFrame(this);
  infoPanel_->setObjectName(QStringLiteral("mapEditorInfoPanel"));
  infoPanel_->setMinimumWidth(235);
  infoPanel_->setMaximumWidth(320);
  auto *infoOuter = new QVBoxLayout(infoPanel_);
  infoOuter->setContentsMargins(0, 0, 0, 0);
  auto *infoScroll = new QScrollArea(infoPanel_);
  infoScroll->setWidgetResizable(true);
  infoScroll->setFrameShape(QFrame::NoFrame);
  auto *infoContent = new QWidget(infoScroll);
  auto *info = new QVBoxLayout(infoContent);
  info->setContentsMargins(12, 12, 12, 12);
  info->setSpacing(9);
  navigationSelectionLabel_ = new QLabel(QStringLiteral("导航选择：未打开地图"), infoContent);
  navigationSelectionLabel_->setObjectName(QStringLiteral("mapEditorNavigationSelection"));
  navigationSelectionLabel_->setWordWrap(true);
  info->addWidget(sectionTitle(QStringLiteral("地图信息"), infoContent));
  info->addWidget(navigationSelectionLabel_);
  info->addWidget(sectionTitle(QStringLiteral("当前地图"), infoContent));
  dimensionsLabel_ = new QLabel(QStringLiteral("尺寸：—"), infoContent);
  resolutionLabel_ = new QLabel(QStringLiteral("分辨率：—"), infoContent);
  info->addWidget(dimensionsLabel_);
  info->addWidget(resolutionLabel_);

  info->addWidget(sectionTitle(QStringLiteral("文件路径"), infoContent));
  auto addPathRow = [infoContent, info, this](const QString &caption, const QString &labelName,
                                             const QString &copyName, QLabel **labelOut) {
    auto *captionLabel = new QLabel(caption, infoContent);
    captionLabel->setStyleSheet(QStringLiteral("color:#667085;"));
    info->addWidget(captionLabel);
    auto *row = new QHBoxLayout;
    auto *pathLabel = new QLabel(QStringLiteral("—"), infoContent);
    pathLabel->setObjectName(labelName);
    pathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    pathLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    pathLabel->setMinimumWidth(0);
    auto *copyButton = new QPushButton(QStringLiteral("复制"), infoContent);
    copyButton->setObjectName(copyName);
    copyButton->setFixedHeight(26);
    copyButton->setMinimumWidth(46);
    row->addWidget(pathLabel, 1);
    row->addWidget(copyButton);
    info->addLayout(row);
    *labelOut = pathLabel;
    connect(copyButton, &QPushButton::clicked, this, [this, pathLabel] {
      const QString fullPath = pathLabel->property("fullPath").toString();
      if (fullPath.isEmpty()) return;
      QApplication::clipboard()->setText(fullPath);
      setOperationMessage(QStringLiteral("已复制路径"), false);
    });
  };
  addPathRow(QStringLiteral("源 YAML"), QStringLiteral("mapEditorSourceYaml"),
             QStringLiteral("mapEditorCopySourceYaml"), &sourceYamlLabel_);
  addPathRow(QStringLiteral("源图片"), QStringLiteral("mapEditorSourceImage"),
             QStringLiteral("mapEditorCopySourceImage"), &sourceImageLabel_);
  addPathRow(QStringLiteral("最近导出"), QStringLiteral("mapEditorExportYaml"),
             QStringLiteral("mapEditorCopyExportYaml"), &exportYamlLabel_);

  technicalToggle_ = new QPushButton(QStringLiteral("技术参数 ▸"), infoContent);
  technicalToggle_->setObjectName(QStringLiteral("mapEditorTechnicalToggle"));
  technicalToggle_->setCheckable(true);
  technicalLabel_ = new QLabel(infoContent);
  technicalLabel_->setObjectName(QStringLiteral("mapEditorTechnicalDetails"));
  technicalLabel_->setWordWrap(true);
  technicalLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  technicalLabel_->setVisible(false);
  info->addWidget(technicalToggle_);
  info->addWidget(technicalLabel_);
  info->addStretch();
  infoScroll->setWidget(infoContent);
  infoOuter->addWidget(infoScroll);

  auto *splitter = new QSplitter(Qt::Horizontal, this);
  splitter->setChildrenCollapsible(false);
  splitter->addWidget(leftPanel);
  splitter->addWidget(canvasContainer);
  splitter->addWidget(infoPanel_);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  splitter->setStretchFactor(2, 0);
  splitter->setSizes({205, std::max(360, targetWidth - 470), 255});
  root->addWidget(splitter, 1);

  // Bottom status area: operation feedback is separate from cursor/zoom state.
  auto *statusPanel = new QFrame(this);
  statusPanel->setObjectName(QStringLiteral("mapEditorStatusBar"));
  auto *statusOuter = new QVBoxLayout(statusPanel);
  statusOuter->setContentsMargins(10, 6, 10, 6);
  statusOuter->setSpacing(4);
  operationMessageLabel_ = new QLabel(QStringLiteral("打开一张已有 YAML 地图开始编辑"), statusPanel);
  operationMessageLabel_->setObjectName(QStringLiteral("mapEditorOperationMessage"));
  operationMessageLabel_->setWordWrap(true);
  operationMessageLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  statusOuter->addWidget(operationMessageLabel_);
  auto *statusGrid = new QGridLayout;
  statusGrid->setHorizontalSpacing(18);
  statusGrid->setVerticalSpacing(2);
  toolSummaryLabel_ = new QLabel(QStringLiteral("工具：画笔 · 障碍"), statusPanel);
  toolSummaryLabel_->setObjectName(QStringLiteral("mapEditorToolSummary"));
  toolSummaryLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  cursorValueLabel_ = new QLabel(QStringLiteral("光标：—"), statusPanel);
  cursorValueLabel_->setObjectName(QStringLiteral("mapEditorCursor"));
  cursorValueLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  fileStateLabel_ = new QLabel(QStringLiteral("状态：未打开"), statusPanel);
  fileStateLabel_->setObjectName(QStringLiteral("mapEditorDirtyState"));
  statusGrid->addWidget(toolSummaryLabel_, 0, 0);
  statusGrid->addWidget(fileStateLabel_, 0, 1, Qt::AlignRight);
  statusGrid->addWidget(cursorValueLabel_, 1, 0, 1, 2);
  statusGrid->setColumnStretch(0, 1);
  statusOuter->addLayout(statusGrid);
  root->addWidget(statusPanel);

  // Local style only: do not affect the main window, RViz or camera panels.
  setStyleSheet(QStringLiteral(R"(
    QDialog#mapEditorDialog { background:#eef1f5; color:#20252b; }
    QFrame#mapEditorTopBar, QFrame#mapEditorToolPanel, QFrame#mapEditorInfoPanel, QFrame#mapEditorStatusBar {
      background:#ffffff; border:1px solid #d8dee6; border-radius:8px;
    }
    QFrame#mapEditorViewBar { background:rgba(255,255,255,235); border:1px solid #d8dee6; border-radius:6px; }
    QPushButton { min-height:30px; padding:0 10px; border:1px solid #c8d0da; border-radius:5px; background:#ffffff; color:#20252b; }
    QPushButton:hover { background:#f3f7fd; border-color:#7fa9e8; }
    QPushButton:checked { background:#e7f0ff; border-color:#2f6fed; color:#174ea6; font-weight:600; }
    QPushButton[primary="true"] { background:#1f6feb; border-color:#1f6feb; color:#ffffff; font-weight:600; }
    QPushButton[primary="true"]:hover { background:#195fc8; }
    QPushButton:disabled { color:#9aa4b2; background:#f5f7f9; border-color:#e1e5ea; }
    QSpinBox { min-height:30px; padding:0 6px; border:1px solid #c8d0da; border-radius:5px; background:#ffffff; }
    QLabel[sectionTitle="true"] { font-weight:600; color:#344054; padding-top:3px; }
    QLabel#mapEditorCurrentFile { font-size:14px; font-weight:600; color:#182230; }
    QLabel#mapEditorDirtyState[dirty="true"] { color:#b42318; font-weight:600; }
    QLabel#mapEditorDirtyState[dirty="false"] { color:#157347; font-weight:600; }
    QLabel#mapEditorNavigationSelection[selected="true"] { color:#157347; font-weight:600; }
    QLabel#mapEditorNavigationSelection[selected="false"] { color:#667085; }
    QLabel#mapEditorNavigationSelection[warning="true"] { color:#b54708; font-weight:600; }
    QLabel#mapEditorOperationMessage { color:#475467; }
    QLabel#mapEditorOperationMessage[error="true"] {
      color:#b42318; background:#fef3f2; border:1px solid #fecdca; border-radius:4px; padding:3px 6px;
    }
    QLabel#mapEditorOperationMessage[error="false"] { color:#475467; }
    QLabel#mapEditorCursor, QLabel#mapEditorToolSummary { color:#475467; }
    QScrollArea { border:none; background:transparent; }
    QSplitter::handle { background:#dfe4ea; width:1px; }
  )"));

  // Tool selection and brush controls.
  connect(shapeBrushButton_, &QPushButton::toggled, this, [this] { updateToolFromButtons(); });
  connect(shapeLineButton_, &QPushButton::toggled, this, [this] { updateToolFromButtons(); });
  connect(cellObstacleButton_, &QPushButton::toggled, this, [this] { updateToolFromButtons(); });
  connect(cellFreeButton_, &QPushButton::toggled, this, [this] { updateToolFromButtons(); });
  connect(cellUnknownButton_, &QPushButton::toggled, this, [this] { updateToolFromButtons(); });
  connect(brushSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
    canvas_->setBrushRadius(value);
    updateBrushSummary();
    updateToolSummary();
  });

  updateToolFromButtons();
  updateBrushSummary();

  // File, view and status actions.
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
  connect(fitButton_, &QPushButton::clicked, canvas_, &MapCanvas::fitToView);
  connect(zoomInButton_, &QPushButton::clicked, canvas_, &MapCanvas::zoomIn);
  connect(zoomOutButton_, &QPushButton::clicked, canvas_, &MapCanvas::zoomOut);
  connect(canvas_, &MapCanvas::cursorMoved, this, [this](double x, double y, bool valid) {
    cursorFullText_ = valid ? QStringLiteral("光标：x=%1 m，y=%2 m")
                                  .arg(x, 0, 'f', 2)
                                  .arg(y, 0, 'f', 2)
                            : QStringLiteral("光标：—");
    cursorValueLabel_->setToolTip(cursorFullText_);
    refreshStatusElision();
  });
  connect(canvas_, &MapCanvas::statusMessage, this, &MapEditorDialog::showHint);
  connect(canvas_, &MapCanvas::viewScaleChanged, this, [this](double scale) {
    zoomValueLabel_->setText(QStringLiteral("%1×").arg(QString::number(scale, 'f', 2)));
  });
  canvas_->setPendingCommandHandler([this](std::unique_ptr<MapEditCommand> command) {
    history_.push(std::move(command));
    updateUi();
  });

  // Navigation feedback is controlled by MainWindow; the editor never decides
  // whether a saved YAML is compatible with the current backend/prefix.
  infoToggleButton_->setChecked(targetWidth >= 1050);
  infoPanel_->setVisible(infoToggleButton_->isChecked());
  connect(infoToggleButton_, &QPushButton::toggled, this, [this](bool checked) {
    infoPanel_->setVisible(checked);
    infoToggleButton_->setText(checked ? QStringLiteral("收起信息") : QStringLiteral("地图信息"));
  });
  infoToggleButton_->setText(infoPanel_->isVisible() ? QStringLiteral("收起信息")
                                                     : QStringLiteral("地图信息"));
  connect(technicalToggle_, &QPushButton::toggled, this, [this](bool checked) {
    technicalLabel_->setVisible(checked);
    technicalToggle_->setText(checked ? QStringLiteral("技术参数 ▾")
                                      : QStringLiteral("技术参数 ▸"));
  });

  auto editingInput = [] {
    QWidget *focus = QApplication::focusWidget();
    return qobject_cast<QLineEdit *>(focus) != nullptr || qobject_cast<QAbstractSpinBox *>(focus) != nullptr;
  };
  auto *undoShortcut = new QShortcut(QKeySequence::Undo, this);
  connect(undoShortcut, &QShortcut::activated, this, [this, editingInput] {
    if (!editingInput()) undoButton_->click();
  });
  auto *redoShortcutY = new QShortcut(QKeySequence(QStringLiteral("Ctrl+Y")), this);
  connect(redoShortcutY, &QShortcut::activated, this, [this, editingInput] {
    if (!editingInput()) redoButton_->click();
  });
  auto *redoShortcutShiftZ = new QShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+Z")), this);
  connect(redoShortcutShiftZ, &QShortcut::activated, this, [this, editingInput] {
    if (!editingInput()) redoButton_->click();
  });
  saveAsShortcut_ = new QShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+S")), this);
  saveAsShortcut_->setObjectName(QStringLiteral("mapEditorSaveAsShortcut"));
  connect(saveAsShortcut_, &QShortcut::activated, this, [this] {
    if (saveAsButton_ && saveAsButton_->isEnabled()) saveAs();
  });
  auto *fitShortcut = new QShortcut(QKeySequence(Qt::Key_F), canvas_);
  fitShortcut->setContext(Qt::WidgetWithChildrenShortcut);
  connect(fitShortcut, &QShortcut::activated, canvas_, &MapCanvas::fitToView);

  updateUi();
  if (!initialYamlPath.trimmed().isEmpty()) {
    QString error;
    if (!loadMap(initialYamlPath.trimmed(), &error)) {
      setOperationMessage(QStringLiteral("默认地图未打开：%1").arg(error), true);
    }
  }
}

bool MapEditorDialog::loadMap(const QString &yamlPath, QString *error) {
  MapDocument loaded;
  if (!MapIo::load(yamlPath, &loaded, error)) return false;
  setDocument(std::make_unique<MapDocument>(std::move(loaded)));
  setOperationMessage(QStringLiteral("已打开：%1").arg(document_->metadata().sourceYamlPath), false);
  return true;
}

bool MapEditorDialog::saveToPath(const QString &targetYamlPath, QString *error) {
  if (!document_) {
    if (error) *error = QStringLiteral("当前没有打开地图");
    setOperationMessage(QStringLiteral("保存失败：当前没有打开地图"), true);
    return false;
  }
  QString backup;
  QString internalError;
  if (!MapIo::saveAs(document_.get(), targetYamlPath, true, &internalError, &backup)) {
    if (error) *error = internalError;
    setOperationMessage(QStringLiteral("保存失败：%1").arg(internalError), true);
    return false;
  }
  savedRevision_ = history_.currentRevision();
  const QString source = document_->metadata().sourceYamlPath;
  updateUi();
  QString message = QStringLiteral("已保存：%1").arg(document_->metadata().lastExportYamlPath);
  if (!backup.isEmpty()) message += QStringLiteral("；原目标备份：%1").arg(backup);
  setOperationMessage(message, false);
  emit mapSaved(document_->metadata().lastExportYamlPath, source);
  if (error) error->clear();
  return true;
}

void MapEditorDialog::updateNavigationSelection(const QString &selectedYamlPath, bool sourceMatches) {
  navigationSelectedYaml_ = selectedYamlPath.trimmed();
  navigationSourceMatches_ = sourceMatches;
  updateNavigationFeedback();
}

void MapEditorDialog::closeEvent(QCloseEvent *event) {
  if (!maybeSave()) {
    event->ignore();
    return;
  }
  event->accept();
}

void MapEditorDialog::keyPressEvent(QKeyEvent *event) {
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
  QString yaml = selected;
  if (QFileInfo(selected).suffix().compare(QStringLiteral("pgm"), Qt::CaseInsensitive) == 0) {
    const QFileInfo pgm(selected);
    const QDir directory(pgm.absolutePath());
    yaml = directory.absoluteFilePath(pgm.completeBaseName() + QStringLiteral(".yaml"));
    if (!QFileInfo::exists(yaml)) {
      yaml = directory.absoluteFilePath(pgm.completeBaseName() + QStringLiteral(".yml"));
    }
    if (!QFileInfo::exists(yaml)) {
      const QString error = QStringLiteral("只有 PGM 而没有对应 YAML 时不能猜测原点或分辨率，请选择地图 YAML。");
      QMessageBox::warning(this, QStringLiteral("缺少 YAML"), error);
      setOperationMessage(error, true);
      return;
    }
  }
  QString error;
  if (!loadMap(yaml, &error)) {
    setOperationMessage(QStringLiteral("打开失败：%1").arg(error), true);
    QMessageBox::critical(this, QStringLiteral("打开失败"), error);
  }
}

bool MapEditorDialog::saveAs() {
  if (!document_) {
    setOperationMessage(QStringLiteral("保存失败：当前没有打开地图"), true);
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
        document_ && document_->metadata().lastExportYamlPath == targetAbsolute &&
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
  navigationSelectedYaml_.clear();
  canvas_->setDocument(document_.get());
  updateUi();
}

void MapEditorDialog::updateUi() {
  undoButton_->setEnabled(history_.canUndo());
  redoButton_->setEnabled(history_.canRedo());
  const bool documentAvailable = document_ != nullptr;
  saveAsButton_->setEnabled(documentAvailable);
  if (saveAsShortcut_) saveAsShortcut_->setEnabled(documentAvailable);
  if (!document_) {
    currentFileLabel_->setText(QStringLiteral("未打开地图"));
    setWindowTitle(QStringLiteral("地图编辑"));
    cursorFullText_ = QStringLiteral("光标：—");
    cursorValueLabel_->setText(cursorFullText_);
    updateFileState();
    updateToolSummary();
    updateBrushSummary();
    updateInfoPanel();
    updateNavigationFeedback();
    return;
  }
  const QString fileName = QFileInfo(document_->metadata().sourceYamlPath).fileName();
  currentFileLabel_->setText(fileName + (isDirty() ? QStringLiteral(" *") : QString()));
  currentFileLabel_->setToolTip(document_->metadata().sourceYamlPath);
  setWindowTitle(QStringLiteral("地图编辑 - %1%2").arg(fileName, isDirty() ? QStringLiteral(" *") : QString()));
  updateFileState();
  updateToolSummary();
  updateBrushSummary();
  updateInfoPanel();
  updateNavigationFeedback();
}

void MapEditorDialog::updateToolFromButtons() {
  const bool line = shapeLineButton_->isChecked();
  MapCanvas::Tool tool = MapCanvas::Tool::Obstacle;
  if (cellFreeButton_->isChecked()) tool = line ? MapCanvas::Tool::FreeLine : MapCanvas::Tool::Free;
  else if (cellUnknownButton_->isChecked()) tool = line ? MapCanvas::Tool::UnknownLine : MapCanvas::Tool::Unknown;
  else tool = line ? MapCanvas::Tool::ObstacleLine : MapCanvas::Tool::Obstacle;
  canvas_->setTool(tool);
  updateToolSummary();
}

void MapEditorDialog::updateBrushSummary() {
  const int radius = brushSpin_ ? brushSpin_->value() : 0;
  const int diameter = radius * 2 + 1;
  if (document_) {
    const double meters = diameter * document_->metadata().resolution;
    brushSummaryLabel_->setText(QStringLiteral("半径 %1 格；覆盖直径 %2 格 ≈ %3 m")
                                    .arg(radius)
                                    .arg(diameter)
                                    .arg(displayNumber(meters)));
  } else {
    brushSummaryLabel_->setText(QStringLiteral("半径 %1 格；覆盖直径 %2 格").arg(radius).arg(diameter));
  }
}

void MapEditorDialog::updateFileState() {
  const bool dirty = document_ && isDirty();
  QString text;
  if (!document_) text = QStringLiteral("状态：未打开");
  else if (dirty) text = QStringLiteral("状态：未保存修改");
  else text = QStringLiteral("状态：已保存");
  fileStateLabel_->setText(text);
  fileStateLabel_->setProperty("dirty", dirty);
  repolish(fileStateLabel_);
}

void MapEditorDialog::updateNavigationFeedback() {
  if (!navigationSelectionLabel_) return;
  bool selected = false;
  bool warning = false;
  QString text;
  if (!document_) {
    text = QStringLiteral("导航选择：未打开地图");
  } else if (document_->metadata().lastExportYamlPath.isEmpty()) {
    text = QStringLiteral("导航选择：本次编辑尚未导出");
  } else if (!navigationSelectedYaml_.isEmpty() &&
             pathIdentity(navigationSelectedYaml_) ==
                 pathIdentity(document_->metadata().lastExportYamlPath)) {
    if (!navigationSourceMatches_) {
      text = QStringLiteral("导航选择：来源模式或地图前缀已变化，待确认");
      warning = true;
    } else {
      text = QStringLiteral("导航选择：已选为下次导航地图");
      selected = true;
    }
  } else if (navigationSelectedYaml_.isEmpty()) {
    text = QStringLiteral("导航选择：已保存，未自动选用");
  } else {
    text = QStringLiteral("导航选择：已保存，但当前选择指向其他 YAML");
  }
  navigationSelectionLabel_->setText(text);
  navigationSelectionLabel_->setProperty("selected", selected);
  navigationSelectionLabel_->setProperty("warning", warning);
  repolish(navigationSelectionLabel_);
}

void MapEditorDialog::updateInfoPanel() {
  if (!dimensionsLabel_) return;
  if (!document_) {
    dimensionsLabel_->setText(QStringLiteral("尺寸：—"));
    resolutionLabel_->setText(QStringLiteral("分辨率：—"));
    for (QLabel *label : {sourceYamlLabel_, sourceImageLabel_, exportYamlLabel_}) {
      if (!label) continue;
      label->setProperty("fullPath", QString());
      label->setText(QStringLiteral("—"));
      label->setToolTip(QString());
    }
    technicalLabel_->setText(QStringLiteral("—"));
    return;
  }
  const MapMetadata &metadata = document_->metadata();
  dimensionsLabel_->setText(QStringLiteral("尺寸：%1 × %2 像素")
                                .arg(document_->width())
                                .arg(document_->height()));
  resolutionLabel_->setText(QStringLiteral("分辨率：%1 m/格").arg(displayNumber(metadata.resolution)));

  const auto setPath = [this](QLabel *label, const QString &path) {
    if (!label) return;
    label->setProperty("fullPath", path);
    label->setText(path.isEmpty() ? QStringLiteral("—") : elidedPath(path));
    label->setToolTip(path);
  };
  setPath(sourceYamlLabel_, metadata.sourceYamlPath);
  setPath(sourceImageLabel_, metadata.sourceImagePath);
  setPath(exportYamlLabel_, metadata.lastExportYamlPath);

  technicalLabel_->setText(
      QStringLiteral("origin：[%1, %2, %3] rad\n"
                     "negate：%4\nfree_thresh：%5\noccupied_thresh：%6\nmode：%7")
          .arg(displayNumber(metadata.originX))
          .arg(displayNumber(metadata.originY))
          .arg(displayNumber(metadata.originYaw))
          .arg(metadata.negate ? 1 : 0)
          .arg(displayNumber(metadata.freeThresh))
          .arg(displayNumber(metadata.occupiedThresh))
          .arg(metadata.mode));
}

void MapEditorDialog::updateToolSummary() {
  if (!toolSummaryLabel_) return;
  const bool line = shapeLineButton_ && shapeLineButton_->isChecked();
  QString cell = QStringLiteral("障碍");
  if (cellFreeButton_ && cellFreeButton_->isChecked()) cell = QStringLiteral("空闲");
  else if (cellUnknownButton_ && cellUnknownButton_->isChecked()) cell = QStringLiteral("未知");
  QString summary = QStringLiteral("工具：%1 · %2 · 半径 %3 格")
                        .arg(line ? QStringLiteral("直线") : QStringLiteral("画笔"), cell)
                        .arg(brushSpin_ ? brushSpin_->value() : 0);
  if (document_) {
    const int diameter = (brushSpin_ ? brushSpin_->value() : 0) * 2 + 1;
    summary += QStringLiteral("（直径 %1 格 / %2 m）")
                   .arg(diameter)
                   .arg(displayNumber(diameter * document_->metadata().resolution));
  }
  toolSummaryFullText_ = summary;
  toolSummaryLabel_->setToolTip(summary);
  refreshStatusElision();
}

void MapEditorDialog::resizeEvent(QResizeEvent *event) {
  QDialog::resizeEvent(event);
  QTimer::singleShot(0, this, [this] { refreshStatusElision(); });
}

void MapEditorDialog::refreshStatusElision() {
  if (toolSummaryLabel_) {
    const int available = std::max(40, toolSummaryLabel_->width());
    toolSummaryLabel_->setText(
        toolSummaryFullText_.isEmpty()
            ? toolSummaryFullText_
            : toolSummaryLabel_->fontMetrics().elidedText(toolSummaryFullText_, Qt::ElideRight, available));
  }
  if (cursorValueLabel_) {
    const int available = std::max(40, cursorValueLabel_->width());
    cursorValueLabel_->setText(
        cursorFullText_.isEmpty()
            ? cursorFullText_
            : cursorValueLabel_->fontMetrics().elidedText(cursorFullText_, Qt::ElideRight, available));
  }
}

void MapEditorDialog::setOperationMessage(const QString &message, bool error) {
  if (!operationMessageLabel_) return;
  operationMessageLabel_->setText(message);
  operationIsError_ = error;
  operationMessageLabel_->setProperty("error", error);
  repolish(operationMessageLabel_);
}

void MapEditorDialog::showHint(const QString &message) {
  if (operationIsError_) return;
  setOperationMessage(message, false);
}

void MapEditorDialog::repolish(QWidget *widget) {
  if (!widget) return;
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
  widget->update();
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

QString MapEditorDialog::elidedPath(const QString &path) const {
  if (path.isEmpty()) return {};
  return QFontMetrics(font()).elidedText(path, Qt::ElideMiddle, 230);
}
