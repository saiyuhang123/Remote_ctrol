#include "ui/widgets/map_canvas.h"
#include "map_editor/map_document.h"

#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace {
bool isLineTool(MapCanvas::Tool tool) {
  return tool == MapCanvas::Tool::ObstacleLine || tool == MapCanvas::Tool::FreeLine ||
         tool == MapCanvas::Tool::UnknownLine;
}

QRgb colorForSemantic(int semantic) {
  if (semantic == 100) return qRgb(34, 38, 42);
  if (semantic == 0) return qRgb(238, 240, 242);
  return qRgb(156, 156, 156);
}
}

MapCanvas::MapCanvas(QWidget *parent) : QWidget(parent) {
  setObjectName(QStringLiteral("mapCanvas"));
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
  setMinimumSize(420, 320);
  setCursor(Qt::CrossCursor);
}

void MapCanvas::setDocument(MapDocument *document) {
  document_ = document;
  resetPendingCommand();
  linePreviewVisible_ = false;
  hoverPreviewVisible_ = false;
  autoFit_ = true;
  image_ = QImage();
  if (document_ && document_->valid()) {
    refreshImage();
    fitToView();
  } else {
    scale_ = 1.0;
    offset_ = QPointF(0.0, 0.0);
  }
  update();
}

void MapCanvas::setTool(Tool tool) {
  tool_ = tool;
  linePreviewVisible_ = false;
  clearHoverPreview();
  resetPendingCommand();
  update();
}

void MapCanvas::setBrushRadius(int cells) {
  brushRadius_ = std::clamp(cells, 0, 200);
  if (hoverPreviewVisible_) update();
}

void MapCanvas::setPendingCommandHandler(
    std::function<void(std::unique_ptr<MapEditCommand>)> handler) {
  commandHandler_ = std::move(handler);
}

void MapCanvas::refreshImage() {
  if (!document_ || !document_->valid()) {
    image_ = QImage();
    return;
  }
  if (image_.width() != document_->width() || image_.height() != document_->height()) {
    image_ = QImage(document_->width(), document_->height(), QImage::Format_Indexed8);
  }
  QVector<QRgb> colorTable(256);
  for (int value = 0; value < 256; ++value) {
    colorTable[value] = colorForSemantic(document_->semanticValue(static_cast<uint8_t>(value)));
  }
  image_.setColorTable(colorTable);
  const int rowBytes = document_->width();
  for (int row = 0; row < document_->height(); ++row) {
    std::copy_n(document_->pixels().data() + static_cast<size_t>(row) * rowBytes, rowBytes,
                image_.scanLine(row));
  }
  update();
}

bool MapCanvas::cancelLinePreview() {
  if (!linePreviewVisible_) return false;
  linePreviewVisible_ = false;
  emit statusMessage(QStringLiteral("已取消直线预览"));
  update();
  return true;
}

QPointF MapCanvas::cellToScreen(int u, int v) const {
  return QPointF(offset_.x() + u * scale_, offset_.y() + v * scale_);
}

bool MapCanvas::screenToCell(const QPointF &point, int *u, int *v) const {
  if (!document_ || scale_ <= 0.0) return false;
  const double imageX = (point.x() - offset_.x()) / scale_;
  const double imageY = (point.y() - offset_.y()) / scale_;
  const int candidateU = static_cast<int>(std::floor(imageX));
  const int candidateV = static_cast<int>(std::floor(imageY));
  if (!document_->contains(candidateU, candidateV)) return false;
  if (u) *u = candidateU;
  if (v) *v = candidateV;
  return true;
}

bool MapCanvas::valueForTool(uint8_t *value) const {
  if (!document_ || !value) return false;
  MapCellState state = MapCellState::Occupied;
  if (tool_ == Tool::Free || tool_ == Tool::FreeLine) state = MapCellState::Free;
  else if (tool_ == Tool::Unknown || tool_ == Tool::UnknownLine) state = MapCellState::Unknown;
  return document_->encodingForState(state, value, nullptr);
}

void MapCanvas::paintEvent(QPaintEvent *event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), QColor(21, 24, 28));
  if (!document_ || image_.isNull()) {
    painter.setPen(QColor(160, 168, 176));
    painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("尚未打开地图"));
    return;
  }

  const QRectF target(offset_.x(), offset_.y(), document_->width() * scale_,
                      document_->height() * scale_);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
  painter.drawImage(target, image_);

  if (scale_ >= 3.0 && std::isfinite(document_->metadata().resolution) &&
      document_->metadata().resolution > 0.0) {
    QPen gridPen(QColor(80, 90, 100, 90));
    gridPen.setWidthF(0.5);
    painter.setPen(gridPen);
    const int stepCells = std::max(
        1, static_cast<int>(std::round(1.0 / document_->metadata().resolution)));
    if (stepCells * scale_ >= 12.0) {
      for (int u = stepCells; u < document_->width(); u += stepCells) {
        const double x = offset_.x() + u * scale_;
        painter.drawLine(QPointF(x, target.top()), QPointF(x, target.bottom()));
      }
      for (int v = stepCells; v < document_->height(); v += stepCells) {
        const double y = offset_.y() + v * scale_;
        painter.drawLine(QPointF(target.left(), y), QPointF(target.right(), y));
      }
    }
  }

  if (hoverPreviewVisible_ && hoverU_ >= 0 && hoverV_ >= 0 && !linePreviewVisible_) {
    QColor fill(31, 111, 235, 55);
    if (tool_ == Tool::Free || tool_ == Tool::FreeLine) fill = QColor(255, 255, 255, 165);
    else if (tool_ == Tool::Unknown || tool_ == Tool::UnknownLine) fill = QColor(156, 156, 156, 120);
    else fill = QColor(15, 18, 22, 62);
    painter.setBrush(fill);
    QPen outline(QColor(31, 111, 235, 230));
    outline.setWidthF(std::max(1.0, 1.5));
    painter.setPen(outline);
    if (brushRadius_ <= 0) {
      painter.drawRect(QRectF(cellToScreen(hoverU_, hoverV_), QSizeF(scale_, scale_)));
    } else {
      const QPointF center =
          cellToScreen(hoverU_, hoverV_) + QPointF(scale_ / 2.0, scale_ / 2.0);
      const double radius = (brushRadius_ + 0.5) * scale_;
      painter.drawEllipse(center, radius, radius);
    }
  }

  if (linePreviewVisible_ && isLineTool(tool_)) {
    const QPointF start =
        cellToScreen(lineStartU_, lineStartV_) + QPointF(scale_ / 2.0, scale_ / 2.0);
    const QPointF end =
        cellToScreen(lineEndU_, lineEndV_) + QPointF(scale_ / 2.0, scale_ / 2.0);
    const double width = std::max(1.0, (brushRadius_ * 2.0 + 1.0) * scale_);
    QColor stateColor(31, 111, 235, 170);
    if (tool_ == Tool::FreeLine) stateColor = QColor(245, 245, 245, 190);
    else if (tool_ == Tool::UnknownLine) stateColor = QColor(156, 156, 156, 190);
    else stateColor = QColor(36, 40, 44, 190);
    QPen outline(QColor(31, 111, 235, 235));
    outline.setWidthF(width + 2.0);
    outline.setCapStyle(Qt::RoundCap);
    painter.setPen(outline);
    painter.drawLine(start, end);
    QPen fillPen(stateColor);
    fillPen.setWidthF(width);
    fillPen.setCapStyle(Qt::RoundCap);
    painter.setPen(fillPen);
    painter.drawLine(start, end);
  }
}

void MapCanvas::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  if (autoFit_) {
    fitToView();
    return;
  }
  const QSize oldSize = event->oldSize();
  if (oldSize.isValid()) {
    offset_ += QPointF((width() - oldSize.width()) / 2.0,
                       (height() - oldSize.height()) / 2.0);
  }
  update();
}

void MapCanvas::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton) {
    panning_ = true;
    setManualView();
    clearHoverPreview();
    lastPanPosition_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }
  if (event->button() != Qt::LeftButton || !document_) {
    event->ignore();
    return;
  }

  int u = 0;
  int v = 0;
  if (!screenToCell(event->localPos(), &u, &v)) {
    emit statusMessage(QStringLiteral("请在有效地图范围内操作"));
    event->accept();
    return;
  }

  clearHoverPreview();
  if (isLineTool(tool_)) {
    linePreviewVisible_ = true;
    lineStartU_ = lineEndU_ = u;
    lineStartV_ = lineEndV_ = v;
    update();
  } else {
    uint8_t value = 0;
    if (!valueForTool(&value)) {
      emit statusMessage(QStringLiteral("当前地图元数据无法表示所选工具"));
      event->accept();
      return;
    }
    pendingCommand_ = std::make_unique<MapEditCommand>(document_);
    painting_ = true;
    pendingCommand_->paintCircle(u, v, brushRadius_, value);
    lastPaintU_ = u;
    lastPaintV_ = v;
    refreshImage();
    update();
  }
  event->accept();
}

void MapCanvas::mouseMoveEvent(QMouseEvent *event) {
  if (panning_) {
    offset_ += QPointF(event->pos() - lastPanPosition_);
    lastPanPosition_ = event->pos();
    setManualView();
    clearHoverPreview();
    update();
    event->accept();
    return;
  }

  int u = 0;
  int v = 0;
  const bool inside = screenToCell(event->localPos(), &u, &v);
  if (linePreviewVisible_ && inside) {
    lineEndU_ = u;
    lineEndV_ = v;
    clearHoverPreview();
    update();
  } else if (painting_ && (event->buttons() & Qt::LeftButton) && inside && pendingCommand_) {
    if (u != lastPaintU_ || v != lastPaintV_) {
      uint8_t value = 0;
      if (valueForTool(&value)) {
        pendingCommand_->paintLine(lastPaintU_, lastPaintV_, u, v, brushRadius_, value);
      }
      lastPaintU_ = u;
      lastPaintV_ = v;
      refreshImage();
      clearHoverPreview();
      update();
    }
  } else if (!painting_ && !linePreviewVisible_) {
    updateHoverPreview(event->localPos());
  }
  emitCursor(event->localPos());
  event->accept();
}

void MapCanvas::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton) {
    panning_ = false;
    setCursor(Qt::CrossCursor);
    event->accept();
    return;
  }
  if (event->button() != Qt::LeftButton) {
    event->ignore();
    return;
  }

  if (linePreviewVisible_ && isLineTool(tool_) && lineStartU_ >= 0) {
    int releaseU = 0;
    int releaseV = 0;
    if (screenToCell(event->localPos(), &releaseU, &releaseV)) {
      lineEndU_ = releaseU;
      lineEndV_ = releaseV;
    }
    uint8_t value = 0;
    if (valueForTool(&value)) {
      auto command = std::make_unique<MapEditCommand>(document_);
      command->paintLine(lineStartU_, lineStartV_, lineEndU_, lineEndV_, brushRadius_, value);
      if (!command->empty() && commandHandler_) commandHandler_(std::move(command));
      refreshImage();
    }
  } else if (painting_) {
    painting_ = false;
    lastPaintU_ = lastPaintV_ = -1;
    if (pendingCommand_ && !pendingCommand_->empty() && commandHandler_) {
      commandHandler_(std::move(pendingCommand_));
    } else {
      pendingCommand_.reset();
    }
  }
  linePreviewVisible_ = false;
  refreshImage();
  updateHoverPreview(event->localPos());
  update();
  event->accept();
}

void MapCanvas::wheelEvent(QWheelEvent *event) {
  if (!document_) {
    event->ignore();
    return;
  }
  const int delta = event->angleDelta().y();
  if (delta == 0) {
    event->ignore();
    return;
  }
  zoomBy(delta > 0 ? 1.25 : 0.8, event->position());
  event->accept();
}

void MapCanvas::leaveEvent(QEvent *event) {
  clearHoverPreview();
  emit cursorMoved(0.0, 0.0, false);
  QWidget::leaveEvent(event);
}

void MapCanvas::zoomBy(double factor, const QPointF &anchor) {
  if (!document_) return;
  setManualView();
  const double newScale = std::clamp(scale_ * factor, 0.02, 200.0);
  if (std::abs(newScale - scale_) < 1e-9) return;
  const QPointF imagePoint((anchor.x() - offset_.x()) / scale_,
                           (anchor.y() - offset_.y()) / scale_);
  scale_ = newScale;
  offset_ = anchor - imagePoint * scale_;
  emit viewScaleChanged(scale_);
  update();
}

void MapCanvas::fitToView() {
  if (!document_ || document_->width() <= 0 || document_->height() <= 0) return;
  autoFit_ = true;
  const double availableWidth = std::max(1, width() - 24);
  const double availableHeight = std::max(1, height() - 24);
  scale_ = std::clamp(std::min(availableWidth / document_->width(),
                               availableHeight / document_->height()),
                      0.02, 200.0);
  offset_ = QPointF((width() - document_->width() * scale_) / 2.0,
                    (height() - document_->height() * scale_) / 2.0);
  emit viewScaleChanged(scale_);
  update();
}

void MapCanvas::zoomIn() {
  zoomBy(1.25, rect().center());
}

void MapCanvas::zoomOut() {
  zoomBy(0.8, rect().center());
}

void MapCanvas::setManualView() {
  autoFit_ = false;
}

void MapCanvas::clearHoverPreview() {
  if (!hoverPreviewVisible_ && hoverU_ < 0 && hoverV_ < 0) return;
  hoverPreviewVisible_ = false;
  hoverU_ = hoverV_ = -1;
  update();
}

void MapCanvas::updateHoverPreview(const QPointF &screenPoint) {
  if (!document_ || panning_ || painting_ || linePreviewVisible_) {
    clearHoverPreview();
    return;
  }
  int u = 0;
  int v = 0;
  if (!screenToCell(screenPoint, &u, &v)) {
    clearHoverPreview();
    return;
  }
  if (!hoverPreviewVisible_ || hoverU_ != u || hoverV_ != v) {
    hoverPreviewVisible_ = true;
    hoverU_ = u;
    hoverV_ = v;
    update();
  }
}

void MapCanvas::emitCursor(const QPointF &screenPoint) {
  if (!document_) {
    emit cursorMoved(0.0, 0.0, false);
    return;
  }
  int u = 0;
  int v = 0;
  if (!screenToCell(screenPoint, &u, &v)) {
    emit cursorMoved(0.0, 0.0, false);
    return;
  }
  const QPointF world = document_->cellCenterWorld(u, v);
  emit cursorMoved(world.x(), world.y(), true);
}

void MapCanvas::resetPendingCommand() {
  pendingCommand_.reset();
  painting_ = false;
  lastPaintU_ = lastPaintV_ = -1;
}
