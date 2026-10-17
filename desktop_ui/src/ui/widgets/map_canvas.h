#pragma once

#include "map_editor/map_edit_command.h"

#include <QImage>
#include <QPointF>
#include <QWidget>
#include <functional>
#include <memory>

class MapDocument;

class MapCanvas : public QWidget {
  Q_OBJECT
public:
  enum class Tool { Obstacle, Free, Unknown, ObstacleLine, FreeLine, UnknownLine };
  Q_ENUM(Tool)

  explicit MapCanvas(QWidget *parent = nullptr);

  void setDocument(MapDocument *document);
  MapDocument *document() const { return document_; }

  void setTool(Tool tool);
  Tool tool() const { return tool_; }
  void setBrushRadius(int cells);
  int brushRadius() const { return brushRadius_; }
  void setPendingCommandHandler(std::function<void(std::unique_ptr<MapEditCommand>)> handler);

  void refreshImage();
  bool cancelLinePreview();
  QPointF cellToScreen(int u, int v) const;
  bool screenToCell(const QPointF &point, int *u, int *v) const;
  double viewScale() const { return scale_; }
  bool autoFitEnabled() const { return autoFit_; }

public slots:
  void fitToView();
  void zoomIn();
  void zoomOut();

signals:
  void cursorMoved(double worldX, double worldY, bool inside);
  void statusMessage(const QString &message);
  void viewScaleChanged(double scale);

protected:
  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void leaveEvent(QEvent *event) override;

private:
  bool valueForTool(uint8_t *value) const;
  void applyPreviewAt(int u, int v);
  void resetPendingCommand();
  void emitCursor(const QPointF &screenPoint);
  void zoomBy(double factor, const QPointF &anchor);
  void updateHoverPreview(const QPointF &screenPoint);
  void clearHoverPreview();
  void setManualView();

  MapDocument *document_ = nullptr;
  QImage image_;
  Tool tool_ = Tool::Obstacle;
  int brushRadius_ = 2;
  double scale_ = 1.0;
  QPointF offset_{0.0, 0.0};
  bool autoFit_ = true;
  bool hoverPreviewVisible_ = false;
  int hoverU_ = -1;
  int hoverV_ = -1;

  bool panning_ = false;
  QPoint lastPanPosition_;

  bool painting_ = false;
  int lastPaintU_ = -1;
  int lastPaintV_ = -1;
  std::unique_ptr<MapEditCommand> pendingCommand_;

  bool linePreviewVisible_ = false;
  int lineStartU_ = -1;
  int lineStartV_ = -1;
  int lineEndU_ = -1;
  int lineEndV_ = -1;

  std::function<void(std::unique_ptr<MapEditCommand>)> commandHandler_;
};
