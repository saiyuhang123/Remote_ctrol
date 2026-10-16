#pragma once

#include <QHash>
#include <cstdint>
#include <memory>
#include <vector>

class MapDocument;

// A single mouse-down-to-mouse-up edit. Only changed cells are stored, with
// their exact before/after values, so undo restores the original bytes.
class MapEditCommand {
public:
  explicit MapEditCommand(MapDocument *document);

  void paintCircle(int centerU, int centerV, int radius, uint8_t value);
  void paintLine(int u0, int v0, int u1, int v1, int radius, uint8_t value);
  void undo();
  void redo();

  bool empty() const { return before_.isEmpty(); }
  int changedCellCount() const { return before_.size(); }
  qint64 approximateBytes() const;

  quint64 beforeRevision() const { return beforeRevision_; }
  quint64 afterRevision() const { return afterRevision_; }
  void setRevisions(quint64 before, quint64 after) {
    beforeRevision_ = before;
    afterRevision_ = after;
  }

private:
  void setCell(int u, int v, uint8_t value);

  MapDocument *document_ = nullptr;
  QHash<int, uint8_t> before_;
  QHash<int, uint8_t> after_;
  quint64 beforeRevision_ = 0;
  quint64 afterRevision_ = 0;
};

class MapHistory {
public:
  explicit MapHistory(int maxSteps = 100, qint64 maxBytes = 64LL * 1024 * 1024);

  bool canUndo() const { return index_ > 0; }
  bool canRedo() const { return index_ < static_cast<int>(commands_.size()); }
  int index() const { return index_; }
  int count() const { return static_cast<int>(commands_.size()); }
  qint64 approximateBytes() const { return bytes_; }
  quint64 currentRevision() const { return currentRevision_; }

  void push(std::unique_ptr<MapEditCommand> command);
  bool undo();
  bool redo();
  void clear();

private:
  void trim();

  std::vector<std::unique_ptr<MapEditCommand>> commands_;
  int index_ = 0;
  quint64 currentRevision_ = 0;
  quint64 nextRevision_ = 1;
  int maxSteps_ = 100;
  qint64 maxBytes_ = 64LL * 1024 * 1024;
  qint64 bytes_ = 0;
};
