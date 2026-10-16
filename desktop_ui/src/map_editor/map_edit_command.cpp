#include "map_editor/map_edit_command.h"
#include "map_editor/map_document.h"

#include <QtGlobal>
#include <algorithm>
#include <cmath>

MapEditCommand::MapEditCommand(MapDocument *document) : document_(document) {}

void MapEditCommand::setCell(int u, int v, uint8_t value) {
  if (!document_ || !document_->contains(u, v)) return;
  const int cell = document_->index(u, v);
  if (!before_.contains(cell)) before_.insert(cell, document_->pixelByIndex(cell));
  after_[cell] = value;
  document_->setPixelByIndex(cell, value);
}

void MapEditCommand::paintCircle(int centerU, int centerV, int radius, uint8_t value) {
  if (!document_ || radius < 0) return;
  const int radiusSquared = radius * radius;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      if (dx * dx + dy * dy <= radiusSquared) setCell(centerU + dx, centerV + dy, value);
    }
  }
}

void MapEditCommand::paintLine(int u0, int v0, int u1, int v1, int radius, uint8_t value) {
  if (!document_) return;
  int x = u0;
  int y = v0;
  const int dx = std::abs(u1 - u0);
  const int sx = u0 < u1 ? 1 : -1;
  const int dy = -std::abs(v1 - v0);
  const int sy = v0 < v1 ? 1 : -1;
  int error = dx + dy;
  while (true) {
    paintCircle(x, y, radius, value);
    if (x == u1 && y == v1) break;
    const int error2 = 2 * error;
    if (error2 >= dy) {
      error += dy;
      x += sx;
    }
    if (error2 <= dx) {
      error += dx;
      y += sy;
    }
  }
}

void MapEditCommand::undo() {
  if (!document_) return;
  for (auto it = before_.constBegin(); it != before_.constEnd(); ++it) {
    document_->setPixelByIndex(it.key(), it.value());
  }
}

void MapEditCommand::redo() {
  if (!document_) return;
  for (auto it = after_.constBegin(); it != after_.constEnd(); ++it) {
    document_->setPixelByIndex(it.key(), it.value());
  }
}

qint64 MapEditCommand::approximateBytes() const {
  // QHash node overhead plus a payload byte and an index int. This is only used
  // as a soft history-memory budget; exact allocator accounting is not needed.
  return static_cast<qint64>(before_.size() + after_.size()) * 48;
}

MapHistory::MapHistory(int maxSteps, qint64 maxBytes)
    : maxSteps_(std::max(1, maxSteps)), maxBytes_(std::max<qint64>(1, maxBytes)) {}

void MapHistory::push(std::unique_ptr<MapEditCommand> command) {
  if (!command || command->empty()) return;

  if (index_ < static_cast<int>(commands_.size())) {
    for (int i = index_; i < static_cast<int>(commands_.size()); ++i) {
      bytes_ -= commands_[static_cast<size_t>(i)]->approximateBytes();
    }
    commands_.erase(commands_.begin() + index_, commands_.end());
  }

  command->setRevisions(currentRevision_, nextRevision_++);
  currentRevision_ = command->afterRevision();
  bytes_ += command->approximateBytes();
  commands_.push_back(std::move(command));
  index_ = static_cast<int>(commands_.size());
  trim();
}

bool MapHistory::undo() {
  if (!canUndo()) return false;
  auto &command = commands_[static_cast<size_t>(index_ - 1)];
  command->undo();
  currentRevision_ = command->beforeRevision();
  --index_;
  return true;
}

bool MapHistory::redo() {
  if (!canRedo()) return false;
  auto &command = commands_[static_cast<size_t>(index_)];
  command->redo();
  currentRevision_ = command->afterRevision();
  ++index_;
  return true;
}

void MapHistory::clear() {
  commands_.clear();
  index_ = 0;
  bytes_ = 0;
  currentRevision_ = 0;
  nextRevision_ = 1;
}

void MapHistory::trim() {
  while (!commands_.empty() &&
         (static_cast<int>(commands_.size()) > maxSteps_ || (bytes_ > maxBytes_ && commands_.size() > 1))) {
    bytes_ -= commands_.front()->approximateBytes();
    commands_.erase(commands_.begin());
    if (index_ > 0) --index_;
  }
}
