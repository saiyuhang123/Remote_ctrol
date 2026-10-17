#pragma once

#include "map_editor/map_document.h"
#include "map_editor/map_edit_command.h"

#include <QDialog>
#include <memory>

class MapCanvas;
class QComboBox;
class QLabel;
class QPushButton;
class QSpinBox;

class MapEditorDialog : public QDialog {
  Q_OBJECT
public:
  explicit MapEditorDialog(const QString &initialYamlPath, QWidget *parent = nullptr);

  bool maybeSave();
  bool loadMap(const QString &yamlPath, QString *error = nullptr);
  bool saveToPath(const QString &targetYamlPath, QString *error = nullptr);
  bool isDirty() const { return history_.currentRevision() != savedRevision_; }
  MapCanvas *mapCanvas() const { return canvas_; }
  MapDocument *document() const { return document_.get(); }
  void reject() override;

protected:
  void closeEvent(QCloseEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;

signals:
  void mapSaved(const QString &yamlPath, const QString &sourceYamlPath);

private slots:
  void chooseOpen();
  bool saveAs();

private:
  void setDocument(std::unique_ptr<MapDocument> document);
  void updateUi();
  QString initialSavePath() const;

  std::unique_ptr<MapDocument> document_;
  MapHistory history_;
  quint64 savedRevision_ = 0;

  MapCanvas *canvas_ = nullptr;
  QComboBox *toolCombo_ = nullptr;
  QSpinBox *brushSpin_ = nullptr;
  QLabel *brushMeters_ = nullptr;
  QLabel *metadataLabel_ = nullptr;
  QLabel *pathLabel_ = nullptr;
  QLabel *statusLabel_ = nullptr;
  QPushButton *openButton_ = nullptr;
  QPushButton *saveAsButton_ = nullptr;
  QPushButton *undoButton_ = nullptr;
  QPushButton *redoButton_ = nullptr;
  QPushButton *fitButton_ = nullptr;
  QPushButton *zoomInButton_ = nullptr;
  QPushButton *zoomOutButton_ = nullptr;
};
