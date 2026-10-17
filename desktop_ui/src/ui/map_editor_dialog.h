#pragma once

#include "map_editor/map_document.h"
#include "map_editor/map_edit_command.h"

#include <QDialog>
#include <memory>

class MapCanvas;
class QLabel;
class QPushButton;
class QSpinBox;
class QFrame;
class QResizeEvent;
class QShortcut;

class MapEditorDialog : public QDialog {
  Q_OBJECT
public:
  explicit MapEditorDialog(const QString &initialYamlPath, QWidget *parent = nullptr);

  bool maybeSave();
  bool loadMap(const QString &yamlPath, QString *error = nullptr);
  bool saveCurrent(QString *error = nullptr);
  bool saveToPath(const QString &targetYamlPath, QString *error = nullptr);
  bool isDirty() const { return history_.currentRevision() != savedRevision_; }
  MapCanvas *mapCanvas() const { return canvas_; }
  MapDocument *document() const { return document_.get(); }

  // MainWindow calls this after it validates whether the saved YAML can be
  // selected for the next navigation launch. The editor only displays the
  // result; it never decides source/backend compatibility itself.
  void updateNavigationSelection(const QString &selectedYamlPath, bool sourceMatches = true);

  void reject() override;

protected:
  void closeEvent(QCloseEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;

signals:
  void mapSaved(const QString &yamlPath, const QString &sourceYamlPath);

private slots:
  void chooseOpen();
  void saveDirect();
  bool saveAs();

private:
  void setDocument(std::unique_ptr<MapDocument> document);
  void updateUi();
  void updateToolFromButtons();
  void updateBrushSummary();
  void updateFileState();
  void updateNavigationFeedback();
  void updateInfoPanel();
  void updateToolSummary();
  void refreshStatusElision();
  void setOperationMessage(const QString &message, bool error);
  void showHint(const QString &message);
  void repolish(QWidget *widget);
  QString initialSavePath() const;
  QString elidedPath(const QString &path) const;

  std::unique_ptr<MapDocument> document_;
  MapHistory history_;
  quint64 savedRevision_ = 0;

  MapCanvas *canvas_ = nullptr;
  QSpinBox *brushSpin_ = nullptr;
  QPushButton *openButton_ = nullptr;
  QPushButton *saveButton_ = nullptr;
  QPushButton *saveAsButton_ = nullptr;
  QPushButton *undoButton_ = nullptr;
  QPushButton *redoButton_ = nullptr;
  QPushButton *fitButton_ = nullptr;
  QPushButton *zoomInButton_ = nullptr;
  QPushButton *zoomOutButton_ = nullptr;
  QPushButton *infoToggleButton_ = nullptr;
  QPushButton *shapeBrushButton_ = nullptr;
  QPushButton *shapeLineButton_ = nullptr;
  QPushButton *cellObstacleButton_ = nullptr;
  QPushButton *cellFreeButton_ = nullptr;
  QPushButton *cellUnknownButton_ = nullptr;
  QPushButton *technicalToggle_ = nullptr;

  QFrame *infoPanel_ = nullptr;
  QLabel *currentFileLabel_ = nullptr;
  QLabel *brushSummaryLabel_ = nullptr;
  QLabel *dimensionsLabel_ = nullptr;
  QLabel *resolutionLabel_ = nullptr;
  QLabel *sourceYamlLabel_ = nullptr;
  QLabel *sourceImageLabel_ = nullptr;
  QLabel *exportYamlLabel_ = nullptr;
  QLabel *technicalLabel_ = nullptr;
  QLabel *navigationSelectionLabel_ = nullptr;
  QLabel *zoomValueLabel_ = nullptr;
  QLabel *toolSummaryLabel_ = nullptr;
  QLabel *cursorValueLabel_ = nullptr;
  QLabel *fileStateLabel_ = nullptr;
  QLabel *operationMessageLabel_ = nullptr;
  QShortcut *saveAsShortcut_ = nullptr;
  QShortcut *saveShortcut_ = nullptr;

  QString navigationSelectedYaml_;
  QString toolSummaryFullText_;
  QString cursorFullText_;
  bool navigationSourceMatches_ = true;
  bool operationIsError_ = false;
};
