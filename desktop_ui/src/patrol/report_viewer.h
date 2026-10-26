#pragma once

#include <QDialog>
#include <QJsonObject>
#include <QPixmap>
#include <QWidget>

class QLabel;
class QPushButton;
class QProcess;
class QTimer;
class QTextBrowser;
class QTableWidget;

class InspectionPhotoView : public QWidget {
public:
  explicit InspectionPhotoView(QWidget *parent = nullptr);
  void setPhoto(const QString &path, const QString &caption = QString());
  static void showPhoto(const QString &path, const QString &title, QWidget *parent);
protected:
  void resizeEvent(QResizeEvent *event) override;
private:
  void updatePreview();
  QLabel *image_, *caption_;
  QPushButton *enlarge_;
  QPixmap preview_;
  QString path_;
};

class PatrolReportDialog : public QDialog {
public:
  PatrolReportDialog(const QString &runId, const QString &projectRoot, QWidget *parent = nullptr);
  ~PatrolReportDialog() override;
private:
  void load();
  void loaded(int code);
  void updateButtons();
  void exportReport(const QString &format);
  QString runId_, projectRoot_, failure_;
  QJsonObject report_;
  QByteArray output_, snapshot_, errors_;
  QProcess *process_;
  QTimer *timeout_;
  QLabel *status_;
  QTextBrowser *overview_;
  QWidget *inspectionGallery_, *visionGallery_;
  QTableWidget *stops_;
  QPushButton *refresh_, *pdf_, *html_, *json_;
  bool loading_ = false, exporting_ = false;
};
