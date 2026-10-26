#include "patrol/report_viewer.h"

#include <QtConcurrent/QtConcurrentRun>
#include <QBuffer>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFutureWatcher>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QHeaderView>
#include <QImageReader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPageSize>
#include <QPdfWriter>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QResizeEvent>
#include <QSaveFile>
#include <QScreen>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTabWidget>
#include <QTextBrowser>
#include <QTextDocument>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>
#include <exception>

namespace {
QString escaped(const QString &text) { return text.toHtmlEscaped().replace('\n', "<br>"); }
QString text(const QJsonObject &v, const char *key, const QString &fallback = QString()) {
  const auto value = v.value(QLatin1String(key));
  if (value.isString()) return value.toString();
  if (value.isDouble()) return QString::number(value.toDouble(), 'g', 16);
  if (value.isBool()) return value.toBool() ? QStringLiteral("是") : QStringLiteral("否");
  return fallback;
}
QString date(const QJsonValue &value) {
  const double ns = value.toDouble();
  return ns > 0 ? QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(ns / 1e6)).toString("yyyy-MM-dd HH:mm:ss") : QStringLiteral("—");
}
QString kindName(const QString &kind) {
  if (kind == "face_capture") return QStringLiteral("人脸");
  if (kind == "person") return QStringLiteral("行人");
  if (kind == "vehicle_plate") return QStringLiteral("车牌");
  return kind;
}
QString mediaName(const QJsonObject &item) {
  if (!text(item, "local_image_path").isEmpty()) return QStringLiteral("可查看");
  return text(item, "media_status") == "outside_root"
    ? QStringLiteral("图片目录未配置") : QStringLiteral("图片缺失或已清理");
}
QString position(const QJsonObject &v) {
  if (!v.value("pose_valid").toBool()) return QStringLiteral("无有效定位");
  return QString("%1, %2 m / %3°").arg(v.value("robot_x").toDouble(), 0, 'f', 2)
      .arg(v.value("robot_y").toDouble(), 0, 'f', 2)
      .arg(v.value("robot_yaw").toDouble() * 180. / 3.141592653589793, 0, 'f', 1);
}
QString inspectionName(const QJsonObject &item) {
  auto name = text(item, "point_name", text(item, "point_id"));
  const auto shot = text(item, "shot_name");
  if (!shot.isEmpty()) name += " · " + shot;
  return name;
}
QString shotMetadata(const QJsonObject &item) {
  const auto raw = text(item, "metadata_json");
  const auto metadata = QJsonDocument::fromJson(raw.toUtf8());
  return metadata.isObject() && !metadata.object().isEmpty()
      ? QString::fromUtf8(metadata.toJson(QJsonDocument::Indented)) : QString();
}
QImage readImage(const QString &path, const QSize &maximum) {
  if (path.isEmpty()) return {};
  QImageReader reader(path);
  reader.setAutoTransform(true);
  const auto size = reader.size();
  if (!size.isValid() || qint64(size.width()) * size.height() > 64000000) return {};
  if (maximum.isValid() && (size.width() > maximum.width() || size.height() > maximum.height()))
    reader.setScaledSize(size.scaled(maximum, Qt::KeepAspectRatio));
  return reader.read();
}
QString tableStart(const QStringList &headers) {
  QString result = "<table width='100%' border='1' cellspacing='0' cellpadding='5'><tr bgcolor='#edf3f8'>";
  for (const auto &h : headers) result += "<th>" + escaped(h) + "</th>";
  return result + "</tr>";
}
QString row(const QStringList &cells) {
  QString result = "<tr>";
  for (const auto &cell : cells) result += "<td>" + escaped(cell) + "</td>";
  return result + "</tr>";
}

QString reportHtml(const QJsonObject &report, bool full, QTextDocument *resources = nullptr) {
  const auto run = report.value("run").toObject(), route = report.value("route").toObject();
  QString body = "<h1>巡检报告</h1><h2>" + escaped(text(route, "name", text(run, "route_id"))) + "</h2>";
  body += tableStart({"项目", "结果"});
  const auto counts = report.value("vision_event_counts").toObject();
  const auto originalSummary = report.value("summary").toObject();
  QStringList plates;
  for (const auto &value : originalSummary.value("plates").toArray()) {
    const auto item = value.toObject();
    plates.append(value.isString() ? value.toString()
        : text(item, "plate") + " × " + text(item, "count", "1"));
  }
  const QList<QPair<QString, QString>> summary{
    {"任务编号", text(report, "run_id")}, {"地图", text(report, "map_name")},
    {"任务状态", text(report, "state_name")}, {"开始", date(run.value("started_at"))},
    {"结束", date(run.value("finished_at"))}, {"耗时", text(report, "duration_seconds") + " 秒"},
    {"路线 / 地图版本", text(run, "route_revision") + " / " + text(run, "map_revision")},
    {"完成圈数 / 点数", text(run, "completed_loops") + " / " + text(run, "completed_steps")},
    {"到点识别成功 / 失败", text(report, "inspection_success_count") + " / " + text(report, "inspection_failure_count")},
    {"归集的人脸 / 行人 / 车牌事件", text(counts, "face_capture", "0") + " / " + text(counts, "person", "0") + " / " + text(counts, "vehicle_plate", "0")},
    {"任务原始统计：人脸 / 行人", text(originalSummary, "face_capture_count", "0") + " / " + text(originalSummary, "person_count", "0")},
    {"任务原始车牌统计", plates.isEmpty() ? "无" : plates.join("、")},
    {"图片缺失或目录未配置", text(report, "missing_image_count", "0")},
    {"任务异常", text(run, "error_code", "无")}, {"任务说明", text(run, "message", "—")}
  };
  for (const auto &entry : summary) body += row({entry.first, entry.second});
  body += "</table>";
  for (const auto &warning : report.value("warnings").toArray())
    body += "<p style='color:#a33b16'>" + escaped(warning.toString()) + "</p>";
  body += "<p>" + escaped(text(report, "vision_association")) + "</p>";
  body += "<h2>路线与巡检点快照</h2>" + tableStart({"点位", "位置 / 朝向", "停留", "到点识别"});
  for (const auto &value : route.value("steps").toArray()) {
    const auto step = value.toObject();
    body += row({text(step, "point_name", text(step, "point_id")),
                 text(step, "x") + ", " + text(step, "y") + " / " + text(step, "yaw") + " rad",
                 text(step, "dwell_seconds") + " 秒", step.value("inspection_enabled").toBool() ? "开启" : "关闭"});
  }
  body += "</table>";
  if (full) {
    int imageIndex = 0;
    auto photo = [&](const QJsonObject &item) {
      const auto picture = readImage(text(item, "local_image_path"), QSize(900, 650));
      if (picture.isNull()) return QString("<p>图片不可用：%1</p>").arg(escaped(text(item, "image_path")));
      QString url;
      if (resources) {
        url = QString("patrol-photo:%1").arg(imageIndex++);
        resources->addResource(QTextDocument::ImageResource, QUrl(url), picture);
      } else {
        QByteArray bytes; QBuffer buffer(&bytes); buffer.open(QIODevice::WriteOnly);
        picture.save(&buffer, "JPEG", 80);
        url = "data:image/jpeg;base64," + QString::fromLatin1(bytes.toBase64());
      }
      const auto width = qMin(picture.width(), 620);
      const auto height = qMax(1, picture.height() * width / picture.width());
      return QString("<p><img src='%1' width='%2' height='%3'></p>").arg(url).arg(width).arg(height);
    };
    body += "<h2>到点抓拍与识别结果</h2>";
    for (const auto &value : report.value("inspection_results").toArray()) {
      const auto item = value.toObject();
      body += "<h3>" + escaped(inspectionName(item)) + "</h3>";
      body += tableStart({"项目", "内容"});
      body += row({"时间", date(item.value("started_at"))});
      body += row({"记录编号 / 拍摄视角", text(item, "result_id") + " / " + text(item, "shot_id")});
      body += row({"圈 / 点序", text(item, "loop_index") + " / " + text(item, "step_index")});
      body += row({"结果", item.value("success").toBool() ? "成功" : "失败"});
      body += row({"识别文本", text(item, "text")});
      body += row({"模型 / 预置位", text(item, "model") + " / " + text(item, "ptz_preset")});
      body += row({"识别要求", text(item, "prompt")});
      const auto metadata = shotMetadata(item);
      if (!metadata.isEmpty()) body += row({"拍摄参数与朝向补偿记录", metadata});
      body += row({"异常 / 说明", text(item, "error_code") + " " + text(item, "message")}) + "</table>";
      body += photo(item);
    }
    body += "<h2>人脸、行人和车牌记录</h2>";
    for (const auto &value : report.value("vision_events").toArray()) {
      const auto item = value.toObject();
      body += "<h3>" + escaped(kindName(text(item, "event_type"))) + " · " + date(item.value("event_time_ns")) + "</h3>";
      body += tableStart({"项目", "内容"}) + row({"识别内容", text(item, "label")});
      body += row({"事件编号", text(item, "event_id")}) + row({"小车位置", position(item)});
      body += row({"地图 / 时间来源", text(item, "map_id") + " / " + text(item, "time_basis")});
      body += row({"置信度", text(item, "confidence", "—")}) + "</table>" + photo(item);
    }
    body += "<h2>到点停车记录</h2>" + tableStart({"圈 / 点序", "尝试", "结果", "位置误差", "朝向误差", "异常"});
    for (const auto &value : report.value("stop_attempts").toArray()) {
      const auto item = value.toObject();
      body += row({text(item, "loop_index") + " / " + text(item, "step_index"), text(item, "attempt_index"), text(item, "result"),
                   item.value("actual_valid").toBool() ? text(item, "position_error") + " m" : "无实测",
                   item.value("actual_valid").toBool() ? text(item, "yaw_error") + " rad" : "无实测", text(item, "error_code")});
    }
    body += "</table>";
  }
  return "<!doctype html><html><head><meta charset='utf-8'><title>巡检报告</title>"
         "<style>body{font-family:sans-serif;color:#172638}td,th{word-wrap:break-word}h2{margin-top:24px}"
         "table{border-collapse:collapse}img{max-width:100%}@media print{tr,img{page-break-inside:avoid}}</style></head><body>" + body + "</body></html>";
}

QString writeReport(const QJsonObject &report, const QByteArray &snapshot,
                    const QString &path, const QString &format) {
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) return file.errorString();
  if (format == "pdf") {
    {
      QPdfWriter writer(&file);
      writer.setTitle("巡检报告 " + text(report, "run_id"));
      writer.setCreator("机器人本地控制端");
      writer.setPageSize(QPageSize(QPageSize::A4));
      writer.setResolution(96);
      writer.setPageMargins(QMarginsF(12, 12, 12, 12));
      QTextDocument document;
      const auto html = reportHtml(report, true, &document);
      document.setHtml(html);
      document.print(&writer);
    }
    if (file.size() == 0 || file.error() != QFileDevice::NoError) {
      const auto error = file.errorString(); file.cancelWriting(); return error;
    }
  } else {
    const auto contents = format == "json" ? snapshot : reportHtml(report, true).toUtf8();
    if (file.write(contents) != contents.size()) {
      const auto error = file.errorString(); file.cancelWriting(); return error;
    }
  }
  return file.commit() ? QString() : file.errorString();
}

class ReportGallery : public QWidget {
public:
  ReportGallery(bool inspections, QWidget *parent) : QWidget(parent), inspections_(inspections) {
    auto *body = new QVBoxLayout(this);
    auto *filters = new QHBoxLayout;
    type_ = new QComboBox(this);
    type_->addItem(inspections ? "全部到点结果" : "全部视觉记录", "");
    if (inspections) { type_->addItem("成功", "success"); type_->addItem("失败", "failure"); }
    else { type_->addItem("人脸", "face_capture"); type_->addItem("行人", "person"); type_->addItem("车牌", "vehicle_plate"); }
    search_ = new QLineEdit(this); search_->setPlaceholderText("搜索点位、识别文字或车牌");
    filters->addWidget(type_); filters->addWidget(search_, 1); body->addLayout(filters);
    auto *split = new QSplitter(Qt::Horizontal, this);
    table_ = new QTableWidget(0, 5, split);
    table_->setHorizontalHeaderLabels(inspections ? QStringList{"时间", "点位", "圈 / 点序", "识别摘要", "状态"}
                                               : QStringList{"时间", "类型", "识别内容", "小车位置", "图片"});
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setSectionResizeMode(inspections ? 3 : 2, QHeaderView::Stretch);
    auto *detail = new QWidget(split); auto *detailLayout = new QVBoxLayout(detail);
    photo_ = new InspectionPhotoView(detail); detailLayout->addWidget(photo_, 3);
    details_ = new QPlainTextEdit(detail); details_->setReadOnly(true); detailLayout->addWidget(details_, 2);
    split->addWidget(table_); split->addWidget(detail); split->setSizes({650, 430}); body->addWidget(split, 1);
    auto *pages = new QHBoxLayout; previous_ = new QPushButton("上一页", this); next_ = new QPushButton("下一页", this);
    count_ = new QLabel(this); pages->addWidget(count_, 1); pages->addWidget(previous_); pages->addWidget(next_); body->addLayout(pages);
    connect(type_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this] { filter(); });
    connect(search_, &QLineEdit::textChanged, this, [this] { filter(); });
    connect(previous_, &QPushButton::clicked, this, [this] { --page_; populate(); });
    connect(next_, &QPushButton::clicked, this, [this] { ++page_; populate(); });
    connect(table_, &QTableWidget::itemSelectionChanged, this, [this] { select(); });
    connect(table_, &QTableWidget::cellDoubleClicked, this, [this](int, int) {
      const int index = page_ * 50 + table_->currentRow();
      if (table_->currentRow() >= 0 && index < filtered_.size()) {
        const auto item = filtered_.at(index).toObject();
        InspectionPhotoView::showPhoto(text(item, "local_image_path"), "巡检照片", this);
      }
    });
    filter();
  }
  void setRecords(const QJsonArray &records) { records_ = records; filter(); }
private:
  void filter() {
    filtered_ = {}; page_ = 0;
    const auto wanted = type_->currentData().toString(), needle = search_->text().trimmed();
    for (const auto &value : records_) {
      const auto item = value.toObject();
      if (inspections_ && !wanted.isEmpty() && item.value("success").toBool() != (wanted == "success")) continue;
      if (!inspections_ && !wanted.isEmpty() && text(item, "event_type") != wanted) continue;
      const auto haystack = inspectionName(item) + " " + text(item, "point_id") + " " + text(item, "text") + " " + text(item, "label") + " " + text(item, "message");
      if (!needle.isEmpty() && !haystack.contains(needle, Qt::CaseInsensitive)) continue;
      filtered_.append(value);
    }
    populate();
  }
  void populate() {
    const QSignalBlocker block(table_);
    table_->setRowCount(0); photo_->setPhoto({}); details_->clear();
    const int begin = page_ * 50, end = qMin(begin + 50, filtered_.size());
    for (int i = begin; i < end; ++i) {
      const auto item = filtered_.at(i).toObject();
      const QStringList values = inspections_
        ? QStringList{date(item.value("started_at")), inspectionName(item),
                      text(item, "loop_index") + " / " + text(item, "step_index"),
                      text(item, "text").isEmpty() ? text(item, "message") : text(item, "text"),
                      item.value("success").toBool() ? "成功" : "失败"}
        : QStringList{date(item.value("event_time_ns")), kindName(text(item, "event_type")),
                      text(item, "label"), position(item), mediaName(item)};
      const int rowIndex = table_->rowCount(); table_->insertRow(rowIndex);
      for (int column = 0; column < values.size(); ++column) {
        auto *cell = new QTableWidgetItem(values[column].simplified().left(180));
        cell->setToolTip(values[column]); table_->setItem(rowIndex, column, cell);
      }
    }
    previous_->setEnabled(page_ > 0); next_->setEnabled(end < filtered_.size());
    count_->setText(QString("共 %1 条 · 第 %2 / %3 页 · 导出包含任务全部记录和图片")
                    .arg(filtered_.size()).arg(page_ + 1).arg(qMax(1, (filtered_.size() + 49) / 50)));
    if (table_->rowCount()) { table_->selectRow(0); select(); }
  }
  void select() {
    const int rowIndex = table_->currentRow(), index = page_ * 50 + rowIndex;
    if (rowIndex < 0 || index >= filtered_.size()) return;
    const auto item = filtered_.at(index).toObject();
    photo_->setPhoto(text(item, "local_image_path"), mediaName(item));
    QString content;
    if (inspections_) {
      content = QString("点位：%1\n时间：%2\n圈 / 点序：%3 / %4\n识别模型：%5\n预置位：%6\n\n识别内容：\n%7\n\n识别要求：\n%8\n\n异常 / 说明：\n%9 %10")
          .arg(inspectionName(item), date(item.value("started_at")),
               text(item, "loop_index"), text(item, "step_index"), text(item, "model"), text(item, "ptz_preset"),
               text(item, "text"), text(item, "prompt"), text(item, "error_code")).arg(text(item, "message"));
      content += "\n\n记录编号：" + text(item, "result_id") + "\n拍摄视角编号：" + text(item, "shot_id");
      const auto metadata = shotMetadata(item);
      if (!metadata.isEmpty()) content += "\n拍摄参数与朝向补偿记录：\n" + metadata;
    } else {
      content = QString("类型：%1\n时间：%2\n内容：%3\n置信度：%4\n小车位置：%5\n地图：%6\n时间来源：%7\n事件编号：%8")
          .arg(kindName(text(item, "event_type")), date(item.value("event_time_ns")), text(item, "label"),
               text(item, "confidence", "—"), position(item), text(item, "map_id"), text(item, "time_basis"), text(item, "event_id"));
    }
    details_->setPlainText(content + "\n\n图片：" + text(item, "image_path"));
  }
  bool inspections_;
  QJsonArray records_, filtered_;
  int page_ = 0;
  QComboBox *type_;
  QLineEdit *search_;
  QTableWidget *table_;
  InspectionPhotoView *photo_;
  QPlainTextEdit *details_;
  QPushButton *previous_, *next_;
  QLabel *count_;
};
}  // namespace

InspectionPhotoView::InspectionPhotoView(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this); layout->setContentsMargins(0, 0, 0, 0);
  image_ = new QLabel("选择记录查看照片", this); image_->setAlignment(Qt::AlignCenter);
  image_->setMinimumSize(180, 140); image_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  image_->setStyleSheet("background:#edf2f7;border:1px solid #d6e0e9;border-radius:6px;color:#64748b");
  caption_ = new QLabel(this); caption_->setWordWrap(true); caption_->setTextFormat(Qt::PlainText);
  enlarge_ = new QPushButton("查看大图 / 缩放", this); enlarge_->setEnabled(false);
  layout->addWidget(image_, 1); layout->addWidget(caption_); layout->addWidget(enlarge_);
  connect(enlarge_, &QPushButton::clicked, this, [this] { showPhoto(path_, caption_->text(), this); });
}
void InspectionPhotoView::setPhoto(const QString &path, const QString &caption) {
  path_ = path; preview_ = QPixmap::fromImage(readImage(path, QSize(1600, 1200)));
  image_->clear(); caption_->setText(caption); enlarge_->setEnabled(!preview_.isNull());
  if (preview_.isNull()) image_->setText(path.isEmpty() ? "没有可预览的图片" : "图片不存在或无法读取");
  else updatePreview();
}
void InspectionPhotoView::updatePreview() {
  if (!preview_.isNull()) image_->setPixmap(preview_.scaled(image_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
void InspectionPhotoView::resizeEvent(QResizeEvent *event) { QWidget::resizeEvent(event); updatePreview(); }
void InspectionPhotoView::showPhoto(const QString &path, const QString &title, QWidget *parent) {
  const auto image = readImage(path, QSize());
  if (image.isNull()) { QMessageBox::information(parent, "巡检照片", "图片不存在、无法读取或尺寸过大。"); return; }
  auto *dialog = new QDialog(parent); dialog->setAttribute(Qt::WA_DeleteOnClose);
  dialog->setWindowTitle(title.isEmpty() ? "巡检照片" : title);
  dialog->setObjectName("patrolPhotoDialog"); dialog->resize(1000, 740);
  auto *layout = new QVBoxLayout(dialog); auto *tools = new QHBoxLayout;
  auto *fit = new QPushButton("适应窗口", dialog), *actual = new QPushButton("100%", dialog);
  auto *zoomIn = new QPushButton("放大", dialog), *zoomOut = new QPushButton("缩小", dialog);
  tools->addWidget(fit); tools->addWidget(actual); tools->addWidget(zoomIn); tools->addWidget(zoomOut); tools->addStretch();
  layout->addLayout(tools); auto *view = new QGraphicsView(dialog); auto *scene = new QGraphicsScene(view);
  auto *item = scene->addPixmap(QPixmap::fromImage(image)); view->setScene(scene);
  view->setDragMode(QGraphicsView::ScrollHandDrag); view->setRenderHint(QPainter::SmoothPixmapTransform);
  layout->addWidget(view, 1);
  auto *caption = new QLabel(path, dialog); caption->setTextFormat(Qt::PlainText); caption->setWordWrap(true);
  caption->setTextInteractionFlags(Qt::TextSelectableByMouse); layout->addWidget(caption);
  connect(fit, &QPushButton::clicked, dialog, [view, item] { view->fitInView(item, Qt::KeepAspectRatio); });
  connect(actual, &QPushButton::clicked, dialog, [view] { view->resetTransform(); });
  connect(zoomIn, &QPushButton::clicked, dialog, [view] { if (view->transform().m11() < 12) view->scale(1.3, 1.3); });
  connect(zoomOut, &QPushButton::clicked, dialog, [view] { if (view->transform().m11() > .02) view->scale(1 / 1.3, 1 / 1.3); });
  dialog->show(); QTimer::singleShot(0, dialog, [view, item] { view->fitInView(item, Qt::KeepAspectRatio); });
}

PatrolReportDialog::PatrolReportDialog(const QString &runId, const QString &projectRoot, QWidget *parent)
    : QDialog(parent), runId_(runId), projectRoot_(projectRoot) {
  setAttribute(Qt::WA_DeleteOnClose); setObjectName("patrolReportDialog");
  setWindowTitle("任务图文报告 · " + runId_);
  const auto available = screen()->availableGeometry();
  resize(qMin(1220, available.width() - 40), qMin(850, available.height() - 60));
  auto *layout = new QVBoxLayout(this); auto *toolbar = new QHBoxLayout;
  auto *title = new QLabel("任务：" + runId_, this); title->setTextFormat(Qt::PlainText); title->setTextInteractionFlags(Qt::TextSelectableByMouse);
  title->setWordWrap(true); title->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  toolbar->addWidget(title, 1);
  refresh_ = new QPushButton("刷新报告", this); pdf_ = new QPushButton("导出 PDF", this);
  html_ = new QPushButton("导出 HTML", this); json_ = new QPushButton("导出 JSON", this);
  toolbar->addWidget(refresh_); toolbar->addWidget(pdf_); toolbar->addWidget(html_); toolbar->addWidget(json_); layout->addLayout(toolbar);
  status_ = new QLabel(this); status_->setWordWrap(true); status_->setTextFormat(Qt::PlainText);
  status_->setTextInteractionFlags(Qt::TextSelectableByMouse); layout->addWidget(status_);
  auto *tabs = new QTabWidget(this); overview_ = new QTextBrowser(tabs); overview_->setOpenExternalLinks(false);
  overview_->setOpenLinks(false); tabs->addTab(overview_, "报告概览与路线");
  inspectionGallery_ = new ReportGallery(true, tabs); tabs->addTab(inspectionGallery_, "到点抓拍与识别");
  visionGallery_ = new ReportGallery(false, tabs); tabs->addTab(visionGallery_, "人脸 / 行人 / 车牌");
  stops_ = new QTableWidget(0, 6, tabs); stops_->setHorizontalHeaderLabels({"圈 / 点序", "尝试", "结果", "位置误差(m)", "朝向误差(rad)", "异常"});
  stops_->setEditTriggers(QAbstractItemView::NoEditTriggers); stops_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  tabs->addTab(stops_, "到点停车记录"); layout->addWidget(tabs, 1);
  process_ = new QProcess(this); timeout_ = new QTimer(this); timeout_->setSingleShot(true); timeout_->setInterval(45000);
  connect(process_, &QProcess::readyReadStandardOutput, this, [this] {
    output_ += process_->readAllStandardOutput();
    if (output_.size() > 128 * 1024 * 1024) { failure_ = "任务记录过大，报告未加载；没有截断或导出不完整数据。"; process_->kill(); }
  });
  connect(process_, &QProcess::readyReadStandardError, this, [this] { errors_ = (errors_ + process_->readAllStandardError()).right(16000); });
  connect(process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
    if (error == QProcess::FailedToStart) {
      timeout_->stop(); loading_ = false; status_->setText("无法启动本地报告读取器：" + process_->errorString()); updateButtons();
    }
  });
  connect(process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this, [this](int code, QProcess::ExitStatus) { loaded(code); });
  connect(timeout_, &QTimer::timeout, this, [this] { failure_ = "读取报告超时，请检查本地数据库是否繁忙。"; process_->kill(); });
  connect(refresh_, &QPushButton::clicked, this, [this] { load(); });
  connect(pdf_, &QPushButton::clicked, this, [this] { exportReport("pdf"); });
  connect(html_, &QPushButton::clicked, this, [this] { exportReport("html"); });
  connect(json_, &QPushButton::clicked, this, [this] { exportReport("json"); });
  updateButtons(); QTimer::singleShot(0, this, [this] { load(); });
}
PatrolReportDialog::~PatrolReportDialog() {
  if (process_->state() != QProcess::NotRunning) { process_->kill(); process_->waitForFinished(1000); }
}
void PatrolReportDialog::updateButtons() {
  const bool ready = !loading_ && !exporting_ && !report_.isEmpty();
  refresh_->setEnabled(!loading_ && !exporting_); pdf_->setEnabled(ready); html_->setEnabled(ready); json_->setEnabled(ready);
}
void PatrolReportDialog::load() {
  if (loading_ || exporting_) return;
  loading_ = true; output_.clear(); errors_.clear(); failure_.clear(); report_ = {}; snapshot_.clear(); updateButtons();
  overview_->setPlainText("正在读取本地任务报告…");
  static_cast<ReportGallery *>(inspectionGallery_)->setRecords({}); static_cast<ReportGallery *>(visionGallery_)->setRecords({}); stops_->setRowCount(0);
  status_->setText("正在读取任务及视觉记录；无需启动网页服务。");
  auto environment = QProcessEnvironment::systemEnvironment(); environment.insert("ROBOT_PROJECT_ROOT", projectRoot_);
  environment.insert("PYTHONUNBUFFERED", "1"); process_->setProcessEnvironment(environment);
  process_->start("python3", {"-m", "robot_patrol.desktop_report", "--run-id", runId_, "--project-root", projectRoot_});
  timeout_->start();
}
void PatrolReportDialog::loaded(int code) {
  timeout_->stop(); loading_ = false; output_ += process_->readAllStandardOutput(); errors_ += process_->readAllStandardError();
  QJsonParseError parseError; const auto document = QJsonDocument::fromJson(output_, &parseError);
  if (code != 0 || parseError.error != QJsonParseError::NoError || !document.isObject() || !failure_.isEmpty()) {
    auto error = failure_.isEmpty() ? QString::fromUtf8(errors_).trimmed() : failure_;
    if (error.contains("No module named")) error = "本地报告模块未安装，请重新构建 robot_patrol，并从加载 ROS 工作空间的入口启动客户端。";
    if (error.isEmpty()) error = "报告数据无效：" + parseError.errorString();
    status_->setText(error); overview_->setPlainText(error); updateButtons(); return;
  }
  report_ = document.object();
  if (text(report_, "run_id") != runId_) { report_ = {}; status_->setText("报告任务编号不匹配，已拒绝显示。"); updateButtons(); return; }
  snapshot_ = output_; output_.clear(); overview_->setHtml(reportHtml(report_, false));
  const auto inspections = report_.value("inspection_results").toArray(), events = report_.value("vision_events").toArray();
  static_cast<ReportGallery *>(inspectionGallery_)->setRecords(inspections); static_cast<ReportGallery *>(visionGallery_)->setRecords(events);
  for (const auto &value : report_.value("stop_attempts").toArray()) {
    const auto item = value.toObject(); const int r = stops_->rowCount(); stops_->insertRow(r);
    const QStringList cells{text(item, "loop_index") + " / " + text(item, "step_index"), text(item, "attempt_index"), text(item, "result"),
      item.value("actual_valid").toBool() ? text(item, "position_error") : "无实测", item.value("actual_valid").toBool() ? text(item, "yaw_error") : "无实测", text(item, "error_code")};
    for (int c = 0; c < cells.size(); ++c) stops_->setItem(r, c, new QTableWidgetItem(cells[c]));
  }
  QString status = QString("已加载 %1 条到点结果、%2 条视觉记录。图片缺失或目录未配置 %3 张。")
      .arg(inspections.size()).arg(events.size()).arg(report_.value("missing_image_count").toInt());
  for (const auto &warning : report_.value("warnings").toArray()) status += "\n" + warning.toString();
  status_->setText(status); updateButtons();
}
void PatrolReportDialog::exportReport(const QString &format) {
  if (loading_ || exporting_ || report_.isEmpty()) return;
  const auto extension = format.toLower();
  const auto directory = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
  const auto suggested = QDir(directory).filePath("inspection-" + runId_ + "." + extension);
  QFileDialog chooser(this, "导出完整巡检报告", suggested, format.toUpper() + " (*." + extension + ")");
  chooser.setAcceptMode(QFileDialog::AcceptSave); chooser.setDefaultSuffix(extension);
  if (chooser.exec() != QDialog::Accepted || chooser.selectedFiles().isEmpty()) return;
  const QString path = chooser.selectedFiles().first();
  exporting_ = true; updateButtons(); status_->setText("正在后台导出全部记录和图片…");
  auto *watcher = new QFutureWatcher<QString>(this);
  connect(watcher, &QFutureWatcher<QString>::finished, this, [this, watcher, path] {
    const auto error = watcher->result(); watcher->deleteLater(); exporting_ = false; updateButtons();
    status_->setText(error.isEmpty() ? "报告已保存：" + path : "报告导出失败：" + error);
  });
  const auto report = report_; const auto snapshot = snapshot_;
  watcher->setFuture(QtConcurrent::run([report, snapshot, path, format] {
    try { return writeReport(report, snapshot, path, format); }
    catch (const std::exception &error) { return QString::fromUtf8(error.what()); }
    catch (...) { return QStringLiteral("导出发生异常，目标文件未完整保存"); }
  }));
}
