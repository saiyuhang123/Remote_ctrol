#include "vision/vision_panel.h"
#include "vision/vision_client.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDialog>
#include <QGridLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QSplitter>
#include <QElapsedTimer>
#include <QHash>
#include <QScreen>
#include <cmath>

VisionPanel::VisionPanel(const DesktopConfig &config, bool offline, QWidget *parent) : QWidget(parent) {
  setObjectName("visionPanel");
  client_ = new VisionClient(config, offline, this);
  auto *layout = new QVBoxLayout(this); layout->setContentsMargins(0, 0, 0, 0);
  auto *hint = new QLabel("上位机方案：YuNet 人脸、YOLO26n 行人、LPD-YuNet 车牌 + 号码识别。无需导航，未定位也保存图片；相机只提供视频。"); hint->setWordWrap(true); layout->addWidget(hint);
  auto *buttons = new QGridLayout;
  auto button = [&](const QString &text, const char *name, int row, int column) {
    auto *b = new QPushButton(text); b->setObjectName(name); b->setEnabled(!offline && client_->supported()); buttons->addWidget(b, row, column); return b;
  };
  auto *faceStart = button("启动上位机人脸", "visionStartFace", 0, 0);
  auto *faceStop = button("停止人脸检测", "visionStopFace", 0, 1);
  auto *hostStart = button("启动行人 / 车牌", "visionStartDetector", 1, 0);
  auto *hostStop = button("停止上位机检测", "visionStopDetector", 1, 1);
  layout->addLayout(buttons);
  auto *choices = new QHBoxLayout;
  auto *person = new QCheckBox("行人"); person->setObjectName("visionPerson"); person->setChecked(true);
  auto *plate = new QCheckBox("车牌"); plate->setObjectName("visionPlate"); plate->setChecked(true);
  choices->addWidget(person); choices->addWidget(plate); choices->addStretch(); layout->addLayout(choices);
  auto *faceState = new QLabel("上位机人脸检测未启动"); auto *hostState = new QLabel("上位机行人 / 车牌未启动");
  auto *recordState = new QLabel(client_->supported() ? "本地记录未启动" : "缺少 vision_interfaces，请先构建视觉模块");
  for (auto *label : {faceState, hostState, recordState}) { label->setWordWrap(true); label->setTextFormat(Qt::PlainText); layout->addWidget(label); }
  auto *history = new QPushButton("查看视觉记录"); history->setObjectName("visionHistory"); history->setEnabled(client_->supported()); layout->addWidget(history);
  connect(faceStart, &QPushButton::clicked, client_, &VisionClient::startFace);
  connect(faceStop, &QPushButton::clicked, client_, &VisionClient::stopFace);
  connect(hostStart, &QPushButton::clicked, this, [=] { client_->startDetector(person->isChecked(), plate->isChecked()); });
  connect(hostStop, &QPushButton::clicked, client_, &VisionClient::stopDetector);
  connect(client_, &VisionClient::cameraRequested, this, &VisionPanel::cameraRequested);
  connect(client_, &VisionClient::message, this, &VisionPanel::message);
  auto ages = std::make_shared<QHash<QString, QElapsedTimer>>();
  connect(client_, &VisionClient::status, this, [=](const QString &kind, const QJsonObject &value) {
    (*ages)[kind].restart();
    if (kind == "detector") {
      const QString face = value.value("face").toBool() ? "已开启（YuNet）"
        : value.value("face_requested").toBool() ? "准备中 / 模型未就绪" : "已关闭";
      faceState->setText("上位机人脸：" + face + "；仅检测并保存小图，不比对身份");
      hostState->setText(QString("上位机：%1 · 行人 %2 / 车牌 %3\n%4")
      .arg(value.value("input_ready").toBool() ? "视频已接入" : "等待视频")
      .arg(value.value("person").toBool() ? "开" : "关").arg(value.value("plate").toBool() ? "开" : "关").arg(value.value("last_error").toString()));
    }
    else if (kind == "recorder") recordState->setText(QString("本地记录 %1 条 · %2\n%3")
      .arg(value.value("records").toDouble(), 0, 'f', 0)
      .arg(value.value("map_id").toString().isEmpty() ? "地图未关联，图片正常保存" : "地图：" + value.value("map_id").toString())
      .arg(value.value("last_error").toString()));
  });
  auto *freshness = new QTimer(this); freshness->setInterval(1000);
  connect(freshness, &QTimer::timeout, this, [=] {
    for (const auto &entry : {qMakePair(QString("detector"), faceState), qMakePair(QString("detector"), hostState), qMakePair(QString("recorder"), recordState)}) {
      if (ages->contains(entry.first) && ages->value(entry.first).elapsed() > 3500) entry.second->setText("组件状态已过期，请检查节点是否运行");
    }
  }); freshness->start();
  connect(history, &QPushButton::clicked, this, [=] {
    if (history_) { history_->show(); history_->raise(); history_->activateWindow(); return; }
    auto *dialog = new QDialog(this); dialog->setAttribute(Qt::WA_DeleteOnClose);
    history_ = dialog;
    dialog->setWindowTitle("视觉记录 · 本地独立执勤");
    const auto available = screen()->availableGeometry();
    dialog->resize(qMin(1000, available.width()-32), qMin(650, available.height()-48)); dialog->setObjectName("visionHistoryDialog");
    auto *body = new QVBoxLayout(dialog);
    auto *filters = new QHBoxLayout; auto *type = new QComboBox;
    type->addItem("全部", ""); type->addItem("人脸抓拍", "face_capture"); type->addItem("行人", "person"); type->addItem("车牌", "vehicle_plate");
    filters->addWidget(type); auto *refresh = new QPushButton("刷新 / 首页"); auto *next = new QPushButton("下一页");
    filters->addWidget(refresh); filters->addWidget(next); body->addLayout(filters);
    auto *map = new QLineEdit; map->setPlaceholderText("可选：填写当前实际定位地图的标记；不启动导航");
    auto *setMap = new QPushButton("关联地图标记"); setMap->setEnabled(!offline);
    auto *context = new QHBoxLayout; context->addWidget(map); context->addWidget(setMap); body->addLayout(context);
    auto *table = new QTableWidget(0, 5); table->setHorizontalHeaderLabels({"时间", "类型", "内容", "小车坐标(m/°)", "定位状态"});
    table->setSelectionBehavior(QAbstractItemView::SelectRows); table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers); table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    body->addWidget(table, 1);
    auto *image = new QLabel("选择一条记录查看图片"); image->setAlignment(Qt::AlignCenter); image->setMinimumHeight(180); body->addWidget(image, 1);
    auto *detail = new QLabel; detail->setWordWrap(true); detail->setTextFormat(Qt::PlainText);
    detail->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred); detail->setTextInteractionFlags(Qt::TextSelectableByMouse); body->addWidget(detail);
    auto cursor = std::make_shared<QString>();
    connect(refresh, &QPushButton::clicked, dialog, [=] { client_->query(type->currentData().toString()); });
    connect(next, &QPushButton::clicked, dialog, [=] { client_->query(type->currentData().toString(), *cursor); });
    connect(setMap, &QPushButton::clicked, dialog, [=] { client_->setMapLabel(map->text()); });
    connect(client_, &VisionClient::records, dialog, [=](const QJsonArray &items, const QString &nextCursor, quint64 total) {
      *cursor = nextCursor; next->setEnabled(!nextCursor.isEmpty()); table->setRowCount(0);
      for (const auto &item : items) {
        const auto event = item.toObject(); const int row = table->rowCount(); table->insertRow(row);
        const QString position = event.value("pose_valid").toBool()
          ? QString("%1, %2, %3°").arg(event.value("robot_x").toDouble(), 0, 'f', 2).arg(event.value("robot_y").toDouble(), 0, 'f', 2).arg(event.value("robot_yaw").toDouble()*180./3.141592653589793, 0, 'f', 1) : QString("—");
        const QHash<QString, QString> names{{"face_capture", "人脸抓拍"}, {"person", "行人"}, {"vehicle_plate", "车牌"}};
        const QHash<QString, QString> poseNames{{"valid", "有效"}, {"map_unassociated", "地图未关联"}, {"tf_unavailable", "未定位"}, {"tf_stale", "定位过期"}, {"context_changed", "地图会话已变化"}};
        const QStringList values{QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(event.value("event_time_ns").toDouble()/1e6)).toString("MM-dd hh:mm:ss"),
          names.value(event.value("event_type").toString()), event.value("label").toString(), position,
          poseNames.value(event.value("pose_status").toString(), "定位无效")};
        for (int column = 0; column < values.size(); ++column) table->setItem(row, column, new QTableWidgetItem(values[column]));
        table->item(row, 0)->setData(Qt::UserRole, event);
      }
      detail->setText(QString("共 %1 条。上位机按视频帧时间记录；未定位不影响图片保存。旧海康记录保留原时间来源。").arg(total));
    });
    connect(table, &QTableWidget::itemSelectionChanged, dialog, [=] {
      const int row = table->currentRow(); if (row < 0) return;
      const auto event = table->item(row, 0)->data(Qt::UserRole).toJsonObject();
      const QPixmap pixmap(event.value("image_path").toString());
      if (pixmap.isNull()) { image->clear(); image->setText("图片缺失或不可读"); }
      else image->setPixmap(pixmap.scaled(image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
      const QHash<QString, QString> times{{"host_alarm_receipt", "海康报警接收时间（近似）"}, {"ros_image_stamp", "视频帧发布时的主机时间"},
          {"host_video_receipt", "主机收帧时间"}, {"host_image_receipt", "主机收帧时间"}};
      detail->setText(QString("地图：%1\n时间来源：%2\n图片：%3").arg(event.value("map_id").toString(),
        times.value(event.value("time_basis").toString(), "时间来源未知"), event.value("image_path").toString()));
    });
    dialog->show(); client_->query();
  });
}
VisionPanel::~VisionPanel() { delete client_; }
bool VisionPanel::setRemoteFeature(const QString &parameter, bool enabled, QString *detail) {
  return client_->setFeature(parameter, enabled, detail);
}
