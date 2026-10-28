#include "patrol/inspection_target_page.h"
#include "patrol/patrol_ui_style.h"
#include "map_editor/map_io.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGraphicsPathItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMouseEvent>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace { constexpr double kPi = 3.14159265358979323846; }

// This map only selects a target. Only the explicit Start button sends it
// through InspectPoint, retaining the patrol engine's ownership and guard.
class InspectionTargetMap : public QGraphicsView {
public:
  explicit InspectionTargetMap(QWidget *parent) : QGraphicsView(parent) {
    setScene(new QGraphicsScene(this));
    setObjectName("inspectionTargetMap");
    setMinimumSize(240, 180);
    setDragMode(ScrollHandDrag);
    setTransformationAnchor(AnchorUnderMouse);
    setRenderHint(QPainter::Antialiasing);
  }
  std::function<void(double, double, double)> picked;
  bool load(const PatrolMapInfo &map, QString *error) {
    scene()->clear(); target_ = nullptr; markers_.clear(); valid_ = false; picking_ = false;
    MapDocument document;
    if (!MapIo::load(map.navigationYaml, &document, error)) return false;
    const auto &meta = document.metadata();
    if (meta.width != map.width || meta.height != map.height ||
        std::abs(meta.resolution - map.resolution) > 1e-9 ||
        std::abs(meta.originX - map.originX) > 1e-6 ||
        std::abs(meta.originY - map.originY) > 1e-6 ||
        std::abs(std::remainder(meta.originYaw - map.originYaw, 2*kPi)) > 1e-6) {
      if (error) *error = QStringLiteral("地图文件与活跃地图版本不一致，请重新登记并确认地图");
      return false;
    }
    meta_ = meta;
    QImage image(meta.width, meta.height, QImage::Format_Grayscale8);
    for (int row = 0; row < meta.height; ++row)
      std::copy_n(document.pixels().data() + row*meta.width, meta.width, image.scanLine(row));
    scene()->addPixmap(QPixmap::fromImage(image));
    scene()->setSceneRect(0, 0, meta.width, meta.height);
    valid_ = true; fit();
    return true;
  }
  void clearMap() {
    scene()->clear(); markers_.clear(); target_ = nullptr; valid_ = false; picking_ = false;
  }
  void fit() { autoFit_ = true; if (valid_) fitInView(sceneRect(), Qt::KeepAspectRatio); }
  bool valid() const { return valid_; }
  QPointF worldToPixel(double x, double y) const {
    const double dx=x-meta_.originX, dy=y-meta_.originY, c=std::cos(meta_.originYaw), s=std::sin(meta_.originYaw);
    return {(c*dx+s*dy)/meta_.resolution, meta_.height-(-s*dx+c*dy)/meta_.resolution};
  }
  QPointF pixelToWorld(const QPointF &p) const {
    const double x=p.x()*meta_.resolution, y=(meta_.height-p.y())*meta_.resolution;
    const double c=std::cos(meta_.originYaw), s=std::sin(meta_.originYaw);
    return {meta_.originX+c*x-s*y, meta_.originY+s*x+c*y};
  }
  void target(double x, double y, double yaw, bool visible) {
    yaw_ = yaw;
    if (!valid_) return;
    if (!target_) {
      target_ = scene()->addPath(QPainterPath());
      target_->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      target_->setZValue(3);
      target_->setPen(QPen(QColor("#d97706"), 3));
    }
    QPainterPath path; path.addEllipse(QPointF(), 7, 7);
    path.moveTo(0,0); path.lineTo(28,0); path.lineTo(20,-6);
    path.moveTo(28,0); path.lineTo(20,6);
    QTransform rotation; rotation.rotate(-(yaw-meta_.originYaw)*180/kPi);
    target_->setPath(rotation.map(path));
    target_->setPos(worldToPixel(x,y)); target_->setVisible(visible);
  }
  void points(const QVector<PatrolPointInfo> &points, const QString &mapId) {
    for (auto *item : markers_) delete item;
    markers_.clear();
    if (!valid_) return;
    for (const auto &point : points) {
      if (point.mapId != mapId) continue;
      auto *marker = scene()->addEllipse(-4,-4,8,8,QPen(QColor("#0d9488"),2),QBrush(Qt::white));
      marker->setFlag(QGraphicsItem::ItemIgnoresTransformations);
      marker->setPos(worldToPixel(point.x,point.y)); marker->setZValue(2);
      marker->setToolTip(point.name); markers_.append(marker);
    }
  }
protected:
  void resizeEvent(QResizeEvent *event) override {
    QGraphicsView::resizeEvent(event); if (autoFit_) fit();
  }
  void wheelEvent(QWheelEvent *event) override {
    const double factor = event->angleDelta().y() > 0 ? 1.2 : 1/1.2;
    const double next = transform().m11()*factor;
    if (next > .005 && next < 100) { autoFit_ = false; scale(factor,factor); }
    event->accept();
  }
  void mousePressEvent(QMouseEvent *event) override {
    if (!valid_ || event->button() != Qt::LeftButton || (event->modifiers() & Qt::ShiftModifier)) {
      QGraphicsView::mousePressEvent(event); return;
    }
    const QPointF p=mapToScene(event->pos());
    if (!sceneRect().contains(p)) return;
    origin_=pixelToWorld(p); picking_=true;
    if (picked) picked(origin_.x(),origin_.y(),yaw_);
    event->accept();
  }
  void mouseMoveEvent(QMouseEvent *event) override {
    if (!picking_) { QGraphicsView::mouseMoveEvent(event); return; }
    const auto p=pixelToWorld(mapToScene(event->pos()));
    const auto delta=p-origin_;
    if (std::hypot(delta.x(),delta.y()) > .03 && picked)
      picked(origin_.x(),origin_.y(),std::atan2(delta.y(),delta.x()));
    event->accept();
  }
  void mouseReleaseEvent(QMouseEvent *event) override {
    if (picking_ && event->button()==Qt::LeftButton) {
      const auto delta=pixelToWorld(mapToScene(event->pos()))-origin_;
      if (std::hypot(delta.x(),delta.y()) > .03 && picked)
        picked(origin_.x(),origin_.y(),std::atan2(delta.y(),delta.x()));
      picking_=false; event->accept();
    }
    else QGraphicsView::mouseReleaseEvent(event);
  }
private:
  MapMetadata meta_;
  bool valid_=false, autoFit_=true, picking_=false;
  double yaw_=0.;
  QPointF origin_;
  QGraphicsPathItem *target_=nullptr;
  QVector<QGraphicsItem *> markers_;
};

InspectionTargetPage::InspectionTargetPage(PatrolClient *client, bool offline,
    std::function<bool(QString *)> permission, QWidget *parent)
    : QWidget(parent), client_(client), offline_(offline), permission_(std::move(permission)) {
  setObjectName("inspectionTargetPage");
  auto *root=new QVBoxLayout(this); root->setContentsMargins(0,8,0,0);
  auto *body=new QHBoxLayout; root->addLayout(body,1);
  auto *scroll=new QScrollArea(this); scroll->setWidgetResizable(true); scroll->setMinimumWidth(270); scroll->setMaximumWidth(350);
  auto *form=new QWidget(scroll); auto *left=new QVBoxLayout(form); left->setContentsMargins(10,10,10,10);
  left->addWidget(patrolui::heading("一键到达与巡检",form));
  left->addWidget(patrolui::hint("选择已有巡检点，或在地图按住拖动设置目标与车头朝向。临时目标无需保存路线。",form));
  points_=new QComboBox(form); points_->setObjectName("inspectionTargetPoint");
  points_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  points_->setMinimumContentsLength(12); left->addWidget(points_);
  auto *fields=new QFormLayout;
  auto number=[form](const char *name, double low, double high, int decimals) {
    auto *spin=new QDoubleSpinBox(form); spin->setObjectName(name); spin->setRange(low,high); spin->setDecimals(decimals); return spin;
  };
  x_=number("inspectionTargetX",-100000,100000,3); y_=number("inspectionTargetY",-100000,100000,3);
  yaw_=number("inspectionTargetYaw",-180,180,1);
  fields->addRow("X (m)",x_); fields->addRow("Y (m)",y_); fields->addRow("朝向 (°)",yaw_); left->addLayout(fields);
  inspect_=new QCheckBox("到达后抓拍识别",form); inspect_->setObjectName("inspectionTargetEnabled"); inspect_->setChecked(true); left->addWidget(inspect_);
  auto *settings=new QFormLayout;
  preset_=new QSpinBox(form); preset_->setObjectName("inspectionTargetPreset"); preset_->setRange(1,255);
  settings->addRow("云台预置位",preset_); left->addLayout(settings);
  prompt_=new QPlainTextEdit(form); prompt_->setObjectName("inspectionTargetPrompt");
  prompt_->setPlaceholderText("识别要求（选填，最多 2000 字）"); prompt_->setMaximumHeight(78); left->addWidget(prompt_);
  left->addWidget(patrolui::hint("已有点位使用其保存的巡检设置及全部拍摄视角；需要临时修改时请选择地图点。",form));
  auto button=[form](const QString &text,const char *name) {auto *b=new QPushButton(text,form); b->setObjectName(name); b->setAutoDefault(false); return b;};
  start_=button("到达并巡检","inspectionTargetStart"); start_->setProperty("primary",true); left->addWidget(start_);
  auto *actions=new QHBoxLayout;
  cancel_=button("取消当前任务","inspectionTargetCancel"); home_=button("一键返回","inspectionTargetHome");
  actions->addWidget(cancel_); actions->addWidget(home_); left->addLayout(actions);
  readiness_=patrolui::hint("等待活跃地图",form); readiness_->setObjectName("inspectionTargetReadiness"); left->addWidget(readiness_); left->addStretch();
  scroll->setWidget(form); body->addWidget(scroll);
  auto *right=new QVBoxLayout; body->addLayout(right,1);
  auto *tools=new QHBoxLayout;
  tools->addWidget(patrolui::hint("地图选点 · 滚轮缩放 · Shift + 拖动平移",this),1);
  auto *fit=new QPushButton("适应窗口",this); fit->setObjectName("inspectionTargetFit"); tools->addWidget(fit); right->addLayout(tools);
  view_=new InspectionTargetMap(this); right->addWidget(view_,1);
  status_=patrolui::hint("尚无活动任务",this); status_->setObjectName("inspectionTargetStatus"); root->addWidget(status_);
  message_=patrolui::hint("请先选择目标。",this); message_->setObjectName("inspectionTargetMessage");
  message_->setTextFormat(Qt::PlainText); root->addWidget(message_);
  connect(fit,&QPushButton::clicked,this,[this]{view_->fit();});
  connect(points_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{chooseSavedPoint();});
  for (auto *spin : {x_,y_,yaw_}) {
    connect(spin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this]{useCoordinates();});
    connect(spin,&QDoubleSpinBox::editingFinished,this,[this]{
      if (selectedPointId_.isEmpty()) useCoordinates();
    });
  }
  connect(inspect_,&QCheckBox::toggled,this,[this]{updateControls();});
  connect(prompt_,&QPlainTextEdit::textChanged,this,[this]{updateControls();});
  connect(start_,&QPushButton::clicked,this,[this]{start();});
  connect(cancel_,&QPushButton::clicked,client_,&PatrolClient::cancelActiveTask);
  connect(home_,&QPushButton::clicked,this,[this]{
    updateControls(); if (!home_->isEnabled()) return;
    client_->returnHome(map_.mapId,map_.revision); message("正在结束当前任务，随后返回待命点。");
  });
  view_->picked=[this](double x,double y,double yaw) {
    updating_=true; x_->setValue(x); y_->setValue(y); yaw_->setValue(yaw*180/kPi); updating_=false;
    useCoordinates();
  };
  connect(client_,&PatrolClient::mapsChanged,this,[this]{refreshMap();});
  connect(client_,&PatrolClient::pointsChanged,this,[this]{refreshPoints();});
  connect(client_,&PatrolClient::homePointChanged,this,[this]{updateControls();});
  connect(client_,&PatrolClient::statusChanged,this,[this]{
    const auto s=client_->status(); status_->setText(PatrolClient::stateName(s.state)+" · "+s.currentPointName+" · "+s.message); updateControls();
  });
  connect(client_,&PatrolClient::operationFinished,this,[this](const QString &op,bool,const QString &text){
    if (op=="一键到达" || op=="到点巡检任务" || op=="一键返回" || op=="返航任务" || op=="取消返航") message(text);
  });
  connect(client_,&PatrolClient::errorOccurred,this,[this](const QString &text){message(text);});
  auto *timer=new QTimer(this); timer->setInterval(500); connect(timer,&QTimer::timeout,this,[this]{updateControls();}); timer->start();
  refreshMap();
}

void InspectionTargetPage::setReturnPermission(std::function<bool(QString *)> permission) {
  returnPermission_=std::move(permission); updateControls();
}

void InspectionTargetPage::refreshMap() {
  PatrolMapInfo next;
  for (const auto &item:client_->maps()) if (item.mapId==client_->activeMapId()) next=item;
  if (next.mapId!=map_.mapId || next.revision!=map_.revision || next.navigationYaml!=map_.navigationYaml || !view_->valid()) {
    map_=next; targetValid_=false; selectedPointId_.clear(); selectedPointRevision_=0;
    if (map_.mapId.isEmpty()) {view_->clearMap(); message("请先在地图与返航设置中确认活跃地图。");}
    else if (map_.navigationYaml.isEmpty()) {view_->clearMap(); message("活跃地图缺少地图文件，请重新登记地图。");}
    else {QString error; if (!view_->load(map_,&error)) message("地图读取失败："+error); else message("地图已加载，请重新选择目标。");}
    refreshPoints();
  }
  updateControls();
}

void InspectionTargetPage::refreshPoints() {
  const QSignalBlocker block(points_);
  points_->clear(); points_->addItem("地图选点 / 临时坐标",QString());
  bool found=selectedPointId_.isEmpty();
  for (const auto &point:client_->points()) {
    if (point.mapId!=map_.mapId) continue;
    points_->addItem(point.name+(point.inspectionEnabled?" · 巡检":" · 仅到达"),point.pointId);
    if (point.pointId==selectedPointId_ && point.revision==selectedPointRevision_) {found=true; points_->setCurrentIndex(points_->count()-1);}
  }
  if (!found) {selectedPointId_.clear(); selectedPointRevision_=0; targetValid_=false; message("巡检点已变化，请重新选择目标。");}
  view_->points(client_->points(),map_.mapId); updateControls();
}

void InspectionTargetPage::chooseSavedPoint() {
  selectedPointId_=points_->currentData().toString(); selectedPointRevision_=0; targetValid_=false;
  if (selectedPointId_.isEmpty()) {message("在地图选择临时目标，或输入坐标。"); updateControls(); return;}
  for (const auto &point:client_->points()) if (point.pointId==selectedPointId_ && point.mapId==map_.mapId) {
    updating_=true;
    x_->setValue(point.x); y_->setValue(point.y); yaw_->setValue(point.yaw*180/kPi);
    inspect_->setChecked(point.inspectionEnabled); preset_->setValue(point.ptzPreset); prompt_->setPlainText(point.recognitionPrompt);
    selectedPointRevision_=point.revision; targetValid_=true; updating_=false;
    message("已选择 "+point.name+"，将使用该点保存的全部巡检设置。"); break;
  }
  updateControls();
}

void InspectionTargetPage::useCoordinates() {
  if (updating_) return;
  selectedPointId_.clear(); selectedPointRevision_=0;
  {const QSignalBlocker block(points_); points_->setCurrentIndex(0);}
  targetValid_=view_->valid(); updateControls();
}

void InspectionTargetPage::updateControls() {
  const bool saved=!selectedPointId_.isEmpty();
  const bool enabled=inspect_->isChecked();
  inspect_->setEnabled(!saved); preset_->setEnabled(!saved && enabled); prompt_->setEnabled(!saved && enabled);
  QString reason; const bool permitted=!permission_ || permission_(&reason);
  if (offline_) reason="离线模式不能执行任务";
  else if (!view_->valid() || map_.mapId.isEmpty()) reason="请先加载有效的活跃地图";
  else if (!client_->pointServerReady()) reason="到点巡检服务未就绪，请启用巡航联锁后启动导航";
  else if (client_->hasActiveOrPendingRun()) reason="已有任务或请求，等待完成或取消；可一键返回";
  else if (!permitted) {if(reason.isEmpty()) reason="当前不允许行驶";}
  else if (!targetValid_) reason="请在地图选择目标或输入坐标";
  else if (prompt_->toPlainText().size()>2000) reason="识别要求最多 2000 字";
  else {
    const auto p=view_->worldToPixel(x_->value(),y_->value());
    if (!view_->sceneRect().contains(p) || p.x()>=map_.width || p.y()<=0) reason="目标超出当前地图范围";
    else reason.clear();
  }
  start_->setText(enabled?"到达并巡检":"一键到达"); start_->setEnabled(reason.isEmpty()); start_->setToolTip(reason);
  readiness_->setText(reason.isEmpty()?"已就绪，点击后前往目标并执行设置的操作。":reason);
  cancel_->setEnabled(!offline_ && client_->hasActiveOrPendingRun());
  QString returnReason; const auto &permission=returnPermission_?returnPermission_:permission_;
  const bool returnAllowed=!permission || permission(&returnReason);
  const auto home=client_->homePoint();
  const bool validHome=home.exists && home.mapId==map_.mapId && home.mapRevision==map_.revision;
  home_->setEnabled(!offline_ && returnAllowed && validHome && client_->returnServerReady() && !client_->returnActiveOrPending());
  home_->setToolTip(!validHome?"先在地图与返航设置中，将停稳后的当前位置保存为返航点":returnReason);
  view_->target(x_->value(),y_->value(),yaw_->value()*kPi/180,targetValid_);
}

void InspectionTargetPage::start() {
  updateControls(); if (!start_->isEnabled()) return;
  client_->inspectPoint(map_.mapId,map_.revision,selectedPointId_,selectedPointRevision_,
      x_->value(),y_->value(),yaw_->value()*kPi/180,inspect_->isChecked(),preset_->value(),prompt_->toPlainText());
  message("正在提交到点巡检任务…"); updateControls();
}

void InspectionTargetPage::message(const QString &text) {message_->setText(text); updateControls();}
