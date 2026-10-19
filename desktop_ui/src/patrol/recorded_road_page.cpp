#include "patrol/recorded_road_page.h"
#include "patrol/patrol_client.h"
#include "patrol/patrol_ui_style.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QSaveFile>
#include <QDir>
#include <QMap>
#include <QFormLayout>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QGraphicsPixmapItem>
#include <QGroupBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QTableWidget>
#include <QTabWidget>
#include <QResizeEvent>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace {
class RoadView : public QGraphicsView {
public:
  using QGraphicsView::QGraphicsView;
  void fitScene() {
    autoFit_ = true;
    if (scene() && !scene()->sceneRect().isEmpty()) fitInView(scene()->sceneRect(), Qt::KeepAspectRatio);
  }
  void zoomBy(double factor) {
    const double next = transform().m11() * factor;
    if (next > .01 && next < 100) { autoFit_ = false; scale(factor, factor); }
  }
protected:
  void resizeEvent(QResizeEvent *event) override {
    QGraphicsView::resizeEvent(event);
    if (autoFit_) fitScene();
  }
  void wheelEvent(QWheelEvent *event) override {
    zoomBy(event->angleDelta().y()>0?1.15:1/1.15);
    event->accept();
  }
private:
  bool autoFit_ = true;
};
constexpr double kScale=40.0;
}

RecordedRoadPage::RecordedRoadPage(PatrolClient *client,QWidget *parent):QWidget(parent),client_(client) {
  setObjectName("recordedRoadPage");
  auto *root = new QVBoxLayout(this);
  root->setContentsMargins(0, 10, 0, 0);
  root->setSpacing(10);
  auto *statusRow = new QHBoxLayout;
  statusRow->addWidget(patrolui::heading("录制与维护道路", this));
  statusRow->addStretch();
  statusLabel_ = new QLabel("等待道路服务", this);
  statusLabel_->setObjectName("roadRecordingStatus");
  statusLabel_->setWordWrap(true);
  statusLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  statusRow->addWidget(statusLabel_, 2);
  root->addLayout(statusRow);
  auto *split = new QSplitter(this);
  split->setObjectName("roadWorkspaceSplitter");
  split->setChildrenCollapsible(false);
  root->addWidget(split, 1);
  auto *controls = patrolui::card(split);
  controls->setMinimumWidth(306);
  controls->setMaximumWidth(360);
  auto *left = new QVBoxLayout(controls);
  left->setContentsMargins(12, 12, 12, 12);
  left->setSpacing(8);
  auto button = [&](const QString &text, const QString &name) {
    auto *b = new QPushButton(text, controls); b->setObjectName(name); b->setAutoDefault(false); return b;
  };
  auto *netHeading = new QHBoxLayout;
  netHeading->addWidget(patrolui::heading("当前路网", controls));
  netHeading->addStretch();
  auto *reload = button("刷新", "roadRefresh");
  netHeading->addWidget(reload);
  left->addLayout(netHeading);
  networks_ = new QComboBox(controls);
  networks_->setObjectName("roadNetworks");
  networks_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  networks_->setMinimumContentsLength(12);
  left->addWidget(networks_);
  networkSummary_ = patrolui::hint("还没有路网，请在“路网管理”中新建或导入。", controls);
  networkSummary_->setObjectName("roadNetworkSummary");
  left->addWidget(networkSummary_);

  auto *controlTabs = new QTabWidget(controls);
  controlTabs->setObjectName("roadControlTabs");
  controlTabs->setDocumentMode(true);
  left->addWidget(controlTabs, 1);
  auto makeTab = [&](const QString &name) {
    auto *scroll = new QScrollArea(controlTabs);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto *body = new QWidget(scroll);
    auto *layout = new QVBoxLayout(body);
    layout->setContentsMargins(0, 10, 4, 4);
    layout->setSpacing(10);
    scroll->setWidget(body);
    controlTabs->addTab(scroll, name);
    return layout;
  };
  auto *recordTab = new QWidget(controlTabs);
  recordTab->setObjectName("roadRecordingTab");
  auto *recordLayout = new QVBoxLayout(recordTab);
  recordLayout->setContentsMargins(0, 8, 0, 0);
  recordLayout->setSpacing(8);
  controlTabs->addTab(recordTab, "录制道路");
  auto *recordScroll = new QScrollArea(recordTab);
  recordScroll->setObjectName("roadRecordingFormScroll");
  recordScroll->setWidgetResizable(true);
  recordScroll->setMinimumHeight(20);
  recordScroll->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Ignored);
  recordScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto *recordFields = new QWidget(recordScroll);
  auto *recordFieldsLayout = new QVBoxLayout(recordFields);
  recordFieldsLayout->setContentsMargins(0, 0, 4, 0);
  recordFieldsLayout->setSpacing(10);
  recordScroll->setWidget(recordFields);
  recordLayout->addWidget(recordScroll, 1);
  auto *rf = new QFormLayout;
  rf->setSpacing(8);
  roadName_ = new QLineEdit("道路", controls);
  roadName_->setObjectName("roadName");
  rf->addRow("道路名称", roadName_);
  startNode_ = new QComboBox(controls);
  endNode_ = new QComboBox(controls);
  startNode_->setObjectName("roadStartNode");
  endNode_->setObjectName("roadEndNode");
  for (auto *combo : {startNode_, endNode_}) {
    combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    combo->setMinimumContentsLength(8);
  }
  rf->addRow("起点连接", startNode_);
  rf->addRow("终点连接", endNode_);
  recordFieldsLayout->addLayout(rf);
  begin_ = button("开始录制", "roadBegin"); begin_->setProperty("primary", true);
  pause_ = button("暂停录制", "roadPause");
  resume_ = button("继续录制", "roadResume");
  finish_ = button("结束预览", "roadFinish");
  save_ = button("保存道路", "roadSave"); save_->setProperty("primary", true);
  discard_ = button("放弃草稿", "roadDiscard"); discard_->setProperty("patrolDanger", true);
  auto *splitRoad = button("此处分段，继续录制", "roadMarkNode");
  auto *grid = new QGridLayout;
  grid->setSpacing(8);
  grid->addWidget(begin_, 0, 0); grid->addWidget(pause_, 0, 1); grid->addWidget(resume_, 0, 2);
  grid->addWidget(finish_, 1, 0); grid->addWidget(save_, 1, 1); grid->addWidget(discard_, 1, 2);
  grid->addWidget(splitRoad, 2, 0, 1, 3);
  for(auto *control : {begin_, pause_, resume_, finish_, save_, discard_})
    control->setStyleSheet("padding-left:4px; padding-right:4px;");
  recordLayout->addLayout(grid);
  recordFieldsLayout->addWidget(patrolui::hint("手动驾驶录制 → 结束预览 → 保存道路。路口转弯和反向通行需分别补录。", controls));
  auto *draftBox = new QGroupBox("恢复上次草稿", controls);
  auto *draftLayout = new QVBoxLayout(draftBox);
  draftsCombo_ = new QComboBox(draftBox);
  draftsCombo_->setObjectName("roadDrafts");
  draftsCombo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  draftsCombo_->setMinimumContentsLength(8);
  draftLayout->addWidget(draftsCombo_);
  auto *recover = button("载入草稿预览", "roadRecover");
  draftLayout->addWidget(recover);
  recordFieldsLayout->addWidget(draftBox);
  recordFieldsLayout->addStretch();

  auto *bindLayout = makeTab("点位绑定");
  bindLayout->addWidget(patrolui::hint("将导航点绑定到道路节点，固定路线才能访问这些点位。", controls));
  auto *bf = new QFormLayout;
  bf->setSpacing(10);
  point_ = new QComboBox(controls); point_->setObjectName("roadBindingPoint");
  bindingNode_ = new QComboBox(controls); bindingNode_->setObjectName("roadBindingNode");
  for (auto *combo : {point_, bindingNode_}) {
    combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    combo->setMinimumContentsLength(8);
  }
  bf->addRow("导航点", point_); bf->addRow("道路节点", bindingNode_);
  bindLayout->addLayout(bf);
  bindingSummary_ = patrolui::hint("请选择导航点查看已有绑定。", controls);
  bindingSummary_->setObjectName("roadBindingSummary");
  bindLayout->addWidget(bindingSummary_);
  auto *bind = button("添加所选节点绑定", "roadBindPoint");
  bindLayout->addWidget(bind);
  auto *unbind = button("移除所选节点绑定", "roadUnbindPoint");
  unbind->setProperty("patrolDanger", true);
  bindLayout->addWidget(unbind);
  bindLayout->addStretch();
  bindLayout->addWidget(patrolui::hint("道路修改和点位绑定完成后，点击下方“校验并发布路网”。", controls));

  auto *manageLayout = makeTab("路网管理");
  auto *newNetwork = new QGroupBox("新建与文件", controls);
  auto *nf = new QVBoxLayout(newNetwork);
  networkName_ = new QLineEdit(newNetwork);
  networkName_->setObjectName("roadNetworkName");
  networkName_->setPlaceholderText("新路网名称，例如：厂区道路");
  nf->addWidget(networkName_);
  auto *create = button("新建路网", "roadCreateNetwork");
  nf->addWidget(create);
  auto *io = new QHBoxLayout;
  auto *import = button("导入路网", "roadImport");
  auto *exportButton = button("导出路网", "roadExport");
  io->addWidget(import); io->addWidget(exportButton); nf->addLayout(io);
  manageLayout->addWidget(newNetwork);
  auto *edgeGroup = new QGroupBox("所选道路设置", controls);
  auto *ef = new QFormLayout(edgeGroup);
  roadSelectionLabel_ = patrolui::hint("请在右侧道路列表中选择一条道路。", edgeGroup);
  ef->addRow(roadSelectionLabel_);
  speed_ = new QDoubleSpinBox(edgeGroup); speed_->setObjectName("roadSpeed");
  speed_->setRange(.03, .5); speed_->setDecimals(2); speed_->setValue(.2); speed_->setSuffix(" m/s");
  edgeEnabled_ = new QCheckBox("允许通行", edgeGroup); edgeEnabled_->setChecked(true);
  ef->addRow("道路限速", speed_); ef->addRow(edgeEnabled_);
  auto *update = button("保存道路设置", "roadUpdateEdge"); ef->addRow(update);
  manageLayout->addWidget(edgeGroup);
  auto *region = button("导入允许通行区域", "roadRegion");
  region->setToolTip("JSON 格式：地图坐标下的多边形顶点数组 [[x,y],…]。未配置时仍受地图障碍和局部绕行范围限制。");
  manageLayout->addWidget(region);
  manageLayout->addStretch();
  publish_ = button("校验并发布路网", "roadPublish");
  publish_->setProperty("primary", true);
  left->addWidget(publish_);

  auto *right = patrolui::card(split);
  auto *rightLayout = new QVBoxLayout(right);
  rightLayout->setContentsMargins(14, 12, 14, 12);
  rightLayout->setSpacing(8);
  auto *viewHeader = new QHBoxLayout;
  viewHeader->addWidget(patrolui::heading("路网预览", right));
  viewHeader->addStretch();
  auto *zoomOut = button("−", "roadZoomOut"); zoomOut->setToolTip("缩小");
  auto *zoomIn = button("+", "roadZoomIn"); zoomIn->setToolTip("放大");
  auto *fit = button("适配视图", "roadFitView");
  viewHeader->addWidget(zoomOut); viewHeader->addWidget(zoomIn); viewHeader->addWidget(fit);
  rightLayout->addLayout(viewHeader);
  auto *legend = new QLabel("<span style='color:#2563eb'>● 前进道路</span>　"
                           "<span style='color:#d97706'>● 倒车道路</span>　"
                           "<span style='color:#16a34a'>● 已验证路线</span>　"
                           "<span style='color:#a855f7'>● 当前录制</span>", right);
  legend->setObjectName("roadLegend");
  legend->setWordWrap(true);
  rightLayout->addWidget(legend);
  scene_ = new QGraphicsScene(this);
  view_ = new RoadView(scene_, right); view_->setObjectName("roadPreview");
  view_->setRenderHint(QPainter::Antialiasing);
  view_->setDragMode(QGraphicsView::ScrollHandDrag);
  view_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  view_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  rightLayout->addWidget(view_, 1);
  auto *emptyLayout = new QVBoxLayout(view_->viewport());
  emptyLayout->setContentsMargins(30, 20, 30, 20);
  emptyLayout->addStretch();
  emptyPreview_ = patrolui::hint("当前路网还没有道路\n先新建路网，再手动驾驶录制第一条道路。", view_->viewport());
  emptyPreview_->setObjectName("roadPreviewEmpty");
  emptyPreview_->setAlignment(Qt::AlignCenter);
  emptyPreview_->setAttribute(Qt::WA_TransparentForMouseEvents);
  emptyLayout->addWidget(emptyPreview_); emptyLayout->addStretch();
  auto *roadsHeader = new QHBoxLayout;
  roadsHeader->addWidget(patrolui::heading("道路列表", right));
  roadsHeader->addStretch();
  roadCount_ = patrolui::hint("0 条道路", right);
  roadsHeader->addWidget(roadCount_);
  rightLayout->addLayout(roadsHeader);
  roads_ = new QTableWidget(0, 4, right); roads_->setObjectName("roadEdgesTable");
  roads_->setHorizontalHeaderLabels({"道路", "起点 → 终点", "运动段", "通行"});
  patrolui::configureTable(roads_);
  roads_->setMinimumHeight(100); roads_->setMaximumHeight(140);
  rightLayout->addWidget(roads_);
  split->setStretchFactor(0, 0); split->setStretchFactor(1, 1);
  split->setSizes({330, 800});
  message_ = patrolui::hint("滚轮缩放，拖动查看；预览仅用于查看路线。", this);
  message_->setTextFormat(Qt::PlainText);
  message_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  root->addWidget(message_);
  connect(zoomOut, &QPushButton::clicked, this, [this] { static_cast<RoadView *>(view_)->zoomBy(1 / 1.2); });
  connect(zoomIn, &QPushButton::clicked, this, [this] { static_cast<RoadView *>(view_)->zoomBy(1.2); });
  connect(fit, &QPushButton::clicked, this, [this] { static_cast<RoadView *>(view_)->fitScene(); });
  connect(point_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &RecordedRoadPage::updateBindingSummary);

  connect(reload,&QPushButton::clicked,this,&RecordedRoadPage::refresh);
  connect(create,&QPushButton::clicked,this,[this]{if(!networkName_->text().trimmed().isEmpty()) command("CREATE",{{"name",networkName_->text().trimmed()}});});
  connect(begin_,&QPushButton::clicked,this,[this]{command("START_RECORD",{{"name",roadName_->text().trimmed()}, {"start_node",startNode_->currentData().toString()}});});
  connect(pause_,&QPushButton::clicked,this,[this]{command("PAUSE_RECORD");});
  connect(resume_,&QPushButton::clicked,this,[this]{command("RESUME_RECORD");});
  connect(finish_,&QPushButton::clicked,this,[this]{command("FINISH_RECORD");});
  connect(splitRoad,&QPushButton::clicked,this,[this]{command("MARK_NODE",{{"end_node",endNode_->currentData().toString()}});});
  connect(save_,&QPushButton::clicked,this,[this]{command("SAVE_RECORD",{{"end_node",endNode_->currentData().toString()}});});
  connect(discard_,&QPushButton::clicked,this,[this]{if(QMessageBox::question(this,"放弃录制","放弃当前录制草稿？原始数据仍保留在本地记录中。") == QMessageBox::Yes) command("DISCARD_RECORD");});
  connect(recover,&QPushButton::clicked,this,[this]{command("RECOVER_DRAFT",{{"draft_id",draftsCombo_->currentData().toString()}});});
  connect(bind,&QPushButton::clicked,this,[this]{
    const auto id=point_->currentData().toString(),node=bindingNode_->currentData().toString();
    auto nodes=selectedNetwork()["document"].toObject()["bindings"].toObject()[id].toArray();
    if(!nodes.contains(node)) nodes.append(node);
    command("BIND_POINT",{{"point_id",id},{"node_ids",nodes}});
  });
  connect(unbind,&QPushButton::clicked,this,[this]{command("UNBIND_POINT",{{"point_id",point_->currentData().toString()},{"node_id",bindingNode_->currentData().toString()}});});
  connect(publish_,&QPushButton::clicked,this,[this]{command("PUBLISH");});
  connect(update,&QPushButton::clicked,this,[this]{int row=roads_->currentRow();if(row>=0) command("UPDATE_EDGE",{{"edge_id",roads_->item(row,0)->data(Qt::UserRole).toString()}, {"enabled",edgeEnabled_->isChecked()},{"speed",speed_->value()}});});
  connect(exportButton,&QPushButton::clicked,this,[this]{command("EXPORT");});
  connect(import,&QPushButton::clicked,this,[this]{
    const auto path=QFileDialog::getOpenFileName(this,"导入录制路网",{},"JSON (*.json)");if(path.isEmpty()) return;
    QFile f(path);if(f.size()>32*1024*1024 || !f.open(QIODevice::ReadOnly)) {message_->setText("文件过大或无法读取");return;}
    const auto value=QJsonDocument::fromJson(f.readAll());if(!value.isObject()) {message_->setText("路网文件格式无效");return;}
    command("IMPORT",{{"export",value.object()}});
  });
  connect(region,&QPushButton::clicked,this,[this]{
    auto path=QFileDialog::getOpenFileName(this,"允许通行区域",{},"JSON (*.json)");if(path.isEmpty()) return;
    QFile f(path);if(!f.open(QIODevice::ReadOnly) || f.size()>100000) return;
    auto value=QJsonDocument::fromJson(f.readAll());if(value.isArray()) command("SET_REGION",{{"polygon",value.array()}});
  });
  connect(networks_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{if(!refreshing_) {plan_={};updateNetworks();command("SHOW_NETWORK");}});
  connect(roads_,&QTableWidget::itemSelectionChanged,this,[this]{
    int row=roads_->currentRow();if(row<0) {roadSelectionLabel_->setText("请在右侧道路列表中选择一条道路。");return;}
    auto edge=selectedNetwork()["document"].toObject()["edges"].toObject()[roads_->item(row,0)->data(Qt::UserRole).toString()].toObject();
    roadSelectionLabel_->setText("当前道路：" + edge["name"].toString());
    speed_->setValue(edge["speed"].toDouble(.2));edgeEnabled_->setChecked(edge["enabled"].toBool(true));
  });
  connect(client_,&PatrolClient::roadReply,this,&RecordedRoadPage::reply);
  connect(client_,&PatrolClient::roadNetworksChanged,this,&RecordedRoadPage::updateNetworks);
  connect(client_,&PatrolClient::pointsChanged,this,&RecordedRoadPage::updatePoints);
  connect(client_,&PatrolClient::mapsChanged,this,[this]{refresh();});
  auto *timer=new QTimer(this);timer->setInterval(1000);
  connect(timer,&QTimer::timeout,this,[this]{
    if(client_->roadServerReady()) client_->roadCommand("STATUS");
    else {statusLabel_->setText("道路服务未就绪：请启动定位导航并勾选“路线巡航联锁”");patrolui::tone(statusLabel_,"warning");begin_->setEnabled(false);}
  });timer->start();refresh();
}

QJsonObject RecordedRoadPage::selectedNetwork() const {
  const auto id=networks_->currentData().toString();
  for(auto value:client_->roadNetworks()) if(value.toObject()["network_id"].toString()==id) return value.toObject();
  return {};
}
void RecordedRoadPage::command(const QString &op,QJsonObject payload) {
  const auto net=selectedNetwork();
  if(!payload.contains("network_id")) payload["network_id"]=net["network_id"];
  if(!payload.contains("expected_revision")) payload["expected_revision"]=net["revision"];
  client_->roadCommand(op,payload);
}
void RecordedRoadPage::refresh() {
  if(client_->roadServerReady()) {client_->roadCommand("LIST");client_->roadCommand("STATUS");}
  updatePoints();
}
void RecordedRoadPage::updatePoints() {
  auto selected=point_->currentData();point_->clear();
  for(auto p:client_->points()) if(p.mapId==client_->activeMapId()) point_->addItem(p.name,p.pointId);
  auto index=point_->findData(selected);if(index>=0) point_->setCurrentIndex(index);
  updateBindingSummary();
}
void RecordedRoadPage::updateBindingSummary() {
  const auto doc = selectedNetwork()["document"].toObject();
  const auto nodes = doc["nodes"].toObject();
  QStringList names;
  for (auto id : doc["bindings"].toObject()[point_->currentData().toString()].toArray()) {
    const auto name = nodes[id.toString()].toObject()["name"].toString();
    names << (name.isEmpty() ? id.toString() : name);
  }
  bindingSummary_->setText(point_->currentIndex() < 0 ? "请先在“点位管理”中录入导航点。"
                          : names.isEmpty() ? "该导航点尚未绑定道路节点。"
                                            : "已绑定：" + names.join("、"));
}
void RecordedRoadPage::updateNetworks() {
  refreshing_=true;
  const auto id=pendingNetworkId_.isEmpty()?networks_->currentData():QVariant(pendingNetworkId_);networks_->clear();
  for(auto item:client_->roadNetworks()) {auto n=item.toObject();networks_->addItem(n["name"].toString()+QString(" · v%1 · %2").arg(n["revision"].toInt()).arg(n["published"].toBool()?"已发布":"待校验"),n["network_id"].toString());}
  int selected=networks_->findData(id);if(selected>=0) {networks_->setCurrentIndex(selected);pendingNetworkId_.clear();}
  auto doc=selectedNetwork()["document"].toObject();auto nodes=doc["nodes"].toObject();
  for(auto *combo:{startNode_,endNode_,bindingNode_}) {
    const auto previous=combo->currentData();combo->clear();
    if(combo!=bindingNode_) combo->addItem("新建独立节点",QString());
    if(combo==endNode_) combo->addItem("本次起点（闭环）",QString("__start__"));
    for(auto it=nodes.begin();it!=nodes.end();++it) {auto n=it.value().toObject();combo->addItem(n["name"].toString()+QString(" · %1°").arg(n["yaw"].toDouble()*180/3.141592653589793,0,'f',0),it.key());}
    int pos=combo->findData(previous);if(pos>=0) combo->setCurrentIndex(pos);
  }
  QString selectedEdge;
  if(roads_->currentRow()>=0 && roads_->item(roads_->currentRow(),0))
    selectedEdge=roads_->item(roads_->currentRow(),0)->data(Qt::UserRole).toString();
  roads_->setRowCount(0);auto edges=doc["edges"].toObject();
  for(auto it=edges.begin();it!=edges.end();++it) {
    auto edge=it.value().toObject();int row=roads_->rowCount();roads_->insertRow(row);
    auto *name=new QTableWidgetItem(edge["name"].toString());name->setData(Qt::UserRole,it.key());roads_->setItem(row,0,name);
    auto nodeName=[&](const QString &key){return nodes[edge[key].toString()].toObject()["name"].toString();};
    roads_->setItem(row,1,new QTableWidgetItem(nodeName("from")+" → "+nodeName("to")));
    QStringList dirs;for(auto b:edge["blocks"].toArray()) dirs<< (b.toObject()["direction"].toInt()>0?"前进":"倒车");
    roads_->setItem(row,2,new QTableWidgetItem(dirs.join(" → ")));
    roads_->setItem(row,3,new QTableWidgetItem(edge["enabled"].toBool(true)?"启用":"停用"));
    for(int column=0;column<4;++column) roads_->item(row,column)->setToolTip(roads_->item(row,column)->text());
  }
  for(int row=0;row<roads_->rowCount();++row)
    if(roads_->item(row,0)->data(Qt::UserRole).toString()==selectedEdge) {roads_->selectRow(row);break;}
  roadCount_->setText(QString("%1 条道路").arg(edges.size()));
  networkSummary_->setText(selectedNetwork().isEmpty() ? "还没有路网，请在“路网管理”中新建或导入。"
                          : QString("%1 个节点 · %2 条道路 · %3").arg(nodes.size()).arg(edges.size())
                            .arg(selectedNetwork()["published"].toBool()?"已发布":"修改后需校验发布"));
  updateBindingSummary();
  refreshing_=false;redraw();
}
void RecordedRoadPage::reply(const QString &op,bool success,const QString &text,const QJsonObject &data) {
  if(op=="STATUS") {
    if(!success) {statusLabel_->setText(text);patrolui::tone(statusLabel_,"danger");return;}
    const auto previousLive=status_["live_samples"];
    status_=data;state_=data["state"].toString("IDLE");
    const auto networkId=data["network_id"].toString();
    if(state_!="IDLE" && !networkId.isEmpty() && networkId!=networks_->currentData().toString()) {
      pendingNetworkId_=networkId;updateNetworks();
      if(!pendingNetworkId_.isEmpty()) client_->roadCommand("LIST");
    }
    if(data["live_samples"]!=previousLive) redraw(true);
    const QMap<QString,QString> names{{"IDLE","空闲"},{"RECORDING","录制中"},{"PAUSED","录制暂停"},{"REVIEW","待保存"},{"INVALID","录制无效"}};
    bool enabled=data["enabled"].toBool();
    statusLabel_->setText(enabled?QString("%1 · %2m · %3个样本 %4").arg(names.value(state_,state_)).arg(data["length"].toDouble(),0,'f',2).arg(data["samples"].toInt()).arg(data["error"].toString()):QString("本会话未启用固定路线：请停止流程，再勾选“路线巡航联锁”并启动定位导航"));
    patrolui::tone(statusLabel_, !enabled || state_=="PAUSED" || state_=="REVIEW" ? "warning"
                               : state_=="INVALID" ? "danger" : state_=="RECORDING" ? "info" : "good");
    const bool idle=state_=="IDLE";begin_->setEnabled(enabled&&idle&&!selectedNetwork().isEmpty()&&!client_->hasActiveOrPendingRun());
    pause_->setEnabled(state_=="RECORDING");resume_->setEnabled(state_=="PAUSED");finish_->setEnabled(state_=="RECORDING"||state_=="PAUSED");
    findChild<QPushButton*>("roadMarkNode")->setEnabled(state_=="RECORDING");
    save_->setEnabled(state_=="REVIEW");discard_->setEnabled(!idle);publish_->setEnabled(enabled&&idle);
    networks_->setEnabled(idle);startNode_->setEnabled(idle);roadName_->setEnabled(idle);
    return;
  }
  if(op=="LIST" && success) {
    drafts_=data["drafts"].toArray();draftsCombo_->clear();
    for(auto draft:drafts_) {auto d=draft.toObject();draftsCombo_->addItem(d["payload"].toObject()["name"].toString()+" · "+d["draft_id"].toString().right(8),d["draft_id"].toString());}
    return;
  }
  if(op=="PLAN") return;
  if(op=="SHOW_NETWORK") return;
  if(!success) {message_->setText(text);patrolui::tone(message_,"danger");return;}
  if(op=="CREATE" || op=="IMPORT") pendingNetworkId_=data["network_id"].toString();
  if(data.contains("review_blocks")) {
    plan_=QJsonObject{{"legs",QJsonArray{QJsonObject{{"blocks",data["review_blocks"]}}}}};redraw();
  }
  if(op=="EXPORT") {
    auto path=QFileDialog::getSaveFileName(this,"导出录制路网",{},"JSON (*.json)");if(path.isEmpty()) return;
    QSaveFile file(path);const auto bytes=QJsonDocument(data).toJson(QJsonDocument::Indented);
    if(!file.open(QIODevice::WriteOnly) || file.write(bytes)!=bytes.size() || !file.commit()) message_->setText("导出失败："+file.errorString());
    else message_->setText("路网已导出："+path);
    return;
  }
  message_->setText(op=="PUBLISH"?"路网已通过校验并发布，请在巡检路线中选择此版本后保存。":"操作完成；修改过的路网需重新校验发布。" );
  patrolui::tone(message_,"info");
  refresh();
}
void RecordedRoadPage::showPlan(const QJsonObject &plan) {
  const auto index=networks_->findData(plan["network_id"].toString());
  if(index>=0) {refreshing_=true;networks_->setCurrentIndex(index);refreshing_=false;updateNetworks();}
  plan_=plan;redraw();
}
void RecordedRoadPage::pauseForClose() {if(state_=="RECORDING") command("PAUSE_RECORD");}
void RecordedRoadPage::redraw(bool preserveView) {
  const auto transform=view_->transform();const auto center=view_->mapToScene(view_->viewport()->rect().center());
  scene_->clear();auto net=selectedNetwork();auto doc=net["document"].toObject();
  for(auto map:client_->maps()) if(map.mapId==client_->activeMapId()) {
    try {
      auto yaml=YAML::LoadFile(map.navigationYaml.toStdString());auto image=QString::fromStdString(yaml["image"].as<std::string>());
      if(QFileInfo(image).isRelative()) image=QFileInfo(map.navigationYaml).dir().filePath(image);
      QPixmap pix(image);if(!pix.isNull()) {
        auto *item=scene_->addPixmap(pix);double s=map.resolution*kScale,c=std::cos(map.originYaw),q=std::sin(map.originYaw);
        item->setTransform(QTransform(c*s,-q*s,q*s,c*s,kScale*map.originX-q*s*pix.height(),-kScale*map.originY-c*s*pix.height()));item->setOpacity(.7);item->setZValue(-1);
      }
    } catch(const std::exception &) {}
    break;
  }
  auto draw=[this](const QJsonArray &points,const QColor &color,double width){
    if(points.isEmpty()) return;QPainterPath path;
    auto first=points[0].toObject();path.moveTo(first["x"].toDouble()*kScale,-first["y"].toDouble()*kScale);
    for(auto point:points) {auto p=point.toObject();path.lineTo(p["x"].toDouble()*kScale,-p["y"].toDouble()*kScale);}
    QPen pen(color,width);pen.setCosmetic(true);
    scene_->addPath(path,pen);
    if(points.size()>2) {
      int i=points.size()/2;auto a=points[qMax(0,i-3)].toObject(),b=points[i].toObject();
      QPointF end(b["x"].toDouble()*kScale,-b["y"].toDouble()*kScale);double theta=std::atan2(-(b["y"].toDouble()-a["y"].toDouble()),b["x"].toDouble()-a["x"].toDouble());
      scene_->addLine(QLineF(end,end+QPointF(std::cos(theta+2.6)*7,std::sin(theta+2.6)*7)),QPen(color,width));
      scene_->addLine(QLineF(end,end+QPointF(std::cos(theta-2.6)*7,std::sin(theta-2.6)*7)),QPen(color,width));
    }
  };
  for(auto value:doc["edges"].toObject()) {
    auto edge=value.toObject();if(!edge["enabled"].toBool(true)) continue;
    for(auto block:edge["blocks"].toArray()) {auto b=block.toObject();draw(b["samples"].toArray(),b["direction"].toInt()>0?QColor("#2563eb"):QColor("#d97706"),1.5);}
  }
  for(auto value:doc["nodes"].toObject()) {
    auto n=value.toObject();QPointF p(n["x"].toDouble()*kScale,-n["y"].toDouble()*kScale);
    scene_->addEllipse(p.x()-2,p.y()-2,4,4,QPen(Qt::darkBlue),QBrush(Qt::white));
    const double yaw=n["yaw"].toDouble();QPointF head=p+QPointF(12*std::cos(yaw),-12*std::sin(yaw));
    scene_->addLine(QLineF(p,head),QPen(QColor("#0f172a"),1.2));
    for(double delta:{-0.5,0.5}) scene_->addLine(QLineF(head,head-QPointF(4*std::cos(yaw+delta),-4*std::sin(yaw+delta))),QPen(QColor("#0f172a"),1.2));
    auto *label=scene_->addText(n["name"].toString());label->setPos(p+QPointF(6,-18));label->setDefaultTextColor(QColor("#0f172a"));
    label->setFlag(QGraphicsItem::ItemIgnoresTransformations);
  }
  auto legs=plan_["legs"].toArray();if(plan_["closure"].isObject()) legs.append(plan_["closure"]);
  for(auto leg:legs) for(auto block:leg.toObject()["blocks"].toArray()) draw(block.toObject()["samples"].toArray(),QColor("#16a34a"),2.8);
  if(state_=="RECORDING" || state_=="PAUSED") draw(status_["live_samples"].toArray(),QColor("#a855f7"),2.5);
  if(!scene_->items().isEmpty()) {
    scene_->setSceneRect(scene_->itemsBoundingRect().adjusted(-15,-15,15,15));
    if(!preserveView) static_cast<RoadView *>(view_)->fitScene();
    else {view_->setTransform(transform);view_->centerOn(center);}
  } else {
    scene_->setSceneRect(-100,-100,200,200);
  }
  const bool hasRoad = !doc["edges"].toObject().isEmpty() || !plan_["legs"].toArray().isEmpty()
                       || !status_["live_samples"].toArray().isEmpty();
  emptyPreview_->setVisible(!hasRoad);
}
