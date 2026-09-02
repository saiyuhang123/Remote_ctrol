#include "main_window.h"
#include <QDateTime>
#include <QMessageBox>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>

namespace hk_camera
{

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    camera_  = HKCamera::instance();
    decoder_ = new HKDecoder(this);

    setupUI();
    setupConnections();

    setWindowTitle("海康相机控制台");
    resize(900, 520);
}

MainWindow::~MainWindow()
{
    decoder_->stop();
    if (user_id_ >= 0)      camera_->logout(user_id_);
}

// ============================================================
// UI 搭建
// ============================================================

void MainWindow::setupUI()
{
    auto* central = new QWidget(this);
    setCentralWidget(central);
    auto* main_layout = new QHBoxLayout(central);
    main_layout->setContentsMargins(4, 4, 4, 4);
    main_layout->setSpacing(4);

    // ---- 左侧: 预览 + 状态 ----
    auto* left_widget = new QWidget();
    auto* left_layout  = new QVBoxLayout(left_widget);
    left_layout->setContentsMargins(0, 0, 0, 0);

    video_label_ = new QLabel("等待视频流...");
    video_label_->setMinimumSize(320, 240);
    video_label_->setAlignment(Qt::AlignCenter);
    video_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    video_label_->setStyleSheet(
        "QLabel { background-color: #1a1a1a; color: #888; "
        "border: 2px solid #555; border-radius: 4px; }");
    left_layout->addWidget(video_label_);

    status_label_ = new QLabel("● 未连接");
    status_label_->setStyleSheet("QLabel { color: #ff6600; font-weight: bold; padding: 2px; }");
    left_layout->addWidget(status_label_);

    main_layout->addWidget(left_widget, 2);

    // ---- 右侧: 控制面板 (可滚动) ----
    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setMaximumWidth(480);
    scroll->setMinimumWidth(380);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto* right_widget = new QWidget();
    auto* right_layout = new QVBoxLayout(right_widget);
    right_layout->setContentsMargins(2, 2, 2, 2);
    right_layout->setSpacing(4);

    // ---- 1. 登录 (紧凑表单) ----
    login_group_ = new QGroupBox("设备登录");
    auto* login_layout = new QGridLayout(login_group_);
    login_layout->setSpacing(2);

    ip_edit_   = new QLineEdit("192.168.1.64");
    port_edit_ = new QLineEdit("8000");
    user_edit_ = new QLineEdit("admin");
    pass_edit_ = new QLineEdit("a1234567");
    pass_edit_->setEchoMode(QLineEdit::Password);
    ip_edit_->setMaximumHeight(26);   port_edit_->setMaximumHeight(26);
    user_edit_->setMaximumHeight(26); pass_edit_->setMaximumHeight(26);
    port_edit_->setMaximumWidth(60);

    login_btn_  = new QPushButton("登录");
    logout_btn_ = new QPushButton("登出");
    logout_btn_->setEnabled(false);
    login_btn_->setMaximumHeight(26);  logout_btn_->setMaximumHeight(26);

    login_layout->addWidget(new QLabel("IP:"),   0, 0);
    login_layout->addWidget(ip_edit_,             0, 1, 1, 3);
    login_layout->addWidget(new QLabel("端口:"),  0, 4);
    login_layout->addWidget(port_edit_,           0, 5);
    login_layout->addWidget(new QLabel("用户:"),  1, 0);
    login_layout->addWidget(user_edit_,           1, 1);
    login_layout->addWidget(new QLabel("密码:"),  1, 2);
    login_layout->addWidget(pass_edit_,           1, 3);
    login_layout->addWidget(login_btn_,           1, 4);
    login_layout->addWidget(logout_btn_,          1, 5);

    right_layout->addWidget(login_group_);

    // ---- 2. 视频控制 (单行) ----
    stream_group_ = new QGroupBox("视频控制");
    auto* stream_layout = new QHBoxLayout(stream_group_);
    stream_layout->setSpacing(3);

    channel_combo_ = new QComboBox();
    for (int i = 1; i <= 16; i++)
        channel_combo_->addItem(QString("通道%1").arg(i), i);
    channel_combo_->setMaximumWidth(70);

    start_stream_btn_ = new QPushButton("取流");
    stop_stream_btn_  = new QPushButton("停止");
    snapshot_btn_     = new QPushButton("快照");
    capture_btn_      = new QPushButton("抓图");
    start_stream_btn_->setEnabled(false);
    stop_stream_btn_->setEnabled(false);
    snapshot_btn_->setEnabled(false);
    capture_btn_->setEnabled(false);
    snapshot_btn_->setCheckable(true);
    for (auto* b : {start_stream_btn_, stop_stream_btn_, snapshot_btn_, capture_btn_})
        b->setMaximumHeight(26);

    stream_layout->addWidget(channel_combo_);
    stream_layout->addWidget(start_stream_btn_);
    stream_layout->addWidget(stop_stream_btn_);
    stream_layout->addWidget(snapshot_btn_);
    stream_layout->addWidget(capture_btn_);

    right_layout->addWidget(stream_group_);

    // ---- 3. 云台 + 镜头控制 (合并) ----
    ptz_group_ = new QGroupBox("云台 / 镜头");
    auto* ptz_layout = new QGridLayout(ptz_group_);
    ptz_layout->setSpacing(2);
    ptz_layout->setContentsMargins(4, 4, 4, 4);

    // 方向键 (第一行)
    ptz_up_btn_    = new QPushButton("▲");
    ptz_down_btn_  = new QPushButton("▼");
    ptz_left_btn_  = new QPushButton("◄");
    ptz_right_btn_ = new QPushButton("►");
    ptz_stop_btn_  = new QPushButton("■");
    for (auto* b : {ptz_up_btn_, ptz_down_btn_, ptz_left_btn_, ptz_right_btn_, ptz_stop_btn_})
        { b->setMaximumHeight(28); b->setMaximumWidth(50); }

    ptz_layout->addWidget(ptz_up_btn_,    0, 1);
    ptz_layout->addWidget(ptz_left_btn_,   1, 0);
    ptz_layout->addWidget(ptz_stop_btn_,   1, 1);
    ptz_layout->addWidget(ptz_right_btn_,  1, 2);
    ptz_layout->addWidget(ptz_down_btn_,   2, 1);

    // 变倍
    auto* zoom_layout = new QHBoxLayout();
    zoom_layout->setSpacing(2);
    zoom_in_btn_  = new QPushButton("放大+");
    zoom_out_btn_ = new QPushButton("缩小-");
    for (auto* b : {zoom_in_btn_, zoom_out_btn_}) b->setMaximumHeight(26);
    zoom_layout->addWidget(zoom_in_btn_);
    zoom_layout->addWidget(zoom_out_btn_);
    ptz_layout->addLayout(zoom_layout, 0, 3, 1, 2);

    // 聚焦按钮
    auto* focus_btn_layout = new QHBoxLayout();
    focus_btn_layout->setSpacing(2);
    focus_near_btn_ = new QPushButton("近焦");
    focus_far_btn_  = new QPushButton("远焦");
    for (auto* b : {focus_near_btn_, focus_far_btn_}) b->setMaximumHeight(26);
    focus_btn_layout->addWidget(focus_near_btn_);
    focus_btn_layout->addWidget(focus_far_btn_);
    ptz_layout->addLayout(focus_btn_layout, 1, 3, 1, 2);

    // 自动聚焦按钮
    auto_focus_btn_ = new QPushButton("自动聚焦");
    auto_focus_btn_->setMaximumHeight(26);
    auto_focus_btn_->setStyleSheet(
        "QPushButton { background-color: #2e7d32; color: white; font-weight: bold; }"
        "QPushButton:hover { background-color: #388e3c; }"
        "QPushButton:disabled { background-color: #555; }");
    ptz_layout->addWidget(auto_focus_btn_, 2, 3, 1, 2);

    // 聚焦位置滑块 (紧凑)
    auto* focus_pos_layout = new QHBoxLayout();
    focus_pos_layout->setSpacing(2);
    focus_pos_slider_ = new QSlider(Qt::Horizontal);
    focus_pos_slider_->setRange(0x1000, 0xC000);
    focus_pos_slider_->setValue(0x6000);
    focus_pos_slider_->setToolTip("手动聚焦位置 (0x1000~0xC000)");
    focus_pos_slider_->setMaximumHeight(22);
    focus_pos_label_ = new QLabel("0x6000");
    focus_pos_label_->setMinimumWidth(48);
    focus_pos_label_->setMaximumHeight(22);
    manual_focus_set_btn_ = new QPushButton("应用");
    manual_focus_set_btn_->setMaximumHeight(22);
    manual_focus_set_btn_->setMaximumWidth(50);
    focus_pos_layout->addWidget(focus_pos_slider_);
    focus_pos_layout->addWidget(focus_pos_label_);
    focus_pos_layout->addWidget(manual_focus_set_btn_);
    ptz_layout->addLayout(focus_pos_layout, 3, 0, 1, 5);

    // 预置点 (紧凑单行)
    auto* preset_layout = new QHBoxLayout();
    preset_layout->setSpacing(2);
    preset_spin_      = new QSpinBox();
    preset_set_btn_   = new QPushButton("设");
    preset_goto_btn_  = new QPushButton("调");
    preset_clear_btn_ = new QPushButton("清");
    preset_clear_all_btn_ = new QPushButton("清全部");
    preset_spin_->setRange(1, 256);
    preset_spin_->setValue(1);
    preset_spin_->setMaximumHeight(24);
    preset_spin_->setMaximumWidth(55);
    for (auto* b : {preset_set_btn_, preset_goto_btn_, preset_clear_btn_})
        { b->setMaximumHeight(24); b->setMaximumWidth(35); }
    preset_clear_all_btn_->setMaximumHeight(24);
    preset_clear_all_btn_->setStyleSheet(
        "QPushButton { background-color: #b71c1c; color: white; font-size: 10px; }"
        "QPushButton:hover { background-color: #c62828; }");
    preset_layout->addWidget(new QLabel("预置点:"));
    preset_layout->addWidget(preset_spin_);
    preset_layout->addWidget(preset_set_btn_);
    preset_layout->addWidget(preset_goto_btn_);
    preset_layout->addWidget(preset_clear_btn_);
    preset_layout->addWidget(preset_clear_all_btn_);
    preset_layout->addStretch();
    ptz_layout->addLayout(preset_layout, 4, 0, 1, 5);

    ptz_group_->setEnabled(false);
    right_layout->addWidget(ptz_group_);

    // ---- 4. 巡航 (紧凑) ----
    cruise_group_ = new QGroupBox("预置点巡航");
    auto* cruise_layout = new QHBoxLayout(cruise_group_);
    cruise_layout->setSpacing(3);

    cruise_layout->addWidget(new QLabel("序列:"));
    cruise_presets_edit_ = new QLineEdit("1");
    cruise_presets_edit_->setToolTip("逗号分隔, 例如: 1,3,5,7");
    cruise_presets_edit_->setMaximumHeight(26);
    cruise_presets_edit_->setMaximumWidth(80);
    cruise_layout->addWidget(cruise_presets_edit_);
    cruise_layout->addWidget(new QLabel("驻留:"));
    cruise_dwell_spin_ = new QSpinBox();
    cruise_dwell_spin_->setRange(1, 60);
    cruise_dwell_spin_->setValue(5);
    cruise_dwell_spin_->setSuffix("秒");
    cruise_dwell_spin_->setMaximumHeight(26);
    cruise_dwell_spin_->setMaximumWidth(65);
    cruise_layout->addWidget(cruise_dwell_spin_);
    cruise_start_btn_ = new QPushButton("▶");
    cruise_stop_btn_  = new QPushButton("■");
    cruise_stop_btn_->setEnabled(false);
    cruise_start_btn_->setMaximumHeight(26); cruise_start_btn_->setMaximumWidth(36);
    cruise_stop_btn_->setMaximumHeight(26);  cruise_stop_btn_->setMaximumWidth(36);
    cruise_layout->addWidget(cruise_start_btn_);
    cruise_layout->addWidget(cruise_stop_btn_);
    cruise_layout->addStretch();

    cruise_group_->setEnabled(false);
    right_layout->addWidget(cruise_group_);

    // ---- 4b. 硬件巡航 (设备端执行) ----
    hw_cruise_group_ = new QGroupBox("硬件巡航 (设备端)");
    auto* hw_cruise_layout = new QGridLayout(hw_cruise_group_);
    hw_cruise_layout->setSpacing(3);
    hw_cruise_layout->setContentsMargins(4, 4, 4, 4);

    // 第一行: 路线号 + 预置点序列
    hw_cruise_layout->addWidget(new QLabel("路线:"), 0, 0);
    hw_cruise_route_spin_ = new QSpinBox();
    hw_cruise_route_spin_->setRange(1, 8);
    hw_cruise_route_spin_->setValue(1);
    hw_cruise_route_spin_->setMaximumHeight(26);
    hw_cruise_route_spin_->setMaximumWidth(50);
    hw_cruise_layout->addWidget(hw_cruise_route_spin_, 0, 1);

    hw_cruise_layout->addWidget(new QLabel("序列:"), 0, 2);
    hw_cruise_presets_edit_ = new QLineEdit("1,2,3");
    hw_cruise_presets_edit_->setToolTip("逗号分隔, 例如: 1,3,5,7");
    hw_cruise_presets_edit_->setMaximumHeight(26);
    hw_cruise_layout->addWidget(hw_cruise_presets_edit_, 0, 3, 1, 3);

    // 第二行: 驻留 + 速度
    hw_cruise_layout->addWidget(new QLabel("驻留:"), 1, 0);
    hw_cruise_dwell_spin_ = new QSpinBox();
    hw_cruise_dwell_spin_->setRange(1, 255);
    hw_cruise_dwell_spin_->setValue(5);
    hw_cruise_dwell_spin_->setSuffix("秒");
    hw_cruise_dwell_spin_->setMaximumHeight(26);
    hw_cruise_dwell_spin_->setMaximumWidth(65);
    hw_cruise_layout->addWidget(hw_cruise_dwell_spin_, 1, 1);

    hw_cruise_layout->addWidget(new QLabel("速度:"), 1, 2);
    hw_cruise_speed_spin_ = new QSpinBox();
    hw_cruise_speed_spin_->setRange(1, 7);
    hw_cruise_speed_spin_->setValue(4);
    hw_cruise_speed_spin_->setMaximumHeight(26);
    hw_cruise_speed_spin_->setMaximumWidth(50);
    hw_cruise_layout->addWidget(hw_cruise_speed_spin_, 1, 3);

    // 第三行: 按钮
    hw_cruise_save_btn_  = new QPushButton("保存路线");
    hw_cruise_start_btn_ = new QPushButton("▶启动");
    hw_cruise_stop_btn_  = new QPushButton("■停止");
    hw_cruise_clear_btn_ = new QPushButton("清除路线");
    hw_cruise_query_btn_ = new QPushButton("查看路线");
    for (auto* b : {hw_cruise_save_btn_, hw_cruise_start_btn_,
                    hw_cruise_stop_btn_, hw_cruise_clear_btn_,
                    hw_cruise_query_btn_})
        b->setMaximumHeight(26);
    hw_cruise_save_btn_->setStyleSheet(
        "QPushButton { background-color: #1565c0; color: white; font-weight: bold; }"
        "QPushButton:hover { background-color: #1976d2; }");
    hw_cruise_start_btn_->setStyleSheet(
        "QPushButton { background-color: #2e7d32; color: white; font-weight: bold; }"
        "QPushButton:hover { background-color: #388e3c; }");
    hw_cruise_stop_btn_->setStyleSheet(
        "QPushButton { background-color: #c62828; color: white; font-weight: bold; }"
        "QPushButton:hover { background-color: #d32f2f; }");

    hw_cruise_layout->addWidget(hw_cruise_save_btn_,  2, 0, 1, 2);
    hw_cruise_layout->addWidget(hw_cruise_start_btn_, 2, 2);
    hw_cruise_layout->addWidget(hw_cruise_stop_btn_,  2, 3);
    hw_cruise_layout->addWidget(hw_cruise_clear_btn_, 2, 4);
    hw_cruise_layout->addWidget(hw_cruise_query_btn_, 2, 5);

    hw_cruise_group_->setEnabled(false);
    right_layout->addWidget(hw_cruise_group_);

    // ---- 5. OCR (紧凑) ----
    ocr_group_ = new QGroupBox("OCR 文字识别");
    auto* ocr_layout = new QVBoxLayout(ocr_group_);
    ocr_layout->setSpacing(3);

    auto* ocr_btn_row = new QHBoxLayout();
    ocr_btn_ = new QPushButton("识别当前画面");
    ocr_btn_->setMaximumHeight(28);
    ocr_btn_->setStyleSheet(
        "QPushButton { background-color: #1a6fb5; color: white; font-weight: bold; }"
        "QPushButton:hover { background-color: #2088d0; }"
        "QPushButton:disabled { background-color: #555; }");
    ocr_status_label_ = new QLabel("就绪");
    ocr_status_label_->setStyleSheet("QLabel { color: #888; }");
    ocr_btn_row->addWidget(ocr_btn_);
    ocr_btn_row->addWidget(ocr_status_label_);
    ocr_layout->addLayout(ocr_btn_row);

    ocr_result_edit_ = new QTextEdit();
    ocr_result_edit_->setReadOnly(true);
    ocr_result_edit_->setMaximumHeight(80);
    ocr_result_edit_->setPlaceholderText("识别结果...");
    ocr_result_edit_->setFont(QFont("monospace", 9));
    ocr_layout->addWidget(ocr_result_edit_);

    ocr_group_->setEnabled(false);
    right_layout->addWidget(ocr_group_);

    // ---- 6. 日志 (可折叠高度) ----
    auto* log_group = new QGroupBox("日志");
    auto* log_layout = new QVBoxLayout(log_group);
    log_layout->setContentsMargins(2, 2, 2, 2);
    log_edit_ = new QTextEdit();
    log_edit_->setReadOnly(true);
    log_edit_->setMaximumHeight(100);
    log_edit_->setFont(QFont("monospace", 8));
    log_layout->addWidget(log_edit_);

    right_layout->addWidget(log_group);

    // 右侧放入滚动区域
    scroll->setWidget(right_widget);
    main_layout->addWidget(scroll);
}

// ============================================================
// 信号槽连接
// ============================================================

void MainWindow::setupConnections()
{
    // 按钮
    connect(login_btn_,  &QPushButton::clicked, this, &MainWindow::onLogin);
    connect(logout_btn_, &QPushButton::clicked, this, &MainWindow::onLogout);
    connect(start_stream_btn_, &QPushButton::clicked,
            this, &MainWindow::onStartStream);
    connect(stop_stream_btn_, &QPushButton::clicked,
            this, &MainWindow::onStopStream);
    connect(snapshot_btn_, &QPushButton::clicked,
            this, &MainWindow::onToggleSnapshot);
    connect(capture_btn_, &QPushButton::clicked,
            this, &MainWindow::onCapture);

    // RTSP 解码器 → 显示
    connect(decoder_, &HKDecoder::frameDecoded,
            this, &MainWindow::onDecodedImage);

    // ---- 云台方向 (按下=动, 松开=停) ----
    connect(ptz_up_btn_,    &QPushButton::pressed,  this, &MainWindow::onPtzUpPressed);
    connect(ptz_up_btn_,    &QPushButton::released, this, &MainWindow::onPtzStop);
    connect(ptz_down_btn_,  &QPushButton::pressed,  this, &MainWindow::onPtzDownPressed);
    connect(ptz_down_btn_,  &QPushButton::released, this, &MainWindow::onPtzStop);
    connect(ptz_left_btn_,  &QPushButton::pressed,  this, &MainWindow::onPtzLeftPressed);
    connect(ptz_left_btn_,  &QPushButton::released, this, &MainWindow::onPtzStop);
    connect(ptz_right_btn_, &QPushButton::pressed,  this, &MainWindow::onPtzRightPressed);
    connect(ptz_right_btn_, &QPushButton::released, this, &MainWindow::onPtzStop);
    // 变倍
    connect(zoom_in_btn_,   &QPushButton::pressed,  this, &MainWindow::onZoomInPressed);
    connect(zoom_in_btn_,   &QPushButton::released, this, &MainWindow::onZoomStop);
    connect(zoom_out_btn_,  &QPushButton::pressed,  this, &MainWindow::onZoomOutPressed);
    connect(zoom_out_btn_,  &QPushButton::released, this, &MainWindow::onZoomStop);
    // 预置点
    connect(preset_set_btn_,   &QPushButton::clicked, this, &MainWindow::onPtzSetPreset);
    connect(preset_goto_btn_,  &QPushButton::clicked, this, &MainWindow::onPtzGotoPreset);
    connect(preset_clear_btn_, &QPushButton::clicked, this, &MainWindow::onPtzClearPreset);
    connect(preset_clear_all_btn_, &QPushButton::clicked, this, &MainWindow::onClearAllPresets);
    // 聚焦
    connect(focus_near_btn_,    &QPushButton::pressed,  this, &MainWindow::onFocusNearPressed);
    connect(focus_near_btn_,    &QPushButton::released, this, &MainWindow::onFocusStop);
    connect(focus_far_btn_,     &QPushButton::pressed,  this, &MainWindow::onFocusFarPressed);
    connect(focus_far_btn_,     &QPushButton::released, this, &MainWindow::onFocusStop);
    connect(auto_focus_btn_,    &QPushButton::clicked,  this, &MainWindow::onAutoFocus);
    connect(manual_focus_set_btn_, &QPushButton::clicked, this, &MainWindow::onSetManualFocus);
    connect(focus_pos_slider_,  &QSlider::valueChanged, this, &MainWindow::onFocusPosChanged);
    // 巡航
    connect(cruise_start_btn_, &QPushButton::clicked, this, &MainWindow::onCruiseStart);
    connect(cruise_stop_btn_,  &QPushButton::clicked, this, &MainWindow::onCruiseStop);
    // 硬件巡航
    connect(hw_cruise_save_btn_,  &QPushButton::clicked, this, &MainWindow::onHwCruiseSave);
    connect(hw_cruise_start_btn_, &QPushButton::clicked, this, &MainWindow::onHwCruiseStart);
    connect(hw_cruise_stop_btn_,  &QPushButton::clicked, this, &MainWindow::onHwCruiseStop);
    connect(hw_cruise_clear_btn_, &QPushButton::clicked, this, &MainWindow::onHwCruiseClear);
    connect(hw_cruise_query_btn_, &QPushButton::clicked, this, &MainWindow::onHwCruiseQuery);

    // OCR
    connect(ocr_btn_, &QPushButton::clicked, this, &MainWindow::onOcrRecognize);

    // 报警 + 错误
    connect(camera_, &HKCamera::alarmReceived,
            this, &MainWindow::onAlarm);
    connect(camera_, &HKCamera::errorOccurred,
            this, &MainWindow::onError);

    // 定时器刷新状态
    status_timer_ = new QTimer(this);
    connect(status_timer_, &QTimer::timeout, this, &MainWindow::updateStatus);
    status_timer_->start(1000);
}

// ============================================================
// 槽函数实现
// ============================================================

void MainWindow::onLogin()
{
    camera_->init();

    CameraLoginInfo info;
    info.ip       = ip_edit_->text().toStdString();
    info.port     = static_cast<uint16_t>(port_edit_->text().toInt());
    info.username = user_edit_->text().toStdString();
    info.password = pass_edit_->text().toStdString();

    log(QString("正在连接 %1:%2 ...")
        .arg(info.ip.c_str()).arg(info.port));

    user_id_ = camera_->login(info);

    if (user_id_ >= 0)
    {
        log(QString("✓ 登录成功, user_id=%1").arg(user_id_));
        setControlsEnabled(true);
    }
    else
    {
        log(QString("✗ 登录失败, 错误码=%1").arg(camera_->getLastError()));
    }
}

void MainWindow::onLogout()
{
    // 停掉所有取流模式
    decoder_->stop();
    if (snapshot_mode_)
    {
        snapshot_mode_ = false;
        snapshot_btn_->setChecked(false);
        snapshot_btn_->setText("快照模式 (1Hz 低CPU)");
        if (snapshot_timer_) { snapshot_timer_->stop(); delete snapshot_timer_; snapshot_timer_ = nullptr; }
    }
    if (cruise_running_) onCruiseStop();
    if (user_id_ >= 0)     { camera_->logout(user_id_);           user_id_     = -1; }

    setControlsEnabled(false);
    login_btn_->setEnabled(true);
    log("已登出");
}

void MainWindow::onStartStream()
{
    if (user_id_ < 0) return;

    // 如果快照模式开着, 先关掉
    if (snapshot_mode_)
    {
        snapshot_mode_ = false;
        snapshot_btn_->setChecked(false);
        snapshot_btn_->setText("快照模式 (1Hz 低CPU)");
        if (snapshot_timer_) { snapshot_timer_->stop(); delete snapshot_timer_; snapshot_timer_ = nullptr; }
    }

    int channel = channel_combo_->currentData().toInt();
    log(QString("开始取流 通道%1 (RTSP)...").arg(channel));

    CameraLoginInfo info;
    info.ip       = ip_edit_->text().toStdString();
    info.port     = static_cast<uint16_t>(port_edit_->text().toInt());
    info.username = user_edit_->text().toStdString();
    info.password = pass_edit_->text().toStdString();

    if (decoder_->start(info, channel))
    {
        log(QString("✓ RTSP 取流已启动"));
        start_stream_btn_->setEnabled(false);
        stop_stream_btn_->setEnabled(true);
    }
    else
    {
        log(QString("✗ RTSP 取流启动失败"));
    }
}

void MainWindow::onStopStream()
{
    decoder_->stop();

    log("取流已停止");
    start_stream_btn_->setEnabled(true);
    stop_stream_btn_->setEnabled(false);
}

void MainWindow::onCapture()
{
    if (user_id_ < 0) return;

    int  channel = channel_combo_->currentData().toInt();
    auto path    = QString("/home/nvidia/kybot_ws/src/hk_camera/pic_capture/%1.jpg")
                   .arg(QDateTime::currentSecsSinceEpoch());

    bool ok = camera_->captureJPEG(user_id_, channel, 0, path.toStdString());
    if (ok)
        log(QString("✓ 抓图成功: %1").arg(path));
    else
        log(QString("✗ 抓图失败, 错误码=%1").arg(camera_->getLastError()));
}

void MainWindow::onToggleSnapshot()
{
    if (snapshot_mode_)
    {
        // 关闭快照模式
        snapshot_mode_ = false;
        snapshot_btn_->setChecked(false);
        snapshot_btn_->setText("快照模式 (1Hz 低CPU)");
        if (snapshot_timer_)
        {
            snapshot_timer_->stop();
            delete snapshot_timer_;
            snapshot_timer_ = nullptr;
        }
        log("快照模式已关闭");
    }
    else
    {
        if (user_id_ < 0)
        {
            log("请先登录");
            snapshot_btn_->setChecked(false);
            return;
        }

        // 如果正在取流, 先停掉
        if (decoder_->isRunning())
        {
            decoder_->stop();
            start_stream_btn_->setEnabled(true);
            stop_stream_btn_->setEnabled(false);
        }

        snapshot_mode_ = true;
        snapshot_btn_->setText("快照模式 (运行中...)");
        log("快照模式已开启 (1Hz, 相机端编码 JPEG, CPU 低负载)");

        // 立即抓一张
        onSnapshotTick();

        // 启动 1Hz 定时器
        snapshot_timer_ = new QTimer(this);
        connect(snapshot_timer_, &QTimer::timeout, this, &MainWindow::onSnapshotTick);
        snapshot_timer_->start(1000);
    }
}

void MainWindow::onSnapshotTick()
{
    if (!snapshot_mode_ || user_id_ < 0) return;

    int  channel = channel_combo_->currentData().toInt();
    // 用固定临时路径, 避免磁盘堆积
    auto path = QString("/tmp/hk_snapshot.jpg");

    if (camera_->captureJPEG(user_id_, channel, 0, path.toStdString()))
    {
        QPixmap pix(path);
        if (!pix.isNull())
        {
            video_label_->setPixmap(
                pix.scaled(video_label_->size(),
                           Qt::KeepAspectRatio,
                           Qt::FastTransformation));
        }
    }
}

// ============================================================
// 云台控制槽
// ============================================================

void MainWindow::ptzDo(DWORD cmd)
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    camera_->ptzControl(user_id_, ch, cmd, 0);  // 0=start
}

void MainWindow::onPtzUpPressed()    { ptzDo(TILT_UP); }
void MainWindow::onPtzDownPressed()  { ptzDo(TILT_DOWN); }
void MainWindow::onPtzLeftPressed()  { ptzDo(PAN_LEFT); }
void MainWindow::onPtzRightPressed() { ptzDo(PAN_RIGHT); }
void MainWindow::onZoomInPressed()   { ptzDo(ZOOM_IN); }
void MainWindow::onZoomOutPressed()  { ptzDo(ZOOM_OUT); }

void MainWindow::onPtzStop()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    // 停所有方向 + 变倍 (0=stop, 但对全部方向发 stop 比较暴力但安全)
    camera_->ptzControl(user_id_, ch, TILT_UP, 1);
    camera_->ptzControl(user_id_, ch, PAN_LEFT, 1);
    camera_->ptzControl(user_id_, ch, ZOOM_IN, 1);
}

void MainWindow::onZoomStop() { onPtzStop(); }

// ============================================================
// 聚焦控制槽
// ============================================================

void MainWindow::onFocusNearPressed()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    camera_->ptzControl(user_id_, ch, FOCUS_NEAR, 0);
    log("手动调焦: 近焦 ←");
}

void MainWindow::onFocusFarPressed()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    camera_->ptzControl(user_id_, ch, FOCUS_FAR, 0);
    log("手动调焦: 远焦 →");
}

void MainWindow::onFocusStop()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    camera_->ptzControl(user_id_, ch, FOCUS_NEAR, 1);
    camera_->ptzControl(user_id_, ch, FOCUS_FAR,  1);
}

void MainWindow::onAutoFocus()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    log("触发自动聚焦...");
    if (camera_->setAutoFocusMode(user_id_, ch, 0))
        log("✓ 自动聚焦模式已设置");
    else
        log(QString("✗ 自动聚焦失败, 错误码=%1").arg(camera_->getLastError()));
}

void MainWindow::onSetManualFocus()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    DWORD pos = static_cast<DWORD>(focus_pos_slider_->value());
    log(QString("设置手动聚焦位置: 0x%1").arg(pos, 1, 16));
    if (camera_->setManualFocus(user_id_, ch, pos))
        log(QString("✓ 手动聚焦已设置 (位置=0x%1)").arg(pos, 1, 16));
    else
        log(QString("✗ 手动聚焦失败, 错误码=%1").arg(camera_->getLastError()));
}

void MainWindow::onFocusPosChanged(int value)
{
    focus_pos_label_->setText(QString("0x%1").arg(value, 1, 16));
}

void MainWindow::onPtzSetPreset()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    int idx = preset_spin_->value();
    camera_->ptzPreset(user_id_, ch, SET_PRESET, idx);
    log(QString("设置预置点 %1").arg(idx));
}

void MainWindow::onPtzGotoPreset()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    int idx = preset_spin_->value();
    camera_->ptzPreset(user_id_, ch, GOTO_PRESET, idx);
    log(QString("转到预置点 %1").arg(idx));
}

void MainWindow::onPtzClearPreset()
{
    if (user_id_ < 0) return;
    int ch = channel_combo_->currentData().toInt();
    int idx = preset_spin_->value();
    camera_->ptzPreset(user_id_, ch, CLE_PRESET, idx);
    log(QString("清除预置点 %1").arg(idx));
}

// ============================================================
// 巡航控制
// ============================================================

void MainWindow::onCruiseStart()
{
    if (user_id_ < 0 || cruise_running_) return;

    // 解析预置点序列
    QString text = cruise_presets_edit_->text();
    QStringList parts = text.split(",", Qt::SkipEmptyParts);
    std::vector<int> presets;
    for (const auto& p : parts)
    {
        bool ok;
        int v = p.trimmed().toInt(&ok);
        if (ok && v >= 1 && v <= 256)
            presets.push_back(v);
    }
    if (presets.empty())
    {
        log("无效的预置点序列");
        return;
    }

    cruise_running_ = true;
    cruise_index_   = 0;
    cruise_start_btn_->setEnabled(false);
    cruise_stop_btn_->setEnabled(true);
    cruise_presets_edit_->setEnabled(false);
    cruise_dwell_spin_->setEnabled(false);

    log(QString("巡航开始, %1个预置点, 驻留%2秒")
        .arg(presets.size()).arg(cruise_dwell_spin_->value()));

    // 立即转到第一个点
    int ch = channel_combo_->currentData().toInt();
    camera_->ptzPreset(user_id_, ch, GOTO_PRESET, presets[0]);
    log(QString("  → 预置点 %1").arg(presets[0]));

    // 启动定时器
    cruise_timer_ = new QTimer(this);
    connect(cruise_timer_, &QTimer::timeout, this, [this]() {
        onCruiseTick();
    });
    cruise_timer_->start(cruise_dwell_spin_->value() * 1000);
}

void MainWindow::onCruiseTick()
{
    if (!cruise_running_ || user_id_ < 0) return;

    // 解析当前序列
    QString text = cruise_presets_edit_->text();
    QStringList parts = text.split(",", Qt::SkipEmptyParts);
    std::vector<int> presets;
    for (const auto& p : parts)
    {
        bool ok;
        int v = p.trimmed().toInt(&ok);
        if (ok && v >= 1 && v <= 256)
            presets.push_back(v);
    }
    if (presets.empty()) { onCruiseStop(); return; }

    // 下一个预置点 (循环)
    cruise_index_ = (cruise_index_ + 1) % presets.size();
    int ch = channel_combo_->currentData().toInt();
    camera_->ptzPreset(user_id_, ch, GOTO_PRESET, presets[cruise_index_]);
    log(QString("  → 预置点 %1").arg(presets[cruise_index_]));
}

void MainWindow::onCruiseStop()
{
    cruise_running_ = false;
    cruise_index_   = 0;
    if (cruise_timer_) { cruise_timer_->stop(); delete cruise_timer_; cruise_timer_ = nullptr; }
    cruise_start_btn_->setEnabled(true);
    cruise_stop_btn_->setEnabled(false);
    cruise_presets_edit_->setEnabled(true);
    cruise_dwell_spin_->setEnabled(true);
    log("巡航已停止");
}

// ============================================================
// 硬件巡航 (设备端执行)
// ============================================================

void MainWindow::onHwCruiseSave()
{
    if (user_id_ < 0) return;

    int ch    = channel_combo_->currentData().toInt();
    int route = hw_cruise_route_spin_->value();

    // 解析预置点序列
    QString text = hw_cruise_presets_edit_->text();
    QStringList parts = text.split(",", Qt::SkipEmptyParts);
    std::vector<int> presets;
    for (const auto& p : parts)
    {
        bool ok;
        int v = p.trimmed().toInt(&ok);
        if (ok && v >= 1 && v <= 256)
            presets.push_back(v);
    }
    if (presets.empty())
    {
        log("硬件巡航: 无效的预置点序列");
        return;
    }

    int dwell = hw_cruise_dwell_spin_->value();
    int speed = hw_cruise_speed_spin_->value();

    log(QString("硬件巡航: 配置路线%1, %2个预置点, 驻留%3秒, 速度%4")
        .arg(route).arg(presets.size()).arg(dwell).arg(speed));

    if (camera_->ptzCruiseSetRoute(user_id_, ch, route, presets, dwell, speed))
        log(QString("  ✓ 路线%1 配置完成, %2个点").arg(route).arg(presets.size()));
    else
        log(QString("  ⚠ 路线%1 配置部分失败, 请检查终端日志").arg(route));

    // 配置后立即查询验证
    std::vector<int> q_presets, q_dwells, q_speeds;
    if (camera_->ptzCruiseQuery(user_id_, ch, route, q_presets, q_dwells, q_speeds))
    {
        log(QString("  验证: 路线%1 实际有%2个点").arg(route).arg(q_presets.size()));
        for (size_t i = 0; i < q_presets.size(); i++)
            log(QString("    [%1] 预置点=%2 驻留=%3秒 速度=%4")
                .arg(i+1).arg(q_presets[i]).arg(q_dwells[i]).arg(q_speeds[i]));
    }
    else
    {
        log(QString("  验证: 查询路线%1 失败, 错误码=%2").arg(route).arg(camera_->getLastError()));
    }
}

void MainWindow::onHwCruiseStart()
{
    if (user_id_ < 0) return;

    int ch    = channel_combo_->currentData().toInt();
    int route = hw_cruise_route_spin_->value();

    if (camera_->ptzCruiseStart(user_id_, ch, route))
        log(QString("✓ 硬件巡航启动 (路线%1)").arg(route));
    else
        log(QString("✗ 硬件巡航启动失败, 错误码=%1").arg(camera_->getLastError()));
}

void MainWindow::onHwCruiseStop()
{
    if (user_id_ < 0) return;

    int ch    = channel_combo_->currentData().toInt();
    int route = hw_cruise_route_spin_->value();

    if (camera_->ptzCruiseStop(user_id_, ch, route))
        log(QString("✓ 硬件巡航停止 (路线%1)").arg(route));
    else
        log(QString("✗ 硬件巡航停止失败, 错误码=%1").arg(camera_->getLastError()));
}

void MainWindow::onHwCruiseClear()
{
    if (user_id_ < 0) return;

    int ch    = channel_combo_->currentData().toInt();
    int route = hw_cruise_route_spin_->value();

    if (camera_->ptzCruiseDeleteRoute(user_id_, ch, route))
        log(QString("✓ 硬件巡航路线%1 已清除").arg(route));
    else
        log(QString("✗ 清除路线失败, 错误码=%1").arg(camera_->getLastError()));
}

void MainWindow::onHwCruiseQuery()
{
    if (user_id_ < 0) return;

    int ch    = channel_combo_->currentData().toInt();
    int route = hw_cruise_route_spin_->value();

    std::vector<int> presets, dwells, speeds;
    if (!camera_->ptzCruiseQuery(user_id_, ch, route, presets, dwells, speeds))
    {
        log(QString("✗ 查询路线%1 失败, 错误码=%2").arg(route).arg(camera_->getLastError()));
        return;
    }

    if (presets.empty())
    {
        log(QString("路线%1: 空 (无预置点)").arg(route));
        return;
    }

    log(QString("路线%1: %2个预置点").arg(route).arg(presets.size()));
    for (size_t i = 0; i < presets.size(); i++)
    {
        log(QString("  [%1] 预置点=%2  驻留=%3秒  速度=%4")
            .arg(i + 1).arg(presets[i]).arg(dwells[i]).arg(speeds[i]));
    }
}

void MainWindow::onClearAllPresets()
{
    if (user_id_ < 0) return;

    int ch = channel_combo_->currentData().toInt();

    auto btn = QMessageBox::question(this, "确认",
        "确定要清除所有预置点吗？\n此操作不可撤销。",
        QMessageBox::Yes | QMessageBox::No);
    if (btn != QMessageBox::Yes) return;

    if (camera_->ptzClearAllPresets(user_id_, ch))
        log("✓ 所有预置点已清除");
    else
        log(QString("✗ 清除全部预置点失败, 错误码=%1").arg(camera_->getLastError()));
}

void MainWindow::onDecodedImage(const QImage& image)
{
    if (image.isNull()) return;

    // 快速缩放显示: FastTransformation 用最近邻插值 (CPU 开销远低于 SmoothTransformation)
    video_label_->setPixmap(
        QPixmap::fromImage(image).scaled(
            video_label_->size(), Qt::KeepAspectRatio, Qt::FastTransformation));

    // ROS 发布: 每 5 帧发一次, 避免每帧都做 format 转换 + 深拷贝
    static int ros_skip = 0;
    ros_skip++;
    if (image_pub_ && (ros_skip % 5 == 0))
    {
        QImage rgb = image.convertToFormat(QImage::Format_RGB888);
        cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                    const_cast<uint8_t*>(rgb.bits()),
                    static_cast<size_t>(rgb.bytesPerLine()));
        auto header = std_msgs::msg::Header();
        header.stamp = ros_node_->now();
        header.frame_id = "hk_camera";
        auto img_msg = cv_bridge::CvImage(header, "rgb8", mat).toImageMsg();
        image_pub_->publish(*img_msg);
    }
}

void MainWindow::onAlarm(const AlarmEvent& alarm)
{
    log(QString("[告警] type=%1, channels detected")
        .arg(alarm.alarm_type));
}

void MainWindow::onError(int code, const QString& msg)
{
    log(QString("[错误 %1] %2").arg(code).arg(msg));
}

void MainWindow::updateStatus()
{
    if (user_id_ >= 0 && decoder_->isRunning())
    {
        status_label_->setText("● 取流中");
        status_label_->setStyleSheet("QLabel { color: #00cc66; font-weight: bold; padding: 2px; }");
    }
    else if (user_id_ >= 0)
    {
        status_label_->setText("● 已连接");
        status_label_->setStyleSheet("QLabel { color: #ffaa00; font-weight: bold; padding: 2px; }");
    }
    else
    {
        status_label_->setText("● 未连接");
        status_label_->setStyleSheet("QLabel { color: #ff6600; font-weight: bold; padding: 2px; }");
    }
}

// ============================================================
// 辅助
// ============================================================

void MainWindow::log(const QString& msg)
{
    QString ts = QDateTime::currentDateTime().toString("hh:mm:ss");
    log_edit_->append(QString("[%1] %2").arg(ts, msg));
}

void MainWindow::setControlsEnabled(bool enabled)
{
    login_btn_->setEnabled(!enabled);
    logout_btn_->setEnabled(enabled);
    start_stream_btn_->setEnabled(enabled);
    stop_stream_btn_->setEnabled(false);
    snapshot_btn_->setEnabled(enabled);
    capture_btn_->setEnabled(enabled);
    ptz_group_->setEnabled(enabled);
    cruise_group_->setEnabled(enabled);
    hw_cruise_group_->setEnabled(enabled);
    ocr_group_->setEnabled(enabled);
}

// ============================================================
// OCR 识别
// ============================================================

void MainWindow::setRosNode(rclcpp::Node::SharedPtr node)
{
    ros_node_ = node;
    if (ros_node_)
    {
        ocr_client_ = ros_node_->create_client<ocr_interfaces::srv::RecognizeText>(
            "/ocr/recognize");
        image_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Image>(
            "/hk_camera/image_raw", 10);
        log("ROS bridge ready (/ocr/recognize + /hk_camera/image_raw)");
    }
}

void MainWindow::onOcrRecognize()
{
    if (!ocr_client_)
    {
        ocr_status_label_->setText("⚠ ROS service client 未初始化");
        ocr_status_label_->setStyleSheet("QLabel { color: #ff6600; }");
        return;
    }

    if (!ocr_client_->wait_for_service(std::chrono::seconds(1)))
    {
        ocr_status_label_->setText("⚠ OCR 服务不可用，请先启动 ocr_node");
        ocr_status_label_->setStyleSheet("QLabel { color: #ff6600; }");
        log("OCR 服务 /ocr/recognize 不可用，请执行: ros2 launch ocr_node ocr_node.launch.py");
        return;
    }

    ocr_btn_->setEnabled(false);
    ocr_status_label_->setText("⏳ 正在识别...");
    ocr_status_label_->setStyleSheet("QLabel { color: #ffaa00; }");
    ocr_result_edit_->clear();

    auto request = std::make_shared<ocr_interfaces::srv::RecognizeText::Request>();
    request->conf_threshold = 0.0f;  // 使用 ocr_node 默认阈值

    // 异步调用，不阻塞 GUI
    auto future = ocr_client_->async_send_request(
        request,
        [this](rclcpp::Client<ocr_interfaces::srv::RecognizeText>::SharedFuture future)
        {
            // 此回调在 ROS spin 线程中执行，需要用 invokeMethod 切回 Qt 主线程更新 UI
            auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]()
                {
                    ocr_btn_->setEnabled(true);

                    if (!response->success)
                    {
                        ocr_status_label_->setText(
                            QString("✗ 识别失败: %1").arg(QString::fromStdString(response->message)));
                        ocr_status_label_->setStyleSheet("QLabel { color: #ff3300; }");
                        log(QString("OCR 失败: %1").arg(QString::fromStdString(response->message)));
                        return;
                    }

                    double proc_ms = response->processing_time_ms;
                    int count = response->detections.size();

                    ocr_status_label_->setText(
                        QString("✓ 识别完成: %1 条文字, %2 ms")
                            .arg(count)
                            .arg(proc_ms, 0, 'f', 0));
                    ocr_status_label_->setStyleSheet("QLabel { color: #00cc66; font-weight: bold; }");

                    // 格式化结果显示
                    QString result_text;
                    for (size_t i = 0; i < response->detections.size(); i++)
                    {
                        const auto& d = response->detections[i];
                        result_text += QString("[%1] %2  (置信度: %3)\n")
                            .arg(i + 1)
                            .arg(QString::fromStdString(d.text))
                            .arg(d.confidence, 0, 'f', 2);
                    }
                    ocr_result_edit_->setPlainText(result_text);

                    // 日志
                    log(QString("OCR: %1 条文字, %2 ms")
                        .arg(count)
                        .arg(proc_ms, 0, 'f', 0));
                    for (size_t i = 0; i < response->detections.size() && i < 3; i++)
                    {
                        log(QString("  [%1] \"%2\" conf=%3")
                            .arg(i + 1)
                            .arg(QString::fromStdString(response->detections[i].text))
                            .arg(response->detections[i].confidence, 0, 'f', 2));
                    }
                },
                Qt::QueuedConnection);
        });
}

} // namespace hk_camera
