#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QLineEdit>
#include <QComboBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSpinBox>
#include <QSlider>
#include <QTimer>
#include <QScrollArea>
#include <memory>

#include "hk_camera/hk_camera.h"
#include "hk_camera/hk_decoder.h"

// ROS2 OCR service (optional — only when building with ROS)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "ocr_interfaces/srv/recognize_text.hpp"

namespace hk_camera
{

/**
 * @brief Qt 主窗口 —— 海康相机控制界面
 *
 * 功能:
 *   - 设备登录/登出
 *   - 视频预览区域 (QImage 显示)
 *   - 取流/停止
 *   - 抓图
 *   - 报警状态显示
 *   - 日志窗口
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onLogin();
    void onLogout();
    void onStartStream();
    void onStopStream();
    void onToggleSnapshot();
    void onSnapshotTick();
    void onCapture();
    void onDecodedImage(const QImage& image);
    void onAlarm(const AlarmEvent& alarm);
    void onError(int code, const QString& msg);
    void updateStatus();

    // ---- 云台控制 ----
    void onPtzUpPressed();
    void onPtzDownPressed();
    void onPtzLeftPressed();
    void onPtzRightPressed();
    void onPtzStop();
    void onZoomInPressed();
    void onZoomOutPressed();
    void onZoomStop();
    void onPtzSetPreset();
    void onPtzGotoPreset();
    void onPtzClearPreset();
    void ptzDo(DWORD cmd);

    // ---- 巡航 ----
    void onCruiseStart();
    void onCruiseStop();
    void onCruiseTick();

    // ---- 硬件巡航 ----
    void onHwCruiseSave();
    void onHwCruiseStart();
    void onHwCruiseStop();
    void onHwCruiseClear();
    void onHwCruiseQuery();
    void onClearAllPresets();

    // ---- 聚焦控制 ----
    void onFocusNearPressed();
    void onFocusFarPressed();
    void onFocusStop();
    void onAutoFocus();
    void onSetManualFocus();
    void onFocusPosChanged(int value);

    // ---- OCR 识别 ----
    void onOcrRecognize();

public:
    /// 注入 ROS2 节点指针（main.cpp 调用），用于 OCR service client
    void setRosNode(rclcpp::Node::SharedPtr node);

private:
    void setupUI();
    void setupConnections();
    void log(const QString& msg);
    void setControlsEnabled(bool enabled);

    // ---- 控件 ----
    QGroupBox*   login_group_;
    QLineEdit*   ip_edit_;
    QLineEdit*   port_edit_;
    QLineEdit*   user_edit_;
    QLineEdit*   pass_edit_;
    QPushButton* login_btn_;
    QPushButton* logout_btn_;

    QGroupBox*   stream_group_;
    QPushButton* start_stream_btn_;
    QPushButton* stop_stream_btn_;
    QPushButton* snapshot_btn_;
    QPushButton* capture_btn_;
    QComboBox*   channel_combo_;

    QLabel*      video_label_;       // 视频预览区域
    QLabel*      status_label_;      // 设备状态
    QTextEdit*   log_edit_;          // 日志

    // ---- 云台控件 ----
    QGroupBox*   ptz_group_;
    QPushButton* ptz_up_btn_;
    QPushButton* ptz_down_btn_;
    QPushButton* ptz_left_btn_;
    QPushButton* ptz_right_btn_;
    QPushButton* ptz_stop_btn_;
    QPushButton* zoom_in_btn_;
    QPushButton* zoom_out_btn_;
    QSpinBox*    preset_spin_;
    QPushButton* preset_set_btn_;
    QPushButton* preset_goto_btn_;
    QPushButton* preset_clear_btn_;
    QPushButton* preset_clear_all_btn_;

    // ---- 聚焦控件 ----
    QGroupBox*   focus_group_;
    QPushButton* focus_near_btn_;
    QPushButton* focus_far_btn_;
    QPushButton* auto_focus_btn_;
    QPushButton* manual_focus_set_btn_;
    QSlider*     focus_pos_slider_;
    QLabel*      focus_pos_label_;   // 当前聚焦位置

    QTimer*      status_timer_;
    QTimer*      snapshot_timer_ = nullptr;
    bool         snapshot_mode_  = false;

    // ---- 巡航 ----
    QGroupBox*   cruise_group_;
    QLineEdit*   cruise_presets_edit_;  // "1,3,5,7"
    QSpinBox*    cruise_dwell_spin_;    // 驻留秒数
    QPushButton* cruise_start_btn_;
    QPushButton* cruise_stop_btn_;
    QTimer*      cruise_timer_ = nullptr;
    int          cruise_index_ = 0;
    bool         cruise_running_ = false;

    // ---- 硬件巡航 ----
    QGroupBox*   hw_cruise_group_;
    QSpinBox*    hw_cruise_route_spin_;    // 路线号 1~8
    QLineEdit*   hw_cruise_presets_edit_;  // "1,3,5,7"
    QSpinBox*    hw_cruise_dwell_spin_;   // 驻留秒数
    QSpinBox*    hw_cruise_speed_spin_;   // 速度 1~7
    QPushButton* hw_cruise_save_btn_;
    QPushButton* hw_cruise_start_btn_;
    QPushButton* hw_cruise_stop_btn_;
    QPushButton* hw_cruise_clear_btn_;
    QPushButton* hw_cruise_query_btn_;

    // ---- OCR 识别 ----
    QGroupBox*   ocr_group_;
    QPushButton* ocr_btn_;
    QTextEdit*   ocr_result_edit_;      // 识别结果显示
    QLabel*      ocr_status_label_;     // 耗时 / 状态
    QLabel*      ocr_preview_label_;    // 当前用于识别的帧缩略图

    // ---- ROS2 OCR client ----
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Client<ocr_interfaces::srv::RecognizeText>::SharedPtr ocr_client_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // ---- 数据 ----
    HKCamera*    camera_;
    HKDecoder*   decoder_;
    int          user_id_     = -1;
};

} // namespace hk_camera

#endif // MAIN_WINDOW_H
