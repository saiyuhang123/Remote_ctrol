#ifndef HK_CAMERA_NODE_H
#define HK_CAMERA_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <atomic>
#include <deque>
#include <mutex>

#include "hk_camera/hk_camera.h"
#include "hk_camera/hk_decoder.h"
#include "hk_camera/image_preview.h"
#include "hk_camera/latest_frame.h"
#include "hk_camera/msg/camera_alarm.hpp"
#include "hk_camera/msg/face_capture.hpp"
#include "hk_camera/srv/capture_picture.hpp"
#include "hk_camera/srv/set_ptz_pose.hpp"
#include "hk_camera/srv/get_ptz_pose.hpp"
#include "hk_camera/srv/control_ptz.hpp"
#include "hk_camera/srv/ptz_preset.hpp"

namespace hk_camera
{

/**
 * @brief ROS2 节点 —— 海康相机驱动
 *
 * Pub:
 *   - /hk_camera/image_raw    (sensor_msgs/Image)  原始视频帧，有订阅者时发布
 *   - /hk_camera/image_preview (sensor_msgs/Image)  限分辨率、限帧率预览
 *   - /hk_camera/alarm         (hk_camera/CameraAlarm) 报警事件
 *   - /hk_camera/face_capture  (hk_camera/FaceCapture) 人脸坐标和图片路径
 *   - /hk_camera/face_image    (sensor_msgs/Image) 带框场景图
 *   - /hk_camera/status        (std_msgs/String)    设备状态
 *
 * Sub:
 *   - /hk_camera/cmd           (std_msgs/String)    控制指令
 *
 * Srv:
 *   - /hk_camera/capture       (CapturePicture)     抓图服务
 *   - /hk_camera/login         (Trigger)            登录
 *   - /hk_camera/logout        (Trigger)            登出
 *   - /hk_camera/start_stream  (Trigger)            开始取流
 *   - /hk_camera/stop_stream   (Trigger)            停止取流
 *
 * Param:
 *   - ip (string)        : 相机 IP, 默认 192.168.1.64
 *   - port (int)         : 端口, 默认 8000
 *   - username (string)  : 用户名, 默认 admin
 *   - password (string)  : 密码, 默认 zmdl@603
 *   - channel (int)      : 通道号, 默认 1
 *   - auto_login (bool)  : 启动时登录, 节点默认 false, launch 默认 true
 *   - auto_stream (bool) : 自动登录后取流, 默认 true
 *   - enable_face_capture (bool): 登录后布防接收人脸抓拍, 默认 true
 *   - face_capture_dir (string): 抓拍图片及坐标保存目录
 *   - raw_publish_fps (double): 原图发布上限 0～30，默认 8；0 表示不限帧，重启生效
 *   - preview_max_width (int): 预览最大宽度 160～4096，默认 640
 *   - preview_publish_fps (double): 预览帧率 0.5～30，默认 5
 */
class HKCameraNode : public rclcpp::Node
{
public:
    explicit HKCameraNode(const std::string& name = "hk_camera_node");
    ~HKCameraNode();

private:
    // ---- ROS2 接口 ----
    void setupPublishers();
    void setupSubscribers();
    void setupServices();
    void loadParameters();

    // ---- 与 HKCamera (Qt 层) 桥接 ----
    void connectCameraSignals();
    void onDecodedImage(const QImage& image);
    void publishFrames(bool preview);
    void publishStreamStatus();
    void onAlarm(const AlarmEvent& alarm);
    void onFaceCapture(const FaceCaptureEvent& event);
    void processFaceCaptures();
    bool startAlarmReception();
    void setFaceCapture(bool enabled, std_srvs::srv::SetBool::Response& response);
    void publishFaceStatus();

    // ---- Service callbacks ----
    void onCapturePicture(
        const srv::CapturePicture::Request::SharedPtr req,
        srv::CapturePicture::Response::SharedPtr res);
    void onLogin(
        const std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res);
    void onLogout(
        const std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res);
    void onStartStream(
        const std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res);
    void onStopStream(
        const std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res);
    void onSetPTZPose(
        const srv::SetPTZPose::Request::SharedPtr req,
        srv::SetPTZPose::Response::SharedPtr res);
    void onGetPTZPose(
        const srv::GetPTZPose::Request::SharedPtr req,
        srv::GetPTZPose::Response::SharedPtr res);
    void onPTZPreset(
        const srv::PTZPreset::Request::SharedPtr req,
        srv::PTZPreset::Response::SharedPtr res);
    void onControlPTZ(const srv::ControlPTZ::Request::SharedPtr req,
                      srv::ControlPTZ::Response::SharedPtr res);
    bool stopPTZ();

    // ---- Sub callback ----
    void onCommand(const std_msgs::msg::String::SharedPtr msg);

    // ---- 参数更新回调 ----
    rcl_interfaces::msg::SetParametersResult onParamChange(
        const std::vector<rclcpp::Parameter>& params);

    // ---- 状态 ----
    HKCamera*              camera_  = nullptr;   // Qt 单例 (非拥有)
    HKDecoder*             decoder_ = nullptr;
    CameraLoginInfo        login_info_;
    std::atomic<int>       user_id_{-1};
    int                    alarm_handle_ = -1;
    int                    channel_     = 1;
    bool                   enable_face_capture_ = true;
    std::string            face_capture_dir_;
    size_t                 face_queue_limit_ = 8;
    std::atomic<bool>      accept_faces_{false};
    std::mutex             face_mutex_;
    std::deque<FaceCaptureEvent> face_queue_;
    std::atomic<uint64_t>  face_dropped_{0};
    uint64_t               face_sequence_ = 0;
    uint64_t               face_dropped_total_ = 0, face_saved_total_ = 0;
    std::string            face_last_error_;
    std::array<uint32_t, 2> face_coordinate_size_{};
    rclcpp::TimerBase::SharedPtr face_timer_;
    rclcpp::TimerBase::SharedPtr face_status_timer_;
    rclcpp::TimerBase::SharedPtr ptz_timer_;
    int active_ptz_command_ = 0, active_ptz_speed_ = 0;
    std::chrono::steady_clock::time_point ptz_deadline_;
    std::vector<QMetaObject::Connection> camera_connections_;
    DecoderOptions decoder_options_;
    LatestFrame latest_frame_;
    std::thread raw_publish_thread_, preview_publish_thread_;
    std::atomic<int64_t> last_raw_publish_ms_{0}, last_preview_publish_ms_{0};
    rclcpp::TimerBase::SharedPtr stream_status_timer_;
    std::string last_stream_state_;
    // A separate DDS participant keeps small control replies out of the
    // full-image transport's queues, including capture service responses.
    std::shared_ptr<rclcpp::Context> sdk_context_;
    rclcpp::Node::SharedPtr sdk_node_;
    std::shared_ptr<rclcpp::Context> preview_context_;
    rclcpp::Node::SharedPtr preview_node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> sdk_executor_;
    std::thread sdk_thread_;
    std::atomic<int64_t> last_sdk_query_ms_{0}, last_sdk_lock_wait_ms_{0};

    // Keep SDK calls and login/logout serialized, but release this before
    // converting a capture or returning its large ROS image response.
    std::recursive_mutex sdk_session_mutex_;
    rclcpp::CallbackGroup::SharedPtr capture_group_;

    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // ---- 发布器 ----
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr     image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr     preview_pub_;
    double raw_publish_fps_ = 8.;
    int preview_max_width_ = 640;
    double preview_publish_fps_ = 5.;
    rclcpp::Publisher<msg::CameraAlarm>::SharedPtr            alarm_pub_;
    rclcpp::Publisher<msg::FaceCapture>::SharedPtr            face_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      face_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       stream_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       face_status_pub_;

    // ---- 订阅器 ----
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr    cmd_sub_;

    // ---- 服务 ----
    rclcpp::Service<srv::CapturePicture>::SharedPtr           capture_srv_;
    rclcpp::Service<srv::SetPTZPose>::SharedPtr               set_ptz_pose_srv_;
    rclcpp::Service<srv::GetPTZPose>::SharedPtr               get_ptz_pose_srv_;
    rclcpp::Service<srv::PTZPreset>::SharedPtr                ptz_preset_srv_;
    rclcpp::Service<srv::ControlPTZ>::SharedPtr               control_ptz_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        login_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        logout_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        start_stream_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        stop_stream_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr        face_capture_srv_;
};

} // namespace hk_camera

#endif // HK_CAMERA_NODE_H
