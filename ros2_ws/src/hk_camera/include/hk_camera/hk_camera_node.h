#ifndef HK_CAMERA_NODE_H
#define HK_CAMERA_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hk_camera/hk_camera.h"
#include "hk_camera/hk_decoder.h"
#include "hk_camera/msg/camera_alarm.hpp"
#include "hk_camera/srv/capture_picture.hpp"
#include "hk_camera/srv/set_ptz_pose.hpp"

namespace hk_camera
{

/**
 * @brief ROS2 节点 —— 海康相机驱动
 *
 * Pub:
 *   - /hk_camera/image_raw    (sensor_msgs/Image)  视频帧
 *   - /hk_camera/alarm         (hk_camera/CameraAlarm) 报警事件
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
 *   - password (string)  : 密码, 默认 a1234567
 *   - channel (int)      : 通道号, 默认 1
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
    void onAlarm(const AlarmEvent& alarm);

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

    // ---- Sub callback ----
    void onCommand(const std_msgs::msg::String::SharedPtr msg);

    // ---- 参数更新回调 ----
    rcl_interfaces::msg::SetParametersResult onParamChange(
        const std::vector<rclcpp::Parameter>& params);

    // ---- 状态 ----
    HKCamera*              camera_  = nullptr;   // Qt 单例 (非拥有)
    HKDecoder*             decoder_ = nullptr;
    CameraLoginInfo        login_info_;
    int                    user_id_     = -1;
    int                    alarm_handle_ = -1;
    int                    channel_     = 1;

    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // ---- 发布器 ----
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr     image_pub_;
    rclcpp::Publisher<msg::CameraAlarm>::SharedPtr            alarm_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr       status_pub_;

    // ---- 订阅器 ----
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr    cmd_sub_;

    // ---- 服务 ----
    rclcpp::Service<srv::CapturePicture>::SharedPtr           capture_srv_;
    rclcpp::Service<srv::SetPTZPose>::SharedPtr               set_ptz_pose_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        login_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        logout_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        start_stream_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        stop_stream_srv_;
};

} // namespace hk_camera

#endif // HK_CAMERA_NODE_H
