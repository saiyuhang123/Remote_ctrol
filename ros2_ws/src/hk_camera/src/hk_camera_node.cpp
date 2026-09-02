#include "hk_camera/hk_camera_node.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace hk_camera
{

HKCameraNode::HKCameraNode(const std::string& name)
    : Node(name)
{
    camera_  = HKCamera::instance();
    decoder_ = new HKDecoder();  // HKCameraNode 不是 QObject, 不传 parent

    loadParameters();
    setupPublishers();
    setupSubscribers();
    setupServices();
    connectCameraSignals();

    RCLCPP_INFO(get_logger(), "HKCameraNode initialized (SDK %s)",
                camera_->getSDKVersion().c_str());
}

HKCameraNode::~HKCameraNode()
{
    decoder_->stop();
    if (alarm_handle_ >= 0) camera_->closeAlarmChan(alarm_handle_);
    if (user_id_ >= 0)      camera_->logout(user_id_);
    camera_->cleanup();
    delete decoder_;
}

// ============================================================
// 参数加载
// ============================================================

void HKCameraNode::loadParameters()
{
    declare_parameter("ip",       "192.168.1.64");
    declare_parameter("port",     8000);
    declare_parameter("username", "admin");
    declare_parameter("password", "a1234567");
    declare_parameter("channel",  1);

    login_info_.ip       = get_parameter("ip").as_string();
    login_info_.port     = static_cast<uint16_t>(get_parameter("port").as_int());
    login_info_.username = get_parameter("username").as_string();
    login_info_.password = get_parameter("password").as_string();
    channel_             = get_parameter("channel").as_int();

    // 参数变更回调
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            return onParamChange(params);
        });

    RCLCPP_INFO(get_logger(), "Params: ip=%s:%d, channel=%d",
                login_info_.ip.c_str(), login_info_.port, channel_);
}

rcl_interfaces::msg::SetParametersResult HKCameraNode::onParamChange(
    const std::vector<rclcpp::Parameter>& params)
{
    (void)params;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Most params require re-login to take effect";
    // 注意: IP/端口/用户名等变更需要重新登录
    return result;
}

// ============================================================
// Pub/Sub/Service 初始化
// ============================================================

void HKCameraNode::setupPublishers()
{
    image_pub_  = create_publisher<sensor_msgs::msg::Image>(
        "/hk_camera/image_raw", 10);
    alarm_pub_  = create_publisher<msg::CameraAlarm>(
        "/hk_camera/alarm", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/hk_camera/status", 10);
}

void HKCameraNode::setupSubscribers()
{
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "/hk_camera/cmd", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            onCommand(msg);
        });
}

void HKCameraNode::setupServices()
{
    capture_srv_ = create_service<srv::CapturePicture>(
        "/hk_camera/capture",
        [this](const srv::CapturePicture::Request::SharedPtr  req,
               srv::CapturePicture::Response::SharedPtr       res) {
            onCapturePicture(req, res);
        });

    login_srv_ = create_service<std_srvs::srv::Trigger>(
        "/hk_camera/login",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr  req,
               std_srvs::srv::Trigger::Response::SharedPtr       res) {
            onLogin(req, res);
        });

    logout_srv_ = create_service<std_srvs::srv::Trigger>(
        "/hk_camera/logout",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr  req,
               std_srvs::srv::Trigger::Response::SharedPtr       res) {
            onLogout(req, res);
        });

    start_stream_srv_ = create_service<std_srvs::srv::Trigger>(
        "/hk_camera/start_stream",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr  req,
               std_srvs::srv::Trigger::Response::SharedPtr       res) {
            onStartStream(req, res);
        });

    stop_stream_srv_ = create_service<std_srvs::srv::Trigger>(
        "/hk_camera/stop_stream",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr  req,
               std_srvs::srv::Trigger::Response::SharedPtr       res) {
            onStopStream(req, res);
        });

    set_ptz_pose_srv_ = create_service<srv::SetPTZPose>(
        "/hk_camera/set_ptz_pose",
        [this](const srv::SetPTZPose::Request::SharedPtr  req,
               srv::SetPTZPose::Response::SharedPtr       res) {
            onSetPTZPose(req, res);
        });
}

// ============================================================
// HKCamera (Qt) → ROS2 桥接
// ============================================================

void HKCameraNode::connectCameraSignals()
{
    // 解码器 → ROS2 publish
    QObject::connect(decoder_, &HKDecoder::frameDecoded,
        [this](const QImage& image) { onDecodedImage(image); });

    // 报警: SDK回调线程 → ROS2 publish
    QObject::connect(camera_, &HKCamera::alarmReceived,
        [this](const AlarmEvent& alarm) { onAlarm(alarm); });

    // 错误处理
    QObject::connect(camera_, &HKCamera::errorOccurred,
        [this](int code, const QString& msg) {
            RCLCPP_ERROR(get_logger(), "HK SDK error [%d]: %s",
                         code, msg.toStdString().c_str());
        });
}

void HKCameraNode::onDecodedImage(const QImage& image)
{
    if (image.isNull()) return;

    // QImage → cv::Mat → sensor_msgs/Image
    QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                const_cast<uint8_t*>(rgb.bits()),
                static_cast<size_t>(rgb.bytesPerLine()));

    auto header = std_msgs::msg::Header();
    header.stamp = now();
    header.frame_id = "hk_camera";

    auto img_msg = cv_bridge::CvImage(header, "rgb8", mat).toImageMsg();
    image_pub_->publish(*img_msg);

    auto status = std_msgs::msg::String();
    status.data = "streaming_active";
    status_pub_->publish(status);
}

// ---- 报警回调 → 发布 CameraAlarm ----
void HKCameraNode::onAlarm(const AlarmEvent& alarm)
{
    auto msg = msg::CameraAlarm();
    msg.alarm_type = alarm.alarm_type;
    msg.alarm_desc = "alarm";
    msg.stamp      = now();

    for (int i = 0; i < 16; i++)
    {
        if (alarm.channels[i] == 1)
            msg.alarm_channels.push_back(i + 1);
    }

    alarm_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Alarm: type=%d, channels=%ld",
                alarm.alarm_type, msg.alarm_channels.size());
}

// ============================================================
// Service 实现
// ============================================================

void HKCameraNode::onLogin(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    camera_->init();
    user_id_ = camera_->login(login_info_);

    if (user_id_ >= 0)
    {
        res->success = true;
        res->message = "Login OK, user_id=" + std::to_string(user_id_);
        RCLCPP_INFO(get_logger(), "Device login OK, user_id=%d", user_id_);
    }
    else
    {
        res->success = false;
        res->message = "Login failed, error=" +
                       std::to_string(camera_->getLastError());
    }
}

void HKCameraNode::onLogout(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (user_id_ >= 0)
    {
        camera_->logout(user_id_);
        user_id_ = -1;
        res->success = true;
        res->message = "Logout OK";
    }
    else
    {
        res->success = false;
        res->message = "Not logged in";
    }
}

void HKCameraNode::onStartStream(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (user_id_ < 0)
    {
        res->success = false;
        res->message = "Not logged in";
        return;
    }

    if (decoder_->isRunning())
    {
        res->success = false;
        res->message = "Stream already active";
        return;
    }

    if (decoder_->start(login_info_, channel_))
    {
        res->success = true;
        res->message = "RTSP stream started";
    }
    else
    {
        res->success = false;
        res->message = "RTSP start failed";
    }
}

void HKCameraNode::onStopStream(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (decoder_->isRunning())
    {
        decoder_->stop();
        res->success = true;
        res->message = "Stream stopped";
    }
    else
    {
        res->success = false;
        res->message = "No active stream";
    }
}

void HKCameraNode::onCapturePicture(
    const srv::CapturePicture::Request::SharedPtr req,
    srv::CapturePicture::Response::SharedPtr res)
{
    if (user_id_ < 0)
    {
        res->success = false;
        res->message = "Not logged in";
        return;
    }

    std::string path = req->save_path;
    if (path.empty())
    {
        path = "/home/nvidia/kybot_ws/src/hk_camera/pic_capture/" +
               std::to_string(now().nanoseconds()) + ".jpg";
    }

    bool ok = camera_->captureJPEG(user_id_, channel_,
                                    req->quality, path);

    res->success = ok;
    if (ok)
    {
        res->message = "Captured: " + path;
        // 尝试读取并发布图片
        cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
        if (!img.empty())
        {
            auto img_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), "bgr8", img).toImageMsg();
            res->image = *img_msg;
        }
    }
    else
    {
        res->message = "Capture failed, error=" +
                       std::to_string(camera_->getLastError());
    }
}

// ============================================================
// PTZ 姿态控制服务
// ============================================================

void HKCameraNode::onSetPTZPose(
    const srv::SetPTZPose::Request::SharedPtr req,
    srv::SetPTZPose::Response::SharedPtr res)
{
    if (user_id_ < 0)
    {
        res->success = false;
        res->message = "Not logged in";
        return;
    }

    // 海康 SDK 没有直接的绝对位置 API, 使用预置点方案:
    // pan 字段作为预置点编号 (1~256)
    int preset_no = static_cast<int>(req->pan);
    if (preset_no < 1 || preset_no > 256)
    {
        res->success = false;
        res->message = "Invalid preset number (pan field, expect 1~256)";
        return;
    }

    if (camera_->ptzGotoPreset(user_id_, channel_, preset_no))
    {
        res->success = true;
        res->message = "PTZ moving to preset " + std::to_string(preset_no);
        RCLCPP_INFO(get_logger(), "PTZ → preset %d", preset_no);
    }
    else
    {
        res->success = false;
        res->message = "PTZ failed, error=" +
                       std::to_string(camera_->getLastError());
    }
}

// ============================================================
// 指令订阅回调
// ============================================================

void HKCameraNode::onCommand(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received command: %s", msg->data.c_str());
    // 指令通过 service 接口处理, 此处仅日志
    (void)msg;
}

} // namespace hk_camera
