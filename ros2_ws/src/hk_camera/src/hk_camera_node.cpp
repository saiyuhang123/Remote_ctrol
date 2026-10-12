#include "hk_camera/hk_camera_node.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <QDir>

namespace hk_camera
{

HKCameraNode::HKCameraNode(const std::string& name)
    : Node(name)
{
    camera_  = HKCamera::instance();
    // The GUI creates this node without the headless entry point's SDK setup.
    if (!camera_->init()) throw std::runtime_error("HK SDK initialization failed");
    decoder_ = new HKDecoder();  // HKCameraNode 不是 QObject, 不传 parent

    loadParameters();
    setupPublishers();
    setupSubscribers();
    setupServices();
    connectCameraSignals();
    face_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                  [this] { processFaceCaptures(); });

    RCLCPP_INFO(get_logger(), "HKCameraNode initialized (SDK %s)",
                camera_->getSDKVersion().c_str());

    if (get_parameter("auto_login").as_bool())
    {
        auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
        onLogin(nullptr, response);
        if (response->success && get_parameter("auto_stream").as_bool())
        {
            onStartStream(nullptr, response);
            if (!response->success)
                RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        }
    }
}

HKCameraNode::~HKCameraNode()
{
    accept_faces_ = false;
    face_timer_->cancel();
    for (const auto& connection : camera_connections_) QObject::disconnect(connection);
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
    declare_parameter("password", "zmdl@603");
    declare_parameter("channel",  1);
    declare_parameter("auto_login", false);
    declare_parameter("auto_stream", true);
    declare_parameter("enable_face_capture", true);
    declare_parameter("face_capture_dir", (QDir::homePath() +
                      "/Remote_ctrol/captures/hk_faces").toStdString());
    declare_parameter("face_queue_limit", 8);

    login_info_.ip       = get_parameter("ip").as_string();
    login_info_.port     = static_cast<uint16_t>(get_parameter("port").as_int());
    login_info_.username = get_parameter("username").as_string();
    login_info_.password = get_parameter("password").as_string();
    channel_             = get_parameter("channel").as_int();
    enable_face_capture_ = get_parameter("enable_face_capture").as_bool();
    face_capture_dir_ = get_parameter("face_capture_dir").as_string();
    const auto queue_limit = get_parameter("face_queue_limit").as_int();
    if (face_capture_dir_.empty() || queue_limit < 1 || queue_limit > 64)
        throw std::invalid_argument("face_capture_dir must be nonempty; face_queue_limit must be 1..64");
    face_queue_limit_ = static_cast<size_t>(queue_limit);

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
    face_pub_ = create_publisher<msg::FaceCapture>(
        "/hk_camera/face_capture", rclcpp::QoS(10).reliable().transient_local());
    face_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/hk_camera/face_image", rclcpp::QoS(1).reliable().transient_local());
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
    camera_connections_.push_back(QObject::connect(decoder_, &HKDecoder::frameDecoded,
        [this](const QImage& image) { onDecodedImage(image); }));

    // 报警: SDK回调线程 → ROS2 publish
    camera_connections_.push_back(QObject::connect(camera_, &HKCamera::alarmReceived,
        [this](const AlarmEvent& alarm) { onAlarm(alarm); }));
    camera_connections_.push_back(QObject::connect(camera_, &HKCamera::faceCaptured,
        [this](const FaceCaptureEvent& event) { onFaceCapture(event); }));

    // 错误处理
    camera_connections_.push_back(QObject::connect(camera_, &HKCamera::errorOccurred,
        [this](int code, const QString& msg) {
            RCLCPP_ERROR(get_logger(), "HK SDK error [%d]: %s",
                         code, msg.toStdString().c_str());
        }));
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
    if (alarm.user_id >= 0 && alarm.user_id != user_id_.load()) return;
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

    RCLCPP_INFO(get_logger(), "Alarm: command=0x%lx type=%u, channels=%zu",
                static_cast<unsigned long>(alarm.command), alarm.alarm_type, msg.alarm_channels.size());
}

void HKCameraNode::onFaceCapture(const FaceCaptureEvent& event)
{
    if (!accept_faces_ || (event.user_id >= 0 && event.user_id != user_id_.load()) ||
        (event.channel != 0 && event.channel != static_cast<uint32_t>(channel_))) return;
    std::lock_guard<std::mutex> guard(face_mutex_);
    if (!accept_faces_) return;
    if (face_queue_.size() >= face_queue_limit_)
    {
        ++face_dropped_;
        return;
    }
    face_queue_.push_back(event);
}

bool HKCameraNode::startAlarmReception()
{
    if (!enable_face_capture_ || alarm_handle_ >= 0) return true;
    try
    {
        std::filesystem::create_directories(face_capture_dir_);
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(get_logger(), "Cannot create face capture directory: %s", error.what());
        return false;
    }
    face_coordinate_size_ = camera_->videoSize(user_id_.load(), channel_);
    if (!face_coordinate_size_[0])
        RCLCPP_WARN(get_logger(), "Cannot read video size; face coordinates will use the full background image");
    else
        RCLCPP_INFO(get_logger(), "Face coordinate video size: %ux%u",
                    face_coordinate_size_[0], face_coordinate_size_[1]);
    accept_faces_ = true;
    alarm_handle_ = camera_->setupAlarmChan(user_id_.load());
    if (alarm_handle_ < 0)
    {
        accept_faces_ = false;
        RCLCPP_ERROR(get_logger(), "Face alarm setup failed, SDK error=%d", camera_->getLastError());
        return false;
    }
    RCLCPP_INFO(get_logger(), "Face alarm reception active; output=%s", face_capture_dir_.c_str());
    return true;
}

void HKCameraNode::processFaceCaptures()
{
    const auto dropped = face_dropped_.exchange(0);
    if (dropped) RCLCPP_WARN(get_logger(), "Face queue full: dropped %lu events", dropped);
    FaceCaptureEvent event;
    {
        std::lock_guard<std::mutex> guard(face_mutex_);
        if (face_queue_.empty()) return;
        event = std::move(face_queue_.front());
        face_queue_.pop_front();
    }
    if (!accept_faces_ || user_id_.load() < 0 ||
        (event.user_id >= 0 && event.user_id != user_id_.load())) return;
    if (event.channel == 0) event.channel = channel_;
    event.coordinate_width = face_coordinate_size_[0];
    event.coordinate_height = face_coordinate_size_[1];
    try
    {
        if (event.background_jpeg.empty() && !event.face_jpeg.empty())
            event.background_jpeg = camera_->findFaceBackground(user_id_.load(), event.face_id);
        const auto stem = std::to_string(event.received_ns) + "_face" +
                          std::to_string(event.face_id) + "_" + std::to_string(++face_sequence_);
        const auto files = saveFaceCapture(event, face_capture_dir_, stem);
        auto message = msg::FaceCapture();
        message.header.stamp = rclcpp::Time(event.received_ns, RCL_SYSTEM_TIME);
        message.header.frame_id = "hk_camera";
        message.channel = event.channel;
        message.face_id = event.face_id;
        message.target_id = event.target_id;
        message.score = event.score;
        message.camera_time_raw = event.camera_time;
        message.end_mark = event.end_mark;
        message.rect_valid = files.rect_valid;
        message.background_width = files.background_width;
        message.background_height = files.background_height;
        message.coordinate_width = files.coordinate_width;
        message.coordinate_height = files.coordinate_height;
        message.rect_normalized = files.rect;
        message.rect_pixels = {files.pixels.x, files.pixels.y, files.pixels.width, files.pixels.height};
        message.face_path = files.face_path;
        message.background_path = files.background_path;
        message.annotated_path = files.annotated_path;
        message.metadata_path = files.metadata_path;
        face_pub_->publish(message);
        if (!files.annotated.empty())
            face_image_pub_->publish(*cv_bridge::CvImage(message.header, "bgr8", files.annotated).toImageMsg());
        for (const auto& warning : files.warnings)
            RCLCPP_WARN(get_logger(), "Face %u: %s", event.face_id, warning.c_str());
        RCLCPP_INFO(get_logger(), "Face %u: face=%s boxed=%s metadata=%s", event.face_id,
                    files.face_path.c_str(), files.annotated_path.c_str(), files.metadata_path.c_str());
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(get_logger(), "Face %u could not be saved: %s", event.face_id, error.what());
    }
}

// ============================================================
// Service 实现
// ============================================================

void HKCameraNode::onLogin(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (user_id_ >= 0)
    {
        res->success = startAlarmReception();
        res->message = res->success ? "Already logged in, user_id=" + std::to_string(user_id_.load())
                                   : "Logged in, but alarm setup failed; call login again to retry";
        return;
    }
    user_id_ = camera_->login(login_info_);

    if (user_id_ >= 0)
    {
        res->success = startAlarmReception();
        res->message = res->success ? "Login OK, user_id=" + std::to_string(user_id_.load())
                                   : "Login OK, but alarm setup failed; call login again to retry";
        RCLCPP_INFO(get_logger(), "Device login OK, user_id=%d", user_id_.load());
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
        accept_faces_ = false;
        if (alarm_handle_ >= 0) camera_->closeAlarmChan(alarm_handle_);
        alarm_handle_ = -1;
        decoder_->stop();
        {
            std::lock_guard<std::mutex> guard(face_mutex_);
            face_queue_.clear();
        }
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
