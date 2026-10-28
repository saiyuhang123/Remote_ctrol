#include "hk_camera/ptz_codec.h"
#include "hk_camera/image_message.h"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <QTemporaryDir>
#include <filesystem>
#include "hk_camera/hk_camera_node.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <cmath>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>

namespace hk_camera
{

namespace
{
void publishImage(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher,
                  const QImage& image, const rclcpp::Time& stamp)
{
    if (image.isNull()) return;
    auto header = std_msgs::msg::Header();
    header.stamp = stamp;
    header.frame_id = "hk_camera";
    publisher->publish(makeImageMessage(image, header));
}
}

HKCameraNode::HKCameraNode(const std::string& name)
    : Node(name)
{
    camera_  = HKCamera::instance();
    // The GUI creates this node without the headless entry point's SDK setup.
    if (!camera_->init()) throw std::runtime_error("HK SDK initialization failed");
    decoder_ = new HKDecoder();  // HKCameraNode 不是 QObject, 不传 parent

    loadParameters();
    rclcpp::InitOptions transport_options;
    transport_options.auto_initialize_logging(false);
    sdk_context_ = std::make_shared<rclcpp::Context>();
    sdk_context_->init(0, nullptr, transport_options);
    sdk_node_ = std::make_shared<rclcpp::Node>("hk_camera_sdk", get_namespace(),
        rclcpp::NodeOptions().context(sdk_context_).use_global_arguments(false)
            .start_parameter_services(false).start_parameter_event_publisher(false));
    preview_context_ = std::make_shared<rclcpp::Context>();
    preview_context_->init(0, nullptr, transport_options);
    preview_node_ = std::make_shared<rclcpp::Node>("hk_camera_preview", get_namespace(),
        rclcpp::NodeOptions().context(preview_context_).use_global_arguments(false)
            .start_parameter_services(false).start_parameter_event_publisher(false));
    rclcpp::ExecutorOptions sdk_executor_options;
    sdk_executor_options.context = sdk_context_;
    sdk_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(sdk_executor_options);
    sdk_executor_->add_node(sdk_node_);
    setupPublishers();
    setupSubscribers();
    setupServices();
    connectCameraSignals();
    stream_status_timer_ = create_wall_timer(std::chrono::seconds(1),
                                           [this] { publishStreamStatus(); });
    face_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                  [this] { processFaceCaptures(); });
    face_status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publishFaceStatus(); });
    try {
        raw_publish_thread_ = std::thread([this] { publishFrames(false); });
        preview_publish_thread_ = std::thread([this] { publishFrames(true); });
        sdk_thread_ = std::thread([this] {
            try { sdk_executor_->spin(); }
            catch (const std::exception&) {
                if (rclcpp::ok(sdk_context_)) RCLCPP_ERROR(get_logger(), "SDK executor stopped unexpectedly");
            }
        });
    } catch (...) {
        sdk_executor_->cancel();
        if (sdk_thread_.joinable()) sdk_thread_.join();
        latest_frame_.close();
        if (raw_publish_thread_.joinable()) raw_publish_thread_.join();
        if (preview_publish_thread_.joinable()) preview_publish_thread_.join();
        delete decoder_;
        throw;
    }

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
    sdk_executor_->cancel();
    sdk_thread_.join();
    stopPTZ();
    ptz_timer_->cancel();
    accept_faces_ = false;
    face_timer_->cancel();
    face_status_timer_->cancel();
    stream_status_timer_->cancel();
    for (const auto& connection : camera_connections_) QObject::disconnect(connection);
    decoder_->stop();
    latest_frame_.close();
    raw_publish_thread_.join();
    preview_publish_thread_.join();
    if (alarm_handle_ >= 0) camera_->closeAlarmChan(alarm_handle_);
    while (true) {
        { std::lock_guard<std::mutex> guard(face_mutex_); if (face_queue_.empty()) break; }
        processFaceCaptures();
    }
    if (user_id_ >= 0)      camera_->logout(user_id_);
    camera_->cleanup();
    delete decoder_;
    sdk_executor_->remove_node(sdk_node_);
    if (rclcpp::ok(sdk_context_)) sdk_context_->shutdown("Camera stopped");
    if (rclcpp::ok(preview_context_)) preview_context_->shutdown("Camera stopped");
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
    // 设备不上报 PTZ 范围（PTZSCOPE 全 0）时会拒绝绝对 pan/tilt。
    // 置 true 可跳过该校验直接下发 wAction=1，用于验证设备究竟收不收绝对角度。
    declare_parameter("force_absolute_ptz", false);
    // 云台控制是否由本节点（SDK）提供。改走 485 时置 false，
    // 由 ptz_pelco 节点提供同名同类型服务。
    declare_parameter("enable_ptz_services", true);
    rcl_interfaces::msg::ParameterDescriptor raw_rate_descriptor;
    raw_rate_descriptor.read_only = true;
    raw_rate_descriptor.description = "Raw image publish limit, 0..30 fps; 0 is unlimited. Restart to change.";
    declare_parameter("raw_publish_fps", 8., raw_rate_descriptor);
    rcl_interfaces::msg::ParameterDescriptor stream_descriptor;
    stream_descriptor.read_only = true;
    stream_descriptor.description = "RTSP timeout/retry settings; restart to change.";
    declare_parameter("reconnect_interval", 5., stream_descriptor);
    declare_parameter("stream_open_timeout_ms", 3000, stream_descriptor);
    declare_parameter("stream_read_timeout_ms", 2000, stream_descriptor);
    declare_parameter("preview_max_width", 640);
    declare_parameter("preview_publish_fps", 5.);

    raw_publish_fps_ = get_parameter("raw_publish_fps").as_double();
    if (!validRawPublishFps(raw_publish_fps_))
        throw std::invalid_argument("raw_publish_fps must be 0..30 (0 is unlimited)");
    preview_max_width_ = static_cast<int>(get_parameter("preview_max_width").as_int());
    preview_publish_fps_ = get_parameter("preview_publish_fps").as_double();
    if (!validPreviewSettings(preview_max_width_, preview_publish_fps_))
        throw std::invalid_argument("preview_max_width must be 160..4096; preview_publish_fps must be 0.5..30");
    const double reconnect = get_parameter("reconnect_interval").as_double();
    const auto open_timeout = get_parameter("stream_open_timeout_ms").as_int();
    const auto read_timeout = get_parameter("stream_read_timeout_ms").as_int();
    if (!std::isfinite(reconnect) || reconnect < .1 || reconnect > 60. ||
        open_timeout < 100 || open_timeout > 30000 || read_timeout < 100 || read_timeout > 30000)
        throw std::invalid_argument("RTSP reconnect_interval must be 0.1..60 s and timeouts 100..30000 ms");
    decoder_options_ = {static_cast<int>(open_timeout), static_cast<int>(read_timeout),
                        static_cast<int>(reconnect*1000.)};

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
    rcl_interfaces::msg::SetParametersResult result;
    for (const auto& parameter : params) {
        if (parameter.get_name() == "enable_face_capture") {
            result.successful = false;
            result.reason = "Use /hk_camera/set_face_capture for runtime control; parameter is the startup default";
            return result;
        }
    }
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
        "/hk_camera/image_raw", rclcpp::QoS(1).reliable());
    preview_pub_ = preview_node_->create_publisher<sensor_msgs::msg::Image>(
        "/hk_camera/image_preview", rclcpp::SensorDataQoS().keep_last(1));
    alarm_pub_  = create_publisher<msg::CameraAlarm>(
        "/hk_camera/alarm", 10);
    status_pub_ = preview_node_->create_publisher<std_msgs::msg::String>(
        "/hk_camera/status", 10);
    stream_status_pub_ = preview_node_->create_publisher<std_msgs::msg::String>(
        "/hk_camera/stream_status", rclcpp::QoS(1).best_effort());
    face_status_pub_ = create_publisher<std_msgs::msg::String>("/hk_camera/face_status", 5);
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
    capture_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    face_capture_srv_ = create_service<std_srvs::srv::SetBool>("/hk_camera/set_face_capture",
        [this](std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr res) { setFaceCapture(req->data, *res); });
    capture_srv_ = create_service<srv::CapturePicture>(
        "/hk_camera/capture",
        [this](const srv::CapturePicture::Request::SharedPtr  req,
               srv::CapturePicture::Response::SharedPtr       res) {
            onCapturePicture(req, res);
        }, rmw_qos_profile_services_default, capture_group_);

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

    // The external RS485 head reports angles; the camera reports and drives
    // the optical lens. Keep SDK services private when ptz_pelco owns the
    // public services so it can combine these two independent devices.
    const bool public_ptz = get_parameter("enable_ptz_services").as_bool();
    const std::string ptz_prefix = public_ptz ? "/hk_camera/" : "/hk_camera/sdk/";
    control_ptz_srv_ = sdk_node_->create_service<srv::ControlPTZ>(ptz_prefix + "control_ptz",
        [this](const srv::ControlPTZ::Request::SharedPtr req,
               srv::ControlPTZ::Response::SharedPtr res) { onControlPTZ(req, res); });
    ptz_timer_ = sdk_node_->create_wall_timer(std::chrono::milliseconds(100), [this] {
        std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
        if (active_ptz_command_ && std::chrono::steady_clock::now() >= ptz_deadline_)
            stopPTZ();  // Retry on SDK errors until a stop succeeds.
    });

    get_ptz_pose_srv_ = sdk_node_->create_service<srv::GetPTZPose>(
        ptz_prefix + "get_ptz_pose",
        [this](const srv::GetPTZPose::Request::SharedPtr req,
               srv::GetPTZPose::Response::SharedPtr res) { onGetPTZPose(req, res); });
    ptz_preset_srv_ = sdk_node_->create_service<srv::PTZPreset>(
        "/hk_camera/preset",
        [this](const srv::PTZPreset::Request::SharedPtr req,
               srv::PTZPreset::Response::SharedPtr res) { onPTZPreset(req, res); });

    set_ptz_pose_srv_ = sdk_node_->create_service<srv::SetPTZPose>(
        ptz_prefix + "set_ptz_pose",
        [this](const srv::SetPTZPose::Request::SharedPtr  req,
               srv::SetPTZPose::Response::SharedPtr       res) {
            onSetPTZPose(req, res);
        });
    if (!public_ptz) {
        RCLCPP_INFO(get_logger(), "公共 PTZ 服务由 ptz_pelco 提供；镜头 SDK 服务位于 /hk_camera/sdk/");
    }
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
    // This callback runs on the RTSP reader. Never serialize or publish a ROS
    // image here: DDS backpressure must not stop draining the camera stream.
    latest_frame_.put(image, now().nanoseconds());
}

void HKCameraNode::publishFrames(bool preview)
{
    uint64_t seen = 0;
    ImageRateGate gate;
    const auto publisher = preview ? preview_pub_ : image_pub_;
    const double fps = preview ? preview_publish_fps_ : raw_publish_fps_;
    while (const auto frame = latest_frame_.waitNext(seen))
    {
        const auto current = std::chrono::steady_clock::now();
        if (!decoder_->isRunning() || current-frame->received > std::chrono::seconds(1) ||
            !gate.due(publisher->get_subscription_count() > 0, fps, current)) continue;
        try {
            publishImage(publisher, preview ? makePreview(frame->image, preview_max_width_) : frame->image,
                         rclcpp::Time(frame->stamp_ns));
            auto& last_publish = preview ? last_preview_publish_ms_ : last_raw_publish_ms_;
            last_publish = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        } catch (const std::exception&) {
            if (rclcpp::ok(get_node_base_interface()->get_context()))
                RCLCPP_ERROR(get_logger(), "%s image publish failed", preview ? "Preview" : "Raw");
        }
    }
}

void HKCameraNode::publishStreamStatus()
{
    const auto health = decoder_->health();
    const std::string state = !health.requested ? "stopped" :
        health.connected && health.frame_age_ms <= decoder_options_.read_timeout_ms ? "streaming" :
        health.connection_attempts > 1 ? "reconnecting" : "connecting";
    if (state != last_stream_state_) {
        if (state == "streaming" || state == "stopped")
            RCLCPP_INFO(get_logger(), "Video state=%s attempts=%lu frames=%lu", state.c_str(),
                        health.connection_attempts, health.frames);
        else
            RCLCPP_WARN(get_logger(), "Video state=%s attempts=%lu frame_age_ms=%ld error=%s",
                        state.c_str(), health.connection_attempts, health.frame_age_ms, health.last_error.c_str());
        last_stream_state_ = state;
    }
    std_msgs::msg::String legacy;
    legacy.data = state == "streaming" ? "streaming_active" : "streaming_" + state;
    status_pub_->publish(legacy);
    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto age = [now_ms](int64_t value) { return value ? now_ms-value : int64_t(-1); };
    QJsonObject object{{"state", QString::fromStdString(state)}, {"requested", health.requested},
        {"connected", health.connected}, {"frames", static_cast<qint64>(health.frames)},
        {"connection_attempts", static_cast<qint64>(health.connection_attempts)},
        {"frame_age_ms", static_cast<qint64>(health.frame_age_ms)},
        {"raw_publish_age_ms", static_cast<qint64>(age(last_raw_publish_ms_))},
        {"preview_publish_age_ms", static_cast<qint64>(age(last_preview_publish_ms_))},
        {"last_sdk_query_ms", static_cast<qint64>(last_sdk_query_ms_)},
        {"last_sdk_lock_wait_ms", static_cast<qint64>(last_sdk_lock_wait_ms_)},
        {"last_error", QString::fromStdString(health.last_error)}};
    std_msgs::msg::String message;
    message.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
    stream_status_pub_->publish(message);
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
    face_dropped_total_ += dropped;
    if (dropped) RCLCPP_WARN(get_logger(), "Face queue full: dropped %lu events", dropped);
    FaceCaptureEvent event;
    {
        std::lock_guard<std::mutex> guard(face_mutex_);
        if (face_queue_.empty()) return;
        event = std::move(face_queue_.front());
        face_queue_.pop_front();
    }
    // Stopping reception must not discard events already accepted into the queue.
    if (user_id_.load() < 0 ||
        (event.user_id >= 0 && event.user_id != user_id_.load())) return;
    if (event.channel == 0) event.channel = channel_;
    event.coordinate_width = face_coordinate_size_[0];
    event.coordinate_height = face_coordinate_size_[1];
    try
    {
        if (event.background_jpeg.empty() && !event.face_jpeg.empty())
        {
            std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
            event.background_jpeg = camera_->findFaceBackground(user_id_.load(), event.face_id);
        }
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
        ++face_saved_total_;
        if (!files.annotated.empty())
            face_image_pub_->publish(*cv_bridge::CvImage(message.header, "bgr8", files.annotated).toImageMsg());
        for (const auto& warning : files.warnings)
            RCLCPP_WARN(get_logger(), "Face %u: %s", event.face_id, warning.c_str());
        RCLCPP_INFO(get_logger(), "Face %u: face=%s boxed=%s metadata=%s", event.face_id,
                    files.face_path.c_str(), files.annotated_path.c_str(), files.metadata_path.c_str());
    }
    catch (const std::exception& error)
    {
        face_last_error_ = error.what();
        RCLCPP_ERROR(get_logger(), "Face %u could not be saved: %s", event.face_id, error.what());
    }
}

// ============================================================
// Service 实现
// ============================================================

void HKCameraNode::setFaceCapture(bool enabled, std_srvs::srv::SetBool::Response& response)
{
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
    if (enabled) {
        if (user_id_ < 0) { response.message = "Camera not logged in; connect camera first"; return; }
        const bool before = enable_face_capture_;
        enable_face_capture_ = true;
        response.success = startAlarmReception();
        if (!response.success) enable_face_capture_ = before;
        response.message = response.success ? "SDK face reception enabled; device facesnap rules must be configured"
                                            : "SDK face alarm setup failed";
    } else {
        accept_faces_ = false;
        if (alarm_handle_ >= 0 && !camera_->closeAlarmChan(alarm_handle_)) {
            accept_faces_ = true;
            response.message = "SDK face alarm close failed; reception remains enabled";
            return;
        }
        alarm_handle_ = -1;
        enable_face_capture_ = false;
        response.success = true;
        response.message = "New face reception stopped; accepted events will finish saving";
    }
    face_last_error_ = response.success ? "" : response.message;
    publishFaceStatus();
}

void HKCameraNode::publishFaceStatus()
{
    size_t queued;
    { std::lock_guard<std::mutex> guard(face_mutex_); queued = face_queue_.size(); }
    QJsonObject value{{"component", "hikvision_face"}, {"logged_in", user_id_.load() >= 0},
        {"enabled", accept_faces_.load()}, {"queue", static_cast<double>(queued)},
        {"saved", static_cast<double>(face_saved_total_)}, {"dropped", static_cast<double>(face_dropped_total_)},
        {"last_error", QString::fromStdString(face_last_error_)},
        {"face_dir", QString::fromStdString(face_capture_dir_)}};
    std_msgs::msg::String message;
    message.data = QJsonDocument(value).toJson(QJsonDocument::Compact).toStdString();
    face_status_pub_->publish(message);
}

void HKCameraNode::onLogin(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
    if (user_id_ >= 0)
    {
        stopPTZ();
        accept_faces_ = false;
        if (alarm_handle_ >= 0) camera_->closeAlarmChan(alarm_handle_);
        alarm_handle_ = -1;
        decoder_->stop();
        latest_frame_.clear();
        while (true) {
            { std::lock_guard<std::mutex> guard(face_mutex_); if (face_queue_.empty()) break; }
            processFaceCaptures();
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
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
    if (user_id_ < 0)
    {
        res->success = false;
        res->message = "Not logged in";
        return;
    }

    if (decoder_->isRunning())
    {
        res->success = true;
        res->message = "Stream already active or reconnecting";
        return;
    }

    latest_frame_.clear();
    if (decoder_->start(login_info_, channel_, decoder_options_))
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
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
    stopPTZ();
    if (decoder_->isRunning())
    {
        decoder_->stop();
        latest_frame_.clear();
        res->success = true;
        res->message = "Stream stopped";
    }
    else
    {
        res->success = true;
        res->message = "Stream already stopped";
    }
}

void HKCameraNode::onCapturePicture(
    const srv::CapturePicture::Request::SharedPtr req,
    srv::CapturePicture::Response::SharedPtr res)
{
    if (req->quality > 2) {
        res->success = false;
        res->message = "Invalid JPEG quality (expected 0..2)";
        return;
    }
    // Empty path means preview only. The SDK needs a filename; remove its
    // temporary JPEG after copying the response, without a machine-specific path.
    QTemporaryDir temporary;
    std::string path = req->save_path;
    if (path.empty()) {
        if (!temporary.isValid()) {
            res->message = "Cannot create temporary capture directory";
            return;
        }
        path = temporary.filePath("preview.jpg").toStdString();
    } else {
        std::error_code error;
        const auto parent = std::filesystem::path(path).parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent, error);
        if (error) {
            res->message = "Cannot create capture directory: " + error.message();
            return;
        }
    }

    const auto started = std::chrono::steady_clock::now();
    bool ok;
    int sdk_error = 0;
    {
        std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
        if (user_id_ < 0) {
            res->message = "Not logged in";
            return;
        }
        ok = camera_->captureJPEG(user_id_, channel_, req->quality, path);
        if (!ok) sdk_error = camera_->getLastError();
    }
    const auto captured = std::chrono::steady_clock::now();

    res->success = ok;
    if (ok)
    {
        res->message = req->save_path.empty() ? "Captured preview" : "Captured: " + path;
        // 尝试读取并发布图片
        cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
        if (!img.empty())
        {
            auto img_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), "bgr8", img).toImageMsg();
            img_msg->header.stamp = now();
            img_msg->header.frame_id = "hk_camera";
            res->image = *img_msg;
        } else {
            res->success = false;
            res->message = "Captured JPEG could not be decoded";
        }
    }
    else
    {
        res->message = "Capture failed, error=" +
                       std::to_string(sdk_error);
    }
    const auto finished = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Capture complete: success=%d SDK=%.1f ms image=%.1f ms bytes=%zu",
        res->success,
        std::chrono::duration<double, std::milli>(captured-started).count(),
        std::chrono::duration<double, std::milli>(finished-captured).count(),
        res->image.data.size());
}

// ============================================================
// PTZ 姿态控制服务
// ============================================================

void HKCameraNode::onSetPTZPose(
    const srv::SetPTZPose::Request::SharedPtr req,
    srv::SetPTZPose::Response::SharedPtr res)
{
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
    if (user_id_ < 0)
    {
        res->success = false;
        res->message = "Not logged in";
        return;
    }

    if (active_ptz_command_) { res->message = "Release manual PTZ control first"; return; }
    if (!ptz::validPose(req->pan, req->tilt, req->zoom)) {
        res->message = "Invalid PTZ coordinates: pan/tilt 0..360 degrees, zoom 1..999.9x";
        return;
    }
    NET_DVR_PTZPOS pose{};
    pose.wAction = 1;  // Absolute pan, tilt and zoom, not a preset number.
    pose.wPanPos = *ptz::encode(req->pan);
    pose.wTiltPos = *ptz::encode(req->tilt);
    pose.wZoomPos = *ptz::encode(req->zoom);
    // Validate against the device's limits when it supports scope queries.
    // force_absolute_ptz=true 跳过整段校验（设备不上报 PTZSCOPE 时用于验证硬件是否接受绝对角度）。
    if (!get_parameter("force_absolute_ptz").as_bool()) {
    NET_DVR_PTZSCOPE scope{};
    DWORD length = 0;
    if (camera_->getDVRConfig(user_id_, NET_DVR_GET_PTZSCOPE, channel_, &scope, &length)) {
        if (scope.wPanPosMin == 0 && scope.wPanPosMax == 0 &&
            scope.wTiltPosMin == 0 && scope.wTiltPosMax == 0) {
            if (req->pan != 0 || req->tilt != 0) {
                res->message = "Device does not report absolute pan/tilt; use manual PTZ direction controls";
                return;
            }
            pose.wAction = 4;  // This IPZoom + RS485 device supports absolute zoom only.
        }
        if (!ptz::withinScope(pose.wPanPos, scope.wPanPosMin, scope.wPanPosMax) ||
            !ptz::withinScope(pose.wTiltPos, scope.wTiltPosMin, scope.wTiltPosMax) ||
            !ptz::withinScope(pose.wZoomPos, scope.wZoomPosMin, scope.wZoomPosMax)) {
            res->message = "PTZ coordinates exceed device limits; read current PTZ first";
            return;
        }
    }
    }
    res->success = camera_->setDVRConfig(user_id_, NET_DVR_SET_PTZPOS, channel_, &pose);
    res->message = res->success ? "PTZ target accepted (read back to confirm arrival)" :
        "Absolute PTZ command failed, SDK error=" + std::to_string(camera_->getLastError());
}

void HKCameraNode::onGetPTZPose(
    const srv::GetPTZPose::Request::SharedPtr,
    srv::GetPTZPose::Response::SharedPtr res)
{
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
    const auto locked = Clock::now();
    last_sdk_lock_wait_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(locked-started).count();
    struct Timing {
        std::atomic<int64_t>& elapsed;
        Clock::time_point start;
        ~Timing() { elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-start).count(); }
    } timing{last_sdk_query_ms_, locked};
    if (user_id_ < 0) { res->message = "Not logged in"; return; }
    NET_DVR_PTZPOS pose{};
    DWORD length = 0;
    if (!camera_->getDVRConfig(user_id_, NET_DVR_GET_PTZPOS, channel_, &pose, &length)) {
        res->message = "Read PTZ failed, SDK error=" + std::to_string(camera_->getLastError());
        return;
    }
    const auto pan = ptz::decode(pose.wPanPos), tilt = ptz::decode(pose.wTiltPos), zoom = ptz::decode(pose.wZoomPos);
    if (!pan || !tilt || !zoom || !ptz::validPose(*pan, *tilt, *zoom)) {
        res->message = "Device returned invalid PTZ coordinates";
        return;
    }
    res->pan = *pan; res->tilt = *tilt; res->zoom = *zoom;
    res->success = true;
    res->message = "Actual PTZ coordinates (SDK degrees / optical zoom)";
    NET_DVR_PTZSCOPE scope{};
    if (camera_->getDVRConfig(user_id_, NET_DVR_GET_PTZSCOPE, channel_, &scope, &length)) {
        const auto pmin = ptz::decode(scope.wPanPosMin), pmax = ptz::decode(scope.wPanPosMax);
        const auto tmin = ptz::decode(scope.wTiltPosMin), tmax = ptz::decode(scope.wTiltPosMax);
        const auto zmin = ptz::decode(scope.wZoomPosMin), zmax = ptz::decode(scope.wZoomPosMax);
        if (pmin && pmax && tmin && tmax && zmin && zmax) {
            res->limits_valid = true;
            res->pan_min = *pmin; res->pan_max = *pmax;
            res->tilt_min = *tmin; res->tilt_max = *tmax;
            res->zoom_min = *zmin; res->zoom_max = *zmax;
        }
    }
}

void HKCameraNode::onPTZPreset(
    const srv::PTZPreset::Request::SharedPtr req,
    srv::PTZPreset::Response::SharedPtr res)
{
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
    if (user_id_ < 0) {
        res->success = false;
        res->message = "Not logged in";
        return;
    }
    if (active_ptz_command_) {
        res->success = false;
        res->message = "Release manual PTZ control first";
        return;
    }
    if (req->preset_no < 1 || req->preset_no > 255) {
        res->success = false;
        res->message = "Preset number must be 1..255";
        return;
    }
    DWORD command = 0;
    const char *verb = "";
    switch (req->action) {
        case 0: command = SET_PRESET; verb = "saved"; break;
        case 1: command = GOTO_PRESET; verb = "goto requested"; break;
        case 2: command = CLE_PRESET; verb = "cleared"; break;
        default:
            res->success = false;
            res->message = "Invalid preset action (expected 0=set, 1=goto, 2=clear)";
            return;
    }
    const bool ok = camera_->ptzPreset(user_id_, channel_, command, req->preset_no);
    res->success = ok;
    res->message = ok
        ? std::string("Preset ") + std::to_string(req->preset_no) + " " + verb +
              " (device-side position)"
        : std::string("Preset command failed, SDK error=") +
              std::to_string(camera_->getLastError());
    if (ok) {
        RCLCPP_INFO(get_logger(), "PTZ preset %u %s", req->preset_no, verb);
    }
}

bool HKCameraNode::stopPTZ()
{
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
    if (!active_ptz_command_) return true;
    if (user_id_ >= 0 && !camera_->ptzControlWithSpeed(user_id_, channel_,
            active_ptz_command_, active_ptz_speed_, 1)) return false;
    active_ptz_command_ = 0;
    return true;
}

void HKCameraNode::onControlPTZ(const srv::ControlPTZ::Request::SharedPtr req,
                               srv::ControlPTZ::Response::SharedPtr res)
{
    std::lock_guard<std::recursive_mutex> lock(sdk_session_mutex_);
    if (req->stop) {
        res->success = stopPTZ();
        res->message = res->success ? "PTZ stopped" : "PTZ stop failed; watchdog will retry";
        return;
    }
    if (user_id_ < 0) { res->message = "Not logged in"; return; }
    const bool direction_ok = req->command >= 21 && req->command <= 28;  // 含 25~28 斜向
    const bool zoom_ok = req->command == 11 || req->command == 12;
    if ((!direction_ok && !zoom_ok) ||
        req->speed < 1 || req->speed > 7) { res->message = "Invalid PTZ direction or speed"; return; }
    if (active_ptz_command_ != req->command || active_ptz_speed_ != req->speed) {
        if (!stopPTZ()) { res->message = "Could not stop previous PTZ command"; return; }
        if (!camera_->ptzControlWithSpeed(user_id_, channel_, req->command, req->speed, 0)) {
            res->message = "Manual PTZ failed, SDK error=" + std::to_string(camera_->getLastError());
            return;
        }
        active_ptz_command_ = req->command;
        active_ptz_speed_ = req->speed;
    }
    ptz_deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    res->success = true;
    res->message = "PTZ lease renewed";
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
