#include "hk_camera/hk_decoder.h"
#include "hk_camera/hk_camera.h"

namespace hk_camera
{

// ============================================================
// RTSP URL 构造
// ============================================================
std::string HKDecoder::buildRtspUrl(const CameraLoginInfo& info, int channel)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "rtsp://%s:%s@%s:554/h264/ch%d/main/av_stream",
             info.username.c_str(),
             info.password.c_str(),
             info.ip.c_str(),
             channel);
    return std::string(buf);
}

// ============================================================
// 低延迟 GStreamer 管道 (方案1: rtspsrc 直接控制缓冲)
//   latency=0          → 禁用 2000ms jitter buffer (关键!)
//   buffer-mode=none   → 跳过缓冲队列
//   drop-on-latency=1  → 延迟帧直接丢弃
//   nvvidconv          → GPU 颜色转换 + 缩放 (零CPU)
//   sync=false          → appsink 不按时间戳等待
//   max-buffers=1       → 只保留1帧, 新帧到达旧帧丢弃
// ============================================================
std::string HKDecoder::buildGstPipeline(const std::string& url)
{
    // rtspsrc 直连 → 跳过 uridecodebin 的内部缓冲
    // 管道输出 RGBA 给 QImage 直接用, 省掉 cv::cvtColor
    std::string pipeline =
        "rtspsrc location=" + url +
        " latency=0 buffer-mode=none drop-on-latency=1" +
        " ! rtph264depay"
        " ! h264parse"
        " ! avdec_h264"
        " ! nvvidconv"
        " ! video/x-raw,format=RGBA"
        " ! appsink drop=1 max-buffers=1 sync=false";

    return pipeline;
}

// ============================================================
// 低延迟降级管道 (方案2: uridecodebin)
//   当 rtspsrc 管道不可用时使用
// ============================================================
static std::string buildFallbackPipeline(const std::string& url)
{
    // uridecodebin 自动选解码器, 仍然加上低延迟 sink
    std::string pipeline =
        "uridecodebin uri=" + url +
        " ! nvvidconv"
        " ! video/x-raw,format=RGBA"
        " ! appsink drop=1 max-buffers=1 sync=false";

    return pipeline;
}

// ============================================================
HKDecoder::HKDecoder(QObject* parent)
    : QObject(parent)
{
}

HKDecoder::~HKDecoder()
{
    stop();
}

bool HKDecoder::start(const CameraLoginInfo& info, int channel)
{
    if (running_) return true;

    login_info_ = info;
    channel_    = channel;
    running_    = true;

    // 优先尝试低延迟 rtspsrc 管道
    thread_ = std::thread(&HKDecoder::captureLoopHW, this);
    return true;
}

void HKDecoder::stop()
{
    running_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

bool HKDecoder::isRunning() const
{
    return running_;
}

// ============================================================
// 主解码路径: 低延迟 GStreamer 管道
// ============================================================
void HKDecoder::captureLoopHW()
{
    std::string url  = buildRtspUrl(login_info_, channel_);
    std::string pipe = buildGstPipeline(url);

    printf("[Decoder] Trying low-latency rtspsrc pipeline...\n");
    printf("[Decoder] Pipeline: %s\n", pipe.c_str());
    fflush(stdout);

    cv::VideoCapture cap;
    cap.open(pipe, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        // 方案1 失败, 尝试方案2: uridecodebin
        fprintf(stderr, "[Decoder] rtspsrc pipeline FAILED, "
                "trying uridecodebin fallback...\n");
        fflush(stderr);

        std::string fallback = buildFallbackPipeline(url);
        printf("[Decoder] Fallback: %s\n", fallback.c_str());
        fflush(stdout);

        cap.open(fallback, cv::CAP_GSTREAMER);
    }

    if (!cap.isOpened())
    {
        // 方案1+2 都失败, 降级到 FFmpeg 软解
        fprintf(stderr, "[Decoder] GStreamer FAILED, "
                "falling back to FFmpeg software decode\n");
        fflush(stderr);
        captureLoop();
        return;
    }

    printf("[Decoder] GStreamer pipeline OPEN OK\n");
    fflush(stdout);

    int frame_count = 0;
    cv::Mat frame;

    while (running_)
    {
        if (!cap.read(frame) || frame.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        frame_count++;

        // 管道输出 RGBA → cv::Mat 为 CV_8UC4
        // 直接构造 QImage, 无需 cv::cvtColor (省 ~10ms/帧)
        QImage::Format fmt;
        if (frame.channels() == 4)
        {
            fmt = QImage::Format_RGBA8888;
        }
        else if (frame.channels() == 3)
        {
            // nvvidconv 在某些版本可能输出 BGR 而非 RGBA
            cv::Mat rgb;
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
            frame = rgb;
            fmt = QImage::Format_RGB888;
        }
        else
        {
            fmt = QImage::Format_RGB888;
        }

        QImage img(frame.data, frame.cols, frame.rows,
                   static_cast<int>(frame.step), fmt);
        QImage copy = img.copy();  // 深拷贝 (frame 在下次循环被复用)

        if (frame_count == 1 || frame_count % 30 == 0)
        {
            printf("[Decoder] frame=%d, size=%dx%d, ch=%d\n",
                   frame_count, copy.width(), copy.height(),
                   frame.channels());
            fflush(stdout);
        }

        emit frameDecoded(copy);
    }

    cap.release();
    printf("[Decoder] loop ended, frames=%d\n", frame_count);
    fflush(stdout);
}

// ============================================================
// 软解码降级路径 (cv::CAP_FFMPEG)
// ============================================================
void HKDecoder::captureLoop()
{
    std::string url = buildRtspUrl(login_info_, channel_);
    printf("[Decoder-SW] FFmpeg fallback: %s\n", url.c_str());
    fflush(stdout);

    cv::VideoCapture cap;

    // FFmpeg 低延迟选项: 设置 RTSP 传输为 UDP + 减少缓冲
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);              // 最小缓冲帧数
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('H','2','6','4'));

    cap.open(url, cv::CAP_FFMPEG);

    if (!cap.isOpened())
    {
        fprintf(stderr, "[Decoder-SW] FFmpeg open FAILED\n");
        fflush(stderr);
        running_ = false;
        return;
    }

    printf("[Decoder-SW] FFmpeg open OK\n");
    fflush(stdout);

    int frame_count = 0;
    cv::Mat frame;

    while (running_)
    {
        if (!cap.read(frame) || frame.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        frame_count++;

        // BGR → RGB 转换 (FFmpeg 后端输出 BGR)
        cv::Mat rgb;
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

        QImage img(rgb.data, rgb.cols, rgb.rows,
                   static_cast<int>(rgb.step),
                   QImage::Format_RGB888);
        QImage copy = img.copy();

        if (frame_count == 1 || frame_count % 30 == 0)
        {
            printf("[Decoder-SW] frame=%d, size=%dx%d\n",
                   frame_count, copy.width(), copy.height());
            fflush(stdout);
        }

        emit frameDecoded(copy);
    }

    cap.release();
    printf("[Decoder-SW] loop ended, frames=%d\n", frame_count);
    fflush(stdout);
}

} // namespace hk_camera
