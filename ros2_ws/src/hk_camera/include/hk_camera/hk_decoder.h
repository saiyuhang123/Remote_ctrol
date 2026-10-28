#ifndef HK_DECODER_H
#define HK_DECODER_H

#include <QObject>
#include <QImage>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <opencv2/opencv.hpp>

#include "hk_camera/hk_camera.h"

namespace hk_camera
{

struct DecoderOptions
{
    int open_timeout_ms = 3000;
    int read_timeout_ms = 2000;
    int reconnect_interval_ms = 5000;
};

struct DecoderHealth
{
    bool requested = false;
    bool connected = false;
    uint64_t connection_attempts = 0;
    uint64_t frames = 0;
    int64_t frame_age_ms = -1;
    std::string last_error;
};

// The real implementation uses OpenCV/FFmpeg. Injection lets recovery tests
// exercise disconnects and stalls without contacting a camera or its SDK.
class VideoSource
{
public:
    virtual ~VideoSource() = default;
    virtual bool open(const std::string& url, const DecoderOptions& options) = 0;
    virtual bool read(cv::Mat& frame) = 0;
};

/**
 * @brief 基于 OpenCV RTSP 的视频解码器
 *
 * 不依赖 PlayM4/libPlayCtrl — 直接用 OpenCV 的 FFmpeg 后端从 RTSP 取流解码。
 * 在独立 std::thread 中循环读取, 通过 Qt 信号显示或发布视频帧。
 *
 * 用法:
 *   decoder = new HKDecoder;
 *   decoder->start(login_info, channel);
 *   // connect decoder->frameDecoded → 你的显示/发布槽
 *   decoder->stop();
 */
class HKDecoder : public QObject
{
    Q_OBJECT

public:
    using SourceFactory = std::function<std::unique_ptr<VideoSource>()>;
    explicit HKDecoder(QObject* parent = nullptr);
    HKDecoder(SourceFactory factory, QObject* parent = nullptr);
    ~HKDecoder();

    /// 启动 RTSP 取流 (在后台线程中循环读取)
    bool start(const CameraLoginInfo& info, int channel = 1,
               DecoderOptions options = {});
    void stop();
    bool isRunning() const;
    DecoderHealth health() const;

signals:
    void frameDecoded(const QImage& image);

private:
    void captureLoop();
    void recordFailure(const char* reason);
    static std::string buildRtspUrl(const CameraLoginInfo& info, int channel);

    std::atomic<bool>    running_{false};
    std::thread           thread_;
    CameraLoginInfo      login_info_;
    int                  channel_ = 1;
    DecoderOptions       options_;
    SourceFactory        source_factory_;
    std::mutex           lifecycle_mutex_, retry_mutex_;
    std::condition_variable retry_condition_;
    mutable std::mutex   health_mutex_;
    DecoderHealth        health_;
    std::chrono::steady_clock::time_point last_frame_{};
};

} // namespace hk_camera

#endif // HK_DECODER_H
