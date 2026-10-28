#include "hk_camera/hk_decoder.h"
#include <QUrl>
#include <algorithm>
#include <cstdio>

namespace hk_camera
{

namespace
{
class OpenCVSource : public VideoSource
{
public:
    bool open(const std::string& url, const DecoderOptions& options) override
    {
        return capture_.open(url, cv::CAP_FFMPEG,
            {cv::CAP_PROP_OPEN_TIMEOUT_MSEC, options.open_timeout_ms,
             cv::CAP_PROP_READ_TIMEOUT_MSEC, options.read_timeout_ms});
    }
    bool read(cv::Mat& frame) override { return capture_.read(frame); }
private:
    cv::VideoCapture capture_;
};
}

std::string HKDecoder::buildRtspUrl(const CameraLoginInfo& info, int channel)
{
    QUrl url;
    url.setScheme("rtsp");
    url.setHost(QString::fromStdString(info.ip));
    url.setPort(554);
    url.setUserName(QString::fromStdString(info.username));
    url.setPassword(QString::fromStdString(info.password));
    url.setPath(QString("/h264/ch%1/main/av_stream").arg(channel));
    return url.toEncoded().toStdString();
}

HKDecoder::HKDecoder(QObject* parent)
    : HKDecoder([] { return std::make_unique<OpenCVSource>(); }, parent) {}

HKDecoder::HKDecoder(SourceFactory factory, QObject* parent)
    : QObject(parent), source_factory_(std::move(factory)) {}

HKDecoder::~HKDecoder()
{
    stop();
}

bool HKDecoder::start(const CameraLoginInfo& info, int channel, DecoderOptions options)
{
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    if (running_) return true;
    if (options.open_timeout_ms < 1 || options.read_timeout_ms < 1 ||
        options.reconnect_interval_ms < 1 || !source_factory_) return false;
    // A failed/finished capture thread must be joined before starting again.
    if (thread_.joinable()) thread_.join();
    login_info_ = info;
    channel_ = channel;
    options_ = options;
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        health_ = {};
        health_.requested = true;
        last_frame_ = {};
    }
    running_ = true;
    thread_ = std::thread(&HKDecoder::captureLoop, this);
    return true;
}

void HKDecoder::stop()
{
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    running_ = false;
    retry_condition_.notify_all();
    if (thread_.joinable()) thread_.join();
    std::lock_guard<std::mutex> lock(health_mutex_);
    health_.requested = false;
    health_.connected = false;
}

bool HKDecoder::isRunning() const
{
    return running_;
}

DecoderHealth HKDecoder::health() const
{
    std::lock_guard<std::mutex> lock(health_mutex_);
    auto result = health_;
    if (last_frame_.time_since_epoch().count())
        result.frame_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now()-last_frame_).count();
    return result;
}

void HKDecoder::recordFailure(const char* reason)
{
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        health_.connected = false;
        health_.last_error = reason;
    }
    // Never log an RTSP URL or exception text: either may contain credentials.
    std::fprintf(stderr, "[Decoder] %s (%s, channel=%d); reconnecting\n",
                 reason, login_info_.ip.c_str(), channel_);
}

void HKDecoder::captureLoop()
{
    int failures = 0;
    while (running_)
    {
        try
        {
            {
                std::lock_guard<std::mutex> lock(health_mutex_);
                ++health_.connection_attempts;
            }
            auto source = source_factory_();
            if (!source || !source->open(buildRtspUrl(login_info_, channel_), options_))
                recordFailure("open_failed");
            else
            {
                std::fprintf(stderr, "[Decoder] RTSP opened (%s, channel=%d)\n",
                             login_info_.ip.c_str(), channel_);
                while (running_)
                {
                    cv::Mat frame;
                    const auto started = std::chrono::steady_clock::now();
                    const bool read = source->read(frame);
                    if (!running_) break;
                    if (!read || frame.empty()) {
                        recordFailure("read_failed");
                        break;
                    }
                    if (std::chrono::steady_clock::now()-started >
                            std::chrono::milliseconds(options_.read_timeout_ms)) {
                        // A CPU/transport stall may return an old buffered frame
                        // instead of failing. Reopen to discard that backlog.
                        recordFailure("frame_stalled");
                        break;
                    }
                    failures = 0;
                    {
                        std::lock_guard<std::mutex> lock(health_mutex_);
                        health_.connected = true;
                        health_.last_error.clear();
                        ++health_.frames;
                        last_frame_ = std::chrono::steady_clock::now();
                    }
                    QImage image(frame.data, frame.cols, frame.rows,
                                 static_cast<int>(frame.step), QImage::Format_BGR888);
                    emit frameDecoded(image.copy());
                }
            }
        }
        catch (const std::exception&) { recordFailure("decode_error"); }
        if (!running_) break;
        const int delay_ms = std::min(options_.reconnect_interval_ms, 250 * (1 << failures));
        failures = std::min(failures+1, 5);
        std::unique_lock<std::mutex> lock(retry_mutex_);
        retry_condition_.wait_for(lock, std::chrono::milliseconds(delay_ms),
                                  [this] { return !running_; });
    }
}

} // namespace hk_camera
