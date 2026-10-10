#include "hk_camera/hk_decoder.h"
#include <QUrl>
#include <cstdio>

namespace hk_camera
{

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

HKDecoder::HKDecoder(QObject* parent) : QObject(parent) {}

HKDecoder::~HKDecoder()
{
    stop();
}

bool HKDecoder::start(const CameraLoginInfo& info, int channel)
{
    if (running_) return true;
    // A failed/finished capture thread must be joined before starting again.
    if (thread_.joinable()) thread_.join();
    login_info_ = info;
    channel_ = channel;
    running_ = true;
    thread_ = std::thread(&HKDecoder::captureLoop, this);
    return true;
}

void HKDecoder::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

bool HKDecoder::isRunning() const
{
    return running_;
}

void HKDecoder::captureLoop()
{
    try
    {
        // FFmpeg works on x86_64 and does not require Jetson's nvvidconv plugin.
        cv::VideoCapture cap;
        if (!cap.open(buildRtspUrl(login_info_, channel_), cv::CAP_FFMPEG,
                      {cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 10000,
                       cv::CAP_PROP_READ_TIMEOUT_MSEC, 5000}))
        {
            std::fprintf(stderr, "[Decoder] RTSP open failed (%s, channel=%d)\n",
                         login_info_.ip.c_str(), channel_);
            running_ = false;
            return;
        }
        std::printf("[Decoder] RTSP opened (%s, channel=%d)\n",
                    login_info_.ip.c_str(), channel_);

        int frame_count = 0;
        cv::Mat frame;
        while (running_)
        {
            if (!cap.read(frame) || frame.empty())
            {
                std::fprintf(stderr, "[Decoder] RTSP frame read failed\n");
                break;
            }

            cv::Mat rgb;
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
            QImage image(rgb.data, rgb.cols, rgb.rows,
                         static_cast<int>(rgb.step), QImage::Format_RGB888);
            emit frameDecoded(image.copy());

            ++frame_count;
            if (frame_count == 1 || frame_count % 150 == 0)
            {
                std::printf("[Decoder] frame=%d, size=%dx%d\n",
                            frame_count, frame.cols, frame.rows);
                std::fflush(stdout);
            }
        }
        cap.release();
        std::printf("[Decoder] stream stopped, frames=%d\n", frame_count);
    }
    catch (const cv::Exception&)
    {
        // OpenCV exception details may include the RTSP URL and credentials.
        std::fprintf(stderr, "[Decoder] OpenCV failed to decode the RTSP stream\n");
    }
    running_ = false;
}

} // namespace hk_camera
