#ifndef HK_DECODER_H
#define HK_DECODER_H

#include <QObject>
#include <QImage>
#include <atomic>
#include <thread>
#include <string>
#include <opencv2/opencv.hpp>

#include "hk_camera/hk_camera.h"

namespace hk_camera
{

/**
 * @brief 基于 OpenCV RTSP 的视频解码器
 *
 * 不依赖 PlayM4/libPlayCtrl — 直接用 OpenCV 的 FFmpeg 后端从 RTSP 取流解码。
 * 在独立 std::thread 中循环读取, 通过 Qt 信号投递到主线程。
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
    explicit HKDecoder(QObject* parent = nullptr);
    ~HKDecoder();

    /// 启动 RTSP 取流 (在后台线程中循环读取)
    bool start(const CameraLoginInfo& info, int channel = 1);
    void stop();
    bool isRunning() const;

signals:
    void frameDecoded(const QImage& image);

private:
    void captureLoop();
    void captureLoopHW();     // GStreamer 硬解码路径
    static std::string buildRtspUrl(const CameraLoginInfo& info, int channel);
    static std::string buildGstPipeline(const std::string& url);

    std::atomic<bool>    running_{false};
    std::thread           thread_;
    CameraLoginInfo      login_info_;
    int                  channel_ = 1;
};

} // namespace hk_camera

#endif // HK_DECODER_H
