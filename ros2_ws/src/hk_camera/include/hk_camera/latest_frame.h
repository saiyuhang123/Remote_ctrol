#ifndef HK_CAMERA_LATEST_FRAME_H
#define HK_CAMERA_LATEST_FRAME_H

#include <QImage>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>

namespace hk_camera
{

// One owned frame shared by independent readers. A slow ROS publisher cannot
// block acquisition or build a queue of old video frames for another reader.
class LatestFrame
{
public:
    struct Frame {
        QImage image;
        int64_t stamp_ns = 0;
        uint64_t sequence = 0;
        std::chrono::steady_clock::time_point received;
    };

    void put(const QImage& image, int64_t stamp_ns)
    {
        if (image.isNull()) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_) return;
            frame_ = {image, stamp_ns, ++sequence_, std::chrono::steady_clock::now()};
        }
        condition_.notify_all();
    }

    std::optional<Frame> waitNext(uint64_t& seen)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&] { return closed_ || (!frame_.image.isNull() && frame_.sequence != seen); });
        if (closed_) return std::nullopt;
        seen = frame_.sequence;
        return frame_;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_ = {};
    }

    void close()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
            frame_ = {};
        }
        condition_.notify_all();
    }

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    bool closed_ = false;
    uint64_t sequence_ = 0;
    Frame frame_;
};

} // namespace hk_camera
#endif
