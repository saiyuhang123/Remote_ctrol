#ifndef HK_IMAGE_PREVIEW_H
#define HK_IMAGE_PREVIEW_H

#include <QImage>
#include <chrono>

namespace hk_camera
{

class ImageRateGate
{
public:
    bool due(bool has_subscribers, double fps,
             std::chrono::steady_clock::time_point now);

private:
    std::chrono::steady_clock::time_point last_{};
};

bool validRawPublishFps(double fps);
bool validPreviewSettings(int max_width, double fps);
QImage makePreview(const QImage& image, int max_width);

} // namespace hk_camera

#endif // HK_IMAGE_PREVIEW_H
