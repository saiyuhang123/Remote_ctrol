#include "hk_camera/image_preview.h"

#include <cmath>

namespace hk_camera
{

bool ImageRateGate::due(bool has_subscribers, double fps,
                        std::chrono::steady_clock::time_point now)
{
    if (!has_subscribers)
    {
        last_ = {};
        return false;
    }
    if (fps > 0. && last_.time_since_epoch() != std::chrono::steady_clock::duration::zero() &&
        now - last_ < std::chrono::duration<double>(1. / fps))
        return false;
    last_ = now;
    return true;
}

bool validPreviewSettings(int max_width, double fps)
{
    return max_width >= 160 && max_width <= 4096 &&
           std::isfinite(fps) && fps >= .5 && fps <= 30.;
}

QImage makePreview(const QImage& image, int max_width)
{
    if (image.isNull() || image.width() <= max_width) return image;
    return image.scaledToWidth(max_width, Qt::SmoothTransformation);
}

} // namespace hk_camera
