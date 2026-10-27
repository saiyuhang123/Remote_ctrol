#include "hk_camera/image_message.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

namespace hk_camera
{

std::unique_ptr<sensor_msgs::msg::Image> makeImageMessage(
    const QImage& image, const std_msgs::msg::Header& header)
{
    if (image.isNull()) return nullptr;
    // The decoder supplies BGR888. Qt-scaled previews may use another format;
    // convert those small preview images only when necessary.
    const QImage pixels = image.format() == QImage::Format_BGR888 ||
                          image.format() == QImage::Format_RGB888
        ? image : image.convertToFormat(QImage::Format_RGB888);
    const char* encoding = pixels.format() == QImage::Format_BGR888 ? "bgr8" : "rgb8";
    const cv::Mat mat(pixels.height(), pixels.width(), CV_8UC3,
                      const_cast<uchar*>(pixels.constBits()),
                      static_cast<size_t>(pixels.bytesPerLine()));
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, encoding, mat).toImageMsg(*message);
    return message;
}

} // namespace hk_camera
