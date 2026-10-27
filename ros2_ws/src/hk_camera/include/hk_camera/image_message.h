#ifndef HK_IMAGE_MESSAGE_H
#define HK_IMAGE_MESSAGE_H

#include <QImage>
#include <memory>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace hk_camera
{

// Preserve native BGR/RGB layout and copy pixels into an owned ROS message.
std::unique_ptr<sensor_msgs::msg::Image> makeImageMessage(
    const QImage& image, const std_msgs::msg::Header& header);

} // namespace hk_camera

#endif // HK_IMAGE_MESSAGE_H
