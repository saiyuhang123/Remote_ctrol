#include <gtest/gtest.h>
#include <QColor>
#include <cv_bridge/cv_bridge.h>
#include "hk_camera/image_message.h"
#include "hk_camera/image_preview.h"

TEST(ImageMessage, PreservesColorsAndOwnsDataAcrossFormats)
{
    for (const auto format : {QImage::Format_BGR888, QImage::Format_RGB888,
                              QImage::Format_ARGB32}) {
        // An odd row size exercises Qt padding versus tightly packed ROS rows.
        QImage source(3, 2, format);
        source.fill(Qt::green);
        source.setPixelColor(0, 0, Qt::red);
        source.setPixelColor(2, 1, Qt::blue);
        std_msgs::msg::Header header;
        header.frame_id = "hk_camera";
        header.stamp.sec = 123;
        auto message = hk_camera::makeImageMessage(source, header);
        ASSERT_NE(message, nullptr);
        EXPECT_EQ(message->encoding, format == QImage::Format_BGR888 ? "bgr8" : "rgb8");
        EXPECT_EQ(message->step, 9u);
        EXPECT_EQ(message->data.size(), 18u);
        EXPECT_EQ(message->header.frame_id, "hk_camera");
        EXPECT_EQ(message->header.stamp.sec, 123);
        source.fill(Qt::black);
        const auto bgr = cv_bridge::toCvCopy(*message, "bgr8")->image;
        EXPECT_EQ(bgr.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 255));
        EXPECT_EQ(bgr.at<cv::Vec3b>(1, 2), cv::Vec3b(255, 0, 0));
    }
}

TEST(ImageMessage, PreviewPreservesColorAndAspectRatio)
{
    QImage source(1920, 1080, QImage::Format_BGR888);
    source.fill(QColor(220, 30, 10));
    const auto preview = hk_camera::makePreview(source, 640);
    const auto message = hk_camera::makeImageMessage(preview, std_msgs::msg::Header());
    ASSERT_NE(message, nullptr);
    EXPECT_EQ(message->width, 640u);
    EXPECT_EQ(message->height, 360u);
    const auto bgr = cv_bridge::toCvCopy(*message, "bgr8")->image;
    EXPECT_EQ(bgr.at<cv::Vec3b>(100, 100), cv::Vec3b(10, 30, 220));
}

TEST(ImageMessage, NullImageDoesNotProduceMessage)
{
    EXPECT_EQ(hk_camera::makeImageMessage(QImage(), std_msgs::msg::Header()), nullptr);
}
