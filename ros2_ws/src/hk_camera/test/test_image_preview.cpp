#include <gtest/gtest.h>
#include <QImage>
#include <limits>
#include "hk_camera/image_preview.h"

using Clock = std::chrono::steady_clock;
using hk_camera::ImageRateGate;

TEST(ImageRateGate, RawFramesRequireSubscribers)
{
    ImageRateGate gate;
    const auto start = Clock::now();
    EXPECT_FALSE(gate.due(false, 0., start));
    EXPECT_TRUE(gate.due(true, 0., start));
    EXPECT_TRUE(gate.due(true, 0., start + std::chrono::milliseconds(1)));
}

TEST(ImageRateGate, PreviewUsesLatestRateAndResetsWithoutSubscribers)
{
    ImageRateGate gate;
    const auto start = Clock::now();
    EXPECT_TRUE(gate.due(true, 5., start));
    EXPECT_FALSE(gate.due(true, 5., start + std::chrono::milliseconds(100)));
    EXPECT_TRUE(gate.due(true, 5., start + std::chrono::milliseconds(200)));
    EXPECT_FALSE(gate.due(false, 5., start + std::chrono::milliseconds(201)));
    EXPECT_TRUE(gate.due(true, 5., start + std::chrono::milliseconds(202)));
}

TEST(ImagePreview, ScalesDownWithoutChangingAspectRatio)
{
    const QImage source(1280, 720, QImage::Format_RGB888);
    const auto preview = hk_camera::makePreview(source, 640);
    ASSERT_FALSE(preview.isNull());
    EXPECT_EQ(preview.size(), QSize(640, 360));
}

TEST(ImagePreview, DoesNotUpscaleAndValidatesSettings)
{
    const QImage source(320, 180, QImage::Format_RGB888);
    EXPECT_EQ(hk_camera::makePreview(source, 640).size(), source.size());
    EXPECT_TRUE(hk_camera::validPreviewSettings(640, 5.));
    EXPECT_FALSE(hk_camera::validPreviewSettings(159, 5.));
    EXPECT_FALSE(hk_camera::validPreviewSettings(640, 0.));
    EXPECT_FALSE(hk_camera::validPreviewSettings(640, 31.));
    EXPECT_FALSE(hk_camera::validPreviewSettings(640,
        std::numeric_limits<double>::quiet_NaN()));
}
