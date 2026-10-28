#include "hk_camera/latest_frame.h"
#include <gtest/gtest.h>
#include <future>

using namespace std::chrono_literals;

TEST(LatestFrame, SlowRawReaderDoesNotQueueOrBlockPreviewReader)
{
    hk_camera::LatestFrame frames;
    QImage image(2, 2, QImage::Format_BGR888);
    image.fill(Qt::red);
    uint64_t raw = 0, preview = 0;
    frames.put(image, 1);
    const auto held_raw = frames.waitNext(raw);
    ASSERT_TRUE(held_raw);
    ASSERT_TRUE(frames.waitNext(preview));
    // Keep the first raw frame alive as if its reliable DDS send is blocked.
    // Preview continues independently; raw gets only the latest on resuming.
    for (int index = 2; index <= 25; ++index) {
        image.fill(Qt::green);
        frames.put(image, index);
        const auto fresh = frames.waitNext(preview);
        ASSERT_TRUE(fresh);
        EXPECT_EQ(fresh->stamp_ns, index);
    }
    const auto fresh_raw = frames.waitNext(raw);
    ASSERT_TRUE(fresh_raw);
    EXPECT_EQ(fresh_raw->stamp_ns, 25);
    EXPECT_EQ(held_raw->image.pixelColor(0, 0), QColor(Qt::red));
    EXPECT_EQ(fresh_raw->image.pixelColor(0, 0), QColor(Qt::green));
}

TEST(LatestFrame, ClearDiscardsOldFrameAndCloseWakesEveryWaitingReader)
{
    hk_camera::LatestFrame frames;
    QImage image(2, 2, QImage::Format_BGR888);
    image.fill(Qt::red);
    frames.put(image, 1);
    frames.clear();
    auto first = std::async(std::launch::async, [&] {
        uint64_t seen = 0;
        return frames.waitNext(seen);
    });
    const auto before_new = first.wait_for(20ms);
    frames.put(image, 2);
    const auto frame = first.get();
    EXPECT_EQ(before_new, std::future_status::timeout);
    ASSERT_TRUE(frame);
    EXPECT_EQ(frame->stamp_ns, 2);
    auto waiting = [&] {
        uint64_t seen = frame->sequence;
        return frames.waitNext(seen);
    };
    auto raw = std::async(std::launch::async, waiting);
    auto preview = std::async(std::launch::async, waiting);
    frames.close();
    EXPECT_FALSE(raw.get());
    EXPECT_FALSE(preview.get());
}
