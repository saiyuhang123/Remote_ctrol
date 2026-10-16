#include "hk_camera/ptz_codec.h"
#include <gtest/gtest.h>
#include <limits>

TEST(PtzCodec, UsesSdkPackedDecimalTenths) {
    EXPECT_EQ(*hk_camera::ptz::encode(175.), 0x1750);
    EXPECT_EQ(*hk_camera::ptz::encode(78.9), 0x0789);
    EXPECT_EQ(*hk_camera::ptz::encode(110.), 0x1100);
    EXPECT_EQ(*hk_camera::ptz::encode(1.), 0x0010);
    for (double value : {0., 1., 2.5, 78.9, 175., 360., 999.9})
        EXPECT_NEAR(*hk_camera::ptz::decode(*hk_camera::ptz::encode(value)), value, .001);
}
TEST(PtzCodec, RejectsInvalidNumbersAndDeviceData) {
    EXPECT_FALSE(hk_camera::ptz::encode(-.1));
    EXPECT_FALSE(hk_camera::ptz::encode(1000));
    EXPECT_FALSE(hk_camera::ptz::encode(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(hk_camera::ptz::decode(0x1a00));
    EXPECT_FALSE(hk_camera::ptz::validPose(361, 0, 1));
    EXPECT_FALSE(hk_camera::ptz::validPose(0, -1, 1));
    EXPECT_FALSE(hk_camera::ptz::validPose(0, 0, .9));
    EXPECT_TRUE(hk_camera::ptz::validPose(0, 350, 2));
}
TEST(PtzCodec, HandlesUnreportedAndWrappedLimits) {
    EXPECT_TRUE(hk_camera::ptz::withinScope(0x1750, 0, 0));
    EXPECT_TRUE(hk_camera::ptz::withinScope(0x0020, 0x0010, 0x0230));
    EXPECT_FALSE(hk_camera::ptz::withinScope(0x0240, 0x0010, 0x0230));
    EXPECT_TRUE(hk_camera::ptz::withinScope(0x3500, 0x2700, 0x0900));
    EXPECT_TRUE(hk_camera::ptz::withinScope(0x0500, 0x2700, 0x0900));
    EXPECT_FALSE(hk_camera::ptz::withinScope(0x1800, 0x2700, 0x0900));
}
