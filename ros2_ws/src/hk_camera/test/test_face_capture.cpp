#include "hk_camera/face_capture.h"
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <limits>
#include <unistd.h>

using namespace hk_camera;

namespace
{
std::vector<uint8_t> jpeg(int width, int height)
{
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(40, 80, 120));
    std::vector<uint8_t> bytes;
    cv::imencode(".jpg", image, bytes);
    return bytes;
}

class CaptureStorage : public testing::Test
{
protected:
    void SetUp() override
    {
        char name[] = "/tmp/hk_face_capture_test_XXXXXX";
        auto* created = mkdtemp(name);
        ASSERT_NE(created, nullptr);
        directory = created;
    }
    void TearDown() override
    {
        std::filesystem::remove_all(directory);
    }
    std::filesystem::path directory;
};
} // namespace

TEST(FaceAlarm, CopiesSdkOwnedImagesAndCoordinatesBeforeCallbackReturns)
{
    auto face = jpeg(50, 50);
    auto background = jpeg(200, 100);
    const auto original = background;
    NET_VCA_FACESNAP_ADDINFO additional{};
    additional.struFacePicRect = {.25f, .25f, .5f, .5f};
    NET_VCA_FACESNAP_RESULT native{};
    native.dwSize = sizeof(native);
    native.dwFacePicID = 7;
    native.dwFaceScore = 92;
    native.struDevInfo.byChannel = 1;
    native.struRect = {.25f, .25f, .5f, .5f};
    native.dwFacePicLen = face.size();
    native.pBuffer1 = face.data();
    native.dwBackgroundPicLen = background.size();
    native.pBuffer2 = background.data();
    native.byAddInfo = 1;
    native.pAddInfoBuffer = reinterpret_cast<BYTE*>(&additional);
    FaceCaptureEvent event;
    std::string error;
    ASSERT_TRUE(copyFaceCapture(reinterpret_cast<char*>(&native), sizeof(native), event, error));
    std::fill(background.begin(), background.end(), 0);
    std::fill(face.begin(), face.end(), 0);
    additional.struFacePicRect = {};
    EXPECT_EQ(event.background_jpeg, original);
    EXPECT_FALSE(cv::imdecode(event.face_jpeg, cv::IMREAD_COLOR).empty());
    EXPECT_EQ(event.face_id, 7u);
    EXPECT_EQ(event.score, 92u);
    EXPECT_EQ(event.face_in_crop_rect[2], .5f);
}

TEST(FaceAlarm, RejectsTruncatedNullAndOversizedBuffers)
{
    FaceCaptureEvent event;
    std::string error;
    EXPECT_FALSE(copyFaceCapture(nullptr, 0, event, error));
    NET_VCA_FACESNAP_RESULT native{};
    native.dwSize = sizeof(native);
    EXPECT_FALSE(copyFaceCapture(reinterpret_cast<char*>(&native), sizeof(native) - 1, event, error));
    native.dwFacePicLen = 1;
    EXPECT_FALSE(copyFaceCapture(reinterpret_cast<char*>(&native), sizeof(native), event, error));
    native.dwFacePicLen = 16 * 1024 * 1024 + 1;
    native.pBuffer1 = reinterpret_cast<BYTE*>(&native);
    EXPECT_FALSE(copyFaceCapture(reinterpret_cast<char*>(&native), sizeof(native), event, error));
}

TEST(FaceAlarm, DoesNotTreatUrlTextAsJpeg)
{
    NET_VCA_FACESNAP_RESULT native{};
    native.dwSize = sizeof(native);
    native.byUploadEventDataType = 1;
    FaceCaptureEvent event;
    std::string error;
    EXPECT_FALSE(copyFaceCapture(reinterpret_cast<char*>(&native), sizeof(native), event, error));
    EXPECT_NE(error.find("URL"), std::string::npos);
}

TEST(FaceRectangle, ClipsAtImageEdgesAndRejectsInvalidCoordinates)
{
    cv::Rect pixels;
    ASSERT_TRUE(normalizedRect({.875f, .875f, .5f, .5f}, 200, 100, pixels));
    EXPECT_EQ(pixels, cv::Rect(175, 87, 25, 13));
    EXPECT_FALSE(normalizedRect({0, 0, 0, 0}, 200, 100, pixels));
    EXPECT_FALSE(normalizedRect({-.1f, 0, .5f, .5f}, 200, 100, pixels));
    EXPECT_FALSE(normalizedRect({std::numeric_limits<float>::quiet_NaN(), 0, .5f, .5f}, 200, 100, pixels));
    EXPECT_FALSE(normalizedRect({0, 0, .5f, .5f}, 0, 100, pixels));
}

TEST_F(CaptureStorage, CropsFaceFromEventBackgroundAndPreservesOriginalJpeg)
{
    FaceCaptureEvent event;
    event.face_id = 42;
    event.score = 80;
    event.background_jpeg = jpeg(200, 100);
    event.crop_rect = {.25f, .25f, .5f, .5f};
    const auto files = saveFaceCapture(event, directory, "background_only");
    const auto face = cv::imread(files.face_path);
    EXPECT_EQ(face.cols, 100);
    EXPECT_EQ(face.rows, 50);
    ASSERT_TRUE(files.rect_valid);
    EXPECT_EQ(files.pixels, cv::Rect(50, 25, 100, 50));
    EXPECT_TRUE(std::filesystem::exists(files.annotated_path));
    EXPECT_TRUE(std::filesystem::exists(files.metadata_path));
    std::ifstream stream(files.background_path, std::ios::binary);
    const std::vector<uint8_t> preserved((std::istreambuf_iterator<char>(stream)), {});
    EXPECT_EQ(preserved, event.background_jpeg);
    const auto before = cv::imdecode(event.background_jpeg, cv::IMREAD_COLOR);
    EXPECT_GT(cv::norm(before, files.annotated, cv::NORM_L1), 0);
}

TEST_F(CaptureStorage, MapsFaceWithinHeadShoulderCropOntoBackground)
{
    FaceCaptureEvent event;
    event.background_jpeg = jpeg(200, 100);
    event.crop_rect = {.25f, .25f, .5f, .5f};
    event.face_in_crop_rect = {.25f, .25f, .5f, .5f};
    const auto files = saveFaceCapture(event, directory, "inner_rect");
    EXPECT_EQ(files.pixels, cv::Rect(75, 37, 50, 26));
    EXPECT_TRUE(files.rect_valid);
}

TEST_F(CaptureStorage, PrefersNativeTargetBoxOverExpandedFaceCrop)
{
    FaceCaptureEvent event;
    event.background_jpeg = jpeg(200, 100);
    event.crop_rect = {.25f, .25f, .5f, .5f};
    event.target_rect = {.375f, .375f, .125f, .125f};
    const auto files = saveFaceCapture(event, directory, "target_rect");
    EXPECT_EQ(files.pixels, cv::Rect(75, 37, 25, 13));
}

TEST_F(CaptureStorage, FaceOnlyEventDoesNotInventBackgroundOrBox)
{
    FaceCaptureEvent event;
    event.face_jpeg = jpeg(50, 50);
    event.crop_rect = {.25f, .25f, .5f, .5f};
    const auto files = saveFaceCapture(event, directory, "face_only");
    EXPECT_TRUE(std::filesystem::exists(files.face_path));
    EXPECT_TRUE(files.background_path.empty());
    EXPECT_TRUE(files.annotated_path.empty());
    EXPECT_FALSE(files.rect_valid);
    EXPECT_FALSE(files.warnings.empty());
}

TEST_F(CaptureStorage, InvalidRectangleSavesBackgroundWithoutFalseAnnotation)
{
    FaceCaptureEvent event;
    event.background_jpeg = jpeg(200, 100);
    event.crop_rect = {std::numeric_limits<float>::quiet_NaN(), 0, .5f, .5f};
    const auto files = saveFaceCapture(event, directory, "invalid_rect");
    EXPECT_FALSE(files.rect_valid);
    EXPECT_TRUE(std::filesystem::exists(files.background_path));
    EXPECT_TRUE(files.annotated_path.empty());
    std::ifstream stream(files.metadata_path);
    const std::string metadata((std::istreambuf_iterator<char>(stream)), {});
    EXPECT_NE(metadata.find("null"), std::string::npos);
    EXPECT_EQ(metadata.find("nan"), std::string::npos);
}

TEST_F(CaptureStorage, CorruptImageIsReportedInsteadOfSavedAsSuccessfulCapture)
{
    FaceCaptureEvent event;
    event.background_jpeg = {0, 1, 2, 3};
    EXPECT_THROW(saveFaceCapture(event, directory, "corrupt"), std::runtime_error);
    EXPECT_TRUE(std::filesystem::is_empty(directory));
}

TEST_F(CaptureStorage, AppendedBackgroundCaptionDoesNotMoveTheFaceBox)
{
    FaceCaptureEvent event;
    event.background_jpeg = jpeg(200, 120);
    event.coordinate_width = 200;
    event.coordinate_height = 100;
    event.crop_rect = {.25f, .25f, .5f, .5f};
    const auto files = saveFaceCapture(event, directory, "caption");
    EXPECT_EQ(files.pixels, cv::Rect(50, 25, 100, 50));
    EXPECT_EQ(files.background_height, 120u);
    EXPECT_EQ(files.coordinate_height, 100u);
    EXPECT_EQ(cv::imread(files.face_path).rows, 50);
}
