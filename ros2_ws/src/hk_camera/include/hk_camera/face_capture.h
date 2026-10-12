#pragma once

#include "HCNetSDK.h"
#include <opencv2/core.hpp>
#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace hk_camera
{
struct FaceCaptureEvent
{
    int user_id = -1;
    uint32_t channel = 0;
    uint32_t face_id = 0;
    uint32_t target_id = 0;
    uint32_t score = 255;
    uint32_t camera_time = 0;
    int64_t received_ns = 0;
    bool end_mark = false;
    uint32_t coordinate_width = 0;
    uint32_t coordinate_height = 0;
    std::array<float, 4> target_rect{};
    std::array<float, 4> crop_rect{};
    std::array<float, 4> face_in_crop_rect{};
    std::vector<uint8_t> face_jpeg;
    std::vector<uint8_t> background_jpeg;
};

// All SDK-owned buffers must be copied before returning from its callback.
bool copyFaceCapture(const char* data, size_t size, FaceCaptureEvent& event,
                     std::string& error);
bool normalizedRect(const std::array<float, 4>& rect, int width, int height,
                    cv::Rect& pixels);

struct FaceCaptureFiles
{
    std::string face_path;
    std::string background_path;
    std::string annotated_path;
    std::string metadata_path;
    std::array<float, 4> rect{};
    cv::Rect pixels;
    bool rect_valid = false;
    uint32_t background_width = 0;
    uint32_t background_height = 0;
    uint32_t coordinate_width = 0;
    uint32_t coordinate_height = 0;
    cv::Mat annotated;
    std::vector<std::string> warnings;
};

FaceCaptureFiles saveFaceCapture(const FaceCaptureEvent& event,
                                const std::filesystem::path& directory,
                                const std::string& stem);
} // namespace hk_camera
