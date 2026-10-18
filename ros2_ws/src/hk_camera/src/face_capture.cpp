#include "hk_camera/face_capture.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace hk_camera
{
namespace
{
constexpr size_t max_image_bytes = 16 * 1024 * 1024;

std::array<float, 4> rectValues(const NET_VCA_RECT& rect)
{
    return {rect.fX, rect.fY, rect.fWidth, rect.fHeight};
}

bool validRect(const std::array<float, 4>& rect)
{
    for (float value : rect)
        if (!std::isfinite(value) || value < 0 || value > 1) return false;
    return rect[0] < 1 && rect[1] < 1 && rect[2] > 0 && rect[3] > 0;
}

void writeBytes(const std::filesystem::path& path, const std::vector<uint8_t>& bytes)
{
    const auto temporary = path.string() + ".tmp";
    try
    {
        std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
        stream.exceptions(std::ios::failbit | std::ios::badbit);
        stream.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
        stream.close();
        std::filesystem::rename(temporary, path);
    }
    catch (...)
    {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        throw;
    }
}

void writeImage(const std::filesystem::path& path, const cv::Mat& image)
{
    std::vector<uint8_t> jpeg;
    if (!cv::imencode(".jpg", image, jpeg))
        throw std::runtime_error("Cannot encode capture JPEG");
    writeBytes(path, jpeg);
}

std::string jsonString(const std::string& value)
{
    std::ostringstream out;
    out << '"';
    for (unsigned char ch : value)
    {
        if (ch == '"' || ch == '\\') out << '\\' << ch;
        else if (ch < 32) out << "\\u" << std::hex << std::setw(4)
                              << std::setfill('0') << static_cast<int>(ch) << std::dec;
        else out << ch;
    }
    return out.str() + '"';
}
} // namespace

bool copyFaceCapture(const char* data, size_t size, FaceCaptureEvent& event,
                     std::string& error)
{
    if (!data || size < sizeof(NET_VCA_FACESNAP_RESULT))
    {
        error = "Truncated NET_VCA_FACESNAP_RESULT";
        return false;
    }
    NET_VCA_FACESNAP_RESULT native{};
    std::memcpy(&native, data, sizeof(native));
    if (native.dwSize != 0 && native.dwSize < sizeof(native))
    {
        error = "Invalid native face capture structure size";
        return false;
    }
    if (native.byUploadEventDataType != 0)
    {
        error = "URL face images are unsupported; request binary alarm images";
        return false;
    }
    if (native.dwFacePicLen > max_image_bytes || native.dwBackgroundPicLen > max_image_bytes ||
        (native.dwFacePicLen && !native.pBuffer1) ||
        (native.dwBackgroundPicLen && !native.pBuffer2))
    {
        error = "Invalid or oversized face capture image buffer";
        return false;
    }
    FaceCaptureEvent copied;
    copied.channel = native.struDevInfo.byChannel;
    copied.face_id = native.dwFacePicID;
    copied.target_id = native.struTargetInfo.dwID;
    copied.score = native.dwFaceScore;
    copied.camera_time = native.dwAbsTime;
    copied.end_mark = native.byAlarmEndMark == 1;
    copied.target_rect = rectValues(native.struTargetInfo.struRect);
    copied.crop_rect = rectValues(native.struRect);
    if (native.byAddInfo && native.pAddInfoBuffer)
    {
        NET_VCA_RECT face_in_crop{};
        std::memcpy(&face_in_crop, native.pAddInfoBuffer, sizeof(face_in_crop));
        copied.face_in_crop_rect = rectValues(face_in_crop);
    }
    if (native.dwFacePicLen)
        copied.face_jpeg.assign(native.pBuffer1, native.pBuffer1 + native.dwFacePicLen);
    if (native.dwBackgroundPicLen)
        copied.background_jpeg.assign(native.pBuffer2, native.pBuffer2 + native.dwBackgroundPicLen);
    event = std::move(copied);
    error.clear();
    return true;
}

bool normalizedRect(const std::array<float, 4>& rect, int width, int height,
                    cv::Rect& pixels)
{
    if (width <= 0 || height <= 0 || !validRect(rect)) return false;
    const int left = std::clamp(static_cast<int>(std::floor(rect[0] * width)), 0, width - 1);
    const int top = std::clamp(static_cast<int>(std::floor(rect[1] * height)), 0, height - 1);
    const int right = std::clamp(static_cast<int>(std::ceil((rect[0] + rect[2]) * width)), left + 1, width);
    const int bottom = std::clamp(static_cast<int>(std::ceil((rect[1] + rect[3]) * height)), top + 1, height);
    pixels = cv::Rect(left, top, right - left, bottom - top);
    return true;
}

FaceCaptureFiles saveFaceCapture(const FaceCaptureEvent& event,
                                const std::filesystem::path& directory,
                                const std::string& stem)
{
    FaceCaptureFiles result;
    cv::Mat background, face;
    if (!event.background_jpeg.empty())
    {
        background = cv::imdecode(event.background_jpeg, cv::IMREAD_COLOR);
        if (background.empty()) throw std::runtime_error("Invalid background JPEG in face alarm");
    }
    if (!event.face_jpeg.empty())
    {
        face = cv::imdecode(event.face_jpeg, cv::IMREAD_COLOR);
        if (face.empty()) throw std::runtime_error("Invalid face JPEG in face alarm");
    }

    result.rect = validRect(event.target_rect) ? event.target_rect : event.crop_rect;
    std::string rect_source = validRect(event.target_rect) ? "target" : "face_crop";
    if (validRect(event.crop_rect) && validRect(event.face_in_crop_rect))
    {
        const auto& crop = event.crop_rect;
        const auto& inner = event.face_in_crop_rect;
        result.rect = {crop[0] + inner[0] * crop[2], crop[1] + inner[1] * crop[3],
                       inner[2] * crop[2], inner[3] * crop[3]};
        rect_source = "face_in_crop";
    }

    std::filesystem::create_directories(directory);
    const auto base = std::filesystem::absolute(directory) / stem;
    if (!background.empty())
    {
        result.background_width = background.cols;
        result.background_height = background.rows;
        result.coordinate_width = background.cols;
        result.coordinate_height = background.rows;
        // Some cameras append a text banner below the original video image.
        // Their normalized detection coordinates exclude that banner.
        if (event.coordinate_width && event.coordinate_height)
        {
            const auto scene_height = std::llround(static_cast<double>(background.cols) *
                                                  event.coordinate_height / event.coordinate_width);
            if (scene_height > 0 && scene_height <= background.rows)
                result.coordinate_height = static_cast<uint32_t>(scene_height);
        }
        result.background_path = base.string() + "_background.jpg";
        writeBytes(result.background_path, event.background_jpeg);
        result.rect_valid = normalizedRect(result.rect, result.coordinate_width,
                                          result.coordinate_height, result.pixels);
        if (result.rect_valid)
        {
            result.annotated = background.clone();
            cv::rectangle(result.annotated, result.pixels, cv::Scalar(0, 255, 0), 3);
            std::string label = "face " + std::to_string(event.face_id);
            if (event.score <= 100) label += " score " + std::to_string(event.score);
            cv::putText(result.annotated, label,
                        cv::Point(result.pixels.x, std::max(20, result.pixels.y - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, .6, cv::Scalar(0, 255, 0), 2);
            result.annotated_path = base.string() + "_boxed.jpg";
            writeImage(result.annotated_path, result.annotated);
        }
        else result.warnings.emplace_back("No valid camera rectangle; background saved without a box");
        if (face.empty())
        {
            cv::Rect crop;
            if (normalizedRect(event.crop_rect, result.coordinate_width, result.coordinate_height, crop) ||
                normalizedRect(result.rect, result.coordinate_width, result.coordinate_height, crop))
                face = background(crop).clone();
        }
    }
    else result.warnings.emplace_back("No matching event background; no background box was drawn");

    if (!face.empty())
    {
        result.face_path = base.string() + "_face.jpg";
        if (!event.face_jpeg.empty()) writeBytes(result.face_path, event.face_jpeg);
        else writeImage(result.face_path, face);
    }
    else result.warnings.emplace_back("No face image or valid background crop in this event");

    result.metadata_path = base.string() + ".json";
    std::ostringstream metadata;
    metadata << "{\n  \"source\": \"hikvision_sdk_face_capture\",\n"
             << "  \"received_ns\": " << event.received_ns
             << ",\n  \"camera_time_raw\": " << event.camera_time
             << ",\n  \"channel\": " << event.channel
             << ",\n  \"face_id\": " << event.face_id
             << ",\n  \"target_id\": " << event.target_id
             << ",\n  \"score\": " << event.score
             << ",\n  \"end_mark\": " << (event.end_mark ? "true" : "false")
             << ",\n  \"rect_source\": " << jsonString(rect_source)
             << ",\n  \"rect_valid\": " << (result.rect_valid ? "true" : "false")
             << ",\n  \"rect_normalized\": [";
    for (size_t i = 0; i < result.rect.size(); ++i)
    {
        if (i) metadata << ", ";
        if (std::isfinite(result.rect[i])) metadata << result.rect[i];
        else metadata << "null";
    }
    metadata << "],\n  \"rect_pixels\": [" << result.pixels.x << ", " << result.pixels.y
             << ", " << result.pixels.width << ", " << result.pixels.height << "]"
             << ",\n  \"background_size\": [" << background.cols << ", " << background.rows << "]"
             << ",\n  \"coordinate_size\": [" << result.coordinate_width << ", " << result.coordinate_height << "]"
             << ",\n  \"face_path\": " << jsonString(result.face_path)
             << ",\n  \"background_path\": " << jsonString(result.background_path)
             << ",\n  \"annotated_path\": " << jsonString(result.annotated_path)
             << ",\n  \"warnings\": [";
    for (size_t i = 0; i < result.warnings.size(); ++i)
    {
        if (i) metadata << ", ";
        metadata << jsonString(result.warnings[i]);
    }
    metadata << "]\n}\n";
    const auto text = metadata.str();
    const auto temporary_metadata = result.metadata_path + ".tmp";
    writeBytes(temporary_metadata, std::vector<uint8_t>(text.begin(), text.end()));
    std::filesystem::rename(temporary_metadata, result.metadata_path);
    return result;
}
} // namespace hk_camera
