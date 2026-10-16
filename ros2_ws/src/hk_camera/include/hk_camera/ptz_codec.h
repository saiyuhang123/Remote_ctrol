#pragma once
#include <cmath>
#include <cstdint>
#include <optional>

namespace hk_camera::ptz {
// HCNetSDK PTZ coordinates are packed decimal digits, in tenths of a degree/x.
inline std::optional<uint16_t> encode(double value) {
    if (!std::isfinite(value) || value < 0 || value > 999.9) return {};
    auto digits = static_cast<unsigned>(std::round(value * 10));
    uint16_t result = 0;
    for (unsigned shift = 0; shift < 16; shift += 4) {
        result |= (digits % 10) << shift;
        digits /= 10;
    }
    return result;
}
inline std::optional<double> decode(uint16_t value) {
    unsigned result = 0, scale = 1;
    for (unsigned shift = 0; shift < 16; shift += 4) {
        unsigned digit = (value >> shift) & 0xf;
        if (digit > 9) return {};
        result += digit * scale;
        scale *= 10;
    }
    return result / 10.;
}
inline bool validPose(double pan, double tilt, double zoom) {
    return encode(pan) && encode(tilt) && encode(zoom) &&
           pan <= 360 && tilt <= 360 && zoom >= 1;
}
inline bool withinScope(uint16_t value, uint16_t lower, uint16_t upper) {
    // Some cameras report 0..0 for axes whose limits are unavailable.
    if (lower == upper) return true;
    return lower < upper ? value >= lower && value <= upper : value >= lower || value <= upper;
}
} // namespace hk_camera::ptz
