#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include "controlcan.h"

namespace USB2CAN
{
// GCAN-202/212 manual, CANet data stream: bit 7 = extended, bit 6 = RTR,
// bit 5 = UDP, low nibble = DLC; CAN ID is big-endian, payload is padded to 8.
inline std::array<std::uint8_t, 13> encode_gcan_udp_frame(const VCI_CAN_OBJ &frame)
{
    if (frame.DataLen > 8 || frame.ID > (frame.ExternFlag ? 0x1fffffffU : 0x7ffU)) {
        throw std::invalid_argument("Invalid CAN identifier or DLC for a GCAN UDP frame");
    }
    std::array<std::uint8_t, 13> packet{};
    packet[0] = static_cast<std::uint8_t>(0x20U | frame.DataLen |
        (frame.ExternFlag ? 0x80U : 0U) | (frame.RemoteFlag ? 0x40U : 0U));
    for (unsigned int index = 0; index < 4; ++index) {
        packet[index + 1] = static_cast<std::uint8_t>(frame.ID >> (24U - 8U * index));
    }
    if (!frame.RemoteFlag) {
        for (unsigned int index = 0; index < frame.DataLen; ++index) {
            packet[index + 5] = frame.Data[index];
        }
    }
    return packet;
}
}  // namespace USB2CAN
