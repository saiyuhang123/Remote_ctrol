#include "gcan_udp_frame.h"
#include <algorithm>
#include <cstdlib>
#include <iostream>

using USB2CAN::VCI_CAN_OBJ;
using USB2CAN::encode_gcan_udp_frame;

void check(bool valid, const char *description)
{
    if (!valid) {
        std::cerr << "FAIL: " << description << '\n';
        std::exit(1);
    }
}

int main()
{
    VCI_CAN_OBJ frame{};
    frame.ID = 0x121;
    frame.DataLen = 8;
    frame.Data[0] = 0x45;
    frame.Data[3] = 4;
    const std::array<std::uint8_t, 13> expected{
        0x28, 0x00, 0x00, 0x01, 0x21, 0x45, 0, 0, 4, 0, 0, 0, 0};
    check(encode_gcan_udp_frame(frame) == expected, "standard control frame requires UDP bit and big-endian ID");

    frame.ExternFlag = 1;
    frame.ID = 0x17904001;
    auto packet = encode_gcan_udp_frame(frame);
    check(packet[0] == 0xa8 && packet[1] == 0x17 && packet[2] == 0x90 &&
          packet[3] == 0x40 && packet[4] == 0x01, "extended UDP header and full 29-bit ID");
    frame.RemoteFlag = 1;
    packet = encode_gcan_udp_frame(frame);
    check(packet[0] == 0xe8 && std::all_of(packet.begin() + 5, packet.end(),
          [](std::uint8_t value) { return value == 0; }), "remote flag and zero data bytes");

    frame.RemoteFlag = 0;
    frame.DataLen = 3;
    packet = encode_gcan_udp_frame(frame);
    check(packet[0] == 0xa3 && std::all_of(packet.begin() + 8, packet.end(),
          [](std::uint8_t value) { return value == 0; }), "short CAN payload is zero-padded");

    frame.DataLen = 9;
    try {
        encode_gcan_udp_frame(frame);
        check(false, "oversized DLC must be rejected");
    } catch (const std::invalid_argument &) {}
    frame.DataLen = 8;
    frame.ExternFlag = 0;
    try {
        encode_gcan_udp_frame(frame);
        check(false, "standard ID wider than 11 bits must be rejected");
    } catch (const std::invalid_argument &) {}
    std::cout << "PASS: GCAN UDP protocol cases; no hardware packet sent\n";
}
