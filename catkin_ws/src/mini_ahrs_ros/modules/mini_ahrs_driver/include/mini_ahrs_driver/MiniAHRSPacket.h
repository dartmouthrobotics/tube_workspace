#pragma once

#include <vector>
#include <cstdint>

namespace mini_ahrs_driver
{

struct HeaderData
{
    uint8_t message_type;
    uint8_t data_identifier;
    uint16_t message_length;
};

struct GetDeviceInfoCommandPacket
{
    // taken from the
    const std::vector<uint8_t> buffer {0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x12, 0x19, 0x00};
};

struct StartOrientationDataStreamCommandPacket
{
    const std::vector<uint8_t> buffer {
        0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x33, 0x3A, 0x00 
    };
};

struct StartUserDefineDataStreamCommandPacket
{
    const std::vector<uint8_t> buffer {
        0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x95, 0x9C, 0x00 
    };
};

struct StopDeviceCommandPacket
{
    const std::vector<uint8_t> buffer {
        0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0xFE, 0x05, 0x01
    };
};

struct ReadMiniAHRSParamsCommandPacket
{
    const std::vector<uint8_t> buffer {
        0xAA, 0x55, 0x00, 0x00, 0x07, 0x00, 0x41, 0x48, 0x00
    };
};

} // namespace mini_ahrs_driver
