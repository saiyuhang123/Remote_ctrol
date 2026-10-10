#pragma once

namespace hk_camera::sdk
{
bool load(const char* directory);
bool isLoaded();
void* symbol(const char* name);
} // namespace hk_camera::sdk

// Keep SDK calls typed against the vendor header while loading the libraries
// locally. libHCCore exports a different tinyxml2 ABI from ROS/Fast DDS.
#define HK_SDK_CALL(function) \
    ([] { \
        static const auto call = reinterpret_cast<decltype(&function)>( \
            hk_camera::sdk::symbol(#function)); \
        return call; \
    }())
