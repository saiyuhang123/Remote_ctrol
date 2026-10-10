#include "hk_sdk_loader.h"
#include <dlfcn.h>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>

namespace hk_camera::sdk
{
namespace
{
void* sdk_handle = nullptr;
std::mutex load_mutex;
}

bool load(const char* directory)
{
    std::lock_guard<std::mutex> lock(load_mutex);
    if (sdk_handle) return true;
    // Preload direct dependencies by absolute path so LD_LIBRARY_PATH is not
    // needed. Keep vendor symbols local and bind SDK internals to their own ABI.
    constexpr int flags = RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND;
    for (const char* library : {"libhpr.so", "libHCCore.so", "libhcnetsdk.so"})
    {
        const std::string path = std::string(directory) + "/" + library;
        void* handle = dlopen(path.c_str(), flags);
        if (!handle)
        {
            std::fprintf(stderr, "[HK SDK] Cannot load %s: %s\n", library, dlerror());
            return false;
        }
        if (std::string(library) == "libhcnetsdk.so") sdk_handle = handle;
    }
    // Handles stay valid until process exit; HKCamera::cleanup stops SDK workers.
    return true;
}

bool isLoaded()
{
    return sdk_handle != nullptr;
}

void* symbol(const char* name)
{
    if (!sdk_handle) throw std::runtime_error("HK SDK is not loaded");
    void* address = dlsym(sdk_handle, name);
    if (!address) throw std::runtime_error(std::string("Missing HK SDK function: ") + name);
    return address;
}
} // namespace hk_camera::sdk
