#pragma once

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>

namespace fast_lio {
// PCL opens with O_TRUNC. Write on the same filesystem and publish only a
// completed file, so shutdown signals cannot empty a previously saved map.
template<class Writer>
void save_map_atomically(const std::filesystem::path &destination, Writer writer)
{
    const auto parent = destination.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::string temporary = (parent / ("." + destination.filename().string() + ".tmp-XXXXXX")).string();
    const int fd = ::mkstemp(temporary.data());
    if (fd < 0) throw std::system_error(errno, std::generic_category(), "Cannot create temporary PCD");
    ::close(fd);
    try {
        if (writer(temporary) != 0 || std::filesystem::file_size(temporary) == 0)
            throw std::runtime_error("PCD writer did not produce a complete map");
        std::filesystem::rename(temporary, destination);
    } catch (...) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        throw;
    }
}
} // namespace fast_lio
