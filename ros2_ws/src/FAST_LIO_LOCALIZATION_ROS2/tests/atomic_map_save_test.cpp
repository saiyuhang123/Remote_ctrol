#include "atomic_map_save.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <csignal>
#include <fstream>
#include <iostream>
#include <sys/wait.h>

namespace fs = std::filesystem;
void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
std::string contents(const fs::path &path) {
    std::ifstream stream(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

int main() {
    std::string pattern = (fs::temp_directory_path() / "fastlio-map-test-XXXXXX").string();
    require(::mkdtemp(pattern.data()) != nullptr, "Cannot create test directory");
    const fs::path root(pattern), destination = root / "map.pcd";
    try {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.push_back(pcl::PointXYZ(1., 2., 3.));
        cloud.push_back(pcl::PointXYZ(4., 5., 6.));
        fast_lio::save_map_atomically(destination, [&cloud](const std::string &path) {
            return pcl::io::savePCDFileBinary(path, cloud);
        });
        pcl::PointCloud<pcl::PointXYZ> loaded;
        require(pcl::io::loadPCDFile(destination.string(), loaded) == 0 && loaded.size() == 2,
                "Saved PCD is unreadable");
        const auto original = contents(destination);
        try {
            fast_lio::save_map_atomically(destination, [](const std::string &path) {
                std::ofstream stream(path); stream << "partial";
                return -1;
            });
            throw std::logic_error("Failed writer should throw");
        } catch (const std::runtime_error &) {}
        require(contents(destination) == original, "Failed writer changed existing PCD");
        try {
            fast_lio::save_map_atomically(destination, [](const std::string &) { return 0; });
            throw std::logic_error("Empty output should throw");
        } catch (const std::runtime_error &) {}
        require(contents(destination) == original, "Empty output changed existing PCD");

        int ready[2];
        require(::pipe(ready) == 0, "Cannot create test pipe");
        const auto child = ::fork();
        require(child >= 0, "Cannot fork test writer");
        if (child == 0) {
            ::close(ready[0]);
            fast_lio::save_map_atomically(destination, [&](const std::string &path) {
                const auto result = pcl::io::savePCDFileBinary(path, cloud);
                const char marker = 'W';
                ::write(ready[1], &marker, 1);
                // Parent interrupts a completed temporary write before rename.
                for (;;) ::pause();
                return result;
            });
            ::_exit(2);
        }
        ::close(ready[1]);
        char marker = 0;
        const auto received = ::read(ready[0], &marker, 1);
        ::close(ready[0]);
        ::kill(child, SIGKILL);
        int status = 0; ::waitpid(child, &status, 0);
        require(received == 1 && marker == 'W', "Writer never reached replacement stage");
        require(contents(destination) == original, "Interrupted writer changed existing PCD");
        require(pcl::io::loadPCDFile(destination.string(), loaded) == 0 && loaded.size() == 2,
                "PCD became unreadable after interruption");
        fs::remove_all(root);
        std::cout << "Atomic PCD success, failure, empty output and interrupted writer checks passed\n";
        return 0;
    } catch (const std::exception &error) {
        fs::remove_all(root);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
