#include "fixed_prior_map.hpp"
#include <pcl/point_types.h>
#include <filesystem>
#include <iostream>
#include <limits>
#include <unistd.h>

void check(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
}
int main() {
    using P = pcl::PointXYZ;
    const auto path = std::filesystem::temp_directory_path() /
        ("fastlio_prior_test_" + std::to_string(getpid()) + ".pcd");
    pcl::PointCloud<P> cloud;
    for (int i=0; i<20; ++i) cloud.push_back(P(float(i),float(i%3),float(i%5)));
    pcl::io::savePCDFileBinary(path.string(), cloud);
    fast_lio_prior::FixedMap<P> map(path.string(), .01);
    const auto identity = map.in_lio_frame(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    const Eigen::Vector3d translation(3.,-4.,2.);
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(.73, Eigen::Vector3d(1,2,3).normalized()));
    const auto moved = map.in_lio_frame(translation, rotation);
    check(map.size()==20 && moved->size()==20, "Map point count changed");
    for (size_t i=0; i<20; ++i) {
        const Eigen::Vector3d restored = rotation * moved->points[i].getVector3fMap().cast<double>() + translation;
        check((restored-identity->points[i].getVector3fMap().cast<double>()).norm()<1e-5,
              "map->LIO inverse or 6DOF transform is wrong");
    }
    const auto again = map.in_lio_frame(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    for (size_t i=0; i<20; ++i)
        check((again->points[i].getVector3fMap()-identity->points[i].getVector3fMap()).norm()<1e-6,
              "Re-localization mutated the original map");
    bool rejected=false;
    try { map.in_lio_frame(translation,Eigen::Quaterniond(0,0,0,0)); } catch (const std::exception&) { rejected=true; }
    check(rejected,"Accepted zero quaternion");
    rejected=false;
    try { fast_lio_prior::FixedMap<P> invalid(path.string()+".missing",.1); } catch (const std::exception&) { rejected=true; }
    check(rejected,"Accepted missing PCD");
    rejected=false;
    try { map.in_lio_frame(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,0),rotation); }
    catch (const std::exception&) { rejected=true; }
    check(rejected,"Accepted NaN alignment");
    std::filesystem::remove(path);
    std::cout << "Fixed prior map transforms, immutability and rejection checks passed\n";
}
