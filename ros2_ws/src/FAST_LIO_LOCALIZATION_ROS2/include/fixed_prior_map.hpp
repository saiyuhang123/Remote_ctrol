#pragma once
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <stdexcept>
#include <string>

namespace fast_lio_prior {
// Cache the immutable map in map coordinates. Each re-localization derives a new
// LIO-frame copy from this original, never from an already transformed copy.
template<class PointT> class FixedMap {
    using Cloud = pcl::PointCloud<PointT>;
    typename Cloud::Ptr map_{new Cloud};
public:
    FixedMap(const std::string &path, double voxel) {
        if (!std::isfinite(voxel) || voxel <= 0.)
            throw std::runtime_error("Prior map voxel size must be positive");
        typename Cloud::Ptr raw(new Cloud);
        if (path.empty() || pcl::io::loadPCDFile<PointT>(path, *raw) < 0)
            throw std::runtime_error("Cannot load prior PCD: " + path);
        typename Cloud::Ptr finite(new Cloud);
        for (const auto &p : raw->points)
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
                finite->push_back(p);
        pcl::VoxelGrid<PointT> filter;
        filter.setInputCloud(finite);
        filter.setLeafSize(voxel, voxel, voxel);
        filter.filter(*map_);
        if (map_->size() < 5) throw std::runtime_error("Prior PCD has fewer than five usable points");
    }
    size_t size() const { return map_->size(); }
    typename Cloud::Ptr in_lio_frame(const Eigen::Vector3d &t, Eigen::Quaterniond q) const {
        if (!t.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-6)
            throw std::runtime_error("Invalid prior-map alignment pose");
        q.normalize();
        Eigen::Isometry3d map_from_lio = Eigen::Isometry3d::Identity();
        map_from_lio.linear() = q.toRotationMatrix();
        map_from_lio.translation() = t;
        typename Cloud::Ptr transformed(new Cloud);
        const Eigen::Matrix4f lio_from_map = map_from_lio.inverse().matrix().cast<float>();
        pcl::transformPointCloud(*map_, *transformed, lio_from_map);
        return transformed;
    }
};
} // namespace fast_lio_prior
