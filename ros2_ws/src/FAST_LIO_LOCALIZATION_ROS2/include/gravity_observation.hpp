#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fast_lio_gravity {
inline Eigen::Vector3d bodyUp(const Eigen::Quaterniond &orientation) {
    return orientation.normalized().conjugate() * Eigen::Vector3d::UnitZ();
}
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d A;
    A << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return A;
}
// Right SO(3) perturbation R*Exp(delta): d(R^T * worldUp)/d(delta).
inline Eigen::Matrix3d upJacobian(const Eigen::Matrix3d &R,
                                 const Eigen::Vector3d &worldUp) {
    return skew(R.transpose() * worldUp);
}
}
