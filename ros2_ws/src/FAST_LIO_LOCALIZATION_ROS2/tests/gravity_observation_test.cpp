#include "gravity_observation.hpp"
#include <iostream>

int main() {
    using namespace Eigen;
    const Matrix3d R = (AngleAxisd(.31, Vector3d::UnitZ()) *
                       AngleAxisd(-.17, Vector3d::UnitY()) *
                       AngleAxisd(.09, Vector3d::UnitX())).toRotationMatrix();
    const Vector3d up = Vector3d(.02, -.01, 1).normalized();
    const Matrix3d H = fast_lio_gravity::upJacobian(R, up);
    for (int i=0; i<3; ++i) {
        const Matrix3d perturbed = R * AngleAxisd(1e-7, Vector3d::Unit(i)).toRotationMatrix();
        const Vector3d numeric = (perturbed.transpose()*up - R.transpose()*up)/1e-7;
        if ((numeric-H.col(i)).norm()>1e-6) return 1;
    }
    const Vector3d n = R.transpose()*up;
    if ((H*n).norm()>1e-12) return 2;
    const Quaterniond q1(R), q2(AngleAxisd(1.4, Vector3d::UnitZ())*q1);
    if ((fast_lio_gravity::bodyUp(q1)-fast_lio_gravity::bodyUp(q2)).norm()>1e-12) return 3;
    std::cout << "Gravity observation: Jacobian and yaw invariance passed\n";
    return 0;
}
