#ifndef EIGEN_OPS_H
#define EIGEN_OPS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace util {

Eigen::Quaterniond quat_from_two_vectors(
    const Eigen::Vector3d &a, const Eigen::Vector3d &b);

inline Eigen::Quaterniond quat_from_zaxis(const Eigen::Vector3d &a) {
  return quat_from_two_vectors(Eigen::Vector3d{0, 0, 1}, a);
}

} // end of namespace util

#endif // EIGEN_OPS_H
