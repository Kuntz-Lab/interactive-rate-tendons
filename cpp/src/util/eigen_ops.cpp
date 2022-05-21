#include "util/eigen_ops.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace E = Eigen;

namespace util {

Eigen::Quaterniond quat_from_two_vectors(
    const E::Vector3d &a, const E::Vector3d &b)
{
  return E::Quaterniond::FromTwoVectors(a, b);
}

} // end of namespace util
