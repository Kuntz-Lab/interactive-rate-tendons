#ifndef TENDON_DERIV_H
#define TENDON_DERIV_H

#include "get_r_info.h"

#include <Eigen/Core>

#include <functional>
#include <vector>

namespace tendon {

using State = std::vector<double>;

// maps (t, p) -> force or torque vector per unit length distributed along the
// backbone
// t: parameter along backbone (i.e., percentage of full length)
// p: 3D point of the backbone at t.
using DistFLFunc =
  std::function<Eigen::Vector3d(double, const Eigen::Vector3d&)>;

// maps p -> point force or torque value at the specified point in space
//using PointFLFunc = std::function<Eigen::Vector3d(const Eigen::Vector3d&)>;

struct TendonSpecs;

void tendon_deriv(const State &x, State &dxdt, const double t,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const Eigen::Matrix3d &K_bt,
    const Eigen::Matrix3d &K_se,
    rInfo &rs,        // storage for get_r_info2() to avoid allocations
    rInfoCache &cache // storage for get_r_info2() to avoid allocations
);

/// Unoptimized version of tendon_deriv()
void tendon_deriv_unopt(const State &x, State &dxdt, const double t,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const Eigen::Matrix3d &K_bt,
    const Eigen::Matrix3d &K_se
);

void tendon_deriv(const State &x, State &dxdt, const double t,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const Eigen::Matrix3d &K_bt,
    const Eigen::Matrix3d &K_se,
    rInfo &rs,         // storage for get_r_info2() to avoid allocations
    rInfoCache &cache, // storage for get_r_info2() to avoid allocations
    const DistFLFunc &f_e,  // external force per length not from tendons
    const DistFLFunc &l_e   // external torque per length not from tendons
);

} // end of namespace tendon


#endif // TENDON_DERIV_H
