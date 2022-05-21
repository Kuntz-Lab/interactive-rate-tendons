#ifndef SOLVE_INITIAL_BENDING_H
#define SOLVE_INITIAL_BENDING_H

#include "get_r_info.h"

#include <Eigen/Core>

#include <tuple>
#include <vector>

namespace tendon {

struct TendonSpecs;

// TODO: move to TendonRobot class
std::tuple<Eigen::Vector3d, Eigen::Vector3d, int>
solve_initial_bending(
    const Eigen::Vector3d &v_guess,
    const Eigen::Vector3d &u_guess,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const Eigen::Matrix3d &K_bt,
    const Eigen::Matrix3d &K_se,
    const Eigen::Matrix3d &K_bt_inv,
    const Eigen::Matrix3d &K_se_inv,
    int iter_max, // = 30,
    double residual_threshold, // = 5e-6,
    double dv_threshold, // = 1e-5,
    double du_threshold, // = 1e-6,
    double s_start, // = 0.0,
    rInfo &info,
    rInfoCache &cache
    );

// unoptimized version of solve_initial_bending()
std::tuple<Eigen::Vector3d, Eigen::Vector3d, int>
solve_initial_bending_unopt(
    const Eigen::Vector3d &v_guess,
    const Eigen::Vector3d &u_guess,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const Eigen::Matrix3d &K_bt,
    const Eigen::Matrix3d &K_se,
    const Eigen::Matrix3d &K_bt_inv,
    const Eigen::Matrix3d &K_se_inv,
    int iter_max = 30,
    double residual_threshold = 5e-6,
    double dv_threshold = 1e-5,
    double du_threshold = 1e-6,
    double s_start = 0.0
    );

} // end of namespace tendon

#endif // SOLVE_INITIAL_BENDING_H
