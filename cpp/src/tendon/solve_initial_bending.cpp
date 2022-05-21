#include "solve_initial_bending.h"

#include "get_r_info.h"
#include "TendonSpecs.h"
#include <util/vector_ops.h>

namespace E = Eigen;

using V3 = E::Vector3d;
using M3 = E::Matrix3d;

namespace tendon {

std::tuple<V3, V3, int>
solve_initial_bending(
    const V3 &v_guess,
    const V3 &u_guess,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const M3 &K_bt,
    const M3 &K_se,
    const M3 &K_bt_inv,
    const M3 &K_se_inv,
    int iter_max,
    double residual_threshold,
    double dv_threshold,
    double du_threshold,
    double s_start,
    rInfo &info,
    rInfoCache &cache
) {
  int iters = 0;
  auto v = v_guess;
  auto u = u_guess;
  auto N_t = tendons.size();

  get_r_info2(tendons, s_start, info, cache);
  auto &[r, r_dot, r_ddot] = info;
  std::vector<M3> rhat(r.size());
  std::transform(r.begin(), r.end(), rhat.begin(), util::hat);
  for (iters = 0; iters < iter_max; ++iters) {
    M3 uhat = util::hat(u);
    V3 Ft = V3::Zero();
    V3 Lt = V3::Zero();

    for (decltype(N_t) k = 0; k < N_t; ++k) {
      V3 pi_dot_unit = (uhat * r[k] + r_dot[k] + v).normalized();
      Ft -= tau[k] * pi_dot_unit;
      Lt -= tau[k] * rhat[k] * pi_dot_unit;
    }

    auto n = K_se * (v - V3(0, 0, 1));
    auto m = K_bt * u;
    double residual = std::sqrt((n - Ft).squaredNorm() + (m - Lt).squaredNorm());
    if (residual < residual_threshold) {
      break; // exit early
    }

    V3 v_new = K_se_inv * Ft + V3(0, 0, 1);
    V3 u_new = K_bt_inv * Lt;
    if ((v_new - v).norm() < dv_threshold * v.norm() &&
        (u_new - u).norm() < du_threshold * u.norm())
    {
      break; // exit early
    }

    // update
    v = v_new;
    u = u_new;
  }

  return std::tuple{v, u, iters};
}

std::tuple<V3, V3, int>
solve_initial_bending_unopt(
    const V3 &v_guess,
    const V3 &u_guess,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const M3 &K_bt,
    const M3 &K_se,
    const M3 &K_bt_inv,
    const M3 &K_se_inv,
    int iter_max,
    double residual_threshold,
    double dv_threshold,
    double du_threshold,
    double s_start
) {
  rInfo info;
  rInfoCache cache;
  const auto N_t = tendons.size();
  info.resize(N_t);
  cache.resize(tendons);
  return solve_initial_bending(
      v_guess, u_guess, tendons, tau,
      K_bt, K_se, K_bt_inv, K_se_inv, iter_max,
      residual_threshold, dv_threshold, du_threshold,
      s_start, info, cache);
}

} // end of namespace tendon
