#include "tendon_deriv.h"
#include "TendonSpecs.h"
#include <util/vector_ops.h>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <algorithm>
#include <iterator>
#include <stdexcept>

namespace E = Eigen;

using util::hat;

namespace tendon {

using V3 = E::Vector3d;
using M3 = E::Matrix3d;
using V6 = E::Matrix<double, 6, 1>;
using M6 = E::Matrix<double, 6, 6>;

inline V6 linsolve(const M6 &M, const V6 &b) {
  // Solves x in Ax = b
  // this is by far the most expensive part of this method
  // Note: You only need to use ComputeThinU and ComputeThinV
  //   for a linear solve.
  //   But they only make a difference if the matrix is not
  //   square, which is not the case for us, M is 6x6
  V6 x =
    //M.bdcSvd(E::ComputeFullU | E::ComputeFullV).solve(xi);
    //M.bdcSvd(E::ComputeThinU | E::ComputeThinV).solve(xi);
    M.jacobiSvd(E::ComputeFullU | E::ComputeFullV).solve(b);
    //M.jacobiSvd(E::ComputeThinU | E::ComputeThinV).solve(xi);
  return x;
}

// This is the old method that was considerably slower
inline V6 linsubsolve1(const M3 &A, const M3 &B, const M3 &C, const M3 &D,
                       const V3 &a, const V3 &b)
{
  // Solves x in
  //   [A  B] x = [a]
  //   [C  D]     [b]
  M6 M;
  M.block<3, 3>(0, 0) = A;
  M.block<3, 3>(0, 3) = B;
  M.block<3, 3>(3, 0) = C;
  M.block<3, 3>(3, 3) = D;

  V6 v;
  v.head<3>() = a;
  v.tail<3>() = b;

  return linsolve(M, v);
}

// This is the new method that directly and analytically solves for the inverse
// of the 6x6 matrix.
inline V6 linsubsolve2(const M3 &A, const M3 &B, const M3 &C, const M3 &D,
                       const V3 &a, const V3 &b)
{
  // Solves x in
  //   [A  B] x = [a]
  //   [C  D]     [b]

  // according to wikipedia:
  //   https://en.wikipedia.org/wiki/Invertible_matrix#Blockwise_inversion
  // a matrix can be inverted by inverting submatrices
  M3 Ai  = A.inverse();
  M3 G   = D - C*Ai*B;
  M3 Gi  = G.inverse();
  M3 AiB = Ai * B;
  M3 CAi = C * Ai;

  M6 Mi;
  Mi.block<3, 3>(0, 0) = Ai + AiB * Gi * CAi;
  Mi.block<3, 3>(0, 3) = -AiB * Gi;
  Mi.block<3, 3>(3, 0) = -Gi * CAi;
  Mi.block<3, 3>(3, 3) = Gi;

  V6 v;
  v.head<3>() = a;
  v.tail<3>() = b;

  return Mi * v;
}

inline V6 linsubsolve(const M3 &A, const M3 &B, const M3 &C, const M3 &D,
                       const V3 &a, const V3 &b)
{
  return linsubsolve2(A, B, C, D, a, b);
}

void tendon_deriv(const State &x, State &dxdt, const double t,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const E::Matrix3d &K_bt,
    const E::Matrix3d &K_se,
    rInfo &rs,        // storage for get_r_info2() to avoid allocations
    rInfoCache &cache // storage for get_r_info2() to avoid allocations
) {
  auto N_t = tendons.size();

  // argument validation
  if (tendons.size() != tau.size()) {
    throw std::out_of_range("tendons and tau need to be the same size");
  }
  if (x.size() != 19 + N_t) {
    throw std::out_of_range("x State size is not correct");
  }
  if (dxdt.size() != 19 + N_t) {
    //throw std::out_of_range("dxdt State size is not correct");
    dxdt.resize(19 + N_t);
  }

  // Unpack the state
  const double *data = &x[0];
  V3 p(data);
  M3 R(data + 3);
  V3 v(data + 12);
  V3 u(data + 15);

  M3 vhat = hat(v);
  M3 uhat = hat(u);
  get_r_info2(tendons, t, rs, cache);
  auto &[r, r_dot, r_ddot] = rs;

  // Initialize used variables
  M3 A, B, G, H;
  A = B = G = H = M3::Zero();
  V3 a, b;
  a = b = V3::Zero();
  E::VectorXd si_dot = E::VectorXd::Zero(N_t);

  for (decltype(N_t) j = 0; j < N_t; j++) {
    M3 rhat         = hat(r[j]);
    V3 pi_dot_b     = uhat * r[j] + r_dot[j] + v;
    M3 pi_dot_b_hat = hat(pi_dot_b);
    si_dot[j]       = pi_dot_b.norm();

    M3 Ai = -tau[j] * pi_dot_b_hat * pi_dot_b_hat
              / (si_dot[j] * si_dot[j] * si_dot[j]);
    M3 Bi = rhat * Ai;
    M3 Gi = -Ai * rhat;
    M3 Hi = -Bi * rhat;

    V3 ai = Ai * (uhat * pi_dot_b + uhat * r_dot[j] + r_ddot[j]);
    V3 bi = rhat * ai;

    A += Ai;
    B += Bi;
    G += Gi;
    H += Hi;
    a += ai;
    b += bi;
  }

  V3 v_minus_vstar = v - V3(0, 0, 1);
  V3 c = -uhat * K_bt * u - vhat * K_se * v_minus_vstar - b;
  V3 d = -uhat * K_se * v_minus_vstar - a;

  // solve: xi_dot = M \ [d;c]
  V6 xi_dot = linsubsolve(K_se + A, G, B, K_bt + H, d, c);

  V3 p_dot = R * v;
  M3 R_dot = R * uhat;
  V3 v_dot = xi_dot.head<3>();
  V3 u_dot = xi_dot.tail<3>();

  // store the solution
  std::copy_n(p_dot.data(), 3, std::begin(dxdt));
  std::copy_n(R_dot.data(), 9, std::begin(dxdt) + 3);
  std::copy_n(v_dot.data(), 3, std::begin(dxdt) + 12);
  std::copy_n(u_dot.data(), 3, std::begin(dxdt) + 15);
  dxdt[18] = v.norm();
  std::copy_n(si_dot.data(), N_t, std::begin(dxdt) + 19);
}

void tendon_deriv_unopt(
    const State &x, State &dxdt, const double t,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const E::Matrix3d &K_bt,
    const E::Matrix3d &K_se
) {
  auto N_t = tendons.size();

  // argument validation
  if (tendons.size() != tau.size()) {
    throw std::out_of_range("tendons and tau need to be the same size");
  }
  if (x.size() != 19 + N_t) {
    throw std::out_of_range("x State size is not correct");
  }
  if (dxdt.size() != 19 + N_t) {
    //throw std::out_of_range("dxdt State size is not correct");
    dxdt.resize(19 + N_t);
  }

  // Unpack the state
  const double *data = &x[0];
  V3 p(data);
  M3 R(data + 3);
  V3 v(data + 12);
  V3 u(data + 15);

  M3 vhat = hat(v);
  M3 uhat = hat(u);
  auto [r, r_dot, r_ddot] = get_r_info(tendons, t);

  // Initialize used variables
  M3 A, B, G, H;
  A = B = G = H = M3::Zero();
  V3 a, b;
  a = b = V3::Zero();
  E::VectorXd si_dot = E::VectorXd::Zero(N_t);

  for (decltype(N_t) j = 0; j < N_t; j++) {
    M3 rhat         = hat(r[j]);
    V3 pi_dot_b     = uhat * r[j] + r_dot[j] + v;
    M3 pi_dot_b_hat = hat(pi_dot_b);
    si_dot[j]       = pi_dot_b.norm();

    M3 Ai = -tau[j] * pi_dot_b_hat * pi_dot_b_hat
              / (si_dot[j] * si_dot[j] * si_dot[j]);
    M3 Bi = rhat * Ai;
    M3 Gi = -Ai * rhat;
    M3 Hi = -Bi * rhat;

    V3 ai = Ai * (uhat * pi_dot_b + uhat * r_dot[j] + r_ddot[j]);
    V3 bi = rhat * ai;

    A += Ai;
    B += Bi;
    G += Gi;
    H += Hi;
    a += ai;
    b += bi;
  }

  V3 v_minus_vstar = v - V3(0, 0, 1);
  V3 c = -uhat * K_bt * u - vhat * K_se * v_minus_vstar - b;
  V3 d = -uhat * K_se * v_minus_vstar - a;

  // solve: xi_dot = M \ [d;c]
  V6 xi_dot = linsubsolve1(K_se + A, G, B, K_bt + H, d, c);

  V3 p_dot = R * v;
  M3 R_dot = R * uhat;
  V3 v_dot = xi_dot.head<3>();
  V3 u_dot = xi_dot.tail<3>();

  // store the solution
  std::copy_n(p_dot.data(), 3, std::begin(dxdt));
  std::copy_n(R_dot.data(), 9, std::begin(dxdt) + 3);
  std::copy_n(v_dot.data(), 3, std::begin(dxdt) + 12);
  std::copy_n(u_dot.data(), 3, std::begin(dxdt) + 15);
  dxdt[18] = v.norm();
  std::copy_n(si_dot.data(), N_t, std::begin(dxdt) + 19);
}

void tendon_deriv(const State &x, State &dxdt, const double t,
    const std::vector<TendonSpecs> &tendons,
    const std::vector<double> &tau,
    const E::Matrix3d &K_bt,
    const E::Matrix3d &K_se,
    rInfo &rs,         // storage for get_r_info2() to avoid allocations
    rInfoCache &cache, // storage for get_r_info2() to avoid allocations
    const DistFLFunc &f_e,  // external force  per length not from tendons
    const DistFLFunc &l_e   // external torque per length not from tendons
) {
  auto N_t = tendons.size();

  // argument validation
  if (tendons.size() != tau.size()) {
    throw std::out_of_range("tendons and tau need to be the same size");
  }
  if (x.size() != 19 + N_t) {
    throw std::out_of_range("x State size is not correct");
  }
  if (dxdt.size() != 19 + N_t) {
    //throw std::out_of_range("dxdt State size is not correct");
    dxdt.resize(19 + N_t);
  }

  // Unpack the state
  const double *data = &x[0];
  V3 p(data);
  M3 R(data + 3);
  V3 v(data + 12);
  V3 u(data + 15);

  M3 vhat = hat(v);
  M3 uhat = hat(u);
  get_r_info2(tendons, t, rs, cache);
  auto &[r, r_dot, r_ddot] = rs;

  // Initialize used variables
  M3 A, B, G, H;
  A = B = G = H = M3::Zero();
  V3 a, b;
  a = b = V3::Zero();
  E::VectorXd si_dot = E::VectorXd::Zero(N_t);

  for (decltype(N_t) j = 0; j < N_t; j++) {
    M3 rhat         = hat(r[j]);
    V3 pi_dot_b     = uhat * r[j] + r_dot[j] + v;
    M3 pi_dot_b_hat = hat(pi_dot_b);
    si_dot[j]       = pi_dot_b.norm();

    M3 Ai = -tau[j] * pi_dot_b_hat * pi_dot_b_hat
              / (si_dot[j] * si_dot[j] * si_dot[j]);
    M3 Bi = rhat * Ai;
    M3 Gi = -Ai * rhat;
    M3 Hi = -Bi * rhat;

    V3 ai = Ai * (uhat * pi_dot_b + uhat * r_dot[j] + r_ddot[j]);
    V3 bi = rhat * ai;

    A += Ai;
    B += Bi;
    G += Gi;
    H += Hi;
    a += ai;
    b += bi;
  }

  M3 RT = R.transpose();
  V3 v_minus_vstar = v - V3(0, 0, 1);
  V3 c = -uhat * K_bt * u - vhat * K_se * v_minus_vstar - b - RT*l_e(t, p);
  V3 d = -uhat * K_se * v_minus_vstar - a - RT*f_e(t, p);

  // solve: xi_dot = M \ [d;c]
  V6 xi_dot = linsubsolve(K_se + A, G, B, K_bt + H, d, c);

  V3 p_dot = R * v;
  M3 R_dot = R * uhat;
  V3 v_dot = xi_dot.head<3>();
  V3 u_dot = xi_dot.tail<3>();

  // store the solution
  std::copy_n(p_dot.data(), 3, std::begin(dxdt));
  std::copy_n(R_dot.data(), 9, std::begin(dxdt) + 3);
  std::copy_n(v_dot.data(), 3, std::begin(dxdt) + 12);
  std::copy_n(u_dot.data(), 3, std::begin(dxdt) + 15);
  dxdt[18] = v.norm();
  std::copy_n(si_dot.data(), N_t, std::begin(dxdt) + 19);
}

} // end of namespace tendon
