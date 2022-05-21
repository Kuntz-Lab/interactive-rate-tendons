/**
 * This code is a manual C++ conversion from Matlab code written by another
 * person
 *
 * Comments from the author:
 *
 *   The comments should give you an idea of how to use the function, but let
 *   us know if the usage is unclear.  We parameterized tendon path in terms of
 *   distance and angle from the backbone, both defined by arbitrary
 *   polynomials whose coefficients are stored in matrices C and D.  You input
 *   the routing coefficient matrices C and D along with a vector of tendon
 *   tensions tau.  An example configuration is implemented if no arguments are
 *   entered.   The program should efficiently solve the problem and integrate
 *   the full shape in about 0.1 seconds.  You can comment out the plots, of
 *   course.
 *
 * The paper first describing the kinematic model:
 *
 *   https://ieeexplore.ieee.org/abstract/document/5957337
 *
 * A recent paper that looks at stiffness (maybe)
 *
 *   https://ieeexplore.ieee.org/document/8606257
 */

#include "TendonRobot.h"


#include "BackboneSpecs.h"
#include "TendonSpecs.h"
#include "get_r_info.h"
#include "solve_initial_bending.h"
#include "tendon_deriv.h"
#include <collision/collision.h>
#include <cpptoml/toml_conversions.h>
#include <csv/Csv.h>
#include <spline/CubicSplineSequence.h>
#include <util/macros.h>
#include <util/poly.h>
#include <util/vector_ops.h>

#include <3rdparty/levmar-2.6/levmar.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/numeric/odeint.hpp>

#include <algorithm>
#include <iterator>
#include <random>
#include <stdexcept>
#include <vector>

namespace E = Eigen;
namespace ode = boost::numeric::odeint;

using util::poly_at;
using util::poly_der;

using V3 = E::Vector3d;
using VX = E::VectorXd;
using M3 = E::Matrix3d;

namespace tendon {

namespace {

std::vector<double> t_range(double start, double end, double dt) {
  // Note: util::range will have constant stepping from start.  We want
  // constant stepping from the end, so we edit this afterwards to make it so.
  auto t = util::range(start, end, dt);

  // Example:
  //   for s_start = 0.2, L = 1.0, s = 0.5
  //   - s is 0.3 from s_start, instead make it 0.3 from L -- > s = 1.0 - 0.3 = 0.7

  for (auto &val : t) { val = end - (val - start); }

  // We now need to reverse this list since it's L --> s_start
  std::reverse(t.begin(), t.end());

  return t;
}

struct StiffnessMatrices {
  M3 K_bt;
  M3 K_se;
  M3 K_bt_inv;
  M3 K_se_inv;
};

/**
 * Creates and returns the stiffness matrices and their inverses
 *
 * @param specs: backbone specifications
 *
 * @return a tuple of four matrices
 *   [K_bt, K_se, K_bt_inv, K_se_inv] = get_stiffness_matrices(specs)
 * @return K_bt: Stiffness of torsion and bending
 * @return K_se: Stiffness of shear and elongation
 * @return K_bt_inv: Inverse of K_bt
 * @return K_se_inv: Inverse of K_se
 */
StiffnessMatrices get_stiffness_matrices(const BackboneSpecs &specs) {
  auto [L, dL, ro, ri, E, nu] = specs;
  UNUSED_VAR(L);
  UNUSED_VAR(dL);
  auto ro2 = ro*ro;
  auto ri2 = ri*ri;
  auto I = (1.0/4.0) * M_PI * (ro2*ro2 - ri2*ri2);
  auto Ar = M_PI * (ro2 - ri2);
  auto J = 2 * I;
  auto Gmod = E / (2 * (1 + nu));

  //std::cout
  //  << "  L:          " << L << "\n"
  //     "  ro:         " << ro << "\n"
  //     "  ri:         " << ri << "\n"
  //     "  E:          " << E << "\n"
  //     "  nu:         " << nu << "\n"
  //     "  I:          " << I << "\n"
  //     "  Ar:         " << Ar << "\n"
  //     "  J:          " << J << "\n"
  //     "  Gmod:       " << Gmod << "\n";

  M3 K_bt = M3::Zero();
  K_bt(0, 0) = E * I;
  K_bt(1, 1) = E * I;
  K_bt(2, 2) = J * Gmod;

  M3 K_bt_inv = M3::Zero();
  K_bt_inv(0, 0) = 1 / (E * I);
  K_bt_inv(1, 1) = 1 / (E * I);
  K_bt_inv(2, 2) = 1 / (J * Gmod);

  M3 K_se = M3::Zero();
  K_se(0, 0) = Gmod * Ar;
  K_se(1, 1) = Gmod * Ar;
  K_se(2, 2) = E * Ar;

  M3 K_se_inv = M3::Zero();
  K_se_inv(0, 0) = 1 / (Gmod * Ar);
  K_se_inv(1, 1) = 1 / (Gmod * Ar);
  K_se_inv(2, 2) = 1 / (E * Ar);

  return {K_bt, K_se, K_bt_inv, K_se_inv};
}

/** Integrate using Simpson's method on a sequence of equally spaced values
 *
 * Note: length of vals needs to be even.  If it is odd, we will do a trapezoid
 * for the last interval.
 *
 * @param vals: evaluations of the function to integrate
 * @param dx: constant interval of dependent variable between each function val
 *
 * @return integral value using the Simpson's method
 */
double simpsons(const std::vector<double> &vals, double dx) {
  auto N = vals.size();
  if (N < 2) { return 0.0; }

  double odd_man_out = 0.0;
  if (N % 2 != 0) { // do trapezoid for the last segment
    odd_man_out = 0.5 * dx * (vals[N-1] + vals[N]);
    N--;
  }

  double integral = vals[0] + vals[N];
  int next_multiplier = 2;
  int multiplier = 4;
  for (size_t i = 1; i < N - 1; i++) {
    integral += multiplier * vals[i];
    std::swap(multiplier, next_multiplier); // alternate coefficients
  }
  return odd_man_out + (integral * dx / 3.0);
}

using IntFunc = std::function<void(double*, double*, int, int)>;
void call_callback(double *a, double *b, int n_a, int n_b, void *cb) {
  IntFunc &callback = *static_cast<IntFunc*>(cb);
  callback(a, b, n_a, n_b);
}

} // end of unnamed namespace

PointForces PointForces::calc_point_forces(
    const std::vector<double> &tau,
    const M3 R,
    const V3 u,
    const V3 v,
    const M3 &K_se,
    const M3 &K_bt,
    const std::vector<V3> &r,
    const std::vector<V3> &r_dot)
{
  PointForces f;
  f.n = R * K_se * (v - V3{0, 0, 1});
  f.m = R * K_bt * u; // (u - V3{0, 0, 0});
  // From Eq. (18) and (19) from Rucker et al.
  //   F_t = sum_i ( -tau[i] * pdot[i](L) / pdot[i](L).norm() )
  //   L_t = sum_i ( -tau[i] * R(L) r[i](L) x pdot[i](L) / pdot[i](L).norm() )
  //       = sum_i ( R(L) r[i](L) x F_t[i] )
  f.F_t = V3::Zero();
  f.L_t = V3::Zero();
  for (size_t i = 0; i < r.size(); i++) {
    auto pdot_i_unit = (R * (u.cross(r[i]) + r_dot[i] + v)).normalized();
    V3 F_ti = -tau[i] * pdot_i_unit;
    auto L_ti = (R * r[i]).cross(F_ti);
    f.F_t += F_ti;
    f.L_t += L_ti;
  }
  f.F_e = f.n - f.F_t;
  f.L_e = f.m - f.L_t;
  return f;
}

std::vector<double> TendonRobot::random_state() const {
  static thread_local std::mt19937 generator;
  static thread_local bool seeded = false;
  if (!seeded) {
    std::random_device rd;
    generator.seed(rd());
    seeded = true;
  }

  std::vector<double> state;

  for (auto &tendon : tendons) {
    std::uniform_real_distribution<double> dist(0.0, tendon.max_tension);
    state.push_back(dist(generator));
  }

  if (enable_rotation) {
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);
    state.push_back(dist(generator));
  }

  if (enable_retraction) {
    std::uniform_real_distribution<double> dist(0.0, specs.L);
    state.push_back(dist(generator));
  }

  return state;
}

/** Calculates the shape at zero tensions (home position) */
TendonResult TendonRobot::home_shape(double s_start) const {
  //const double tol = 1e-10;
  //if (s_start < -tol || s_start > specs.L + tol) {
  //  throw std::invalid_argument("s_start outside of backbone length range");
  //}
  //if (std::isnan(s_start)) {
  //  throw std::invalid_argument("s_start is NaN (not a number)");
  //}
  if (s_start <   0.0  ) { s_start =   0.0;   } // truncate
  if (s_start > specs.L) { s_start = specs.L; } // truncate

  TendonResult res{};
  res.u_i = res.u_f = V3::Zero();
  res.v_i = res.v_f = V3{0, 0, 1};
  if (s_start == specs.L) {
    res.t.emplace_back(s_start);
    res.p.emplace_back(collision::Point{0.0, 0.0, 0.0});
    res.R.emplace_back(M3::Identity());
    res.L = 0.0;
    res.L_i = std::vector<double>(tendons.size(), 0.0);
    return res;
  }

  res.t = t_range(s_start, specs.L, specs.dL);
  auto N = res.t.size();
  res.p.resize(N);
  std::transform(res.t.begin(), res.t.end(), res.p.begin(),
      [s_start](double t) { return V3{0.0, 0.0, t - s_start}; });
  res.R = decltype(res.R)(N, M3::Identity());
  res.L = specs.L - s_start;

  // populate res.L_i
  for (auto &tendon : tendons) {
    // check for closed form
    if (tendon.is_straight()) {

      res.L_i.emplace_back(res.L);

    } else if (tendon.is_helix()) {

      auto &d0 = tendon.D[0];
      auto &c1 = tendon.C[1];
      double scaling = std::sqrt(1 + d0*d0*c1*c1);
      res.L_i.emplace_back(res.L * scaling);

    } else {

      // do numerical integration
      auto Cdot = poly_der(tendon.C);
      auto Ddot = poly_der(tendon.D);
      auto &D = tendon.D;
      std::vector<double> ldot_vals(res.t.size());
      std::transform(res.t.begin(), res.t.end(), ldot_vals.begin(),
          [&D, &Cdot, &Ddot](double t) {
            return std::sqrt(
                std::pow(poly_at(Ddot, t), 2)
                + std::pow(poly_at(D, t), 2) * std::pow(poly_at(Cdot, t), 2)
                + 1);
          });
      res.L_i.emplace_back(simpsons(ldot_vals, specs.dL));

    }
  }

  return res;
}

/**
 * Calculates the shape of the tendon-actuated robot given the tendon paths and
 * tensions.
 *
 * @param tau: Tensions of each tendon
 * @param s_start: starting value of s (i.e., retraction amount)
 *
 * @return TendonResult (see struct TendonResult)
 */
TendonResult TendonRobot::tension_shape(const std::vector<double> &tau,
                                        double s_start) const
{
  //const double tol = 1e-10;
  //std::vector<double> tau_copy = tau;
  const auto N = tendons.size();
  if (tendons.size() != tau.size()) {
    throw std::out_of_range("tendons and tau are not the same length");
  }

  //if (std::any_of(tau.begin(), tau.end(),
  //                [&tol](double tension) { return tension < -tol; }))
  //{
  //  using util::operator<<;
  //  std::cerr << "Negative tensions requested: tau = " << tau << std::endl;
  //  throw std::invalid_argument("negative tensions are not allowed");
  //}

  // truncate
  //std::transform(tau_copy.begin(), tau_copy.end(), tau_copy.begin(),
  //               [](double val) { return (val < 0 ? 0 : val); });

  TendonResult res{};

  //if (s_start < -tol || s_start > specs.L + tol) {
  //  throw std::invalid_argument("s_start outside of backbone length range");
  //}
  //if (std::isnan(s_start)) {
  //  throw std::invalid_argument("s_start is NaN (not a number)");
  //}

  // allow it to go past L for the sake of calculating the finite difference
  // jacobian
  //if (s_start <   0.0  ) { s_start =   0.0;   } // truncate
  if (s_start > specs.L) { s_start = specs.L; } // truncate

  if (s_start == specs.L) {
    res.t.emplace_back(s_start);
    res.p.emplace_back(collision::Point{0.0, 0.0, 0.0});
    res.R.emplace_back(M3::Identity());
    res.L = 0.0;
    res.L_i = std::vector<double>(tendons.size(), 0.0);
    res.u_i = V3{0, 0, 0};
    res.u_f = V3{0, 0, 0};
    res.v_i = V3{0, 0, 1};
    res.v_f = V3{0, 0, 1};
    return res;
  }

  //auto [K_bt, K_se, K_bt_inv, K_se_inv] = get_stiffness_matrices(specs);
  const auto Ks = get_stiffness_matrices(specs);

  // calculate initial conditions
  const V3 v_guess(0, 0, 1);
  const V3 u_guess(0, 0, 0);

  rInfo rs;
  rs.resize(N);
  rInfoCache cache;
  cache.resize(this->tendons);

  //std::cout
  //  << "  K_bt:       [" << K_bt.row(0) << "]\n"
  //     "              [" << K_bt.row(1) << "]\n"
  //     "              [" << K_bt.row(2) << "]\n"
  //     "  K_se:       [" << K_se.row(0) << "]\n"
  //     "              [" << K_se.row(1) << "]\n"
  //     "              [" << K_se.row(2) << "]\n"
  //     "  K_bt_inv:   [" << K_bt_inv.row(0) << "]\n"
  //     "              [" << K_bt_inv.row(1) << "]\n"
  //     "              [" << K_bt_inv.row(2) << "]\n"
  //     "  K_se_inv:   [" << K_se_inv.row(0) << "]\n"
  //     "              [" << K_se_inv.row(1) << "]\n"
  //     "              [" << K_se_inv.row(2) << "]\n"
  //     "  v_guess:    [" << v_guess.transpose() << "]\n"
  //     "  u_guess:    [" << u_guess.transpose() << "]\n";
  const int fp_max_iters = 1000; // 30;
  const double dv_rel_threshold = 1e-9; // 1e-5;
  const double du_rel_threshold = 1e-9; // 1e-6;

  const auto [v0, u0, iters] = solve_initial_bending(v_guess, u_guess,
      tendons, tau, Ks.K_bt, Ks.K_se, Ks.K_bt_inv, Ks.K_se_inv, fp_max_iters,
      residual_threshold, dv_rel_threshold, du_rel_threshold,
      s_start, rs, cache);
  UNUSED_VAR(iters);
  res.u_i = u0;
  res.v_i = v0;

  //std::cout << "  iterations: " << iters << "\n"
  //          << "  v0:         [" << v0.transpose() << "]\n"
  //          << "  u0:         [" << u0.transpose() << "]\n";

  State x_init(19 + N, 0.0);
  // Initial rotation is identity
  x_init[3] = 1;
  x_init[7] = 1;
  x_init[11] = 1;
  std::copy_n(v0.data(), 3, std::begin(x_init) + 12);
  std::copy_n(u0.data(), 3, std::begin(x_init) + 15);

  // solve initial value ODE
  auto deriv = [this, &tau, &Ks, &rs, &cache]
               (const State &x, State &dxdt, const double t)
  {
    tendon_deriv(x, dxdt, t, this->tendons, tau, Ks.K_bt, Ks.K_se,
                 rs, cache);
  };

  res.t = t_range(s_start, specs.L, specs.dL);
  res.p.reserve(res.t.size());
  res.R.reserve(res.t.size());
  std::vector<VX> states;
  //std::vector<VX> state_derivs;
  states.reserve(res.t.size());
  //state_derivs.reserve(res.t.size());
  auto observer =
      [/*&state_derivs, */&states, &res/*, &deriv*/](const State &x, double time) {
        //State dxdt;
        //deriv(x, dxdt, time);
        (void)time; // unused

        VX v_x(x.size());
        //VX v_dxdt(dxdt.size());
        std::copy_n(&x[0], x.size(), v_x.data());
        //std::copy_n(&dxdt[0], dxdt.size(), v_dxdt.data());

        //res.t.emplace_back(time);
        res.p.emplace_back(v_x.head<3>());
        res.R.emplace_back(v_x.data() + 3);
        states.emplace_back(std::move(v_x));
        //state_derivs.emplace_back(v_dxdt);
      };

  ode::runge_kutta4<State> stepper;
  //ode::integrate_const(stepper, deriv, x_init, s_start, specs.L,
  //                     std::min(specs.dL, specs.L - s_start), observer);
  ode::integrate_times(stepper, deriv, x_init, res.t.begin(), res.t.end(),
                       specs.dL, observer);

  res.L = states.back()[18];
  res.L_i.resize(N);
  std::copy_n(states.back().data() + 19, N, res.L_i.begin());
  std::copy_n(states.back().data() + 12, 3, res.v_f.data());
  std::copy_n(states.back().data() + 15, 3, res.u_f.data());

  get_r_info2(tendons, s_start, rs, cache);
  auto base_force = PointForces::calc_point_forces(
      tau, res.R.front(), u0, v0, Ks.K_se, Ks.K_bt, rs.r, rs.r_dot);

  res.converged = (base_force.residual() <= residual_threshold);

  //// populate res
  //// use cubic interpolation to get points every specs.dL
  //spline::CubicSplineSequence splines(t, states, state_derivs);
  //res.t = t_range(s_start, specs.L, specs.dL);
  //std::vector<VX> interp_states(res.t.size());
  //std::transform(res.t.begin(), res.t.end(), interp_states.begin(),
  //    [&splines](double time) {
  //      return splines(time);
  //    });
  //res.p.resize(interp_states.size());
  //res.R.resize(interp_states.size());
  //res.L_i.resize(N);
  //std::transform(interp_states.begin(), interp_states.end(), res.p.begin(),
  //    [](const VX &x) {
  //      return x.head<3>();
  //    });
  //std::transform(interp_states.begin(), interp_states.end(), res.R.begin(),
  //    [](const VX &x) {
  //      return M3(x.data() + 3);
  //    });
  //res.L = states.back()[18];
  //std::copy_n(states.back().data() + 19, N, res.L_i.begin());

  return res;
}

TendonResult TendonRobot::tension_shape_unopt(const std::vector<double> &tau,
                                              double s_start) const
{
  //const double tol = 1e-10;
  //std::vector<double> tau_copy = tau;
  auto N = tendons.size();
  if (tendons.size() != tau.size()) {
    throw std::out_of_range("tendons and tau are not the same length");
  }

  //if (std::any_of(tau.begin(), tau.end(),
  //                [&tol](double tension) { return tension < -tol; }))
  //{
  //  using util::operator<<;
  //  std::cerr << "Negative tensions requested: tau = " << tau << std::endl;
  //  throw std::invalid_argument("negative tensions are not allowed");
  //}

  // truncate
  //std::transform(tau_copy.begin(), tau_copy.end(), tau_copy.begin(),
  //               [](double val) { return (val < 0 ? 0 : val); });

  TendonResult res{};

  //if (s_start < -tol || s_start > specs.L + tol) {
  //  throw std::invalid_argument("s_start outside of backbone length range");
  //}
  //if (std::isnan(s_start)) {
  //  throw std::invalid_argument("s_start is NaN (not a number)");
  //}

  // allow it to go past L for the sake of calculating the finite difference
  // jacobian
  //if (s_start <   0.0  ) { s_start =   0.0;   } // truncate
  if (s_start > specs.L) { s_start = specs.L; } // truncate

  if (s_start == specs.L) {
    res.t.emplace_back(s_start);
    res.p.emplace_back(collision::Point{0.0, 0.0, 0.0});
    res.R.emplace_back(M3::Identity());
    res.L = 0.0;
    res.L_i = std::vector<double>(tendons.size(), 0.0);
    res.u_i = V3{0, 0, 0};
    res.u_f = V3{0, 0, 0};
    res.v_i = V3{0, 0, 1};
    res.v_f = V3{0, 0, 1};
    return res;
  }

  //auto [K_bt, K_se, K_bt_inv, K_se_inv] = get_stiffness_matrices(specs);
  const auto Ks = get_stiffness_matrices(specs);

  // calculate initial conditions
  const V3 v_guess(0, 0, 1);
  const V3 u_guess(0, 0, 0);

  //std::cout
  //  << "  K_bt:       [" << K_bt.row(0) << "]\n"
  //     "              [" << K_bt.row(1) << "]\n"
  //     "              [" << K_bt.row(2) << "]\n"
  //     "  K_se:       [" << K_se.row(0) << "]\n"
  //     "              [" << K_se.row(1) << "]\n"
  //     "              [" << K_se.row(2) << "]\n"
  //     "  K_bt_inv:   [" << K_bt_inv.row(0) << "]\n"
  //     "              [" << K_bt_inv.row(1) << "]\n"
  //     "              [" << K_bt_inv.row(2) << "]\n"
  //     "  K_se_inv:   [" << K_se_inv.row(0) << "]\n"
  //     "              [" << K_se_inv.row(1) << "]\n"
  //     "              [" << K_se_inv.row(2) << "]\n"
  //     "  v_guess:    [" << v_guess.transpose() << "]\n"
  //     "  u_guess:    [" << u_guess.transpose() << "]\n";
  const int fp_max_iters = 1000; // 30
  const double dv_rel_threshold = 1e-9; // 1e-5;
  const double du_rel_threshold = 1e-9; // 1e-6;

  const auto [v0, u0, iters] = solve_initial_bending_unopt(v_guess, u_guess,
      tendons, tau, Ks.K_bt, Ks.K_se, Ks.K_bt_inv, Ks.K_se_inv, fp_max_iters,
      residual_threshold, dv_rel_threshold, du_rel_threshold, s_start);
  UNUSED_VAR(iters);
  res.u_i = u0;
  res.v_i = v0;

  //std::cout << "  iterations: " << iters << "\n"
  //          << "  v0:         [" << v0.transpose() << "]\n"
  //          << "  u0:         [" << u0.transpose() << "]\n";

  State x_init(19 + N, 0.0);
  // Initial rotation is identity
  x_init[3] = 1;
  x_init[7] = 1;
  x_init[11] = 1;
  std::copy_n(v0.data(), 3, std::begin(x_init) + 12);
  std::copy_n(u0.data(), 3, std::begin(x_init) + 15);

  // solve initial value ODE
  auto deriv = [this, &tau, &Ks] (const State &x, State &dxdt, const double t) {
    tendon_deriv_unopt(x, dxdt, t, this->tendons, tau, Ks.K_bt, Ks.K_se);
  };

  //res.t = t_range(s_start, specs.L, specs.dL);
  //res.p.reserve(res.t.size());
  //res.R.reserve(res.t.size());
  std::vector<VX> states;
  std::vector<VX> state_derivs;
  //states.reserve(res.t.size());
  //state_derivs.reserve(res.t.size());
  auto observer =
      [&state_derivs, &states, &res, &deriv](const State &x, double time) {
        State dxdt;
        deriv(x, dxdt, time);
        (void)time; // unused

        VX v_x(x.size());
        VX v_dxdt(dxdt.size());
        std::copy_n(&x[0], x.size(), v_x.data());
        std::copy_n(&dxdt[0], dxdt.size(), v_dxdt.data());

        res.t.emplace_back(time);
        //res.p.emplace_back(v_x.head<3>());
        //res.R.emplace_back(v_x.data() + 3);
        states.emplace_back(std::move(v_x));
        state_derivs.emplace_back(v_dxdt);
      };

  ode::runge_kutta4<State> stepper;
  ode::integrate_const(stepper, deriv, x_init, s_start, specs.L,
                       std::min(specs.dL, specs.L - s_start), observer);
  //ode::integrate_times(stepper, deriv, x_init, res.t.begin(), res.t.end(),
  //                     specs.dL, observer);

  res.L = states.back()[18];
  res.L_i.resize(N);

  // populate res
  // use cubic interpolation to get points every specs.dL
  spline::CubicSplineSequence splines(res.t, states, state_derivs);
  res.t = t_range(s_start, specs.L, specs.dL);
  std::vector<VX> interp_states(res.t.size());
  std::transform(res.t.begin(), res.t.end(), interp_states.begin(),
      [&splines](double time) {
        return splines(time);
      });
  res.p.resize(interp_states.size());
  res.R.resize(interp_states.size());
  res.L_i.resize(N);
  std::transform(interp_states.begin(), interp_states.end(), res.p.begin(),
      [](const VX &x) {
        return x.head<3>();
      });
  std::transform(interp_states.begin(), interp_states.end(), res.R.begin(),
      [](const VX &x) {
        return M3(x.data() + 3);
      });
  res.L = states.back()[18];
  std::copy_n(states.back().data() + 19, N, res.L_i.begin());
  std::copy_n(states.back().data() + 12, 3, res.v_f.data());
  std::copy_n(states.back().data() + 15, 3, res.u_f.data());

  auto rinfo = get_r_info(tendons, s_start);
  auto base_force = PointForces::calc_point_forces(
      tau, res.R.front(), u0, v0, Ks.K_se, Ks.K_bt, rinfo.r, rinfo.r_dot);

  res.converged = (base_force.residual() <= residual_threshold);

  return res;
}

PointForces TendonRobot::tip_forces(
    const std::vector<double> &tau, const TendonResult &result) const
{
  auto rs = get_r_info(tendons, result.t.back());
  auto Ks = get_stiffness_matrices(specs);
  return PointForces::calc_point_forces(
      tau, result.R.back(), result.u_f, result.v_f,
      Ks.K_se, Ks.K_bt, rs.r, rs.r_dot);
}

PointForces TendonRobot::base_forces(
    const std::vector<double> &tau, const TendonResult &result) const
{
  auto rs = get_r_info(tendons, result.t.front());
  auto Ks = get_stiffness_matrices(specs);
  return PointForces::calc_point_forces(
      tau, result.R.front(), result.u_i, result.v_i,
      Ks.K_se, Ks.K_bt, rs.r, rs.r_dot);
}

TendonResult TendonRobot::general_tension_shape(
    const std::vector<double> &tau,
    double s_start,
    const DistFLFunc &f_e,
    const DistFLFunc &l_e,
    const Eigen::Vector3d &F_e,
    const Eigen::Vector3d &L_e,
    const Eigen::Vector3d &u_guess,
    const Eigen::Vector3d &v_guess,
    int max_iters,
    double mu_init,
    double stop_threshold_JT_err_inf,
    double stop_threshold_Dp,
    double finite_difference_delta,
    bool verbose
    ) const
{
  using util::operator<<;

  auto N = tendons.size();
  if (tendons.size() != tau.size()) {
    throw std::out_of_range("tendons and tau are not the same length");
  }

  TendonResult res{};

  // allow it to go past L for the sake of calculating the finite difference
  // jacobian
  if (s_start > specs.L) { s_start = specs.L; } // truncate

  if (s_start == specs.L) {
    res.t.emplace_back(s_start);
    res.p.emplace_back(collision::Point{0.0, 0.0, 0.0});
    res.R.emplace_back(M3::Identity());
    res.L = 0.0;
    res.L_i = std::vector<double>(tendons.size(), 0.0);
    return res;
  }

  auto Ks = get_stiffness_matrices(specs);

  rInfo rs;
  rs.resize(N);
  rInfoCache cache;
  cache.resize(this->tendons);

  res.t = t_range(s_start, specs.L, specs.dL);
  res.p.reserve(res.t.size());
  res.R.reserve(res.t.size());
  std::vector<VX> states;
  states.reserve(res.t.size());

  auto observer = [&states, &res](const State &x, double time) {
    UNUSED_VAR(time);

    VX v_x(x.size());
    std::copy_n(&x[0], x.size(), v_x.data());

    res.p.emplace_back(v_x.head<3>());
    res.R.emplace_back(v_x.data() + 3);
    states.emplace_back(std::move(v_x));
  }; // end of lambda observer()

  // solve initial value ODE
  auto deriv = [this, &tau, &Ks, &rs, &cache, f_e, l_e]
               (const State &x, State &dxdt, const double t)
  {
    tendon_deriv(x, dxdt, t, this->tendons, tau, Ks.K_bt, Ks.K_se,
                 rs, cache, f_e, l_e);
  }; // end of lambda deriv()

  bool first_integrate_call = true;
  V3 F_e0, L_e0; // estimated F_e and L_e from the first integration
  IntFunc integrate = [&](double *vu, double *FL_e_out, int vu_dim, int FL_e_dim) {
    UNUSED_VAR(vu_dim);
    UNUSED_VAR(FL_e_dim);

    // clear the last run's capture values
    res.p.clear();
    res.R.clear();
    states.clear();

    // Create initial state for integration
    State x_init(19 + N, 0.0);
    // Initial rotation is identity
    x_init[3] = 1;
    x_init[7] = 1;
    x_init[11] = 1;
    std::copy_n(vu, 6, std::begin(x_init) + 12);
    //std::copy_n(v0.data(), 3, std::begin(x_init) + 12);
    //std::copy_n(u0.data(), 3, std::begin(x_init) + 15);

    ode::runge_kutta4<State> stepper;
    ode::integrate_times(stepper, deriv, x_init, res.t.begin(), res.t.end(),
                         specs.dL, observer);

    // rotation matrix R(L)
    M3 R(states.back().data() + 3);

    // These are u(L) and v(L)
    V3 v(states.back().data() + 12);
    V3 u(states.back().data() + 15);
    get_r_info2(tendons, specs.L, rs, cache);
    auto tip_forces = PointForces::calc_point_forces(
        tau, R, u, v, Ks.K_se, Ks.K_bt, rs.r, rs.r_dot);

    //// calculate the estimated F_e = n(L) - F_t(L), L_e = m(L) - L_t(L)
    //V3 n = R * Ks.K_se * (v - V3{0, 0, 1});
    //V3 m = R * Ks.K_bt * (u - V3{0, 0, 0});
    //V3 F_t = V3::Zero();
    //V3 L_t = V3::Zero();
    //auto &[r, r_dot, r_ddot] = rs;
    //for (size_t i = 0; i < tau.size(); i++) {
    //  V3 pdot_i_unit = (R * (u.cross(r[i]) + r_dot[i] + v)).normalized();
    //  V3 F_ti = -tau[i] * pdot_i_unit;
    //  V3 L_ti = (R * r[i]).cross(F_ti);
    //  F_t += F_ti;
    //  L_t += L_ti;
    //}

    //V3 F_e_estimated = n - F_t;
    //V3 L_e_estimated = m - L_t;
    std::copy_n(tip_forces.F_e.data(), 3, FL_e_out);
    std::copy_n(tip_forces.L_e.data(), 3, FL_e_out + 3);

    // capture the values from the first call
    if (first_integrate_call) {
      first_integrate_call = false;
      F_e0 = tip_forces.F_e;
      L_e0 = tip_forces.L_e;
    }
    //V3 v0(vu);
    //V3 u0(vu + 3);
    //std::cout <<
    //  "    integrate(v: [" << v0.transpose() << "], "
    //                "u: [" << u0.transpose() << "])\n"
    //  "      F_t:      [" << F_t.transpose() << "]\n"
    //  "      L_t:      [" << L_t.transpose() << "]\n"
    //  "      n(L):     [" << n.transpose() << "]\n"
    //  "      m(L):     [" << m.transpose() << "]\n"
    //  "      F_e:      [" << tip_forces.F_e.transpose() << "]\n"
    //  "      L_e:      [" << tip_forces.L_e.transpose() << "]\n"
    //  "      FL_e_out  ["
    //     << FL_e_out[0] << ", "
    //     << FL_e_out[1] << ", "
    //     << FL_e_out[2] << ", "
    //     << FL_e_out[3] << ", "
    //     << FL_e_out[4] << ", "
    //     << FL_e_out[5] << "]\n";

  }; // end of lambda integrate()

  double vu[6];
  std::copy_n(v_guess.data(), 3, vu);
  std::copy_n(u_guess.data(), 3, vu + 3);

  // at the end of integration, n(L) = F_t + F_e, m(L) = L_t + L_e
  double FL_e[6];
  std::copy_n(F_e.data(), 3, FL_e);
  std::copy_n(L_e.data(), 3, FL_e + 3);

  double info[LM_INFO_SZ];
  double levmar_opt[LM_OPTS_SZ];

  levmar_opt[0] = mu_init;
  levmar_opt[1] = stop_threshold_JT_err_inf;

  // these thresholds in levmar are on the square norm, so square them
  levmar_opt[2] = stop_threshold_Dp * stop_threshold_Dp;
  levmar_opt[3] = residual_threshold * residual_threshold;

  // negative for central-difference instead of forward-difference
  levmar_opt[4] = finite_difference_delta;

  // allocate the levmar workspace buffer, known only at runtime, but local to
  // this thread
  static thread_local double *workspace = nullptr;
  static thread_local size_t workspace_size = 0;
  size_t new_workspace_size = LM_DIF_WORKSZ(6, 6);
  if (workspace != nullptr && workspace_size < new_workspace_size) {
    delete[] workspace; // free the old one that's too small
    workspace = nullptr;
  }
  if (workspace == nullptr) {
    workspace_size = new_workspace_size;
    workspace = new double[workspace_size];
  }

  // run Levenberg-Marquardt
  dlevmar_dif(&call_callback,
              vu, FL_e, 6, 6,
              max_iters,
              levmar_opt,
              info,
              workspace,
              nullptr, // do not need the covariance matrix
              static_cast<void*>(&integrate));

  // print verbose diagnostics
  if (verbose) {
    std::string term_expl;
    switch (int(info[6])) {
      case 1: term_expl = "small |J^T e|_inf"; break;
      case 2: term_expl = "small |Dp|"; break;
      case 3: term_expl = "max iter"; break;
      case 4: term_expl = "singular matrix (recommend increasing mu from result)"; break;
      case 5: term_expl = "progress not possible (recommend restart and increasing mu)"; break;
      case 6: term_expl = "small |e|"; break;
      case 7: term_expl = "invalid values from integration (NaN or Inf)"; break;
    }
    integrate(vu, FL_e, 6, 6); // TODO: replace with just F & L computation
    V3 F_e_final(FL_e);
    V3 L_e_final(FL_e + 3);
    V3 v_final(vu);
    V3 u_final(vu + 3);
    std::cout <<
      "\n"
      "general_tension_shape:\n"
      "  initial state:\n"
      "    u initial:        [" << u_guess.transpose()       << "]\n"
      "    v initial:        [" << v_guess.transpose()       << "]\n"
      "  initial estimates:\n"
      "    F_e initial:      [" << F_e0.transpose()          << "]\n"
      "    L_e initial:      [" << L_e0.transpose()          << "]\n"
      "  desired wrench:\n"
      "    F_e desired:      [" << F_e.transpose()           << "]\n"
      "    L_e desired:      [" << L_e.transpose()           << "]\n"
      "  solution:\n"
      "    u final:          [" << u_final.transpose()       << "]\n"
      "    v final:          [" << v_final.transpose()       << "]\n"
      "  reached estimates:\n"
      "    F_e final:        [" << F_e_final.transpose()     << "]\n"
      "    L_e final:        [" << L_e_final.transpose()     << "]\n"
      "  |e|_init:           " << std::sqrt(info[0])         << "\n"
      "  |e|:                " << std::sqrt(info[1])
      << "    (threshold:    " << residual_threshold         << ")\n"
      "  |J^T e|_inf:        " << info[2]
      << "    (threshold:    " << stop_threshold_JT_err_inf  << ")\n"
      "  |Dp|:               " << std::sqrt(info[3])
      << "    (threshold:    " << stop_threshold_Dp          << ")\n"
      "  mu/max(J^T J):      " << info[4]
      << "    (mu_init: "      << mu_init                    << ")\n"
      "  iters:              " << info[5]                    << "\n"
      "  term condition:     " << int(info[6])
      << ",   reason: "        << term_expl                  << "\n"
      "  # integrations:     " << info[7]                    << "\n"
      "  # J calls:          " << info[8]                    << "\n"
      "  # linear solves:    " << info[9]                    << "\n"
      "\n" << std::flush;
  }

  // return results
  res.L = states.back()[18];
  res.L_i.resize(N);
  std::copy_n(states.back().data() + 19, N, res.L_i.begin());
  std::copy_n(states.front().data() + 12, 3, res.v_i.data());
  std::copy_n(states.front().data() + 15, 3, res.u_i.data());
  std::copy_n(states.back().data() + 12, 3, res.v_f.data());
  std::copy_n(states.back().data() + 15, 3, res.u_f.data());

  res.converged = (info[6] == 6);

  return res;
}


bool TendonRobot::is_valid(const std::vector<double> &state,
                           const TendonResult &_home_shape,
                           const TendonResult &_shape) const
{
  auto dl = calc_dl(_home_shape.L_i, _shape.L_i);
  if (!is_within_length_limits(dl)) {
    return false;
  }
  for (size_t i = 0; i < tendons.size(); i++) {
    if (state[i] < 0.0 || tendons[i].max_tension < state[i]) {
      return false;
    }
  }
  // check for self-collision
  return !collides_self(_shape);
}

bool TendonRobot::collides_self(const TendonResult &_shape) const {
  return collision::collides_self(collision::CapsuleSequence{_shape.p, this->r});
}

std::vector<std::vector<double>> TendonRobot::read_config_csv(std::istream &in) {
  csv::CsvReader reader(in);

  auto *header = reader.header();
  std::vector<int> indices;
  for (size_t i = 0; i < tendons.size(); ++i) {
    indices.emplace_back(header->index("tau_" + std::to_string(i+1)));
  }
  if (enable_rotation) {
    indices.emplace_back(header->index("theta"));
  }
  if (enable_retraction) {
    indices.emplace_back(header->index("s_start"));
  }

  // read the rows
  std::vector<std::vector<double>> configs;
  csv::CsvRow row;
  while (reader >> row) {
    std::vector<double> config;
    for (auto i : indices) {
      config.emplace_back(std::stod(row.at(i)));
    }
    configs.emplace_back(std::move(config));
  }
  return configs;
}

std::vector<std::vector<double>> TendonRobot::load_config_csv(
    const std::string &csv_file)
{
  std::ifstream in;
  util::openfile_check(in, csv_file);
  return read_config_csv(in);
}

cpptoml::table_ptr TendonRobot::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("tendon_robot", tbl);
  tbl->insert("radius", r);

  auto specs_tbl = specs.to_toml()->get("backbone_specs")->as_table();
  container->insert("backbone_specs", specs_tbl);

  if (!tendons.empty()) {
    auto tbl_array = cpptoml::make_table_array();
    for (auto &t : tendons) {
      tbl_array->push_back(t.to_toml());
    }
    container->insert("tendons", tbl_array);
  }

  tbl->insert("enable_rotation", enable_rotation);
  tbl->insert("enable_retraction", enable_retraction);
  tbl->insert("residual_threshold", residual_threshold);

  return container;
}

TendonRobot TendonRobot::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  auto robot_tbl = tbl->get("tendon_robot")->as_table();
  if (!robot_tbl) {
    throw cpptoml::parse_exception(
        "Wrong type detected for 'tendon_robot': not a table");
  }

  auto r = robot_tbl->get("radius")->as<double>();
  if (!r) {
    throw cpptoml::parse_exception(
        "Wrong type detected for 'r': not a double");
  }
  TendonRobot robot;
  robot.r = r->get();
  robot.specs = tendon::BackboneSpecs::from_toml(tbl);

  if (tbl->contains("tendons")) {
    auto tendons = tbl->get("tendons")->as_table_array();
    if (!tendons) {
      throw cpptoml::parse_exception("Wrong type detected for 'tendons'");
    }
    for (auto &sub_tbl : tendons->get()) {
      robot.tendons.emplace_back(tendon::TendonSpecs::from_toml(sub_tbl));
    }
  }

  if (robot_tbl->contains("enable_rotation")) {
    auto enable_rotation = robot_tbl->get("enable_rotation")->as<bool>();
    if (!enable_rotation) {
      throw cpptoml::parse_exception("Wrong type detected for enable_rotation");
    }
    robot.enable_rotation = enable_rotation->get();
  }

  if (robot_tbl->contains("enable_retraction")) {
    auto enable_retraction = robot_tbl->get("enable_retraction")->as<bool>();
    if (!enable_retraction) {
      throw cpptoml::parse_exception("Wrong type detected for enable_retraction");
    }
    robot.enable_retraction = enable_retraction->get();
  }

  if (robot_tbl->contains("residual_threshold")) {
    auto residual_threshold = robot_tbl->get("residual_threshold")->as<double>();
    if (!residual_threshold) {
      throw cpptoml::parse_exception("Wrong type detected for residual_threshold");
    }
    robot.residual_threshold = residual_threshold->get();
  }

  return robot;
}

} // end of namespace tendon
