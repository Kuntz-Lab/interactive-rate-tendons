#ifndef TENDON_ROBOT_H
#define TENDON_ROBOT_H

#include "BackboneSpecs.h"
#include "TendonResult.h"
#include "TendonSpecs.h"
#include "get_r_info.h"

#include <collision/Point.h>

#include <Eigen/Core>

#include <functional>
#include <memory>
#include <ostream>
#include <vector>

#include <cmath>

namespace cpptoml {
class table;
}

namespace tendon {

using DistFLFunc =
  std::function<Eigen::Vector3d(double, const Eigen::Vector3d&)>;

struct PointForces {
  Eigen::Vector3d F_e;  // external force  exerted on the robot tip
  Eigen::Vector3d L_e;  // external torque exerted on the robot tip
  Eigen::Vector3d F_t;  // force  on the robot tip by the tendons
  Eigen::Vector3d L_t;  // torque on the robot tip by the tendons
  Eigen::Vector3d n;    // internal force  carried by the backbone
  Eigen::Vector3d m;    // internal torque carried by the backbone

  static PointForces calc_point_forces(
    const std::vector<double> &tau,
    const Eigen::Matrix3d R,
    const Eigen::Vector3d u,
    const Eigen::Vector3d v,
    const Eigen::Matrix3d &K_se,
    const Eigen::Matrix3d &K_bt,
    const std::vector<Eigen::Vector3d> &r,
    const std::vector<Eigen::Vector3d> &r_dot);

  double residual() const {
    return std::sqrt(F_e.squaredNorm() + L_e.squaredNorm());
  }
};

struct TendonRobot {
  double r = 0.015;                         // radius in meters
  tendon::BackboneSpecs specs{};            // backbone specifications
  std::vector<tendon::TendonSpecs> tendons; // tendons
  bool enable_rotation    = false;          // enable rotation control
  bool enable_retraction  = false;          // enable retraction control
  double residual_threshold = 5e-6;         // threshold for converged FK

  size_t state_size() const {
    auto Ntau = tendons.size();
    auto Ntot = Ntau + (enable_rotation ? 1 : 0) + (enable_retraction ? 1 : 0);
    return Ntot;
  }

  std::vector<double> random_state() const;

  std::vector<collision::Point>
  forward_kinematics(const std::vector<double> &state) const {
    auto results = this->shape(state);
    return results.p;
  }

  /** shape at the zero tension config (efficiently computed)
   *
   * Note, the s_start will be used even if retraction is disabled.
   *
   * @param s_start: starting point for shape computation.  Needs to be between
   *   0 and specs.L.  Represents control of robot insertion, i.e., only the
   *   robot from s_start to specs.L is in the workspace.
   * @return shape with zero tension
   */
  TendonResult home_shape(double s_start = 0) const;

  /** shape at zero tension
   *
   * Will pull the s_start out of the state only if retraction is enabled
   *
   * @param state: full configuration state in which retraction will be last if
   *   it is enabled.
   * @return shape with zero tension
   */
  TendonResult home_shape(const std::vector<double> &state) const {
    double retract = enable_retraction ? state.back() : 0.0;
    return home_shape(retract);
  }

  /** computes shape from state
   *
   * @param state: tendon tensions, rotation (if enabled), then retraction (if
   *     enabled)
   *
   * @return shape at the given state
   */
  TendonResult shape(const std::vector<double> &state) const {
    auto Ntau = tendons.size();
    if (state.size() != state_size()) {
      throw std::invalid_argument("State is not the right size");
    }
    std::vector<double> tau(state.begin(), state.begin() + Ntau);
    double rotate = enable_rotation ? state[Ntau] : 0.0;
    double retract = enable_retraction ? state.back() : 0.0;
    return shape(tau, rotate, retract);
  }

  /** Computes the robot shape
   *
   * @param tau: tendon tensions controls
   * @param rotation: rotation control (only used if enable_rotation)
   * @param retraction: retraction control (only used if enable_retraction)
   */
  TendonResult shape(const std::vector<double> &tau, double rotation,
                     double retraction) const
  {
    double retract = enable_retraction ? retraction : 0.0;
    auto result = tension_shape(tau, retract);
    if (enable_rotation) {
      result.rotate_z(rotation);
    }
    return result;
  }

  TendonResult shape_unopt(const std::vector<double> &state) const {
    auto Ntau = tendons.size();
    if (state.size() != state_size()) {
      throw std::invalid_argument("State is not the right size");
    }
    std::vector<double> tau(state.begin(), state.begin() + Ntau);
    double rotate = enable_rotation ? state[Ntau] : 0.0;
    double retract = enable_retraction ? state.back() : 0.0;
    return shape_unopt(tau, rotate, retract);
  }

  TendonResult shape_unopt(const std::vector<double> &tau, double rotation,
                           double retraction) const
  {
    double retract = enable_retraction ? retraction : 0.0;
    auto result = tension_shape_unopt(tau, retract);
    if (enable_rotation) {
      result.rotate_z(rotation);
    }
    return result;
  }

  /// Wraps around general_shape() with tau, rot, and ret each specifieid
  TendonResult general_shape(
      const std::vector<double> &state,
      const DistFLFunc &f_e,
      const DistFLFunc &l_e,
      const Eigen::Vector3d &F_e,
      const Eigen::Vector3d &L_e,
      const Eigen::Vector3d &u_guess = {0, 0, 0},
      const Eigen::Vector3d &v_guess = {0, 0, 1},
      int max_iters = 100,
      double mu_init = 0.1,
      double stop_threshold_JT_err_inf = 1e-12,
      double stop_threshold_Dp = 1e-9,
      double finite_difference_delta = 1e-6,
      bool verbose = true) const
  {
    auto Ntau = tendons.size();
    if (state.size() != state_size()) {
      throw std::invalid_argument("State is not the right size");
    }
    std::vector<double> tau(state.begin(), state.begin() + Ntau);
    double rotate = enable_rotation ? state[Ntau] : 0.0;
    double retract = enable_retraction ? state.back() : 0.0;
    return general_shape(
        tau, rotate, retract,
        f_e, l_e, F_e, L_e, u_guess, v_guess,
        max_iters, mu_init,
        stop_threshold_JT_err_inf,
        stop_threshold_Dp,
        finite_difference_delta,
        verbose);
  }

  /** Wraps around general_tension_shape(), rotating the result after */
  TendonResult general_shape(
      const std::vector<double> &tau,
      double rotation,
      double retraction,
      const DistFLFunc &f_e,
      const DistFLFunc &l_e,
      const Eigen::Vector3d &F_e,
      const Eigen::Vector3d &L_e,
      const Eigen::Vector3d &u_guess = {0, 0, 0},
      const Eigen::Vector3d &v_guess = {0, 0, 1},
      int max_iters = 100,
      double mu_init = 0.1,
      double stop_threshold_JT_err_inf = 1e-9,
      double stop_threshold_Dp = 1e-4,
      double finite_difference_delta = 1e-6,
      bool verbose = true) const
  {
    double retract = enable_retraction ? retraction : 0.0;
    auto result = general_tension_shape(
        tau, retract,
        f_e, l_e, F_e, L_e, u_guess, v_guess,
        max_iters, mu_init,
        stop_threshold_JT_err_inf,
        stop_threshold_Dp,
        finite_difference_delta,
        verbose);
    if (enable_rotation) {
      result.rotate_z(rotation);
    }
    return result;
  }

  bool is_valid(const std::vector<double> &state,
                const TendonResult &_home_shape) const
  { return is_valid(state, _home_shape, shape(state)); }

  bool is_valid(const std::vector<double> &state,
                const TendonResult &_home_shape,
                const TendonResult &_shape) const;

  bool collides_self(const TendonResult &_shape) const;

  std::pair<TendonResult, std::vector<double>>
  shape_and_lengths(const std::vector<double> &state) const {
    return this->shape_and_lengths(state, this->home_shape(state));
  }

  std::pair<TendonResult, std::vector<double>>
  shape_and_lengths(
      const std::vector<double> &state,
      const TendonResult &_home_shape)
  const
  {
    auto _shape = this->shape(state);
    auto _lengths = this->calc_dl(_home_shape.L_i, _shape.L_i);
    return {_shape, _lengths};
  }

  std::vector<double> calc_dl(const std::vector<double> &home_l,
                              const std::vector<double> &other_l) const
  {
    if (home_l.size() != other_l.size()) {
      throw std::out_of_range("vector size mismatch");
    }
    std::vector<double> dl(home_l.size());
    for (size_t i = 0; i < home_l.size(); i++) {
      // string extension means other is shorter
      dl[i] = home_l[i] - other_l[i];
    }
    return dl;
  }

  bool is_within_length_limits(const std::vector<double> &home_l,
                               const std::vector<double> &other_l) const
  {
    auto dl = calc_dl(home_l, other_l);
    return is_within_length_limits(dl);
  }

  bool is_within_length_limits(const std::vector<double> &dl) const {
    if (dl.size() != tendons.size()) {
      throw std::out_of_range("length mismatch");
    }
    for (size_t i = 0; i < dl.size(); i++) {
      if (dl[i] < tendons[i].min_length || tendons[i].max_length < dl[i]) {
        return false;
      }
    }
    return true;
  }

  bool operator==(const TendonRobot &other) const {
    return r                 == other.r
        && specs             == other.specs
        && tendons           == other.tendons
        && enable_rotation   == other.enable_rotation
        && enable_retraction == other.enable_retraction;
  }

  std::vector<std::vector<double>> read_config_csv(std::istream &in);
  std::vector<std::vector<double>> load_config_csv(const std::string &csv_file);

  std::shared_ptr<cpptoml::table> to_toml() const;
  static TendonRobot from_toml(std::shared_ptr<cpptoml::table> tbl);

  /** Calculate tip forces from a TendonResult.
   * The tau vector is only expected to have tensions.  You can include
   * rotation and retraction if you want, they will simply be ignored.
   */
  PointForces tip_forces(const std::vector<double> &tau,
                         const TendonResult &result) const;

  /** Calculate base forces from a TendonResult.
   * The tau vector is only expected to have tensions.  You can include
   * rotation and retraction if you want, they will simply be ignored.
   */
  PointForces base_forces(const std::vector<double> &tau,
                          const TendonResult &result) const;

  /** shape at the given tendon tensions and external loads
   *
   * The f_e, l_e, F_e, and L_e are separate and in addition to the forces and
   * torques on the backbone provided by the tendons.
   *
   * @param tau: tendon tensions, to be between tendon limits, but not checked.
   * @param s_start: starting point for shape computation.  Needs to be between
   *   0 and specs.L.  Represents control of robot insertion, i.e., only the
   *   robot from s_start to specs.L is in the workspace.
   * @param f_e: function to evaluate the external force per unit length along
   *   the backbone at backbone parameter t and at workspace position p.
   * @param l_e: function to evaluate the external torque per unit length along
   *   the backbone at backbone parameter t and at workspace position p.
   * @param F_e: External point force at the tip of the robot
   * @param L_e: External point torque at the tip of the robot
   *
   * @return predicted shape of the tendon robot
   */
  TendonResult general_tension_shape(const std::vector<double> &tau,
                                     double s_start,
                                     const DistFLFunc &f_e,
                                     const DistFLFunc &l_e,
                                     const Eigen::Vector3d &F_e,
                                     const Eigen::Vector3d &L_e,
                                     const Eigen::Vector3d &u_guess = {0, 0, 0},
                                     const Eigen::Vector3d &v_guess = {0, 0, 1},
                                     int max_iters = 100,
                                     double mu_init = 0.1,
                                     double stop_threshold_JT_err_inf = 1e-9,
                                     double stop_threshold_Dp = 1e-4,
                                     double finite_difference_delta = 1e-6,
                                     bool verbose = true) const;

private:
  /** shape at the given tendon tensions
   *
   * @param tau: tendon tensions, to be between tendon limits, but not checked.
   * @param s_start: starting point for shape computation.  Needs to be between
   *   0 and specs.L.  Represents control of robot insertion, i.e., only the
   *   robot from s_start to specs.L is in the workspace.
   * @return shape at tau starting at s_start
   */
  TendonResult tension_shape(const std::vector<double> &tau, double s_start) const;

  /// Old unoptimized version of tension_shape()
  TendonResult tension_shape_unopt(const std::vector<double> &tau, double s_start) const;

};

inline std::ostream& operator<<(std::ostream &out, const TendonRobot &robot) {
  out << "TendonRobot{r=" << robot.r << ", specs=" << robot.specs << ", "
                      "tendons=[";
  bool first = true;
  for (auto &tendon : robot.tendons) {
    if (!first) { out << ", "; }
    out << tendon;
    first = false;
  }
  out << "], "
         "enable_rotation=" << (robot.enable_rotation ? "true" : "false")
      << ", enable_retraction=" << (robot.enable_retraction ? "true" : "false")
      << "}";
  return out;
}

} // end of namespace tendon

#endif // TENDON_ROBOT_H
