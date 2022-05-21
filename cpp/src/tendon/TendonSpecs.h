#ifndef TENDON_SPECS_H
#define TENDON_SPECS_H

#include <Eigen/Core>

#include <memory>
#include <ostream>

namespace cpptoml {
class table;
}

namespace tendon{

/** Specs for a tendon on a tendon-actuated robot
 *
 * @param C: Polynomial coefficients of the theta coordinate of the polar
 *   coordinate position relative to the backbone position.
 *     theta_i(t) = C[i][0] + C[i][1]*t + C[i][2]*t^2 + ...
 * @param D: Polynomial coefficients of the r coordintae of the polar
 *   coordinate position relative to the backbone position.
 *     r_i(t) = D[i][0] + D[i][1]*t + D[i][2]*t^2 + ...
 * @param max_tension: upper limit on tension that can be put on this tendon
 */
struct TendonSpecs {
  Eigen::VectorXd C;           // theta polynomial coefficients of tendons
  Eigen::VectorXd D;           // r polynomial coefficients of tendons
  double max_tension = 20.0;   // Newtons
  double min_length  = -0.015; // minimum string retraction (m)
  double max_length  =  0.035; // maximum string extension (m)

  // polynomial degree.  if (coef <= eps) then consider it zero
  size_t r_degree(double eps = 0.0) const;
  size_t theta_degree(double eps = 0.0) const;

  bool is_straight(double eps = 0.0) const {
    return r_degree(eps) == 0 && theta_degree(eps) == 0;
  }

  bool is_helix(double eps = 0.0) const {
    return r_degree(eps) == 0 && theta_degree(eps) == 1;
  }

  bool operator ==(const TendonSpecs &other) const {
    return C == other.C && D == other.D && max_tension == other.max_tension;
  }

  std::shared_ptr<cpptoml::table> to_toml() const;
  static TendonSpecs from_toml(std::shared_ptr<cpptoml::table> tbl);
};

std::ostream& operator<<(std::ostream &out, const TendonSpecs &specs);

} // end of namespace tendon

#endif // TENDON_SPECS_H
