#ifndef UTIL_POLY_H
#define UTIL_POLY_H

#include <Eigen/Core>

namespace util {

/// Evaluate a polynomial at a particular location
inline double poly_at(const Eigen::VectorXd &coef, double t) {
  double val = 0.0;
  double tpow = 1;
  for (decltype(coef.size()) i = 0; i < coef.size(); i++) {
    val += coef[i] * tpow;
    tpow *= t;
  }
  return val;
}

/// Returns the coefficients of the polynomial's derivative
inline Eigen::VectorXd poly_der(const Eigen::VectorXd &coef) {
  auto N = coef.size();
  Eigen::VectorXd derivative = Eigen::VectorXd::Zero(N);
  if (N < 2) { return derivative; }
  for (decltype(N) i = 1; i < N; i++) {
    derivative[i-1] = i * coef[i];
  }
  return derivative;
}

} // end of namespace util

#endif // UTIL_POLY_H
