#ifndef UTIL_ANGLES_H
#define UTIL_ANGLES_H

#include <boost/math/constants/constants.hpp>

#include <cmath>  // for std::abs()

namespace util {

using boost::math::double_constants::pi;

/// Like std::fmod(), but puts it in [0, b) instead of (-b, b)
template <typename T>
T fmod_floor(const T a, const T b) {
  return std::fmod(std::fmod(a, b) + b, b);
}

/** Maps val to [lower, upper] in a wrapping space (like in SO(2))
 *
 * For example, implement SO(2) with
 *
 *    util::wrap_range(theta, -pi, pi);
 */
template <typename T>
T wrap_range(const T val, const T lower, const T upper) {
  return fmod_floor(val - lower, upper - lower) + lower;
}

/// Wraps an angle from SO(2) to [-pi, pi)
template <typename T>
T canonical_angle(T theta) {
  return wrap_range(theta, T(-pi), T(pi));
}

/** Returns an equivalent angle of "theta" that is closest to the reference in
 * SO2 space, basically from [reference - pi, reference + pi)
 */
template <typename T>
T angle_close_to(T theta, T reference) {
  return wrap_range(theta, reference - T(pi), reference + T(pi));
}


} // end of namespace util

#endif // UTIL_ANGLES_H
