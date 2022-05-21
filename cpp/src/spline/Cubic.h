#ifndef CUBIC_H
#define CUBIC_H

#include <iostream>

namespace spline {

/**
 * Represents a cubic polynomial
 *
 * T is a type that has implemented operator+() and operator*()
 */
template <typename T=double>
struct Cubic {
  using ValueType = T;
  T c0 = T();
  T c1 = T();
  T c2 = T();
  T c3 = T();

  /**
   * Evaluate the polynomial at x
   *
   * x can be a different type, as long as it can be multiplied by itself and
   * multiplied by the constants defined by type T.
   *
   * Return type is automatically determined by the type system depending on
   * types T and U.
   */
  template <typename U>
  auto operator() (U x) const {
    return c0 + c1*x + c2*x*x + c3*x*x*x;
  }

  /// Returns a cubic that is the derivative of this one
  Cubic<T> deriv() const {
    return Cubic<T>{c1, 2*c2, 3*c3, T()};
  }

  bool operator==(const Cubic<T> &other) const {
    return c0 == other.c0
        && c1 == other.c1
        && c2 == other.c2
        && c3 == other.c3;
  }
};

// for nice printing
template<typename T>
std::ostream& operator<< (std::ostream& out, const Cubic<T> &c) {
  out << "Cubic{y = "
      << c.c0 << " + "
      << c.c1 << " x + "
      << c.c2 << " x^2 + "
      << c.c3 << " x^3}";
  return out;
}

} // end of namespace spline

#endif // CUBIC_H
